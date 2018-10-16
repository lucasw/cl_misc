#include "cl_misc.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>


// load the opencl program from the disk
// TBD optionally provide the old program, if it hasn't changed
// then don't rebuild it
bool loadProg(std::vector<cl::Device> &devices, cl::Context &context,
              cl::Program &program, std::string &old_text,
              const std::string cl_name = "misc.cl") {

  // look at using inotfiy to see changes to file
  // http://stackoverflow.com/questions/4664975/monitoring-file-using-inotify
  // http://en.highscore.de/cpp/boost/asio.html
  // http://boost.2283326.n4.nabble.com/ASIO-file-monitoring-help-td4645105.html
  // http://stackoverflow.com/questions/12564039/alternatives-to-inotify-to-detect-when-a-new-file-is-created-under-a-folder/24970801#24970801
  try {
    std::ifstream cl_file;
    cl_file.open(cl_name.c_str()); //, std::ios::in);
    if (!cl_file.is_open()) {
      ROS_ERROR_STREAM("Could not open " << cl_name.c_str());
      return false;
    }
    std::string big_line;
    std::string line;
    while (std::getline(cl_file, line)) {
      big_line += line + "\n";
    }
    // std::cout << big_line << std::endl;
    if (old_text.compare(big_line) == 0) {
      // don't bother trying to load program if it hasn't changed
      return 1;
    }
    old_text = big_line;

    cl::Program::Sources source(
        1, std::make_pair(big_line.c_str(), strlen(big_line.c_str())));

    program = cl::Program(context, source);
    cl_int rv = program.build(devices);

    return rv;
  } catch (cl::Error err) {
    ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");

    ROS_INFO_STREAM(
        "build status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(
            devices[0]));
    ROS_INFO_STREAM("build options:\t"
                    << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(
                           devices[0]));
    ROS_INFO_STREAM("build log:\t "
                    << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]));

    return err.err();
  }
}

// provide the location of the .cl file in the first argument
// then optionally provide the video number, 0 for /dev/video0 and so on,
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cl_misc");
  ros::NodeHandle nh;

  int video_num = 0;
  std::string cl_file = "misc.cl";

  if (argc > 1)
    cl_file = argv[1];
  if (argc > 2)
    video_num = atoi(argv[2]);

  ROS_INFO_STREAM("live coding with " << cl_file << " using /dev/video"
                                      << video_num << " as data source");

  cv::VideoCapture cap(video_num);
  if (!cap.isOpened()) {
    ROS_ERROR_STREAM("could not open /dev/video" << video_num);
    return -1;
  }

  cl_int err = CL_SUCCESS;
  try {

    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    if (platforms.size() == 0) {
      ROS_INFO_STREAM("Platform size 0");
      return EXIT_FAILURE;
    }

    for (cl::Platform &p : platforms) {
      ROS_INFO_STREAM("Platform name: "
                      << p.getInfo<CL_PLATFORM_NAME>() << "\n"
                      << "Vendor: " << p.getInfo<CL_PLATFORM_VENDOR>() << "\n"
                      << "Version: " << p.getInfo<CL_PLATFORM_VERSION>() << "\n"
                      << "Profile: " << p.getInfo<CL_PLATFORM_PROFILE>() << "\n"
                      << "Extenstions: "
                      << p.getInfo<CL_PLATFORM_EXTENSIONS>());
    }

    cl_context_properties properties[] = {
        CL_CONTEXT_PLATFORM, (cl_context_properties)(platforms[0])(),
        0 // the callback
    };
    // cl::Context context(CL_DEVICE_TYPE_CPU, properties);
    cl::Context context(CL_DEVICE_TYPE_GPU, properties);

    std::vector<cl::Device> devices = context.getInfo<CL_CONTEXT_DEVICES>();

    for (cl::Device &d : devices) {
      // TODO(lucasw) INFO_STREAM didn't print out much of this, just the first line
      // ROS_INFO_STREAM(
      std::cout <<
          "Device name: "
          << d.getInfo<CL_DEVICE_NAME>() << ", "
          << "extensions: " << d.getInfo<CL_DEVICE_EXTENSIONS>() << ", "
          << "global mem size: "
          << d.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>() * 9.53674e-7 << "mb"
          << "\n"
          << "local mem size: "
          << d.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>() * 0.000976562 << "kb"
          << "\n"
          << "availability: "
          << (d.getInfo<CL_DEVICE_AVAILABLE>() == true ? "true" : "false")
          << "\n"
          << "max work item dim: "
          << d.getInfo<CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS>() << "\n"
          << "image processing support: "
          << (d.getInfo<CL_DEVICE_IMAGE_SUPPORT>() == CL_TRUE ? "true"
                                                              : "false") << "\n";

      auto work_sizes = d.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>();
      for (int i = 0; i < work_sizes.size(); ++i)
        ROS_INFO_STREAM("dimension: " << i << " size: " << work_sizes.at(i));
    }

    cl::Event event;
    cl::CommandQueue queue(context, devices[0], 0, &err);

    // const int wd = 512;
    // const int ht = 512;
    // cv::Mat imc = cv::Mat(cv::Size(wd,ht), CV_8UC1, cv::Scalar::all(0));
    // cv::circle(imc, cv::Point(wd/2, ht/2), wd/3, cv::Scalar::all(255), -1);
    cv::Mat imc;
    cap >> imc;
    const int wd = imc.cols;
    const int ht = imc.rows;

    unsigned char im[wd * ht];

    for (int i = 0; i < ht; i++) {
      for (int j = 0; j < ht; j++) {
        int val = int(imc.data[i * imc.step + j]);
        im[wd * i + j] = val;
        // if ((j%8 == 0) && (i%8 == 0))
        //  std::cout << "   " << val;
      }
      // std::cout << std::endl;
    }

    // unsigned char im_out[wd * ht];

    cl::size_t<3> origin;
    origin[0] = 0;
    origin[1] = 0, origin[2] = 0;
    cl::size_t<3> region;
    region[0] = wd;
    region[1] = ht;
    region[2] = 1;

    cl::Image2D cl_image;
    cl::Image2D cl_result;
    cl::ImageFormat format(CL_R, CL_UNSIGNED_INT8); // single channel
    ROS_INFO_STREAM(wd << " " << ht);
    try {
      // std::vector<cl::Image2D>
      cl_image = cl::Image2D(context,
                             CL_MEM_READ_ONLY, // | CL_MEM_COPY_HOST_PTR,
                             format, wd, ht, 0,
                             NULL, //(void*) &im, // some random memory
                             &err);

    } catch (cl::Error err) {
      ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");
      return EXIT_FAILURE;
    }

    try {
      cl_result = cl::Image2D(context,
                              CL_MEM_WRITE_ONLY, // | CL_MEM_COPY_HOST_PTR,
                              format, wd, ht, 0,
                              NULL, //(void*) &im, // some random memory
                              &err);
    } catch (cl::Error err) {
      ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");
      return EXIT_FAILURE;
    }

    cl::NDRange global_size(wd, ht);
    // local size doesn't mean much without having local memory
    // the null range causes automatic setting of local size,
    // and because the cpu has four cores it looks like it sets the
    // local size to be the global_size divided by 3 (leaving 1 core free for
    // other tasks) though other times it looks like sets the local size to the
    // full size
    // cl::NDRange local_size= cl::NullRange;
    cl::NDRange local_size(16, 16);
    // this is just a global offset, don't think global size has to shrink
    // by same amount
    // Null and 0,0 should be the same
    // cl::NDRange offset(0, 0);
    cl::NDRange offset = cl::NullRange;

    cl::Kernel kernel;
    bool loaded_one_good_program = false;
    std::string old_text = "";

    int i = 0;
    while (true) {
      cap >> imc;
      if ((imc.rows != ht) || (imc.cols != wd)) {
        ROS_ERROR_STREAM("bad image " << imc.cols << " " << imc.rows
                                      << " != " << wd << " " << ht);
        continue;
      }
      cv::Mat gray;
      cv::cvtColor(imc, gray, CV_BGR2GRAY);

      // only do this intermittently
      if (i % 10 == 0) {
        cl::Program program;
        cl_int rv = loadProg(devices, context, program, old_text, cl_file);
        if (rv == CL_SUCCESS) {

          try {
            cl::Kernel new_kernel = cl::Kernel(program, "hello", &err);
            new_kernel.setArg(0, cl_image);
            new_kernel.setArg(1, cl_result);
            loaded_one_good_program = true;
            kernel = new_kernel;
          } catch (cl::Error err) {
            ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err()
                                       << ")");
          }
        }
      } // load/reload program
      if (!loaded_one_good_program)
        continue;

      queue.enqueueWriteImage(cl_image,
                              CL_TRUE, // blocking write
                              origin, region, 0, 0,
                              gray.data //,
                                        // NULL,
                                        //&event
      );

      queue.enqueueNDRangeKernel(kernel, offset, global_size, local_size, NULL,
                                 &event);

      queue.enqueueReadImage(cl_result,
                             CL_TRUE, // blocking read
                             origin, region, 0, 0,
                             im //_out //,
                                // NULL,
                                //&event
      );

      cv::Mat imt = cv::Mat(cv::Size(wd, ht), CV_8UC1, im);
      cv::imshow("processed image", imt);
      int ch = cv::waitKey(5);

      if (ch == 'q')
        break;

      i++;
    } // for loop

    ////
    if (false) {
      ROS_INFO_STREAM("output ");
      for (int i = 0; i < ht; i++) {
        for (int j = 0; j < ht; j++) {
          std::cout << "   " << int(im[i * wd + j]);
        }
        std::cout << std::endl;
      }
    }
    ////
  } catch (cl::Error err) {
    ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");
    return EXIT_FAILURE;
  }
  // cv::waitKey(0);
  // ping = !ping;
  return EXIT_SUCCESS;
}
