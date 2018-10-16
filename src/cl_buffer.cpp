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
              const std::string cl_name = "buffer.cl") {

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
  ros::init(argc, argv, "cl_buffer");
  ros::NodeHandle nh;

  std::string cl_file = "buffer.cl";

  if (argc > 1)
    cl_file = argv[1];

  ROS_INFO_STREAM("using " << cl_file);

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

    const int wd = 256;
    const int ht = 256;
    cv::Mat input_image = cv::Mat(cv::Size(wd,ht), CV_8UC1, cv::Scalar::all(0));
    cv::circle(input_image, cv::Point(wd/2, ht/2), wd/3, cv::Scalar::all(255), -1);

#if 0
    unsigned char im[wd * ht];

    for (int i = 0; i < ht; i++) {
      for (int j = 0; j < ht; j++) {
        // int val = int(input_image.data[i * input_image.step + j]);
        im[wd * i + j] = val;
        // if ((j%8 == 0) && (i%8 == 0))
        //  std::cout << "   " << val;
      }
      // std::cout << std::endl;
    }
#endif

    size_t origin = 0;
    size_t region = wd * ht;

    const size_t size = wd * ht;
    cl::Buffer cl_image;
    cl::Buffer cl_result;
    ROS_INFO_STREAM(wd << " " << ht);
    try {
      // std::vector<cl::Image2D>
      cl_image = cl::Buffer(context,
                            CL_MEM_READ_ONLY, // | CL_MEM_COPY_HOST_PTR,
                            size,  // TODO(lucasw) * sizeof(float) when using float
                            (void *)NULL, //(void*) &im, // some random memory
                            &err);

    } catch (cl::Error err) {
      ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");
      return EXIT_FAILURE;
    }

    try {
      cl_result = cl::Buffer(context,
                             CL_MEM_WRITE_ONLY, // | CL_MEM_COPY_HOST_PTR,
                             size,
                             NULL, //(void*) &im, // some random memory
                             &err);
    } catch (cl::Error err) {
      ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");
      return EXIT_FAILURE;
    }

    cl::NDRange global_size(wd * ht);
    // local size doesn't mean much without having local memory
    // the null range causes automatic setting of local size,
    // and because the cpu has four cores it looks like it sets the
    // local size to be the global_size divided by 3 (leaving 1 core free for
    // other tasks) though other times it looks like sets the local size to the
    // full size
    cl::NDRange local_size = cl::NullRange;
    // cl::NDRange local_size(16, 16);
    // this is just a global offset, don't think global size has to shrink
    // by same amount
    // Null and 0,0 should be the same
    // cl::NDRange offset(0, 0);
    cl::NDRange offset = cl::NullRange;

    cl::Kernel kernel;
    std::string old_text = "";

    cl::Program program;
    cl_int rv = loadProg(devices, context, program, old_text, cl_file);
    if (rv == CL_SUCCESS) {

      try {
        kernel = cl::Kernel(program, "pressure", &err);
        kernel.setArg(0, cl_image);
        kernel.setArg(1, cl_result);
        kernel.setArg(2, wd);
        kernel.setArg(3, ht);
      } catch (cl::Error err) {
        ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err()
                                   << ")");
        return EXIT_FAILURE;
      }
    }

    cv::imshow("input image", input_image);
    cv::Mat imt = cv::Mat(cv::Size(wd, ht), CV_8UC1, cv::Scalar::all(0));
    cv::imshow("processed image", imt);
    int ch = cv::waitKey(0);

    int i = 0;
    // while (ros::ok()) {
    {
      ROS_INFO_STREAM("step");
      queue.enqueueWriteBuffer(cl_image,
                               CL_TRUE, // blocking write
                               origin, region,
                               input_image.data //,
                                        // NULL,
                                        //&event
      );

      queue.enqueueNDRangeKernel(kernel, offset, global_size, local_size, NULL,
                                 &event);
      // queue.enqueueNDRangeKernel(kernel, offset, global_size, local_size, NULL,
      //                            &event);

      queue.enqueueReadBuffer(cl_result,
                              CL_TRUE, // blocking read
                              origin, region,
                              imt.data //_out //,
                                // NULL,
                                //&event
      );

      cv::imshow("processed image", imt);
      ch = cv::waitKey(0);
      // if (ch == 'q')
      //   break;

      i++;
    } // for loop

    ////
    #if 1
    if (false) {
      ROS_INFO_STREAM("output ");
      for (int i = 0; i < ht; i++) {
        for (int j = 0; j < wd; j++) {
          std::cout << "   " << int(imt.at<char>(wd, ht));
        }
        std::cout << std::endl;
      }
    }
    #endif
    ////
  } catch (cl::Error err) {
    ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");
    return EXIT_FAILURE;
  }
  // cv::waitKey(0);
  // ping = !ping;
  return EXIT_SUCCESS;
}
