#include "cl_misc.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

void pressure(const cv::Mat& past,
              const cv::Mat& pres,
              cv::Mat& futu,
              int width, int height) {

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float val = 0;
      if (x > 0)
        val += pres.at<float>(y, x - 1) / 2.0;
      if (y > 0)
        val += pres.at<float>(y - 1, x) / 2.0;
      if (x < width - 1)
        val += pres.at<float>(y, x + 1) / 2.0;
      if (y < height - 1)
        val += pres.at<float>(y + 1, x) / 2.0;

      futu.at<float>(y, x) = val - past.at<float>(y, x);
    }
  }
};

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

  bool use_gpu = true;
  ros::param::get("~use_gpu", use_gpu);

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

    int wd = 400;
    ros::param::get("~width", wd);
    int ht = 400;
    ros::param::get("~height", ht);
    cv::Mat input_image = cv::Mat(cv::Size(wd,ht), CV_32FC1, cv::Scalar::all(0));
    cv::circle(input_image, cv::Point(wd/2, ht/2), wd/3, cv::Scalar::all(0.2), 3);
    cv::Mat blank_image = cv::Mat(cv::Size(wd,ht), CV_32FC1, cv::Scalar::all(0));

    // for non gpu test
    std::array<cv::Mat, 3> cv_image;
    for (size_t i = 0; i < cv_image.size(); ++i) {
      cv_image[i] = blank_image.clone();
    }
    cv_image[1] = input_image.clone();

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
    size_t region = wd * ht * sizeof(float);

    const size_t buffer_size = wd * ht * sizeof(float);
    // past, present, future
    std::array<cl::Buffer, 3> cl_image;
    ROS_INFO_STREAM(wd << " " << ht << " " << buffer_size);
    try {
      for (size_t i = 0; i < cl_image.size(); ++i) {
        cl_image[i] = cl::Buffer(context,
                              0,  // CL_MEM_READ_ONLY, // | CL_MEM_COPY_HOST_PTR,
                              buffer_size,  // TODO(lucasw) * sizeof(float) when using float
                              (void *)NULL, //(void*) &im, // some random memory
                              &err);
      }
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

    std::array<cl::Kernel, cl_image.size()> kernels;
    std::string old_text = "";

    cl::Program program;
    cl_int rv = loadProg(devices, context, program, old_text, cl_file);
    if (rv != CL_SUCCESS) {
      return EXIT_FAILURE;
    }
    try {
      for (size_t i = 0; i < kernels.size(); ++i) {
        kernels[i] = cl::Kernel(program, "pressure", &err);
        kernels[i].setArg(0, cl_image[i]);
        kernels[i].setArg(1, cl_image[(i + 1) % kernels.size()]);
        kernels[i].setArg(2, cl_image[(i + 2) % kernels.size()]);
        kernels[i].setArg(3, wd);
        kernels[i].setArg(4, ht);
      }
    } catch (cl::Error err) {
      ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err()
                                 << ")");
      return EXIT_FAILURE;
    }

    cv::imshow("input image", input_image);
    // cv::Mat imt = cv::Mat(cv::Size(wd, ht), CV_8UC1, cv::Scalar::all(0));
    // cv::imshow("processed image", imt);
    int ch = cv::waitKey(0);

    // clear all the images
    if (use_gpu) {
      for (size_t i = 0; i < cl_image.size(); ++i) {
      queue.enqueueWriteBuffer(cl_image[0],
                               CL_TRUE, // blocking write
                               origin, region,
                               blank_image.data //,
                                        // NULL,
                                        //&event
      );
      }
      queue.enqueueBarrierWithWaitList();
      // now write the circle to the 'present' buffer
      queue.enqueueWriteBuffer(cl_image[1],
                               CL_TRUE, // blocking write
                               origin, region,
                               input_image.data //,
                                        // NULL,
                                        //&event
      );
      queue.enqueueBarrierWithWaitList();
    }

    int num_kernel_loops = 100;
    ros::param::get("~loops", num_kernel_loops);
    bool save_images = false;
    ros::param::get("~save_images", save_images);

    int outer_loops = 0;
    int inner_loops = 0;
    while (ros::ok()) {
      ROS_DEBUG_STREAM("step " << outer_loops << " " << inner_loops);
      int num = 0;
      ros::Time t0 = ros::Time::now();

      const size_t runs = cl_image.size() * num_kernel_loops;
      if (use_gpu) {
        // const size_t runs = kernels.size() * num_kernel_loops;
        for (size_t i = 0; i < runs; ++i) {
          queue.enqueueNDRangeKernel(kernels[num % kernels.size()], offset,
                                     global_size,  // * sizeof(float),
                                     local_size, NULL,
                                     &event);
          queue.enqueueBarrierWithWaitList();
          num++;
          inner_loops++;
        }
        queue.enqueueReadBuffer(cl_image[(num + 1) % cl_image.size()],
                                CL_TRUE, // blocking read
                                origin, region,
                                input_image.data //_out //,
                                  // NULL,
                                  //&event
        );
      } else {
        for (size_t i = 0; i < runs; ++i) {
          const int past = (num + 0) % cv_image.size();
          const int pres = (num + 1) % cv_image.size();
          const int futu = (num + 2) % cv_image.size();
          pressure(cv_image[past], cv_image[pres], cv_image[futu],
                   wd, ht);
        }
        input_image = cv_image[0].clone();
      }
      ros::Time t1 = ros::Time::now();
      const float runs_per_sec = runs / (t1 - t0).toSec();
      ROS_INFO_STREAM("runs per sec " << runs_per_sec << ", nodes per second "
                      << wd * ht * runs_per_sec);

      if (save_images) {
        std::stringstream ss;
        ss << (outer_loops + 1000);
        std::string name = "cl_prop_" + ss.str().substr(1) + ".png";
        cv::imwrite(name, input_image * 255);
      }
      cv::imshow("input image", input_image);
      ch = cv::waitKey(5);
      if (ch == 'q')
        break;
      if (ch == 'r') {

      }
      ++outer_loops;
    } // for loop

    ////
    #if 1
    if (false) {
      ROS_INFO_STREAM("output ");
      for (int i = 0; i < ht; i++) {
        for (int j = 0; j < wd; j++) {
          std::cout << "   " << int(input_image.at<float>(wd, ht));
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
