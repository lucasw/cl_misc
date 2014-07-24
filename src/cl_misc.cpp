#include "cl_misc.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(void)
{
  cl_int err = CL_SUCCESS;
  try {

    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    if (platforms.size() == 0) {
      std::cout << "Platform size 0\n";
      return EXIT_FAILURE;
    }

    cl_context_properties properties[] = {   
        CL_CONTEXT_PLATFORM, 
        (cl_context_properties)(platforms[0])(), 
        0 // the callback
        };
    cl::Context context(CL_DEVICE_TYPE_CPU, properties); 

    std::vector<cl::Device> devices = context.getInfo<CL_CONTEXT_DEVICES>();

    std::string cl_name = "misc.cl";
    std::ifstream cl_file;
    cl_file.open(cl_name.c_str()); //, std::ios::in);
    if (!cl_file.is_open()) {
      std::cerr << "Could not open " << cl_name.c_str() << std::endl;
      return EXIT_FAILURE;
    }
    std::string big_line;
    std::string line;
    while (std::getline(cl_file, line)) {
      big_line += line + "\n"; 
    }
    std::cout << big_line << std::endl;

    cl::Program::Sources source(
        1,
        std::make_pair(big_line.c_str(), strlen( big_line.c_str() ))
        );
    cl::Program program_ = cl::Program(context, source);
    program_.build(devices);


    cl::Event event;
    cl::CommandQueue queue(context, devices[0], 0, &err);
   
    const int wd = 512;
    const int ht = 512;

    cv::Mat imc = cv::Mat(cv::Size(wd,ht), CV_8UC1, cv::Scalar::all(0));
    cv::circle(imc, cv::Point(wd/2, ht/2), wd/3, cv::Scalar::all(255), -1);
    
    cv::imshow("orig", imc);
    unsigned char im[wd * ht];
   
    for (int i = 0; i < ht; i++) {
    for (int j = 0; j < ht; j++) {
      int val =  int( imc.data[i*imc.step + j] );
      im[wd * i + j] = val;
      //if ((j%8 == 0) && (i%8 == 0)) 
      //  std::cout << "   " << val; 
    }
      //std::cout << std::endl;
    }
    
    //unsigned char im_out[wd * ht];

    cl::size_t<3> origin;
    origin[0] = 0; origin[1] = 0, origin[2] = 0;
    cl::size_t<3> region;
    region[0] = wd; region[1] = ht; region[2] = 1;
    
    cl::Kernel kernel(program_, "hello", &err);

    //bool ping = true;

    cl::Image2D cl_image = cl::Image2D(
        context,
        CL_MEM_READ_ONLY, // | CL_MEM_COPY_HOST_PTR,
        cl::ImageFormat(CL_R, CL_UNSIGNED_INT8),  // single channel
        wd,
        ht,
        0,
        NULL, //(void*) &im, // some random memory
        &err
        );

    cl::Image2D cl_result = cl::Image2D(
        context,
        CL_MEM_WRITE_ONLY, // | CL_MEM_COPY_HOST_PTR,
        cl::ImageFormat(CL_R, CL_UNSIGNED_INT8),  // single channel
        wd,
        ht,
        0,
        NULL, //(void*) &im, // some random memory
        &err
        );
      
    kernel.setArg(0, cl_image);
    kernel.setArg(1, cl_result);
    cl::NDRange global_size(wd, ht);
    cl::NDRange local_size(16, 16);

    //for (int i = 0; i < 100; i++)
    while (true) 
    {
      
      queue.enqueueWriteImage(
          cl_image, 
          CL_TRUE, // blocking write 
          origin,
          region,
          0,0,
          im //,
          //NULL,
          //&event
          );
       
      queue.enqueueNDRangeKernel(
          kernel, 
          cl::NullRange,
          global_size,
          local_size,
          NULL,
          &event); 

      queue.enqueueReadImage(
          cl_result, 
          CL_TRUE, // blocking read 
          origin,
          region,
          0,0,
          im //_out //,
          //NULL,
          //&event
          );
    
      cv::Mat imt = cv::Mat(cv::Size(wd,ht), CV_8UC1, im);
      cv::imshow("processed image", imt);
      int ch = cv::waitKey(5);

      if (ch == 'q') break;
      //ping = !ping;
    } // for loop      

    if (false) {
    std::cout << "output " << std::endl;
    for (int i = 0; i < ht; i++) {
    for (int j = 0; j < ht; j++) {
      std::cout << "   " << int( im[i*wd + j] );
    }
      std::cout << std::endl;
    }
    }
  }
  catch (cl::Error err) {
    std::cerr 
      << "ERROR: "
      << err.what()
      << "("
      << err.err()
      << ")"
      << std::endl;
    return EXIT_FAILURE;
  }
  cv::waitKey(0);
      //ping = !ping;
  return EXIT_SUCCESS;
}
