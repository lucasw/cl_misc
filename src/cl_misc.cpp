#include "cl_misc.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

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
   
    const int wd = 16;
    const int ht = 16;
    unsigned char im[wd * ht];
    
    for (int i = 0; i < ht; i++) {
    for (int j = 0; j < ht; j++) {
      std::cout << "   " << int( im[i*wd + j] );
    }
      std::cout << std::endl;
    }

    cl::Image2D cl_image = cl::Image2D(
        context,
         CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
         cl::ImageFormat(CL_R, CL_UNSIGNED_INT8),  // single channel
         wd,
         ht,
         0,
         (void*) &im, // some random memory
         &err
         );

    unsigned char im_out[wd * ht];
    cl::Buffer cl_result = cl::Buffer(
        context, 
        CL_MEM_WRITE_ONLY, // | CL_MEM_COPY_HOST_PTR,
        sizeof(unsigned char) * wd * ht //,
        //im_out
        );
    
    //std::cout << "made buffer" << std::endl;

    cl::Kernel kernel(program_, "hello", &err);
    kernel.setArg(0, cl_image);
    kernel.setArg(1, cl_result);

    queue.enqueueNDRangeKernel(
        kernel, 
        cl::NullRange,
        cl::NDRange(4,4),
        cl::NullRange,
        NULL,
        &event); 

    //float 
    queue.enqueueReadBuffer(
        cl_result, 
        CL_TRUE, // blocking read 
        0, 
        sizeof(unsigned char) * wd * ht, 
        im_out //,
        //NULL,
        //&event
        );
           
    //event.wait();
 
    std::cout << "output " << std::endl;
    for (int i = 0; i < ht; i++) {
    for (int j = 0; j < ht; j++) {
      std::cout << "   " << int( im_out[i*wd + j] );
    }
      std::cout << std::endl;
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
  return EXIT_SUCCESS;
}
