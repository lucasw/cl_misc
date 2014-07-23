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

    cl::Kernel kernel(program_, "hello", &err);

    cl::Event event;
    cl::CommandQueue queue(context, devices[0], 0, &err);
    queue.enqueueNDRangeKernel(
        kernel, 
        cl::NullRange,
        cl::NDRange(4,4),
        cl::NullRange,
        NULL,
        &event); 

    event.wait();
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
