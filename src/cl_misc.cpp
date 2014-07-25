#include "cl_misc.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// load the opencl program from the disk
// TBD optionally provide the old program, if it hasn't changed
// then don't rebuild it
bool loadProg(
    std::vector<cl::Device>& devices, 
    cl::Context& context,
    cl::Program& program, 
    std::string cl_name = "misc.cl"
    )
{

  try {
    std::ifstream cl_file;
    cl_file.open(cl_name.c_str()); //, std::ios::in);
    if (!cl_file.is_open()) {
      std::cerr << "Could not open " << cl_name.c_str() << std::endl;
      return false;
    }
    std::string big_line;
    std::string line;
    while (std::getline(cl_file, line)) {
      big_line += line + "\n"; 
    }
    //std::cout << big_line << std::endl;

    cl::Program::Sources source(
        1,
        std::make_pair(big_line.c_str(), strlen( big_line.c_str() ))
        );

    program = cl::Program(context, source);
    cl_int rv = program.build(devices);
    return rv;
  } 
  catch (cl::Error err) {
    std::cerr 
      << "ERROR: "
      << err.what()
      << "("
      << err.err()
      << ")"
      << std::endl;
    return err.err();
  }
}

int main(void)
{
  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    std::cerr << "could not open /dev/video0" << std::endl;
    return -1;
  }

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


    cl::Event event;
    cl::CommandQueue queue(context, devices[0], 0, &err);
   
    //const int wd = 512;
    //const int ht = 512;
    //cv::Mat imc = cv::Mat(cv::Size(wd,ht), CV_8UC1, cv::Scalar::all(0));
    //cv::circle(imc, cv::Point(wd/2, ht/2), wd/3, cv::Scalar::all(255), -1);
    cv::Mat imc;
    cap >> imc;
    const int wd = imc.cols;
    const int ht = imc.rows;

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
    
    
    //std::vector<cl::Image2D>
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
      
    cl::NDRange global_size(wd, ht);
    // local size doesn't mean much without having local memory
    // the null range causes automatic setting of local size,
    // and because the cpu has four cores it looks like it sets the
    // local size to be the global_size divided by 3 (leaving 1 core free for other tasks)
    // though other times it looks like sets the local size to the full size
    //cl::NDRange local_size= cl::NullRange; 
    cl::NDRange local_size(16, 16);
    // this is just a global offset, don't think global size has to shrink
    // by same amount
    // Null and 0,0 should be the same
    //cl::NDRange offset(0, 0);
    cl::NDRange offset = cl::NullRange; 

    cl::Kernel kernel;
    
    int i = 0;
    while (true) 
    {
      cap >> imc;
      if ((imc.rows != ht) || 
          (imc.cols != wd)) {
        std::cerr << "bad image " << imc.cols << " " << imc.rows 
            << " != " << wd << " " << ht
            << std::endl;
        continue;
      }
      cv::Mat gray;
      cv::cvtColor(imc, gray, CV_BGR2GRAY);
      
      // only do this intermittently
      if (i % 10 == 0) {
        cl::Program program;
        if (loadProg(devices, context, program) != CL_SUCCESS) { 
          std::cerr << "bad program" << std::endl;
          continue;
        }
        kernel = cl::Kernel(program, "hello", &err);
        kernel.setArg(0, cl_image);
        kernel.setArg(1, cl_result);
      }

      queue.enqueueWriteImage(
          cl_image, 
          CL_TRUE, // blocking write 
          origin,
          region,
          0,0,
          gray.data //,
          //NULL,
          //&event
          );
      
      
      queue.enqueueNDRangeKernel(
          kernel, 
          offset,
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

      i++;
    } // for loop      


    ////
    if (false) {
    std::cout << "output " << std::endl;
    for (int i = 0; i < ht; i++) {
    for (int j = 0; j < ht; j++) {
      std::cout << "   " << int( im[i*wd + j] );
    }
      std::cout << std::endl;
    }
    }
    ////
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
  //cv::waitKey(0);
      //ping = !ping;
  return EXIT_SUCCESS;
}
