cl_misc
=======

OpenCL experimentation

Ubuntu 14.04 setup
------------------


Video card:
NVIDIA Corporation GF108M [GeForce GT 525M]
or Intel HD 3000

Need to use optirun to use Nvidia?

Which nvidia version?  304.117
  dpkg -l | grep nvidia
  ii  bumblebee-nvidia                                      3.2.1-5                                             amd64        NVIDIA Optimus support using the proprietary NVIDIA driver
  ii  nvidia-304                                            304.117-0ubuntu1                                    amd64        NVIDIA legacy binary driver - version 304.117
  ii  nvidia-current                                        304.117-0ubuntu1                                    amd64        Transitional package for nvidia-current
  ii  nvidia-libopencl1-304                                 304.117-0ubuntu1                                    amd64        NVIDIA OpenCL Driver and ICD Loader library
  ii  nvidia-opencl-icd-304                                 304.117-0ubuntu1                                    amd64        NVIDIA OpenCL ICD
  ii  nvidia-settings                                       331.20-0ubuntu8                                     amd64        Tool for configuring the NVIDIA graphics driver

  
  dpkg -l | grep opencl
  ii  nvidia-libopencl1-304                                 304.117-0ubuntu1                                    amd64        NVIDIA OpenCL Driver and ICD Loader library
  rc  nvidia-libopencl1-331                                 331.38-0ubuntu7                                     amd64        NVIDIA OpenCL Driver and ICD Loader library
  ii  nvidia-opencl-icd-304                                 304.117-0ubuntu1                                    amd64        NVIDIA OpenCL ICD
  ii  opencl-headers                                        1.2-2013.10.23-1                                    all          OpenCL (Open Computing Language) header files
  ii  unity-scope-openclipart                               0.1+13.10.20130723-0ubuntu1                         all          OpenClipArt scope for Unity



 clinfo isn't looking too good:

  sudo apt-get install clinfo

  $ clinfo
  clinfo: /usr/lib/x86_64-linux-gnu/libOpenCL.so.1: no version information available (required by clinfo)
  I: ICD loader reports no usable platforms
  $ optirun clinfo 
  [ 2932.786458] [ERROR]Cannot access secondary GPU - error: [XORG] (EE) NVIDIA(0): Failed to initialize the NVIDIA GPU at PCI:1:0:0.  Please

  [ 2932.786585] [ERROR]Aborting because fallback start is disabled.

