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


Trying nvidia-331, install is long process

Rebooted and now sudo glxinfo works, but now optirun clinfo gives the same info as plain clinfo.

sudo apt-get install ocl-icd-dev

No improvement


/etc/OpenCL/vendors/nvidia.icd is missing?

No symlink to /usr/lib/x86_64-linux-gnu/libOpenCL.so, there is just a so.1, so.1.0, so.1.0.0

sudo ln -s /usr/lib/x86_64-linux-gnu/libOpenCL.so.1 /usr/lib/x86_64-linux-gnu/libOpenCL.so

But the clinfo message was already looking at so.1

give up on nvidia, try intel

Intel OpenCL
------------



