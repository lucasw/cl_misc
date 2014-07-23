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


intel_sdk_for_ocl_applications_2014_ubuntu_4.4.0.117_x64.tgz

ran install script


(Reading database ... 486473 files and directories currently installed.)
Preparing to unpack opencl-1.2-base-4.4.0.117-1.x86_64.deb ...
Unpacking opencl-base (1.2-4.4.0.117) over (1.2-4.4.0.117) ...
Setting up opencl-base (1.2-4.4.0.117) ...
/var/lib/dpkg/info/opencl-base.postinst: 2: /var/lib/dpkg/info/opencl-base.postinst: [[: not found
update-alternatives: priority is out of range

It is trying to send a large negative number to update-alternatives

(don't do this see xfanzone solution below) Try dpkg -x on opencl base deb and copying in intel_sdk_for_ocl_applications_2014_ubuntu_4.4.0.117_x64/temp/opt/intel/opencl-1.2-4.4.0.117/lib64/libOpenCL manually

But get  sudo dpkg -i  opencl-1.2-intel-cpu-4.4.0.117-1.x86_64.deb opencl-1.2-devel-4.4.0.117-1.x86_64.deb opencl-1.2-intel-devel-4.4.0.117-1.x86_64.deb opencl-1.2-intel-devel-android-4.4.0.117-1.x86_64.deb 

dpkg: dependency problems prevent configuration of opencl-intel-cpu:
 opencl-intel-cpu depends on opencl-base (>= 1.2-4.4.0.117); however:
   Package opencl-base is not configured yet.

How does it know whether it is configured?

dpkg -x all the other debs except the android one, looks like a mess: qt libs, and boost libs that are out of sync with my system.

Follow these instructions:
http://xfanzone.me/fixing-opencl-deb.html

Had to remove some CL libs and includes that were left over from previous failed installs, then the install script works!

  clinfo
  Number of platforms:         1
    Platform Profile:        FULL_PROFILE
    Platform Version:        OpenCL 1.2 LINUX
    Platform Name:         Intel(R) OpenCL
    Platform Vendor:         Intel(R) Corporation
    Platform Extensions:         cl_khr_icd cl_khr_global_int32_base_atomics cl_khr_global_int32_extended_atomics cl_khr_local_int32_base_atomics cl_khr_local_int32_extended_atomics cl_khr_byte_addressable_store cl_khr_spir cl_intel_exec_by_local_thread cl_khr_depth_images cl_khr_3d_image_writes cl_khr_fp64 


    Platform Name:         Intel(R) OpenCL
  Number of devices:         1
    Device Type:           CL_DEVICE_TYPE_CPU
    Device ID:           32902
    Max compute units:         4
    Max work items dimensions:       3
      Max work items[0]:         8192
      Max work items[1]:         8192
      Max work items[2]:         8192
    Max work group size:         8192
    Preferred vector width char:       1
    Preferred vector width short:      1
    Preferred vector width int:      1
    Preferred vector width long:       1
    Preferred vector width float:      1
    Preferred vector width double:     1
    Native vector width char:      16
    Native vector width short:       8
    Native vector width int:       4
    Native vector width long:      2
    Native vector width float:       8
    Native vector width double:      4
    Max clock frequency:         2800Mhz
    Address bits:          64
    Max memory allocation:       2065366016
    Image support:         Yes
    Max number of images read arguments:     480
    Max number of images write arguments:    480
    Max image 2D width:        16384
    Max image 2D height:         16384
    Max image 3D width:        2048
    Max image 3D height:         2048
    Max image 3D depth:        2048
    Max samplers within kernel:      480
    Max size of kernel argument:       3840
    Alignment (bits) of base address:    1024
    Minimum alignment (bytes) for any datatype:  128
    Single precision floating point capability
      Denorms:           Yes
      Quiet NaNs:          Yes
      Round to nearest even:       Yes
      Round to zero:         No
      Round to +ve and infinity:       No
      IEEE754-2008 fused multiply-add:     No
    Cache type:          Read/Write
    Cache line size:         64
    Cache size:          262144
    Global memory size:        8261464064
    Constant buffer size:        131072
    Max number of constant args:       480
    Local memory type:         Global
    Local memory size:         32768
    Error correction support:      0
    Unified memory for Host and Device:    1
    Profiling timer resolution:      1
    Device endianess:        Little
    Available:           Yes
    Compiler available:        Yes
    Execution capabilities:        
      Execute OpenCL kernels:      Yes
      Execute native function:       Yes
    Queue properties:        
      Out-of-Order:        Yes
      Profiling :          Yes
    Platform ID:           0x24f1aa0
    Name:                   Intel(R) Core(TM) i7-2640M CPU @ 2.80GHz
    Vendor:          Intel(R) Corporation
    Device OpenCL C version:       OpenCL C 1.2 
    Driver version:        1.2.0.117
    Profile:           FULL_PROFILE
    Version:           OpenCL 1.2 (Build 117)
    Extensions:          cl_khr_icd cl_khr_global_int32_base_atomics cl_khr_global_int32_extended_atomics cl_khr_local_int32_base_atomics cl_khr_local_int32_extended_atomics cl_khr_byte_addressable_store cl_khr_spir cl_intel_exec_by_local_thread cl_khr_depth_images cl_khr_3d_image_writes cl_khr_fp64 
All it appears to be showing is the quad core cpu, so it isn't that exciting.

Try on another pc next.


---

Try installing libnuma-dev, some pages suggest it

What about the opencl runtime from Intel?

opencl_runtime_14.1_x64_4.4.0.117.tgz

The install-cpu+mic.sh script is for Xeon Phi coprocessor, which I don't have.  Should the HD 3000 graphics have a gpu?

This page says no:
http://www.intel.com/support/graphics/sb/CS-033757.htm

Move on to making some opencl test program

OpenCL test program
-------------------

Start with the one in the cl.hpp comments.


