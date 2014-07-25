__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | 
    CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void hello(
    __read_only image2d_t image,
    __write_only image2d_t im_out
    )
{ 
  
  const int2 pos = {get_global_id(0), get_global_id(1)};
  uint4 val = read_imageui(image, sampler, pos);
  
  const int2 pos2 = {get_global_id(0), get_global_id(1) + 1};
  uint4 val2 = read_imageui(image, sampler, pos2);

  const int fr = 4;
  write_imageui(im_out, pos, get_local_id(0)*fr + get_local_id(1)*fr); // (val + val2)/2 );
};

