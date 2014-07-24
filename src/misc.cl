__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | 
    CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void hello(
    __read_only image2d_t image,
    __write_only image2d_t im_out
    )
{ 
  
  const int2 pos = {get_global_id(0), get_global_id(1)};
  uint4 val = read_imageui(image, sampler, pos);
  
  const int2 pos2 = {get_global_id(0) + 1, get_global_id(1)};
  uint4 val2 = read_imageui(image, sampler, pos2);

  write_imageui(im_out, pos, val - val2/2); // (val + val2)/2 );
};

