__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | 
    CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void hello(
    __read_only image2d_t image,
    __global uint8* result
    )
{ 
  
  //const int2 pos = {get_global_id(0), get_global_id(1)};
  
  //result[ pos.x + pos.y * get_global_size(0)] = 3;
  result[0] = get_global_id(0);
  result[1] = get_global_id(1);
  //result[1] = get_work_dim();
  result[2] = get_global_size(0);
  result[3] = get_global_size(1);
};

