__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | 
    CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

__kernel void hello(
    __read_only image2d_t image,
    __read_only image2d_t im_out
    //__global uint8* result
    )
{ 
  
  const int2 pos = {get_global_id(0), get_global_id(1)};
 
  uint4 val = read_imageui(image, sampler, pos);
  
  //const int2 pos2 = {get_global_id(0) + 1, get_global_id(1)};

  write_imageui(im_out, pos, val);
  //result[ pos.x + pos.y * get_global_size(0)] = 3;
  
  // messing around with writing to output array 
  //for (uint8 i = 0; i < 16; i++) {
  //uint8 i = 10;  
  //result[0] = i;
  //result[7] = i*2;
  //result[8] = i*3;
  //result[15] = i*3;
  //}
  //result[get_global_id(0)] = get_global_id(0);
  //result[get_global_id(1)] = get_global_id(1);
  /*
  result[2] = get_work_dim();
  result[3] = get_global_size(0);
  result[4] = get_global_size(1);
  result[5] = 15;
  */
};

