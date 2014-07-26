__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | 
    CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

// these produce build errors
//__local uint4 val_max = 0;
//__local uint4 val_max;

__kernel void hello(
    __read_only image2d_t image,
    __write_only image2d_t im_out
    )
{
  // can't do this
  //__local uint4 val_max = 0;
  __local uint4 val_max;
  // do local units execute in order?  It looks like they do
  if ((get_local_id(0) == 0) && (get_local_id(1) == 0)) {
    val_max = 0;
  }

  const int2 pos = {get_global_id(0), get_global_id(1)};
  uint4 val = read_imageui(image, sampler, pos);
  
  val_max = max(val, val_max);

  //const int2 pos2 = {get_global_id(0), get_global_id(1) + 1};
  //uint4 val2 = read_imageui(image, sampler, pos2);

  //const int fr = 4;
  write_imageui(im_out, pos, 32 * (val_max-val) ); // + val_max/32 );
  //write_imageui(im_out, pos, get_local_id(0)*fr + get_local_id(1)*fr); // (val + val2)/2 );
};

