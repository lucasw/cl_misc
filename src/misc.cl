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
  //if ((get_local_id(0) == 0) && (get_local_id(1) == 0)) {
  //  val_max = 0;
  //}

  float coeff[5] = { 0.1, -0.1, 1.8, -0.1, 0.1 };
  //uint coeff[7] = { 0, 0, 0, 1, -1, 0, 0 };
  //uint4 coeff[7] = { 3, 2, 2, 0, 2, 2, 3 };

  float fval = 0;
  uint ival = 0;

  for (int i = 0; i < 5; i++) {
  for (int j = 0; j < 5; j++) {
    const int2 pos = {get_global_id(0) + j - 2, get_global_id(1) + i - 2};
    uint4 val = read_imageui(image, sampler, pos); 
    fval += (float)val.x * coeff[i] * coeff[j]; // fval + val * coeff[i];//val;
    //ival += (val * coeff[i]) * (val * coeff[j]); 
    //val += coeff[i] * 
  }}
  
  uint4 val2;
  val2.x = fval;
  const int2 pos = {get_global_id(0), get_global_id(1)};
  write_imageui(im_out, pos, val2 );
/*
  const int2 pos = {get_global_id(0), get_global_id(1)};
  uint4 val = read_imageui(image, sampler, pos);
  
  //val_max = max(val, val_max);

  const int2 pos2 = {get_global_id(0), get_global_id(1) + 1};
  uint4 val2 = read_imageui(image, sampler, pos2);
  
  const int2 pos3 = {get_global_id(0), get_global_id(1) - 1};
  uint4 val3 = read_imageui(image, sampler, pos3);

  //const int fr = 4;
  write_imageui(im_out, pos, val/2 + val2/4 + val3/4 ); //+ pos2/2 + pos3/2 ); // + val_max/32 );
  */
  //write_imageui(im_out, pos, get_local_id(0)*fr + get_local_id(1)*fr); // (val + val2)/2 );
};

