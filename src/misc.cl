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
  
  const int2 posa = {100, 100};
  uint4 val = read_imageui(image, sampler, posa); 
  
  const int2 posb = {200, 200};
  uint4 valb = read_imageui(image, sampler, posb); 
  //__local int sig;
  uint sig = val.x/ (valb.x/10);
  //sig += 1;
  //if (sig > 32) sig = 0;
  // do local units execute in order?  It looks like they do
  //if ((get_local_id(0) == 0) && (get_local_id(1) == 0)) {
  //  val_max = 0;
  //}

  //float coeff[7] = { -0.1, 0.4, 0.5, 4.7, 0.5, 0.4, -0.1 };
  //float coeff[7] = { -0.3, 0.3, -0.5, 0.6, -0.5, 0.3, -0.3 };
  //float coeff[7] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  float coeff[7] = { -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0 };
  const float gain = 1.0;
  const int num = sizeof(coeff)/sizeof(coeff[0]);
  
  // scale the coefficients based on desired gain
  float sum = 0.0;
  for (int i = 0; i < num; i++) {
    sum += coeff[i];
  }
  for (int i = 0; i < num; i++) {
    coeff[i] *= gain/sum;
  }

  float fval = 0;

  for (int i = 0; i < num; i++) {
  //int j = num/2;
  for (int j = 0; j < num; j++) 
  {
    int aa = sig;
    const int2 pos = {get_global_id(0) + (j - num/2)*aa, get_global_id(1) + (i - num/2) * aa};
    uint4 val = read_imageui(image, sampler, pos); 
    fval += (float)val.x * coeff[i] * coeff[j]; 
  }}
  
  uint4 val2;
  val2.x = fval;
  const int2 pos = {get_global_id(0), get_global_id(1)};
  write_imageui(im_out, pos, val2 );

/*
  const int2 pos = {get_global_id(0), get_global_id(1)};
  uint4 val = read_imageui(image, sampler, pos);
  
  //val_max = max(val, val_max);

  //const int fr = 4;
  write_imageui(im_out, pos, val ); // + val_max/32 );
  */
  //write_imageui(im_out, pos, get_local_id(0)*fr + get_local_id(1)*fr); // (val + val2)/2 );
};

