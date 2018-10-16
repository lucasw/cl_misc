// these produce build errors

__kernel void pressure(__global uchar* image,
                    __global uchar* im_out,
                    int width, int height) {
  const int pos = get_global_id(0);
  const int x = pos % width;
  const int y = (pos - x) / width;
  // im_out[pos] = image[pos] / 2;
  if (x > 0)
     im_out[pos] = image[pos - 1];

  // if (x > 0)
  //   im_out[pos] += image[pos - 1] / 4;
  // if (y > 0)
  //   im_out[pos] += image[pos - width] / 4;
  // if (x < width - 1)
  //   im_out[pos] += image[pos + 1] / 4;
  // if (y < height - 1)
  //   im_out[pos] += image[pos + width] / 4;
};
