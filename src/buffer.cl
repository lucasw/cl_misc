// these produce build errors

__kernel void pressure(__global uchar* past,
                       __global uchar* pres,
                       __global uchar* futu,
                       int width, int height) {
  const int pos = get_global_id(0);
  const int x = pos % width;
  const int y = (pos - x) / width;
  // if (x > 0)
  //    im_out[pos] = image[pos - 1];

  // im_out[pos] = image[y * width + ((x + 1) % width)];

  futu[pos] = 0;
  if (x > 0)
    futu[pos] += pres[pos - 1] / 2;
  if (y > 0)
    futu[pos] += pres[pos - width] / 2;
  if (x < width - 1)
    futu[pos] += pres[pos + 1] / 2;
  if (y < height - 1)
    futu[pos] += pres[pos + width] / 2;

  futu[pos] -= past[pos];
};
