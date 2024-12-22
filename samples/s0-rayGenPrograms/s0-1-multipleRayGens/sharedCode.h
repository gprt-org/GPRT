// This file contains structures which we can use as parameters for our device code.

#include "gprt.h"

struct RayGenData {
  float3 color0;
  float3 color1;
  uint *frameBuffer;
};
