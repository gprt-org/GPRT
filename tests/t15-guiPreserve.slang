#include "gprt.slang"

[shader("compute")]
[numthreads(1, 1, 1)]
void readColor(uniform DescriptorHandle<Texture2D<float4>> image,
               uniform DescriptorHandle<Texture2D<float>> depth, uniform float4 *result) {
  result[0] = image.Load(int3(63, 63, 0));
  result[1] = image.Load(int3(4, 4, 0));
  result[2] = depth.Load(int3(4, 4, 0));
}
