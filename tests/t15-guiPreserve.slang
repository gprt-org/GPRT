#include "gprt.slang"

[shader("compute")]
[numthreads(1, 1, 1)]
void readColor(uniform DescriptorHandle<Texture2D<float4>> image, uniform float4 *result) {
  *result = image.Load(int3(63, 63, 0));
}
