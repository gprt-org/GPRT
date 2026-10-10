#include "gprt.slang"

[shader("compute")]
[numthreads(1, 1, 1)]
void produce(uint3 tid : SV_DispatchThreadID, uniform uint32_t *values, uniform uint32_t base) {
  values[tid.x] = base + tid.x;
}
