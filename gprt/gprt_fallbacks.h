#include "gprt.h"

struct LSSBoundsParameters {
  float4 *vertices;
  uint2 *indices;
  float3 *aabbs;
  uint32_t offset;
  uint32_t count;
};

struct LSSParameters {
  float4 *vertices;
  uint2 *indices;
  uint32_t endcap0 : 1;    // true: endcap0 enabled, false: endcap0 disabled
  uint32_t endcap1 : 1;    // true: endcap1 enabled, false: endcap1 disabled
  uint32_t exitTest : 1;   // false: return entry hits, true: return exit hits
  uint32_t numLSS : 29;
};
