#pragma once 


#include "gprt.h"

struct LSSBoundsParameters {
  float4 *vertices;
  uint2 *indices;
  float3 *aabbs;
  uint32_t offset;
  uint32_t count;
  uint32_t endcap0 : 1;    // true: endcap0 enabled, false: endcap0 disabled
  uint32_t endcap1 : 1;    // true: endcap1 enabled, false: endcap1 disabled
};

struct LSSParameters {
  float4 *vertices;
  uint2 *indices;
  uint32_t endcap0 : 1;    // true: endcap0 enabled, false: endcap0 disabled
  uint32_t endcap1 : 1;    // true: endcap1 enabled, false: endcap1 disabled
  uint32_t exitTest : 1;   // false: return entry hits, true: return exit hits
  uint32_t numLSS : 29;
};

struct SphereBoundsParameters {
  float4 *vertices;
  float3 *aabbs;
  uint32_t offset;
  uint32_t count;
};

struct SphereParameters {
  float4 *vertices;
  uint32_t exitTest;   // false: return entry hits, true: return exit hits
};

struct SolidParameters {
  float4 *vertices;
  uint4 *indices;
  uint8_t *types;
  float4 *aabbs;
  uint32_t offset;
  uint32_t count;
  uint32_t typesOffset;
  uint32_t typesStride;
  uint32_t indicesOffset;
  uint32_t indicesStride;
  uint32_t verticesOffset;
  uint32_t verticesStride;
};