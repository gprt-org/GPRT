#include "gprt.h"

#define AA 3 // used for antialiasing

/* variables available to all programs */

/* variables for the triangle mesh geometry */
struct AABBGeomData {
  float3 *aabbs;
};

struct RayGenData {
  uint *frameBuffer;
  SurfaceAccelerationStructure world;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

/* Constants that change each frame */
struct Constants {
  float time;
  struct Camera {
    float3 pos;
    float3 dir_00;
    float3 dir_du;
    float3 dir_dv;
  } camera;
};