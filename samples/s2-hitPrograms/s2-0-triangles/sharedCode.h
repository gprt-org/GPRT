#include "gprt.h"

#define AA 3 // used for antialiasing

/* variables available to all programs */

/* variables for the triangle mesh geometry */
struct TrianglesGeomData {
  int tmp;   // unused for now
};

struct RayGenData {
  uint *frameBuffer;
  gprt::Accel world;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

/* A small structure of constants that can change every frame without rebuilding the
  shader binding table. (must be 128 bytes or less) */
struct PushConstants {
  float time;
};