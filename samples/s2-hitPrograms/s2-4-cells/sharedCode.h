#include "gprt.h"

#define AA 3 // used for antialiasing
#define NUM_STEPS 32 // Used for ray marching
#define EXPOSURE 2.5

/* variables available to all programs */

/* variables for the triangle mesh geometry */
struct AABBGeomData {
  float3 *aabbs;
};

struct RayGenData {
  uint *frameBuffer;
  gprt::Accel world;
  gprt::Texture stbn;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

/* Constants that change each frame */
struct PushConstants {
  float time;
  uint frameID;
};

gprt::Tetrahedron CreateTetrahedron(float time) {
  gprt::Tetrahedron tet;

  const float angle_offset = M_2PI / 3.0;

  // Base tri
  tet.v[0] = float3(sin(1.0 * angle_offset), -0.75, cos(1.0 * angle_offset));
  tet.v[1] = float3(sin(2.0 * angle_offset), -0.75, cos(2.0 * angle_offset));
  tet.v[2] = float3(sin(3.0 * angle_offset), -0.75, cos(3.0 * angle_offset));

  // Top point
  tet.v[3] = float3(0.0, .75, 0.0);

  return tet;
}

gprt::Pyramid CreatePyramid(float time) {
  gprt::Pyramid pyr;
  
  const float angle_offset = M_2PI / 4.0;

  // Base quad
  pyr.v[0] = float3(cos(1.0 * angle_offset), -0.75, sin(1.0 * angle_offset));
  pyr.v[1] = float3(cos(2.0 * angle_offset), -0.75, sin(2.0 * angle_offset));
  pyr.v[2] = float3(cos(3.0 * angle_offset), -0.75, sin(3.0 * angle_offset));
  pyr.v[3] = float3(cos(4.0 * angle_offset), -0.75, sin(4.0 * angle_offset));

  // Top point
  pyr.v[4] = float3( 0.0, .75, 0.0);

  return pyr;
}

gprt::Wedge CreateWedge(float time) {
  gprt::Wedge w;
  
  const float angle_offset = M_2PI / 3.0;
  
  // Bottom triangle
  w.v[0] = float3(-1., .75*cos(1.0 * angle_offset + cos(time)), .75*sin(1.0 * angle_offset + cos(time)));
  w.v[1] = float3(-1., .75*cos(2.0 * angle_offset + cos(time)), .75*sin(2.0 * angle_offset + cos(time)));
  w.v[2] = float3(-1., .75*cos(3.0 * angle_offset + cos(time)), .75*sin(3.0 * angle_offset + cos(time)));

  // Top triangle of wedge
  w.v[3] = float3(+1., .75*cos(1.0 * angle_offset + cos(time + 1.0)), .75*sin(1.0 * angle_offset + cos(time + 1.0)));
  w.v[4] = float3(+1., .75*cos(2.0 * angle_offset + cos(time + 1.0)), .75*sin(2.0 * angle_offset + cos(time + 1.0)));
  w.v[5] = float3(+1., .75*cos(3.0 * angle_offset + cos(time + 1.0)), .75*sin(3.0 * angle_offset + cos(time + 1.0)));
  
  return w;
}

gprt::Hexahedron CreateHexahedron(float time) {
  gprt::Hexahedron h;
  
  const float angle_offset = M_2PI / 4.0;

  // Top quad
  h.v[0] = float3(-1., .75*cos(1.0 * angle_offset + cos(time)), .75*sin(1.0 * angle_offset + cos(time)));
  h.v[1] = float3(-1., .75*cos(2.0 * angle_offset + cos(time)), .75*sin(2.0 * angle_offset + cos(time)));
  h.v[2] = float3(-1., .75*cos(3.0 * angle_offset + cos(time)), .75*sin(3.0 * angle_offset + cos(time)));
  h.v[3] = float3(-1., .75*cos(4.0 * angle_offset + cos(time)), .75*sin(4.0 * angle_offset + cos(time)));
  
  // Bottom quad
  h.v[4] = float3(+1., .75*cos(1.0 * angle_offset + cos(time + 1.0)), .75*sin(1.0 * angle_offset + cos(time + 1.0)));
  h.v[5] = float3(+1., .75*cos(2.0 * angle_offset + cos(time + 1.0)), .75*sin(2.0 * angle_offset + cos(time + 1.0)));
  h.v[6] = float3(+1., .75*cos(3.0 * angle_offset + cos(time + 1.0)), .75*sin(3.0 * angle_offset + cos(time + 1.0)));
  h.v[7] = float3(+1., .75*cos(4.0 * angle_offset + cos(time + 1.0)), .75*sin(4.0 * angle_offset + cos(time + 1.0)));
  
  return h;
}

gprt::PentagonalPrism CreatePentagonalPrism(float time) {
  gprt::PentagonalPrism p;
  
  p.v[0] = float3(-1., .7*cos(M_2PI * (1.0 / 5.0) + cos(time)), .7*sin(M_2PI * (1.0 / 5.0) + cos(time)));
  p.v[1] = float3(-1., .7*cos(M_2PI * (2.0 / 5.0) + cos(time)), .7*sin(M_2PI * (2.0 / 5.0) + cos(time)));
  p.v[2] = float3(-1., .7*cos(M_2PI * (3.0 / 5.0) + cos(time)), .7*sin(M_2PI * (3.0 / 5.0) + cos(time)));
  p.v[3] = float3(-1., .7*cos(M_2PI * (4.0 / 5.0) + cos(time)), .7*sin(M_2PI * (4.0 / 5.0) + cos(time)));
  p.v[4] = float3(-1., .7*cos(M_2PI * (5.0 / 5.0) + cos(time)), .7*sin(M_2PI * (5.0 / 5.0) + cos(time)));

  // Top pentagram
  p.v[5] = float3(+1., .7*cos(M_2PI * (1.0 / 5.0) + cos(time + 1.0)), .7*sin(M_2PI * (1.0 / 5.0) + cos(time + 1.0)));
  p.v[6] = float3(+1., .7*cos(M_2PI * (2.0 / 5.0) + cos(time + 1.0)), .7*sin(M_2PI * (2.0 / 5.0) + cos(time + 1.0)));
  p.v[7] = float3(+1., .7*cos(M_2PI * (3.0 / 5.0) + cos(time + 1.0)), .7*sin(M_2PI * (3.0 / 5.0) + cos(time + 1.0)));
  p.v[8] = float3(+1., .7*cos(M_2PI * (4.0 / 5.0) + cos(time + 1.0)), .7*sin(M_2PI * (4.0 / 5.0) + cos(time + 1.0)));
  p.v[9] = float3(+1., .7*cos(M_2PI * (5.0 / 5.0) + cos(time + 1.0)), .7*sin(M_2PI * (5.0 / 5.0) + cos(time + 1.0)));

  return p;
}

gprt::HexagonalPrism CreateHexagonalPrism(float time) {
  gprt::HexagonalPrism p;

  // Bottom hexagon
  p.v[0] = float3(-1., .7*cos(M_2PI * (1.0 / 6.0) + cos(time)), .7*sin(M_2PI * (1.0 / 6.0) + cos(time)));
  p.v[1] = float3(-1., .7*cos(M_2PI * (2.0 / 6.0) + cos(time)), .7*sin(M_2PI * (2.0 / 6.0) + cos(time)));
  p.v[2] = float3(-1., .7*cos(M_2PI * (3.0 / 6.0) + cos(time)), .7*sin(M_2PI * (3.0 / 6.0) + cos(time)));
  p.v[3] = float3(-1., .7*cos(M_2PI * (4.0 / 6.0) + cos(time)), .7*sin(M_2PI * (4.0 / 6.0) + cos(time)));
  p.v[4] = float3(-1., .7*cos(M_2PI * (5.0 / 6.0) + cos(time)), .7*sin(M_2PI * (5.0 / 6.0) + cos(time)));
  p.v[5] = float3(-1., .7*cos(M_2PI * (6.0 / 6.0) + cos(time)), .7*sin(M_2PI * (6.0 / 6.0) + cos(time)));

  // Top hexagon
  p.v[ 6] = float3(+1., .7*cos(M_2PI * (1.0 / 6.0) + cos(time + 1.0)), .7*sin(M_2PI * (1.0 / 6.0) + cos(time + 1.0)));
  p.v[ 7] = float3(+1., .7*cos(M_2PI * (2.0 / 6.0) + cos(time + 1.0)), .7*sin(M_2PI * (2.0 / 6.0) + cos(time + 1.0)));
  p.v[ 8] = float3(+1., .7*cos(M_2PI * (3.0 / 6.0) + cos(time + 1.0)), .7*sin(M_2PI * (3.0 / 6.0) + cos(time + 1.0)));
  p.v[ 9] = float3(+1., .7*cos(M_2PI * (4.0 / 6.0) + cos(time + 1.0)), .7*sin(M_2PI * (4.0 / 6.0) + cos(time + 1.0)));
  p.v[10] = float3(+1., .7*cos(M_2PI * (5.0 / 6.0) + cos(time + 1.0)), .7*sin(M_2PI * (5.0 / 6.0) + cos(time + 1.0)));
  p.v[11] = float3(+1., .7*cos(M_2PI * (6.0 / 6.0) + cos(time + 1.0)), .7*sin(M_2PI * (6.0 / 6.0) + cos(time + 1.0)));

  return p;
};

gprt::QuadraticTetrahedron CreateQuadraticTetrahedron(float time) {
  gprt::QuadraticTetrahedron e;

  // Corner vertices
  e.v[0] = float3(cos((1.0 / 3.0) * M_2PI), -.75, sin((1.0 / 3.0) * M_2PI));
  e.v[1] = float3(cos((2.0 / 3.0) * M_2PI), -.75, sin((2.0 / 3.0) * M_2PI));
  e.v[2] = float3(cos((3.0 / 3.0) * M_2PI), -.75, sin((3.0 / 3.0) * M_2PI));
  e.v[3] = float3( 0.0, .75, 0.0);
  
  float3 cen = (e.v[0] + e.v[1] + e.v[2] + e.v[3]) / 4.0f;

  // Edge vertices
  e.v[4] = float3((e.v[0] + e.v[1]) * .5f); // -> midpoint between 0 and 1
  e.v[5] = float3((e.v[1] + e.v[2]) * .5f); // -> midpoint between 1 and 2
  e.v[6] = float3((e.v[0] + e.v[2]) * .5f); // -> midpoint between 0 and 2
  e.v[7] = float3((e.v[0] + e.v[3]) * .5f); // -> midpoint between 0 and 3
  e.v[8] = float3((e.v[1] + e.v[3]) * .5f); // -> midpoint between 1 and 3
  e.v[9] = float3((e.v[2] + e.v[3]) * .5f); // -> midpoint between 2 and 3
  
  // Make the edge vertices wiggle
  e.v[4] = e.v[4] + .2f * normalize(e.v[4] - cen) * cos(time * .1f);
  e.v[5] = e.v[5] + .2f * normalize(e.v[5] - cen) * cos(time * .2f);
  e.v[6] = e.v[6] + .2f * normalize(e.v[6] - cen) * cos(time * .3f);
  e.v[7] = e.v[7] + .2f * normalize(e.v[7] - cen) * cos(time * .4f);
  e.v[8] = e.v[8] + .2f * normalize(e.v[8] - cen) * cos(time * .5f);
  e.v[9] = e.v[9] + .2f * normalize(e.v[9] - cen) * cos(time * .6f);
  
  return e;
};

gprt::QuadraticPyramid CreateQuadraticPyramid(float time) {
  gprt::QuadraticPyramid e;

  // Corner vertices
  e.v[0] = float3(cos((1.0 / 4.0) * M_2PI), -.75, sin((1.0 / 4.0) * M_2PI));
  e.v[1] = float3(cos((2.0 / 4.0) * M_2PI), -.75, sin((2.0 / 4.0) * M_2PI));
  e.v[2] = float3(cos((3.0 / 4.0) * M_2PI), -.75, sin((3.0 / 4.0) * M_2PI));
  e.v[3] = float3(cos((4.0 / 4.0) * M_2PI), -.75, sin((4.0 / 4.0) * M_2PI));
  e.v[4] = float3( 0.0, .75, 0.0);
  
  float3 cen = (e.v[0] + e.v[1] + e.v[2] + e.v[3] + e.v[4]) / 5.0f;

  // Edge vertices 
  e.v[ 5] = float3((e.v[0] + e.v[1]) * .5f); // -> midpoint between 0 and 1
  e.v[ 6] = float3((e.v[1] + e.v[2]) * .5f); // -> midpoint between 1 and 2
  e.v[ 7] = float3((e.v[2] + e.v[3]) * .5f); // -> midpoint between 2 and 3
  e.v[ 8] = float3((e.v[3] + e.v[0]) * .5f); // -> midpoint between 3 and 0
  e.v[ 9] = float3((e.v[0] + e.v[4]) * .5f); // -> midpoint between 0 and 4
  e.v[10] = float3((e.v[1] + e.v[4]) * .5f); // -> midpoint between 1 and 4
  e.v[11] = float3((e.v[2] + e.v[4]) * .5f); // -> midpoint between 2 and 4
  e.v[12] = float3((e.v[3] + e.v[4]) * .5f); // -> midpoint between 3 and 4 
  
  // Make the edge vertices wiggle
  e.v[ 5] = e.v[ 5] + .2f * normalize(e.v[ 5] - cen) * cos(time * .1f);
  e.v[ 6] = e.v[ 6] + .2f * normalize(e.v[ 6] - cen) * cos(time * .2f);
  e.v[ 7] = e.v[ 7] + .2f * normalize(e.v[ 7] - cen) * cos(time * .3f);
  e.v[ 8] = e.v[ 8] + .2f * normalize(e.v[ 8] - cen) * cos(time * .4f);
  e.v[ 9] = e.v[ 9] + .2f * normalize(e.v[ 9] - cen) * cos(time * .5f);
  e.v[10] = e.v[10] + .2f * normalize(e.v[10] - cen) * cos(time * .6f);
  e.v[11] = e.v[11] + .2f * normalize(e.v[11] - cen) * cos(time * .7f);
  e.v[12] = e.v[12] + .2f * normalize(e.v[12] - cen) * cos(time * .8f);
 
  return e;
};

gprt::QuadraticWedge CreateQuadraticWedge(float time) {
  gprt::QuadraticWedge e;

  // Corner vertices
  e.v[0] = float3(-1., .75*cos((1.0 / 3.0) * M_2PI), .75*sin((1.0 / 3.0) * M_2PI));
  e.v[1] = float3(-1., .75*cos((2.0 / 3.0) * M_2PI), .75*sin((2.0 / 3.0) * M_2PI));
  e.v[2] = float3(-1., .75*cos((3.0 / 3.0) * M_2PI), .75*sin((3.0 / 3.0) * M_2PI));
  e.v[3] = float3(+1., .75*cos((1.0 / 3.0) * M_2PI), .75*sin((1.0 / 3.0) * M_2PI));
  e.v[4] = float3(+1., .75*cos((2.0 / 3.0) * M_2PI), .75*sin((2.0 / 3.0) * M_2PI));
  e.v[5] = float3(+1., .75*cos((3.0 / 3.0) * M_2PI), .75*sin((3.0 / 3.0) * M_2PI));

  float3 cen = (e.v[0] + e.v[1] + e.v[2] + e.v[3] + e.v[4] + e.v[5]) / 6.0f;

  // Edge vertices
  e.v[ 6] = float3((e.v[0] + e.v[1]) * .5f); // -> midpoint between 0 and 1
  e.v[ 7] = float3((e.v[1] + e.v[2]) * .5f); // -> midpoint between 1 and 2
  e.v[ 8] = float3((e.v[2] + e.v[0]) * .5f); // -> midpoint between 2 and 0

  e.v[ 9] = float3((e.v[3] + e.v[4]) * .5f); // -> midpoint between 3 and 4
  e.v[10] = float3((e.v[4] + e.v[5]) * .5f); // -> midpoint between 4 and 5
  e.v[11] = float3((e.v[5] + e.v[3]) * .5f); // -> midpoint between 5 and 3

  e.v[12] = float3((e.v[0] + e.v[3]) * .5f); // -> midpoint between 0 and 3
  e.v[13] = float3((e.v[1] + e.v[4]) * .5f); // -> midpoint between 1 and 4
  e.v[14] = float3((e.v[2] + e.v[5]) * .5f); // -> midpoint between 2 and 5
  
  // Make the edge vertices wiggle
  //v05 = v05 + .2 * normalize(v05 - cen) * cos(time * .10);
  e.v[ 6] = e.v[ 6] + .2f * normalize(e.v[ 6] - cen) * cos(time * .15f);
  e.v[ 7] = e.v[ 7] + .2f * normalize(e.v[ 7] - cen) * cos(time * .20f);
  e.v[ 8] = e.v[ 8] + .2f * normalize(e.v[ 8] - cen) * cos(time * .25f);
  e.v[ 9] = e.v[ 9] + .2f * normalize(e.v[ 9] - cen) * cos(time * .30f);
  e.v[10] = e.v[10] + .2f * normalize(e.v[10] - cen) * cos(time * .35f);
  e.v[11] = e.v[11] + .2f * normalize(e.v[11] - cen) * cos(time * .40f);
  e.v[12] = e.v[12] + .2f * normalize(e.v[12] - cen) * cos(time * .45f);
  e.v[13] = e.v[13] + .2f * normalize(e.v[13] - cen) * cos(time * .50f);
  e.v[14] = e.v[14] + .2f * normalize(e.v[14] - cen) * cos(time * .55f);

  return e;
};

gprt::QuadraticHexahedron CreateQuadraticHexahedron(float time) {
  gprt::QuadraticHexahedron e;

  // Corner vertices
  e.v[0] = float3(-1., .75f*cos((1.f / 4.f) * M_2PI), .75f*sin((1.f / 4.f) * M_2PI));
  e.v[1] = float3(-1., .75f*cos((2.f / 4.f) * M_2PI), .75f*sin((2.f / 4.f) * M_2PI));
  e.v[2] = float3(-1., .75f*cos((3.f / 4.f) * M_2PI), .75f*sin((3.f / 4.f) * M_2PI));
  e.v[3] = float3(-1., .75f*cos((4.f / 4.f) * M_2PI), .75f*sin((4.f / 4.f) * M_2PI));
  e.v[4] = float3(+1., .75f*cos((1.f / 4.f) * M_2PI), .75f*sin((1.f / 4.f) * M_2PI));
  e.v[5] = float3(+1., .75f*cos((2.f / 4.f) * M_2PI), .75f*sin((2.f / 4.f) * M_2PI));
  e.v[6] = float3(+1., .75f*cos((3.f / 4.f) * M_2PI), .75f*sin((3.f / 4.f) * M_2PI));
  e.v[7] = float3(+1., .75f*cos((4.f / 4.f) * M_2PI), .75f*sin((4.f / 4.f) * M_2PI));

  float3 cen = (e.v[0] + e.v[1] + e.v[2] + e.v[3] + e.v[4] + e.v[5] + e.v[6] + e.v[7]) / 8.0f;

  // Edge vertices
  e.v[ 8] = float3((e.v[0] + e.v[1]) * .5f); // -> midpoint between 0 and 1
  e.v[ 9] = float3((e.v[1] + e.v[2]) * .5f); // -> midpoint between 1 and 2
  e.v[10] = float3((e.v[2] + e.v[3]) * .5f); // -> midpoint between 2 and 3
  e.v[11] = float3((e.v[3] + e.v[0]) * .5f); // -> midpoint between 3 and 0

  e.v[12] = float3((e.v[4] + e.v[5]) * .5f); // -> midpoint between 4 and 5
  e.v[13] = float3((e.v[5] + e.v[6]) * .5f); // -> midpoint between 5 and 6
  e.v[14] = float3((e.v[6] + e.v[7]) * .5f); // -> midpoint between 6 and 7
  e.v[15] = float3((e.v[7] + e.v[4]) * .5f); // -> midpoint between 7 and 4

  e.v[16] = float3((e.v[0] + e.v[4]) * .5f); // -> midpoint between 0 and 4
  e.v[17] = float3((e.v[1] + e.v[5]) * .5f); // -> midpoint between 1 and 5
  e.v[18] = float3((e.v[2] + e.v[6]) * .5f); // -> midpoint between 2 and 6
  e.v[19] = float3((e.v[3] + e.v[7]) * .5f); // -> midpoint between 3 and 7
  
  // Make the edge vertices wiggle
  e.v[ 7] = e.v[ 7] + .1f * normalize(e.v[ 7] - cen) * cos(time * .10f);
  e.v[ 8] = e.v[ 8] + .1f * normalize(e.v[ 8] - cen) * cos(time * .15f);
  e.v[ 9] = e.v[ 9] + .1f * normalize(e.v[ 9] - cen) * cos(time * .20f);
  e.v[10] = e.v[10] + .1f * normalize(e.v[10] - cen) * cos(time * .25f);
  e.v[11] = e.v[11] + .1f * normalize(e.v[11] - cen) * cos(time * .30f);
  e.v[12] = e.v[12] + .1f * normalize(e.v[12] - cen) * cos(time * .35f);
  e.v[13] = e.v[13] + .1f * normalize(e.v[13] - cen) * cos(time * .40f);
  e.v[14] = e.v[14] + .1f * normalize(e.v[14] - cen) * cos(time * .45f);
  e.v[15] = e.v[15] + .1f * normalize(e.v[15] - cen) * cos(time * .50f);
  e.v[16] = e.v[16] + .1f * normalize(e.v[16] - cen) * cos(time * .55f);
  e.v[17] = e.v[17] + .1f * normalize(e.v[17] - cen) * cos(time * .60f);
  e.v[18] = e.v[18] + .1f * normalize(e.v[18] - cen) * cos(time * .65f);
  e.v[19] = e.v[19] + .1f * normalize(e.v[19] - cen) * cos(time * .70f);

  return e;
};

#if defined(__SLANG_COMPILER__) // <- causes code below to only be included in the device code.

float3 viridis(float t) {
  t = clamp(t, 0.0, 1.0);

  const float3 c0 = float3(0.2777273272234177, 0.005407344544966578, 0.3340998053353061);
  const float3 c1 = float3(0.1050930431085774, 1.404613529898575, 1.384590162594685);
  const float3 c2 = float3(-0.3308618287255563, 0.214847559468213, 0.09509516302823659);
  const float3 c3 = float3(-4.634230498983486, -5.799100973351585, -19.33244095627987);
  const float3 c4 = float3(6.228269936347081, 14.17993336680509, 56.69055260068105);
  const float3 c5 = float3(4.776384997670288, -13.74514537774601, -65.35303263337234);
  const float3 c6 = float3(-5.435455855934631, 4.645852612178535, 26.3124352495832);

  float3 srgb = c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
  return pow(srgb, float3(2.2));
}

float4 over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  if (result.a == 0.f)
    return a;   // avoid NaN
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

#endif