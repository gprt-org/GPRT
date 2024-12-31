#pragma once
#include "gprt.h"

using namespace gprt;



static int GPRT_TETRA = 10;
static int GPRT_VOXEL = 11;
static int GPRT_HEXAHEDRON = 12;
static int GPRT_WEDGE = 13;
static int GPRT_PYRAMID = 14;
static int GPRT_PENTAGONAL_PRISM = 15;
static int GPRT_HEXAGONAL_PRISM = 16;

// #define GPRT_TETRA 10
// #define GPRT_VOXEL 11
// #define GPRT_HEXAHEDRON 12
// #define GPRT_WEDGE 13
// #define GPRT_PYRAMID 14
// #define GPRT_PENTAGONAL_PRISM 15
// #define GPRT_HEXAGONAL_PRISM 16

#define NEWTON_ITERATIONS 5
#define CONVERGED_ERROR 1e-3
#define DIVERGED_ERROR  1e6

// CELL Type Structures

struct Tetrahedron {
  // v0, v1, v2 -> Bottom triangle (counter-clockwise order)
  // v3 -> Top triangle
  float3 v[4];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[4] IsoToSupport(float3 rst) {
    float[4] w;
    w[0] = 1.f - (rst.x + rst.y + rst.z);
    w[1] = rst.x;
    w[2] = rst.y;
    w[3] = rst.z;
    return w;
  };

  static bool IsoIsContained(float3 rst) {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    return (r >= 0.0f) &&
            (s >= 0.0f) &&
            (t >= 0.0f) &&
            ((r + s + t) <= 1.0f);
  };
#endif
};

struct Pyramid {
  // v0, v1, v2, v3 := Bottom quad (counter-clockwise order)
  // v4 := Top point
  float3 v[5];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[5] IsoToSupport(float3 rst) {
    float3 rstm = 1.0 - rst;
    float[5] w;
    w[0] = rstm.x * rstm.y * rstm.z;
    w[1] = rst.x * rstm.y * rstm.z;
    w[2] = rst.x * rst.y * rstm.z;
    w[3] = rstm.x * rst.y * rstm.z;
    w[4] = rst.z;
    return w;
  };

  static bool IsoIsContained(float3 rst)
  {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    return (r >= 0.0f) &&
            (r <= 1.0f) &&
            (s >= 0.0f) &&
            (s <= 1.0f) &&
            (t >= 0.0f) &&
            (t <= 1.0f);
  }
#endif
};

// Node positions, with "z" up and data values stored in "w"
struct Wedge {
  // v0, v1, v2 := Bottom triangle (counter-clockwise order)
  // v3, v4, v5 := Top Triangle (counter-clockwise order)
  float3 v[6];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[6] IsoToSupport(float3 rst) {
    float[6] w;
    w[0] = (1.0 - rst.x - rst.y) * (1.0 - rst.z);
    w[1] = rst.x * (1.0 - rst.z);
    w[2] = rst.y * (1.0 - rst.z);
    w[3] = (1.0 - rst.x - rst.y) * rst.z;
    w[4] = rst.x * rst.z;
    w[5] = rst.y * rst.z;
    return w;
  };

  static bool IsoIsContained(float3 rst)
  {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    return (r >= 0.0f) &&
            (s >= 0.0f) &&
            (r + s <= 1.0f) &&
            (t >= 0.0f) &&
            (t <= 1.0f);
  }
#endif
};

// Node positions, with "z" up and data values stored in "w"
struct Hexahedron {
  // v0, v1, v2, v3 := Bottom quad vertices (counter-clockwise order)
  // v4, v5, v6, v7 := Top quad vertices (counter-clockwise order)
  float3 v[8];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[8] IsoToSupport(float3 rst) {
    float3 rstm = 1.0 - rst;
    float[8] w;
    w[0] = rstm.x * rstm.y * rstm.z;
    w[1] = rst.x * rstm.y * rstm.z;
    w[2] = rst.x * rst.y * rstm.z;
    w[3] = rstm.x * rst.y * rstm.z;
    w[4] = rstm.x * rstm.y * rst.z;
    w[5] = rst.x * rstm.y * rst.z;
    w[6] = rst.x * rst.y * rst.z;
    w[7] = rstm.x * rst.y * rst.z;
    return w;
  };

  static bool IsoIsContained(float3 rst)
  {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    return (r >= 0.0f) &&
            (r <= 1.0f) &&
            (s >= 0.0f) &&
            (s <= 1.0f) &&
            (t >= 0.0f) &&
            (t <= 1.0f);
  }
#endif
};

struct PentagonalPrism {
  // v0, v1, v2, v3, v4 := Bottom pentagon
  // v5, v6, v7, v8, v9 := Top pentagon
  float3 v[10];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[10] IsoToSupport(float3 rst) {
    // Compute iso-parametric interpolation functions
    // http://dilbert.engr.ucdavis.edu/~suku/nem/papers/polyelas.pdf

    // transforming parametric coordinates [0,1] to isoparametric [-1,1]
    float x = 2.0 * (rst.x - .5);
    float y = 2.0 * (rst.y - .5);
    float z = rst.z;   // t is in [0,1]

    // From Appendix A.1 Pentagonal reference element (n = 5)
    float b = 87.05 - 12.7004 * x * x - 12.7004 * y * y;
    float brcp = rcp(b);

    float a0 = -0.0929370 * (3.23607 + 4.0 * x) * (-3.80423 + 3.80423 * x - 2.76393 * y) * (15.2169 + 5.81234 * x + 17.8885 * y);
    float a1 = -0.0790569 * (3.80423 - 3.80423 * x - 2.76393 * y) * (-3.80423 + 3.80423 * x - 2.76393 * y) * (15.2169 + 5.81234 * x + 17.8885 * y);
    float a2 = -0.0790569 * (15.2169 + 5.81234 * x - 17.8885 * y) * (3.80423 - 3.80423 * x - 2.76393 * y) * (-3.80423 + 3.80423 * x - 2.76393 * y);
    float a3 = +0.0929370 * (3.23607 + 4.0 * x) * (15.2169 + 5.81234 * x - 17.8885 * y) * (3.80423 - 3.80423 * x - 2.76393 * y);
    float a4 = +0.0232343 * (3.23607 + 4.0 * x) * (15.2169 + 5.81234 * x - 17.8885 * y) * (15.2169 + 5.81234 * x + 17.8885 * y);

    float[10] w;
    w[0] = -(a0 * brcp) * (z - 1.0);
    w[5] = +(a0 * brcp) * (z - 0.0);

    w[1] = -(a1 * brcp) * (z - 1.0);
    w[6] = +(a1 * brcp) * (z - 0.0);

    w[2] = -(a2 * brcp) * (z - 1.0);
    w[7] = +(a2 * brcp) * (z - 0.0);

    w[3] = -(a3 * brcp) * (z - 1.0);
    w[8] = +(a3 * brcp) * (z - 0.0);

    w[4] = -(a4 * brcp) * (z - 1.0);
    w[9] = +(a4 * brcp) * (z - 0.0);
    return w;
  }

  static bool IsoIsContained(float3 rst)
  {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    if (t < 0.0 || t > 1.0) return false;

    float angleOffset = 90.0f;   // degrees
    float dTheta = 360.0f / 5;   // = 72 degrees between each vertex

    float x = 2.0 * (rst.x - .5);
    float y = 2.0 * (rst.y - .5);
    float2 pt = float2(x, y);

    // Half-plane test for each pentagon edge in CCW order.
    // We'll loop over the edges [V0->V1, V1->V2, ..., V4->V0].
    // If 'pt' is to the right of any edge (cross < 0), it's outside.
    [unroll]
    for (int i = 0; i < 5; i++)
    {
      int j = (i + 1) % 5;   // next vertex index, wrapping around

      float angleDegI = angleOffset + i * dTheta;
      float angleDegJ = angleOffset + j * dTheta;

      float2 pti = float2(cos(radians(angleDegI)), sin(radians(angleDegI)));
      float2 ptj = float2(cos(radians(angleDegJ)), sin(radians(angleDegJ)));

      float2 edge = ptj - pti;   // edge vector
      float2 rel = pt - pti;     // vector from pent[i] to pt

      // Cross product in 2D: edge.x*rel.y - edge.y*rel.x
      float cross = edge.x * rel.y - edge.y * rel.x;

      // If cross < 0 => 'pt' is to the right => outside
      if (cross < 0.0)
        return false;
    }

    // If we never found cross < 0, 'pt' is inside or on boundary
    return true;
  }
#endif
};

struct HexagonalPrism {
  // v0, v1, v2, v3, v4, v5 := Bottom hexagon
  // v6, v7, v8, v9, v10, v11 := Top hexagon
  float3 v[12];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[12] IsoToSupport(float3 rst) {
    float r, s, t;
    r = rst.x;
    s = rst.y;
    t = rst.z;

    const float a = 0.933012701892219298;
    const float b = 0.066987298107780702;

    // First hexagon
    float[12] w;
    w[0] = -16. / 3. * (r - a) * (r - b) * (s - 1.0) * (t - 1.0);
    w[1] = 16. / 3. * (r - 0.5) * (r - b) * (s - 0.75) * (t - 1.0);
    w[2] = -16. / 3. * (r - 0.5) * (r - b) * (s - 0.25) * (t - 1.0);
    w[3] = 16. / 3. * (r - a) * (r - b) * (s - 0.0) * (t - 1.0);
    w[4] = -16. / 3. * (r - 0.5) * (r - a) * (s - 0.25) * (t - 1.0);
    w[5] = 16. / 3. * (r - 0.5) * (r - a) * (s - 0.75) * (t - 1.0);

    // Second hexagon
    w[6] = 16. / 3. * (r - a) * (r - b) * (s - 1.0) * (t - 0.0);
    w[7] = -16. / 3. * (r - 0.5) * (r - b) * (s - 0.75) * (t - 0.0);
    w[8] = 16. / 3. * (r - 0.5) * (r - b) * (s - 0.25) * (t - 0.0);
    w[9] = -16. / 3. * (r - a) * (r - b) * (s - 0.0) * (t - 0.0);
    w[10] = 16. / 3. * (r - 0.5) * (r - a) * (s - 0.25) * (t - 0.0);
    w[11] = -16. / 3. * (r - 0.5) * (r - a) * (s - 0.75) * (t - 0.0);
    return w;
  }

  static bool IsoIsContained(float3 rst)
  {
    if (rst.z < 0.0 || rst.z > 1.0) return false;
    const float2x2 rot = float2x2(0.86602540378, -.5, .5, 0.86602540378);
    const float3 k = float3(-0.866025404, 0.5, 0.577350269);
    float2 p = (rst.xy - 0.5) * (2.0);
    p = abs(mul(rot, p));

    p -= 2.0 * min(dot(k.xy, p), 0.0) * k.xy;
    p -= float2(clamp(p.x, -k.z, k.z), 1.0);

    return p.y <= 0.0;
  }
#endif
};

struct QuadraticTetrahedron {
  float3 v[10];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[10] IsoToSupport(float3 rst) {
    float[10] w;

    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    float u = 1.0 - r - s - t;
    w[0] = u * (2.0 * u - 1.0);
    w[1] = r * (2.0 * r - 1.0);
    w[2] = s * (2.0 * s - 1.0);
    w[3] = t * (2.0 * t - 1.0);

    w[4] = 4.0 * u * r;
    w[5] = 4.0 * r * s;
    w[6] = 4.0 * s * u;
    w[7] = 4.0 * u * t;
    w[8] = 4.0 * r * t;
    w[9] = 4.0 * s * t;

    return w;
  };

  static bool IsoIsContained(float3 rst) {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    return (r >= 0.0f) &&
            (s >= 0.0f) &&
            (t >= 0.0f) &&
            ((r + s + t) <= 1.0f);
  };
#endif
};

struct QuadraticPyramid {
  float3 v[13];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[13] IsoToSupport(float3 rst) {
    float[13] w;
    // Parameteric (0,1) -> isoparametric (-1,1)
    float r = 2.0 * (rst[0] - 0.5);
    float s = 2.0 * (rst[1] - 0.5);
    float t = 2.0 * (rst[2] - 0.5);
    float r2 = r * r;
    float s2 = s * s;
    float t2 = t * t;

    w[0] = -(1. - r) * (1. - s) * (1. - t) * (4. + 3. * r + 3. * s + 2. * r * s + 2. * t + r * t + s * t + 2. * r * s * t) / 16.0;
    w[1] = -(1. + r) * (1. - s) * (1. - t) * (4. - 3. * r + 3. * s - 2. * r * s + 2. * t - r * t + s * t - 2. * r * s * t) / 16.0;
    w[2] = -(1. + r) * (1. + s) * (1. - t) * (4. - 3. * r - 3. * s + 2. * r * s + 2. * t - r * t - s * t + 2. * r * s * t) / 16.0;
    w[3] = -(1. - r) * (1. + s) * (1. - t) * (4. + 3. * r - 3. * s - 2. * r * s + 2. * t + r * t - s * t - 2. * r * s * t) / 16.0;

    w[4] = t * (1. + t) / 2.0;

    w[5] = (1. - r2) * (1. - s) * (1. - t) * (2. + s + s * t) / 8.0;
    w[6] = (1. + r) * (1. - s2) * (1. - t) * (2. - r - r * t) / 8.0;
    w[7] = (1. - r2) * (1. + s) * (1. - t) * (2. - s - s * t) / 8.0;
    w[8] = (1. - r) * (1. - s2) * (1. - t) * (2. + r + r * t) / 8.0;

    w[9] = (1. - r) * (1. - s) * (1. - t2) / 4.0;
    w[10] = (1. + r) * (1. - s) * (1. - t2) / 4.0;
    w[11] = (1. + r) * (1. + s) * (1. - t2) / 4.0;
    w[12] = (1. - r) * (1. + s) * (1. - t2) / 4.0;
    return w;
  };

  static bool IsoIsContained(float3 rst)
  {
    float3 p = float3(rst.x - 0.5, rst.z, rst.y - 0.5);

    // symmetry
    p.xz = abs(p.xz);   // do p=abs(p) instead for double pyramid
    p.xz = (p.z > p.x) ? p.zx : p.xz;
    p.xz -= 0.5;

    // project into face plane (2D)
    float qz = p.x + 0.5 * p.y;
    float si = sign(max(qz, -p.y));
    return si < 0.0;
  }
#endif
};

struct QuadraticWedge {
  float3 v[15];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[15] IsoToSupport(float3 rst) {
    float[15] w;
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    // corners
    w[0] = 2. * (1. - r - s) * (1. - t) * (.5 - r - s - t);
    w[1] = 2. * r * (1. - t) * (r - t - 0.5);
    w[2] = 2. * s * (1. - t) * (s - t - 0.5);
    w[3] = 2. * (1. - r - s) * t * (t - r - s - 0.5);
    w[4] = 2. * r * t * (r + t - 1.5);
    w[5] = 2. * s * t * (s + t - 1.5);

    // midsides of triangles
    w[6] = 4. * r * (1. - r - s) * (1. - t);
    w[7] = 4. * r * s * (1. - t);
    w[8] = 4. * (1. - r - s) * s * (1. - t);
    w[9] = 4. * r * (1. - r - s) * t;
    w[10] = 4. * r * s * t;
    w[11] = 4. * (1. - r - s) * s * t;

    // midsides of rectangles
    w[12] = 4. * t * (1. - r - s) * (1. - t);
    w[13] = 4. * t * r * (1. - t);
    w[14] = 4. * t * s * (1. - t);
    return w;
  };

  static bool IsoIsContained(float3 rst)
  {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    return (r >= 0.0f) &&
            (s >= 0.0f) &&
            (r + s <= 1.0f) &&
            (t >= 0.0f) &&
            (t <= 1.0f);
  }
#endif
};

struct QuadraticHexahedron {
  float3 v[20];

#if defined(__SLANG_COMPILER__)
  [Differentiable]
  static float[20] IsoToSupport(float3 rst) {
    float[20] w;
    // Compute support function values for the current position in "rst" space
    w[0] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[1] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[2] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[3] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[4] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[5] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[6] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[7] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[8] = 0.250 * (1.0 - (2.0 * (rst.x - 0.5)) * (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5)));
    w[9] = 0.250 * (1.0 - (2.0 * (rst.y - 0.5)) * (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5)));
    w[10] = 0.250 * (1.0 - (2.0 * (rst.x - 0.5)) * (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5)));
    w[11] = 0.250 * (1.0 - (2.0 * (rst.y - 0.5)) * (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5)));
    w[12] = 0.250 * (1.0 - (2.0 * (rst.x - 0.5)) * (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5)));
    w[13] = 0.250 * (1.0 - (2.0 * (rst.y - 0.5)) * (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5)));
    w[14] = 0.250 * (1.0 - (2.0 * (rst.x - 0.5)) * (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5)));
    w[15] = 0.250 * (1.0 - (2.0 * (rst.y - 0.5)) * (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5)));
    w[16] = 0.250 * (1.0 - (2.0 * (rst.z - 0.5)) * (2.0 * (rst.z - 0.5))) * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5)));
    w[17] = 0.250 * (1.0 - (2.0 * (rst.z - 0.5)) * (2.0 * (rst.z - 0.5))) * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5)));
    w[18] = 0.250 * (1.0 - (2.0 * (rst.z - 0.5)) * (2.0 * (rst.z - 0.5))) * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5)));
    w[19] = 0.250 * (1.0 - (2.0 * (rst.z - 0.5)) * (2.0 * (rst.z - 0.5))) * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5)));
    return w;
  };

  static bool IsoIsContained(float3 rst)
  {
    float r = rst.x;
    float s = rst.y;
    float t = rst.z;

    return (r >= 0.0f) &&
          (r <= 1.0f) &&
          (s >= 0.0f) &&
          (s <= 1.0f) &&
          (t >= 0.0f) &&
          (t <= 1.0f);
  }
#endif
};

// Returns true if the point is within the tetrahedron.
bool invertInterpolant(float3 P, in Tetrahedron e, inout float3 ic) {
    float rcp_vol_all = rcp(determinant(float3x3(e.v[1] - e.v[0], e.v[2] - e.v[0], e.v[3] - e.v[0])));
    
    // Determine how far the point is from each of the faces. 
    float w0 = determinant(float3x3(P - e.v[1], e.v[3] - e.v[1], e.v[2] - e.v[1])) * rcp_vol_all;
    if (w0 < 0.0) return false;
    float w1 = determinant(float3x3(P - e.v[0], e.v[2] - e.v[0], e.v[3] - e.v[0])) * rcp_vol_all;
    if (w1 < 0.0) return false;
    float w2 = determinant(float3x3(P - e.v[0], e.v[3] - e.v[0], e.v[1] - e.v[0])) * rcp_vol_all;
    if (w2 < 0.0) return false;
    float w3 = determinant(float3x3(P - e.v[0], e.v[1] - e.v[0], e.v[2] - e.v[0])) * rcp_vol_all;
    if (w3 < 0.0) return false;

    // 1 = w0 + w1 + w2 + w3, so 1 - w1 - w2 - w3 = w0
    ic = float3(w1, w2, w3);
    return true;
};

[Differentiable]
float[8] IsoToSupport(int numNodes, float3 rst) {
  float[8] w = { 0., 0., 0., 0., 0., 0., 0., 0. };
  float3 rstm = 1.0 - rst;
  if (numNodes == 4) {
    w[0] = 1.f - (rst.x + rst.y + rst.z);
    w[1] = rst.x;
    w[2] = rst.y;
    w[3] = rst.z;
  }
  if (numNodes == 5) {
    w[0] = rstm.x * rstm.y * rstm.z;
    w[1] = rst.x * rstm.y * rstm.z;
    w[2] = rst.x * rst.y * rstm.z;
    w[3] = rstm.x * rst.y * rstm.z;
    w[4] = rst.z;
  }
  if (numNodes == 6) {
    w[0] = (1.0 - rst.x - rst.y) * (1.0 - rst.z);
    w[1] = rst.x * (1.0 - rst.z);
    w[2] = rst.y * (1.0 - rst.z);
    w[3] = (1.0 - rst.x - rst.y) * rst.z;
    w[4] = rst.x * rst.z;
    w[5] = rst.y * rst.z;
  }
  if (numNodes == 8) {
    w[0] = rstm.x * rstm.y * rstm.z;
    w[1] = rst.x * rstm.y * rstm.z;
    w[2] = rst.x * rst.y * rstm.z;
    w[3] = rstm.x * rst.y * rstm.z;
    w[4] = rstm.x * rstm.y * rst.z;
    w[5] = rst.x * rstm.y * rst.z;
    w[6] = rst.x * rst.y * rst.z;
    w[7] = rstm.x * rst.y * rst.z;
  }
  return w;
};

static float3 GetInitialRST(int numNodes) {
  float3 center = float3(.5);
  if (numNodes == 4) center = float3(.25);
  if (numNodes == 5) center.z = .2;
  return center;
};

static bool IsoIsContained(int numNodes, float3 rst) {
  float r = rst.x;
  float s = rst.y;
  float t = rst.z;
  bool contained = all(rst >= 0.0) && all(rst <= 1.0);
  if (numNodes == 4) {
    contained &= ((r + s + t) <= 1.0f);
  }
  if (numNodes == 6) {
    contained &= r + s <= 1.0f;
  }
  return contained;
};

bool invertInterpolant(float3 P, in Pyramid e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = GetInitialRST(5);

  // Iteration for Newton's method
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = IsoToSupport(1, rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < 5; ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(IsoToSupport)(5, diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(IsoToSupport)(5, diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(IsoToSupport)(5, diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < 5; ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < 5; ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < 5; ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  // Check for containment
  return IsoIsContained(5, rst);
}

bool invertInterpolant(float3 P, in Wedge e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = GetInitialRST(6);

  // Iteration for Newton's method
  bool converged = false;
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = IsoToSupport(6, rst);

    // If the "rst" location matches the query point, we can now use our support function values
    if (converged) break;

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < 6; ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(IsoToSupport)(6, diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(IsoToSupport)(6, diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(IsoToSupport)(6, diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < 6; ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < 6; ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < 6; ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return IsoIsContained(6, rst);
}

bool invertInterpolant(float3 P, in Hexahedron e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = GetInitialRST(8);

  // Iteration for Newton's method
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = IsoToSupport(8, rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < 8; ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(IsoToSupport)(8, diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(IsoToSupport)(8, diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(IsoToSupport)(8, diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < 8; ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < 8; ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < 8; ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  // Check for containment of the final cannonical point location
  return IsoIsContained(8, rst);
}

// General method for inverting linear elements
bool invertInterpolant(float3 P, int numNodes, in float3[8] v, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = GetInitialRST(numNodes);

  // Iteration for Newton's method
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = IsoToSupport(numNodes, rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < 8; ++i) fcol += v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(IsoToSupport)(numNodes, diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(IsoToSupport)(numNodes, diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(IsoToSupport)(numNodes, diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < 8; ++i) rcol += v[i] * dwr[i];
    [unroll] for (int i = 0; i < 8; ++i) scol += v[i] * dws[i];
    [unroll] for (int i = 0; i < 8; ++i) tcol += v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  // Check for containment of the final cannonical point location
  return IsoIsContained(numNodes, rst);
}

bool invertInterpolant(float3 P, in PentagonalPrism e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.5);
  
  // Iteration for Newton's method
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = PentagonalPrism::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(PentagonalPrism::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(PentagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(PentagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return PentagonalPrism::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in HexagonalPrism e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.5);

  // Iteration for Newton's method
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = HexagonalPrism::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(HexagonalPrism::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(HexagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(HexagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return HexagonalPrism::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticTetrahedron e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.5);

  // Iteration for Newton's method
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = QuadraticTetrahedron::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(QuadraticTetrahedron::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(QuadraticTetrahedron::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(QuadraticTetrahedron::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return QuadraticTetrahedron::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticPyramid e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.2);

  // Iteration for Newton's method
  bool converged = false;
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute difference in reconstructed query position vs actual query position
    float3 fcol = -P;
    let w = QuadraticPyramid::IsoToSupport(rst);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // Use this difference to compute rst derivatives...
    float3 rcol = float3(0.f);
    let dwr = fwd_diff(QuadraticPyramid::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];

    float3 scol = float3(0.f);
    let dws = fwd_diff(QuadraticPyramid::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];

    float3 tcol = float3(0.f);
    let dwt = fwd_diff(QuadraticPyramid::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants to generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;

    if (all(abs(fcol) < CONVERGED_ERROR)) converged = true;
  }

  return converged && QuadraticPyramid::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticWedge e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.2);

  // Iteration for Newton's method
  bool converged = false;
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute difference in reconstructed query position vs actual query position
    float3 fcol = -P;
    let w = QuadraticWedge::IsoToSupport(rst);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // Use this difference to compute rst derivatives...
    float3 rcol = float3(0.f);
    let dwr = fwd_diff(QuadraticWedge::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];

    float3 scol = float3(0.f);
    let dws = fwd_diff(QuadraticWedge::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];

    float3 tcol = float3(0.f);
    let dwt = fwd_diff(QuadraticWedge::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants to generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;

    if (all(abs(fcol) < CONVERGED_ERROR)) converged = true;
  }

  return converged && QuadraticWedge::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticHexahedron e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.2);

  // Iteration for Newton's method
  bool converged = false;
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute difference in reconstructed query position vs actual query position
    float3 fcol = -P;
    let w = QuadraticHexahedron::IsoToSupport(rst);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // Use this difference to compute rst derivatives...
    float3 rcol = float3(0.f);
    let dwr = fwd_diff(QuadraticHexahedron::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];

    float3 scol = float3(0.f);
    let dws = fwd_diff(QuadraticHexahedron::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];

    float3 tcol = float3(0.f);
    let dwt = fwd_diff(QuadraticHexahedron::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants to generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;

    if (all(abs(fcol) < CONVERGED_ERROR)) converged = true;
  }

  return converged && QuadraticHexahedron::IsoIsContained(rst);
}