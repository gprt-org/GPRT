#pragma once

// #define COLLECT_STATS

// The higher this number is, the more primitives we can store in our tree.
#define MAX_LEVELS 10

// The higher this number is, the more clusters we're going to touch
// relative to the number of primitives.
#define BRANCHING_FACTOR 4

// With a branching factor of 4 and 12 levels (10 excluding the leaves),
// we can support up to 8 million primitives per nearest neighbor tree.

// Edit: nevermind... I had a bug with my previous minMaxDist function which was giving me some
// incorrect intuition. I'm finding now that this is very helpful for the utah teapot.
// Edit edit: Downward pruning seems to fall apart when nodes are quantized. The reason
// for why is because the nearest face might differ between the quantized and unquantized boxes.
// Nearest face only matches when quantized box is just a scaling up or down of the original.
// It's very rare that this happens, but also, the performance improvements from this aren't very good.
//
// Edit edit edit: When using OBBs, it seems like the maximal distance to the box gives good performance
// improvements.
// More updates, the "minMaxDistance" metric doesn't work for internal OBB nodes, since internal faces aren't guaranteed
// to contain primitives. Only OBB leaves can guarantee this. So, for internal nodes, we instead use the "maxDistance"
// for downward culling.
// More more updates... So long as internal OBBs are built to tightly fit the underlying primitives, then minMaxDist is
// "free game". However, it introduces some compute complexity which doesn't seem worth the effort
// #define ENABLE_DOWNAWARD_CULLING

// Enables an LBVH reference, similar to Jakob and Guthe's knn.
// Note, we use this LBVH as a top level tree for our method, so
// this is just about if we're using RT cores or not
#define ENABLE_LBVH_REFERENCE

// Uses a quantized representation for the active cluster queue
// I've bounced back and forth on if this helps or not.
//   For untruncated queries, the ultimate bottleneck is when tree traversal fails and many prims
//   are all approximately the same distance. There, its better to have an accurate queue for culling.
//   However, I suspect the issue with culling has more to do with using AABBs over something tigher fitting
// #define ENABLE_QUEUE_QUANTIZATION

// Enables oriented bounding boxes in an attempt to improve culling performance.
// Prior experiments seem to suggest that the minimum distance to an axis aligned bounding
// box tends to be a poor fit for the minimum distance to the contained primitive. This
// issue seems to become more and more extreme as primitives are clustered together.
// This mode uses a singular value decomposition to compute the orientation of a bounding box
// that best fits the underlying primitive distribution
// #define ENABLE_OBBS

// #define ENABLE_AABBS

// #define ENABLE_6DOPS

// Honestly I don't see much improvement by avoiding square roots. I don't think this is what the bottleneck is
// at least on my problems... Just going to take square roots for simplicity...
// #define USE_SQUARED_DISTS

// Minimum guaranteed PC size (might require RADV on AMD)
#define PUSH_CONSTANTS_LIMIT 256

// By limiting ourselves to the following, we can improve compatibility across
// AMD, NVIDIA and Intel with compile time checks
// Intel ARC defines a 65535 limit on workgroup size per dimension
// Both Intel and AMD define a 1024 thread limit per workgroup
#define WORKGROUP_LIMIT   65535
#define THREADGROUP_LIMIT 1024

// Some constants for the device parallel scan implementation
#define SCAN_PARTITON_SIZE   8192
#define SCAN_PARTITION       (1 << 0)
#define SCAN_SELECT          (1 << 1)
#define SCAN_SELECT_POSITIVE (1 << 2)

#if defined(__SLANG_COMPILER__)
#define __CLASS_PUBLIC__ public
#define __PRIVATE__      private
#define __PUBLIC__       public
#define __MUTATING__     [mutating]
#else
#define __CLASS_PUBLIC__
#define __PRIVATE__ private:
#define __PUBLIC__  public:
#define __MUTATING__
#endif

// NOTE, struct must be synchronized with declaration in gprt_host.h
namespace gprt {


struct Texture {
  uint32_t index;
};

struct Sampler {
  uint32_t index;
};

// Update: now, we simply use Slang's pointer type for buffers
// Shared between Slang and C++
// struct Buffer {
//   // eventually will become address as features progress
//   uint32_t index;
//   uint32_t temp;
// };

/** The reference to the acceleration structure.
 * @note this handle is subject to change if the memory backing the accel changes.
 */
struct Accel {
  // A VkDeviceAddress
  uint64_t address;

  // A host-side index for SBT management
  uint32_t index;
  uint32_t numGeometries;
};

struct Instance {
  float3x4 transform;

  uint32_t instanceCustomIndex : 24;
  uint32_t mask : 8;
  uint32_t __gprtSBTOffset : 24;
  uint32_t flags : 8;

  uint64_t __gprtAccelAddress;
};


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
            ((r+s+t) <= 1.0f);
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
    w[1] = rst.x  * rstm.y * rstm.z;
    w[2] = rst.x  * rst.y  * rstm.z;
    w[3] = rstm.x * rst.y  * rstm.z;
    w[4] = rstm.x * rstm.y * rst.z;
    w[5] = rst.x  * rstm.y * rst.z;
    w[6] = rst.x  * rst.y  * rst.z;
    w[7] = rstm.x * rst.y  * rst.z;
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
    float z = rst.z; // t is in [0,1]
    
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

    float angleOffset = 90.0f;      // degrees
    float dTheta      = 360.0f / 5; // = 72 degrees between each vertex

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

        float2 edge = ptj - pti; // edge vector
        float2 rel  = pt  - pti; // vector from pent[i] to pt

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
    w[0]  = -16./3. * (r - a  ) * (r - b) * (s - 1.0 ) * (t - 1.0);
    w[1]  =  16./3. * (r - 0.5) * (r - b) * (s - 0.75) * (t - 1.0);
    w[2]  = -16./3. * (r - 0.5) * (r - b) * (s - 0.25) * (t - 1.0);
    w[3]  =  16./3. * (r - a  ) * (r - b) * (s - 0.0 ) * (t - 1.0);
    w[4]  = -16./3. * (r - 0.5) * (r - a) * (s - 0.25) * (t - 1.0);
    w[5]  =  16./3. * (r - 0.5) * (r - a) * (s - 0.75) * (t - 1.0);

    // Second hexagon
    w[6]  =  16./3. * (r - a  ) * (r - b) * (s - 1.0 ) * (t - 0.0);
    w[7]  = -16./3. * (r - 0.5) * (r - b) * (s - 0.75) * (t - 0.0);
    w[8]  =  16./3. * (r - 0.5) * (r - b) * (s - 0.25) * (t - 0.0);
    w[9]  = -16./3. * (r - a  ) * (r - b) * (s - 0.0 ) * (t - 0.0);
    w[10] =  16./3. * (r - 0.5) * (r - a) * (s - 0.25) * (t - 0.0);
    w[11] = -16./3. * (r - 0.5) * (r - a) * (s - 0.75) * (t - 0.0);
    return w;
  }

  static bool IsoIsContained(float3 rst)
  {
    if (rst.z < 0.0 || rst.z > 1.0) return false;
    const float2x2 rot = float2x2(0.86602540378, -.5, .5,  0.86602540378);
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
            ((r+s+t) <= 1.0f);
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

    w[5] = (1. - r2) * (1. - s ) * (1. - t) * (2. + s + s * t) / 8.0;
    w[6] = (1. + r ) * (1. - s2) * (1. - t) * (2. - r - r * t) / 8.0;
    w[7] = (1. - r2) * (1. + s ) * (1. - t) * (2. - s - s * t) / 8.0;
    w[8] = (1. - r ) * (1. - s2) * (1. - t) * (2. + r + r * t) / 8.0;

    w[9 ] = (1. - r) * (1. - s) * (1. - t2) / 4.0;
    w[10] = (1. + r) * (1. - s) * (1. - t2) / 4.0;
    w[11] = (1. + r) * (1. + s) * (1. - t2) / 4.0;
    w[12] = (1. - r) * (1. + s) * (1. - t2) / 4.0;
    return w;
  };

  static bool IsoIsContained(float3 rst)
  {
    float3 p = float3(rst.x - 0.5, rst.z, rst.y - 0.5);
    
    // symmetry
    p.xz = abs(p.xz); // do p=abs(p) instead for double pyramid
    p.xz = (p.z>p.x) ? p.zx : p.xz;
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
    w[ 0] = 2. * (1. - r - s) * (1. - t) * (.5 - r - s - t);
    w[ 1] = 2. * r * (1. - t) * (r - t - 0.5);
    w[ 2] = 2. * s * (1. - t) * (s - t - 0.5);
    w[ 3] = 2. * (1. - r - s) * t * (t - r - s - 0.5);
    w[ 4] = 2. * r * t * (r + t - 1.5);
    w[ 5] = 2. * s * t * (s + t - 1.5);

    // midsides of triangles
    w[ 6] = 4. * r * (1. - r - s) * (1. - t);
    w[ 7] = 4. * r * s * (1. - t);
    w[ 8] = 4. * (1. - r - s) * s * (1. - t);
    w[ 9] = 4. * r * (1. - r - s) * t;
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
    w[ 0] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 1] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 2] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 3] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) - (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 4] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 5] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) - (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 6] = 0.125 * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * ((2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 7] = 0.125 * (1.0 - (2.0 * (rst.x - 0.5))) * (1.0 + (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.z - 0.5))) * (-(2.0 * (rst.x - 0.5)) + (2.0 * (rst.y - 0.5)) + (2.0 * (rst.z - 0.5)) - 2.0);
    w[ 8] = 0.250 * (1.0 - (2.0 * (rst.x - 0.5)) * (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.y - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5)));
    w[ 9] = 0.250 * (1.0 - (2.0 * (rst.y - 0.5)) * (2.0 * (rst.y - 0.5))) * (1.0 + (2.0 * (rst.x - 0.5))) * (1.0 - (2.0 * (rst.z - 0.5)));
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

// Set this using the "setAccel" function.
// // Meant for internal GPRT use. Call "gprtGetInstance" to get an instance object referencing a BLAS.
// void __setAccel(Accel blas) {
//   accelAddress = blas.address;
//   instanceCustomIndex = blas.index;
// }

// __PUBLIC__ uint64_t getAccelAddress() { return accelAddress; }
// __PUBLIC__ uint32_t getAccelIndex() { return instanceCustomIndex; }
// __PUBLIC__ uint32_t getSBTOffset() { return instanceShaderBindingTableRecordOffset; }

// __MUTATING__
// void setSBTOffset(uint32_t TLASOffset, uint32_t BLASOffsetWithinTLAS) {
//   instanceShaderBindingTableRecordOffset = TLASOffset + BLASOffsetWithinTLAS;
// }

// // Some useful constructors
// #if defined(__SLANG_COMPILER__)
// #else

//   // Instance(Accel blas) {
//   //   mask = 0b11111111;
//   //   transform =
//   //       float3x4(float4(1.0f, 0.0f, 0.0f, 0.0f), float4(0.0f, 1.0f, 0.0f, 0.0f), float4(0.0f, 0.0f, 1.0f,
//   0.0f));
//   //   flags = 0;
//   //   accelAddress = blas.address;
//   //   instanceCustomIndex = blas.index;
//   // };

//   Instance() {
//     mask = 0b11111111;
//     transform =
//         float3x4(float4(1.0f, 0.0f, 0.0f, 0.0f), float4(0.0f, 1.0f, 0.0f, 0.0f), float4(0.0f, 0.0f, 1.0f, 0.0f));
//     flags = 0;
//     accelAddress = 0;
//     instanceCustomIndex = 0;
//   };
// #endif


// Temporarily disabling... This is blocking us from dropping the buffers descriptor array

// // Parameters to the built-in scan compute kernels
// struct ScanParams {
//   uint32_t size;
//   uint32_t flags;
//   gprt::Buffer input;
//   gprt::Buffer output;
//   Buffer state;
// };


};   // namespace gprt