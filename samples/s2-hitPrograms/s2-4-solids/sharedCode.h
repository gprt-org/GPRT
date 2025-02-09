#include "gprt.h"

#define AA 3 // used for antialiasing
#define NUM_STEPS 32 // Used for ray marching
#define EXPOSURE 2.5

/* variables available to all programs */

/* variables for the triangle mesh geometry */
struct SolidGeomData {
  uint tmp; // unused for now...
};

struct RayGenData {
  uint *frameBuffer;
  SolidAccelerationStructure world;
  DescriptorHandle<Texture2D> stbn;
};

/* Constants that change each frame */
struct PushConstants {
  float time;
  uint frameID;
};

#if defined(__SLANG_COMPILER__) // <- causes code below to only be included in the device code.

// A simple viridis colormap
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

// Helper function which composites A over B
float4 over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  if (result.a == 0.f)
    return a;   // avoid NaN
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}


void GetTetWeights(float3 rst, out float4 w) {
  w = float4(1.0 - rst.x - rst.y - rst.z, rst);
}

void GetPyrWeights(float3 rst, out float4 w0, out float w1) {
  float4 wxy = float4((1.0 - rst.x) * (1.0 - rst.y), rst.x * (1.0 - rst.y), rst.x * rst.y, (1.0 - rst.x) * rst.y);
  w0 = wxy * (1.0 - rst.z);
  w1 = rst.z;
}

void GetWedWeights(float3 rst, out float3 w0, out float3 w1) {
  float3 wxy = float3(1.0 - rst.x - rst.y, rst.xy);
  w0 = wxy * (1.0 - rst.z);
  w1 = wxy * rst.z;
}

void GetHexWeights(float3 rst, out float4 w0, out float4 w1) {
  float4 wxy = float4((1.0 - rst.x) * (1.0 - rst.y), rst.x * (1.0 - rst.y), rst.x * rst.y, (1.0 - rst.x) * rst.y);
  w0 = wxy * (1.0 - rst.z);
  w1 = wxy * rst.z;
}

uint2 GetSTBNCoordinate(int m, int n, int frameID, int2 pixelID) {
  int frame = (frameID * AA * AA + m * AA + n) % 128;
  uint2 gridCoord = int2(frame % 8, (frame / 8) % 16);
  uint2 texCoord = int2(pixelID.x % 256, pixelID.y % 256);
  uint2 coord = gridCoord * 256 + texCoord;
  return coord;
}

#else

#include <vector> 

// Helper function which generates one of each supported solid type. For simplicity, 
// we'll assume a stride of 8 integers between elements, padding with -1's where needed.
void generateSolids(std::vector<float4> &vertices, std::vector<uint4> &indices, std::vector<uint8_t> &types) {
  // A helper function which translates and scales a solid prim, then appends it's vertices and indices to a global array.
  auto addSolid = [&](float4 disp, float scale, std::vector<float4> verts, std::vector<uint4> inds, uint8_t type) {
    uint32_t currentOff = vertices.size();
    for (auto &ind : inds) ind += uint4(currentOff);
    for (auto &vert : verts) vert = disp + scale * vert;
    vertices.insert(vertices.end(), verts.begin(), verts.end());
    indices.insert(indices.end(), inds.begin(), inds.end());
    types.push_back(type);
  };

  // Some helper values for generating vertex positions...
  #define M_2PI_OVER_3 (1.0 * (M_2PI / 3.0))
  #define M_6PI_OVER_3 (2.0 * (M_2PI / 3.0))
  #define M_9PI_OVER_3 (3.0 * (M_2PI / 3.0))
  #define M_1PI_OVER_2 (1.0 * (M_2PI / 4.0))
  #define M_3PI_OVER_2 (2.0 * (M_2PI / 4.0))
  #define M_9PI_OVER_4 (3.0 * (M_2PI / 4.0))
  #define M_3PI_OVER_1 (4.0 * (M_2PI / 4.0))

  // Solids are represented using 4 to 8 positions, depending on the type.
  // Each vertex stores a position (XYZ) and density (W).

  // A tetrahedron is the simplest solid type, and is defined using four vertices
  std::vector<float4> tetVerts = {
    // Three base points defining a counterclockwise triangle
    {sin(M_2PI_OVER_3), -0.75, cos(M_2PI_OVER_3), 1.0},
    {sin(M_6PI_OVER_3), -0.75, cos(M_6PI_OVER_3), 1.0},
    {sin(M_9PI_OVER_3), -0.75, cos(M_9PI_OVER_3), 1.0},
    // One top point
    {0.0, .75, 0.0, 1.0}
  };
  std::vector<uint4> tetIndices = {uint4(0, 1, 2, 3), uint4(-1, -1, -1, -1)}; 
  addSolid(float4(-1.5, 0., 0., 0.), .5, tetVerts, tetIndices, GPRT_TETRAHEDRON);

  // Pyramids are slighly more complex than tetrahedra, as they have a quad face
  // at the bottom which can bend bilinearly. 
  std::vector<float4> pyrVerts = {
    // Four base points defining a counterclockwise quad
    {cos(M_1PI_OVER_2), -0.75, sin(M_1PI_OVER_2), 1.0},
    {cos(M_3PI_OVER_2), -0.75, sin(M_3PI_OVER_2), 1.0},
    {cos(M_9PI_OVER_4), -0.75, sin(M_9PI_OVER_4), 1.0},
    {cos(M_3PI_OVER_1), -0.75, sin(M_3PI_OVER_1), 1.0},
    // One top point
    { 0.0, .75, 0.0, 1.0}
  };
  std::vector<uint4> pyrIndices = {uint4(0, 1, 2, 3), uint4(4, -1, -1, -1)}; 
  addSolid(float4(-.5, 0., 0., 0.), .5, pyrVerts, pyrIndices, GPRT_PYRAMID);

  // Wedges two triangular faces which connect together via three bilinear quad faces
  std::vector<float4> wedVerts = {
    // Three base points defining a counterclockwise triangle
    {-.5, .75*cos(M_2PI_OVER_3), .75*sin(M_2PI_OVER_3), 1.0},
    {-.5, .75*cos(M_6PI_OVER_3), .75*sin(M_6PI_OVER_3), 1.0},
    {-.5, .75*cos(M_9PI_OVER_3), .75*sin(M_9PI_OVER_3), 1.0},
    // Three top points defining a counterclockwise triangle
    {+.5, .75*cos(M_2PI_OVER_3), .75*sin(M_2PI_OVER_3), 1.0},
    {+.5, .75*cos(M_6PI_OVER_3), .75*sin(M_6PI_OVER_3), 1.0},
    {+.5, .75*cos(M_9PI_OVER_3), .75*sin(M_9PI_OVER_3), 1.0},
  };
  std::vector<uint4> wedIndices = {uint4(0, 1, 2, 3), uint4(4, 5, -1, -1)}; 
  addSolid(float4(+.5, 0., 0., 0.), .5, wedVerts, wedIndices, GPRT_WEDGE);
  
  // Hexahedra have six bilinear quad faces
  std::vector<float4> hexVerts = {
    // Four base points defining a counterclockwise quad
    {-.5, .75*cos(M_1PI_OVER_2), .75*sin(M_1PI_OVER_2), 1.0},
    {-.5, .75*cos(M_3PI_OVER_2), .75*sin(M_3PI_OVER_2), 1.0},
    {-.5, .75*cos(M_9PI_OVER_4), .75*sin(M_9PI_OVER_4), 1.0},
    {-.5, .75*cos(M_3PI_OVER_1), .75*sin(M_3PI_OVER_1), 1.0},
    // Four top points defining a counterclockwise quad
    {+.5, .75*cos(M_1PI_OVER_2), .75*sin(M_1PI_OVER_2), 1.0},
    {+.5, .75*cos(M_3PI_OVER_2), .75*sin(M_3PI_OVER_2), 1.0},
    {+.5, .75*cos(M_9PI_OVER_4), .75*sin(M_9PI_OVER_4), 1.0},
    {+.5, .75*cos(M_3PI_OVER_1), .75*sin(M_3PI_OVER_1), 1.0},
  };
  std::vector<uint4> hexIndices = {uint4(0, 1, 2, 3), uint4(4, 5, 6, 7)}; 
  addSolid(float4(+1.5, 0., 0., 0.), .5, hexVerts, hexIndices, GPRT_HEXAHEDRON);
};
#endif