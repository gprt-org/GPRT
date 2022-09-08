// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "deviceCode.h"
#include "gprt.h"

struct Data{
  float3 color;
  bool debug;
};
struct Payload
{
[[vk::location(0)]] Data data;
};

GPRT_RAYGEN_PROGRAM(simpleRayGen, RayGenData)
                   (in RayGenData RGSBTData) 
{
 
  // printf("TEST %lu\n", RGSBTData.index);

  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  payload.data.debug = (pixelID.x == (RGSBTData.fbSize.x / 2) && pixelID.y == (RGSBTData.fbSize.y / 2) );
  if (payload.data.debug) {
  //   printf("raygen %lld\n", RGSBTData.index);
  //   int NUM_INDICES = 12;
  //   for (int i = 0; i < NUM_INDICES; ++i) {
  //     int3 indices = vk::RawBufferLoad<int3>(RGSBTData.index + sizeof(int3) * i);
  //     printf("%d %d %d\n", indices.x, indices.y, indices.z);
  //   }
    printf("RAYGEN TEST %d, %d\n", RGSBTData.three, RGSBTData.four);
  }
  

  // uint64_t addr = RGSBTData.index;
  // const int   test1  = vk::RawBufferLoad<int>(addr + 1ul * sizeof(int));
  // const int   test2  = vk::RawBufferLoad<int>(addr + 2ul * sizeof(int));
  // printf("TEST %lul %d\n", addr, test2);

  float2 screen = (float2(pixelID) + 
                  float2(.5f, .5f)) / float2(RGSBTData.fbSize);
  RayDesc rayDesc;
  rayDesc.Origin = RGSBTData.camera.pos;
  rayDesc.Direction = 
    normalize(RGSBTData.camera.dir_00
    + screen.x * RGSBTData.camera.dir_du
    + screen.y * RGSBTData.camera.dir_dv
  );
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(RGSBTData.world);
  TraceRay(
    world, // the tree
    RAY_FLAG_FORCE_OPAQUE, // ray flags
    0xff, // instance inclusion mask
    0, // ray type
    1, // number of ray types
    0, // miss type
    rayDesc, // the ray to trace
    payload // the payload IO
  );
  const int fbOfs = pixelID.x + RGSBTData.fbSize.x * pixelID.y;
  vk::RawBufferStore<uint32_t>(RGSBTData.fbPtr + fbOfs * sizeof(uint32_t), 
    gprt::make_rgba(payload.data.color));
}


GPRT_CLOSEST_HIT_PROGRAM(TriangleMesh, TrianglesGeomData, Payload)
                        (in TrianglesGeomData geomSBTData, inout Payload prd)// , in float2 attribs
{
  if (prd.data.debug) {  
    printf("CHIT TEST %d, %d\n", geomSBTData.one, geomSBTData.two);
    // 301380000
    // printf("TEST\n");
    // uint64_t addr = 301380000ul; //geomSBTData.index;
    // const int   test  = vk::RawBufferLoad<int>(addr + 2ul * sizeof(int));
    // printf("TEST %lu\n", geomSBTData.index);
    
  }


  // // // int3 indices[12] =
  // // // {
  // // //   { 0,1,3 }, { 2,3,0 },
  // // //   { 5,7,6 }, { 5,6,4 },
  // // //   { 0,4,5 }, { 0,5,1 },
  // // //   { 2,3,7 }, { 2,7,6 },
  // // //   { 1,5,7 }, { 1,7,3 },
  // // //   { 4,0,2 }, { 4,2,6 }
  // // // };

  // // // float3 vertices[8] =
  // // // {
  // // //   { -1.f,-1.f,-1.f },
  // // //   { +1.f,-1.f,-1.f },
  // // //   { -1.f,+1.f,-1.f },
  // // //   { +1.f,+1.f,-1.f },
  // // //   { -1.f,-1.f,+1.f },
  // // //   { +1.f,-1.f,+1.f },
  // // //   { -1.f,+1.f,+1.f },
  // // //   { +1.f,+1.f,+1.f }
  // // // };

  // // compute normal:
  // const uint    primID = PrimitiveIndex();
  // const uint64_t indexAddr = prd.data.indexAddr;
  // // const int3   index  = vk::RawBufferLoad<int3>(geomSBTData.index + sizeof(int3) * primID);
  // const int3   index  = vk::RawBufferLoad<int3>(indexAddr + sizeof(int3) * primID);
  // // // // uint64_t addr = geomSBTData.index;
  // // // // const int   test1  = vk::RawBufferLoad<int>(addr + 1ul * sizeof(int));
  // // // // const int   test2  = vk::RawBufferLoad<int>(addr + 2ul * sizeof(int));
  // // // // printf("TEST %d\n", test2);
  // // // // const int   test3  = vk::RawBufferLoad<int>(addr + 3 * sizeof(int));





  // // // // const int3   index  = vk::RawBufferLoad<int3>(addr);
  // // // // const int3 index = indices[primID];

  // // // // if (primID == 0) { //primID != 4 && primID != 5) {
  // // // //   // printf("%d %f %f %f addr %d \n", primID, C.x, C.y, C.z, index.z);
  // // // //   printf("%d %d %d %d \n", primID, index.x, index.y, index.z);
  // // // // }

  // // // // if (index.x == 0 && index.y == 0 && index.z == 0) 
  // // // // {
  // // //   // if (prd.data.debug) 
  // // //   // {
  // // //     // printf("TEST\n");
  // // //     // printf("%lu\n", geomSBTData.index);
  // // //   //   // printf("chit %lld\n", geomSBTData.index);
  // // //   //   const int3   index  = vk::RawBufferLoad<int3>(geomSBTData.index  + sizeof(int3) * primID);
  // // //   //   printf("%d %d %d %d\n", primID, index.x, index.y, index.z);


  // // //   //   // int NUM_INDICES = 12;
  // // //   //   // for (int i = 0; i < NUM_INDICES; ++i) {
  // // //   //   //   int3 indices = vk::RawBufferLoad<int3>(geomSBTData.index + sizeof(int3) * i);
  // // //   //   //   printf("%d %d %d %d\n", primID, indices.x, indices.y, indices.z);
  // // //   //   // }
  // // //   // }

  // // // //   // printf("%d %f %f %f addr %d \n", primID, C.x, C.y, C.z, index.z);
  // // // //   printf("%d %d %d %d \n", primID, index.x, index.y, index.z);
  // // // // }

  
  // // // // printf("TEST\n");

  
  
  // const uint64_t vertexAddr = prd.data.vertexAddr;
  // // const float3 A      = vk::RawBufferLoad<float3>(geomSBTData.vertex + sizeof(float3) * index.x);
  // // const float3 B      = vk::RawBufferLoad<float3>(geomSBTData.vertex + sizeof(float3) * index.y);
  // // const float3 C      = vk::RawBufferLoad<float3>(geomSBTData.vertex + sizeof(float3) * index.z);
  // const float3 A      = vk::RawBufferLoad<float3>(vertexAddr + sizeof(float3) * index.x);
  // const float3 B      = vk::RawBufferLoad<float3>(vertexAddr + sizeof(float3) * index.y);
  // const float3 C      = vk::RawBufferLoad<float3>(vertexAddr + sizeof(float3) * index.z);

  // // // // float3 A = vertices[index.x];
  // // // // float3 B = vertices[index.y];
  // // // // float3 C = vertices[index.z];
  // const float3 Ng     = normalize(cross(B-A,C-A));
  // const float3 rayDir = WorldRayDirection();

  // prd.data.color = (.2f + .8f * abs(dot(rayDir,Ng)));// * geomSBTData.color;
}

GPRT_MISS_PROGRAM(miss, MissProgData, Payload)
                 (in MissProgData missSBTData, inout Payload prd) 
{
  uint2 pixelID = DispatchRaysIndex().xy;  
  int pattern = (pixelID.x / 8) ^ (pixelID.y/8);
  prd.data.color = (pattern & 1) ? missSBTData.color1 : missSBTData.color0;
}
