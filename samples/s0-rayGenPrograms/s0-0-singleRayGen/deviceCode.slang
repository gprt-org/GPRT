// This file contains GPU "entry points". An entry point in Slang is like the main() function
// in a C++ program, marked with attributes like [shader(...)] to define its purpose. Unlike a
// regular function, an entry point runs many instances in parallel on the GPU, making it
// ideal for handling large datasets or computations efficiently.

#include "sharedCode.h"   // contains "struct RayGenData{...}"

// Entry points with "raygeneration" initiate rays to trace into the scene. 
[shader("raygeneration")]
void simpleRayGen(ShaderRecord<RayGenData> record) {
  uint2 pixelID = DispatchRaysIndex().xy; // Which thread is currently running?
  uint2 fbSize = DispatchRaysDimensions().xy; // How many threads were launched together?

  // Have the first thread print text to console
  if (pixelID.x == 0 && pixelID.y == 0)
    printf("Hello from your first raygen program!\n");

  // For now, no rays, just a simple test pattern, showing that pixel (0,0) 
  // is in the upper left corner
  int pattern = (pixelID.x*pixelID.x + pixelID.y*pixelID.y) / 10000;
  float3 color = bool(pattern & 1) ? record.color1 : record.color0;

  // Find the frame buffer location (x + width*y) and put the result there
  int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(color);
}
