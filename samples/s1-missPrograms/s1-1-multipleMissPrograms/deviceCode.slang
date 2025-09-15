#include "sharedCode.h"

// In a ray tracing pipeline, a payload is a container for data that travels with a ray
// and can be updated or accessed throughout its traversal.
struct Payload {
  float2 uv;    // example input written by raygen program.
  float3 color; // example output written by miss program.
};

// When our rays "miss", then this miss program will be called. 
// We'll return a color using the ray's payload.
[shader("miss")]
void firstMiss(ShaderRecord<MissProgData> record, PayloadState<Payload> payload) {
  // Read the uv parameter passed in by the program that called "TraceRay"
  float2 uv = payload.uv;
  float3 C = genSMPTETestPattern(uv);

  // Read a constant from the miss entry point's parameters
  int iters = record.maxIterations;

  // Use parameters above to generate a little mandelbrot set.
  float2 z = float2(0.0);
  float l = 0.0;
  for (l = 0.0; l < iters; l += 1.0) {
    z = float2(z.x * z.x - z.y * z.y, 2.0 * z.x * z.y) + uv;   // FORMULA: z = z^2 + c
    if (dot(z, z) > 65536.0) break;
  }
  float j = l - log2(log2(dot(z, z))) + 4.0;

  // Write a color back into the payload, which can be read once TraceRay returns.
  payload.color = float3((j > 99.0) ? 0.0 : pow(sin(j / 20.0), 1.5)) * C;
}

// When our rays "miss", then this miss program will be called.
// We'll return a color using the ray's payload.
[shader("miss")]
void secondMiss(ShaderRecord<MissProgData> record, PayloadState<Payload> payload) {
  // Read the uv parameter passed in by the program that called "TraceRay"
  float2 uv = payload.uv;
  float3 C = genSMPTETestPattern(uv);

  // Read a constant from the miss entry point's parameters
  int iters = record.maxIterations;

  // Use parameters above to generate a little mandelbrot set.
  float2 z = float2(0.0);
  float l = 0.0;
  for (l = 0.0; l < iters; l += 1.0) {
    z = float2(z.x * z.x - z.y * z.y, 2.0 * z.x * z.y) + uv;   // FORMULA: z = z^2 + c
    if (dot(z, z) > 65536.0) break;
  }
  float j = l - log2(log2(dot(z, z))) + 4.0;

  // Write a color back into the payload, which can be read once TraceRay returns.
  payload.color = float3((j > 99.0) ? 0.0 : pow(sin(j / 20.0), 1.5)) * C;
}

// To keep things simple, we'll use this ray generation program to trace a ray that will always miss. 
// Then, we'll store the color returned by our miss program into the image.
[shader("raygeneration")]
void raygen(ShaderRecord<RayGenData> record) {
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;

  Payload payload;

  // Data that we'll pass with the ray, which can be read by the miss program.
  payload.uv = ((2.0 * float2(pixelID) - fbSize.xy) / float(fbSize.y)) * 1.5 - float2(0.5, 0);

  RayDesc Ray;   // For now, just tracing a ray that will always miss.
  TraceRay(gprt::getNullAccelHandle(), RAY_FLAG_NONE, 0xff, 0, 0, record.secondMissIndex, Ray, payload);

  // Get the results written by our miss program
  float3 color = payload.color;

  // Find the frame buffer location (x + width*y) and put the result there
  int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(color);
}

