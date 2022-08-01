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

[[vk::shader_record_ext]]
ConstantBuffer<RayGenData> raygenSBTData;
[shader("raygeneration")]
void simpleRayGen() {
  uint2 threadIdx = DispatchRaysIndex().xy;

  if (threadIdx.x == 0 && threadIdx.y == 0) {
    uint32_t test = vk::RawBufferLoad<uint32_t>(raygenSBTData.fbPtr, 4);
    printf("Hello from the raygen program! %d %d \n", 
      raygenSBTData.fbPtr, test);
  }
}

struct Payload {
  float4 tmp;
};

[[vk::shader_record_ext]]
ConstantBuffer<MissProgData> missSBTData;
[shader("miss")]
void simpleMissProg(inout Payload MyPayload) {
  printf("Hello from the miss program! %d %d \n", 
    missSBTData.third, missSBTData.fourth);
}



 // float test = vk::RawBufferLoad<float>(sbtData.addr, 4);
 