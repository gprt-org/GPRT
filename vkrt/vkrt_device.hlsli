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

namespace vkrt {
  inline uint32_t make_8bit(const float f)
  {
    return min(255,max(0,int(f*256.f)));
  }

  inline uint32_t make_rgba(const float3 color)
  {
    return
      (make_8bit(color.x) << 0) +
      (make_8bit(color.y) << 8) +
      (make_8bit(color.z) << 16) +
      (0xffU << 24);
  }

  [[vk::ext_instruction(4447)]]
  RaytracingAccelerationStructure getAccelHandle(uint64_t ptr);
};

#ifndef VKRT_DEVICE_STAGES
#define VKRT_DEVICE_STAGES

struct PushConsts {
	uint64_t instanceBufferAddr;
	uint64_t transformBufferAddr;
	uint64_t accelReferencesAddr;
};
[[vk::push_constant]] PushConsts pushConsts;

typedef enum VkGeometryInstanceFlagBitsKHR {
    VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR = 0x00000001,
    VK_GEOMETRY_INSTANCE_TRIANGLE_FLIP_FACING_BIT_KHR = 0x00000002,
    VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR = 0x00000004,
    VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR = 0x00000008,
    VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR = VK_GEOMETRY_INSTANCE_TRIANGLE_FLIP_FACING_BIT_KHR,
    VK_GEOMETRY_INSTANCE_TRIANGLE_CULL_DISABLE_BIT_NV = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR,
    VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_NV = VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR,
    VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_NV = VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR,
    VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_NV = VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR,
    VK_GEOMETRY_INSTANCE_FLAG_BITS_MAX_ENUM_KHR = 0x7FFFFFFF
} VkGeometryInstanceFlagBitsKHR;

typedef struct VkTransformMatrixKHR {
    float    mat[3][4];
} VkTransformMatrixKHR;

typedef struct VkAccelerationStructureInstanceKHR {
    float3x4                      transform;
    uint32_t                      instanceCustomIndex24Mask8;
    uint32_t                      instanceShaderBindingTableRecordOffset24Flags8;
    uint64_t                      accelerationStructureReference;
} VkAccelerationStructureInstanceKHR;

[shader("compute")]
[numthreads(1, 1, 1)]
void vkrtFillInstanceData( uint3 DTid : SV_DispatchThreadID )
{
  printf("Hello from compute shader! %d\n", DTid.x);
  printf("Address 1 is %d\n", pushConsts.instanceBufferAddr);
  printf("Address 2 is %d\n", pushConsts.transformBufferAddr);
  printf("Address 3 is %d\n", pushConsts.accelReferencesAddr);

  VkAccelerationStructureInstanceKHR instance;
  
  instance.instanceCustomIndex24Mask8 = 0 | 0xFF << 24;
  instance.instanceShaderBindingTableRecordOffset24Flags8 = 0 
  | 0x00 << 24;

  instance.transform = vk::RawBufferLoad<float3x4>(
    pushConsts.transformBufferAddr + sizeof(VkTransformMatrixKHR) * DTid.x);
  
  instance.accelerationStructureReference = vk::RawBufferLoad<uint64_t>(
    pushConsts.accelReferencesAddr + sizeof(uint64_t) * DTid.x
  );

  // printf("accel addr is %d\n", instance.accelerationStructureReference);

  // printf("sizeof instance is %d\n", sizeof(VkAccelerationStructureInstanceKHR));
  // vk::RawBufferStore<VkAccelerationStructureInstanceKHR>(
  //   pushConsts.instanceBufferAddr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
  //   instance
  // );

  // for whatever reason, can't just store all values in this structure at once... 
  vk::RawBufferStore<float3x4>(
    pushConsts.instanceBufferAddr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
    instance.transform
  );

  vk::RawBufferStore<uint32_t>(
    pushConsts.instanceBufferAddr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(VkTransformMatrixKHR),
    instance.instanceCustomIndex24Mask8
  );

  vk::RawBufferStore<uint32_t>(
    pushConsts.instanceBufferAddr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(VkTransformMatrixKHR) + sizeof(uint32_t),
    instance.instanceShaderBindingTableRecordOffset24Flags8
  );

  vk::RawBufferStore<uint64_t>(
    pushConsts.instanceBufferAddr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(VkTransformMatrixKHR) + sizeof(uint32_t) + sizeof(uint32_t),
    instance.accelerationStructureReference
  );
}
#endif
