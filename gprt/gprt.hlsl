#include "gprt.h"

// typedef enum VkGeometryInstanceFlagBitsKHR {
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR = 0x00000001,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FLIP_FACING_BIT_KHR = 0x00000002,
//     VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR = 0x00000004,
//     VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR = 0x00000008,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR = VK_GEOMETRY_INSTANCE_TRIANGLE_FLIP_FACING_BIT_KHR,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_CULL_DISABLE_BIT_NV = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_NV =
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR, VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_NV =
//     VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR, VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_NV =
//     VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR, VK_GEOMETRY_INSTANCE_FLAG_BITS_MAX_ENUM_KHR = 0x7FFFFFFF
// } VkGeometryInstanceFlagBitsKHR;

typedef float4x4 VkTransformMatrixKHR;
typedef uint32_t VkGeometryInstanceFlagsKHR;

typedef struct VkAccelerationStructureInstanceKHR {
  // VkTransformMatrixKHR transform;
  float4 transforma;
  float4 transformb;
  float4 transformc;
  uint32_t instanceCustomIndex : 24;
  uint32_t mask : 8;
  uint32_t instanceShaderBindingTableRecordOffset : 24;
  VkGeometryInstanceFlagsKHR flags : 8;
  uint64_t accelerationStructureReference;
}
VkAccelerationStructureInstanceKHR;

struct PushConstants {
  uint64_t r[16];
};
[[vk::push_constant]] PushConstants pc;

[shader("compute")]
[numthreads(1, 1, 1)]
void
__compute__gprtFillInstanceData(uint3 DTid: SV_DispatchThreadID) {
  uint64_t instanceBufferPtr = pc.r[0];
  uint64_t transformBufferPtr = pc.r[1];
  uint64_t accelReferencesPtr = pc.r[2];
  uint64_t instanceShaderBindingTableRecordOffset = pc.r[3];
  uint64_t transformOffset = pc.r[4];
  uint64_t transformStride = pc.r[5];
  uint64_t instanceOffsetsBufferPtr = pc.r[6];
  uint64_t instanceVisibilityMasksPtr = pc.r[7];

  VkAccelerationStructureInstanceKHR instance;
  // float3x4 transform = vk::RawBufferLoad<float3x4>(
  //     transformBufferPtr + sizeof(float3x4) * DTid.x);;

  // The instance custom index we currently assume is a value from
  // 0 -> num instances.

  int mask = 0XFF;
  if (instanceVisibilityMasksPtr != -1) {
    mask = vk::RawBufferLoad<int32_t>(instanceVisibilityMasksPtr + sizeof(int32_t) * DTid.x);
  }
  instance.instanceCustomIndex = uint(DTid.x);
  instance.mask = mask;

  int blasOffset = vk::RawBufferLoad<int32_t>(instanceOffsetsBufferPtr + sizeof(int32_t) * DTid.x);
  instance.instanceShaderBindingTableRecordOffset = int(instanceShaderBindingTableRecordOffset + blasOffset);
  instance.flags = 0;

  // If given transforms, copy them over.
  if (transformBufferPtr != -1) {
    // this is gross, but AMD has a bug where loading 3x4 transforms causes a random crash when creating shader modules.
    instance.transforma = vk::RawBufferLoad<float4>(transformBufferPtr + transformOffset + transformStride * DTid.x);
    instance.transformb =
        vk::RawBufferLoad<float4>(transformBufferPtr + transformOffset + transformStride * DTid.x + sizeof(float4));
    instance.transformc = vk::RawBufferLoad<float4>(transformBufferPtr + transformOffset + transformStride * DTid.x +
                                                    sizeof(float4) + sizeof(float4));
  }
  // otherwise, assume identity.
  else {
    instance.transforma = float4(1.0, 0.0, 0.0, 0.0);
    instance.transformb = float4(0.0, 1.0, 0.0, 0.0);
    instance.transformc = float4(0.0, 0.0, 1.0, 0.0);
  }

  instance.accelerationStructureReference = vk::RawBufferLoad<uint64_t>(accelReferencesPtr + sizeof(uint64_t) * DTid.x);

  // // vk::RawBufferStore<VkAccelerationStructureInstanceKHR>(
  // //   instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
  // //   instance
  // // );

  // // for whatever reason, can't just store all values in this structure at once...
  // vk::RawBufferStore<float3x4>(
  //   instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
  //   transform
  // );
  vk::RawBufferStore<float4>(instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
                             instance.transforma);
  vk::RawBufferStore<float4>(instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(float4),
                             instance.transformb);
  vk::RawBufferStore<float4>(instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(float4) +
                                 sizeof(float4),
                             instance.transformc);

  uint32_t instanceCustomIndex24Mask8 = instance.instanceCustomIndex | instance.mask << 24;
  vk::RawBufferStore<uint32_t>(instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x +
                                   sizeof(float3x4),
                               instanceCustomIndex24Mask8);

  uint32_t instanceShaderBindingTableRecordOffset24Flags8 =
      instance.instanceShaderBindingTableRecordOffset | instance.flags << 24;
  vk::RawBufferStore<uint32_t>(instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x +
                                   sizeof(float3x4) + sizeof(uint32_t),
                               instanceShaderBindingTableRecordOffset24Flags8);

  vk::RawBufferStore<uint64_t>(instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x +
                                   sizeof(float3x4) + sizeof(uint32_t) + sizeof(uint32_t),
                               instance.accelerationStructureReference);
}
