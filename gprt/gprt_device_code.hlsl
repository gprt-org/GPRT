#include "gprt.h"

// typedef enum VkGeometryInstanceFlagBitsKHR {
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR = 0x00000001,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FLIP_FACING_BIT_KHR = 0x00000002,
//     VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR = 0x00000004,
//     VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR = 0x00000008,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR = VK_GEOMETRY_INSTANCE_TRIANGLE_FLIP_FACING_BIT_KHR,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_CULL_DISABLE_BIT_NV = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR,
//     VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_NV = VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR,
//     VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_NV = VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR,
//     VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_NV = VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR,
//     VK_GEOMETRY_INSTANCE_FLAG_BITS_MAX_ENUM_KHR = 0x7FFFFFFF
// } VkGeometryInstanceFlagBitsKHR;


struct VkAccelerationStructureInstanceKHR {
    // float3x4                      transform;
    float4                        transforma;
    float4                        transformb;
    float4                        transformc;
    uint32_t                      instanceCustomIndex24Mask8;
    uint32_t                      instanceShaderBindingTableRecordOffset24Flags8;
    uint64_t                      accelerationStructureReference;
};

[shader("compute")]
[numthreads(1, 1, 1)]
void __compute__gprtFillInstanceData( uint3 DTid : SV_DispatchThreadID )
{
  uint64_t instanceBufferPtr = pc.r[0];
  uint64_t transformBufferPtr = pc.r[1];
  uint64_t accelReferencesPtr = pc.r[2];
  uint64_t instanceShaderBindingTableRecordOffset = pc.r[3];
  uint64_t transformOffset = pc.r[4];
  uint64_t transformStride = pc.r[5];
  uint64_t instanceOffsetsBufferPtr = pc.r[6];

  VkAccelerationStructureInstanceKHR instance;
  // float3x4 transform = vk::RawBufferLoad<float3x4>(
  //     transformBufferPtr + sizeof(float3x4) * DTid.x);;
  
  instance.instanceCustomIndex24Mask8 = 0 | 0xFF << 24;

  int blasOffset = vk::RawBufferLoad<int32_t>(
    instanceOffsetsBufferPtr + sizeof(int32_t) * DTid.x
  );

  // printf("blas %d offset %d\n", DTid.x, instanceShaderBindingTableRecordOffset + blasOffset);

  instance.instanceShaderBindingTableRecordOffset24Flags8 = 
    int(instanceShaderBindingTableRecordOffset + blasOffset) | 0x00 << 24;

  // If given transforms, copy them over.
  if (transformBufferPtr != -1) {
    // this is gross, but AMD has a bug where loading 3x4 transforms causes a random crash when creating shader modules.
    instance.transforma = 
      vk::RawBufferLoad<float4>(
        transformBufferPtr + transformOffset + transformStride * DTid.x);
    instance.transformb = 
      vk::RawBufferLoad<float4>(
        transformBufferPtr + transformOffset + transformStride * DTid.x + sizeof(float4));
    instance.transformc = 
      vk::RawBufferLoad<float4>(
        transformBufferPtr + transformOffset + transformStride * DTid.x + sizeof(float4) + sizeof(float4));
  }
  // otherwise, assume identity.
  else {
    instance.transforma = float4(1.0, 0.0, 0.0, 0.0);
    instance.transformb = float4(0.0, 1.0, 0.0, 0.0);
    instance.transformc = float4(0.0, 0.0, 1.0, 0.0);
  }

  instance.accelerationStructureReference = vk::RawBufferLoad<uint64_t>(
    accelReferencesPtr + sizeof(uint64_t) * DTid.x
  );

  // // vk::RawBufferStore<VkAccelerationStructureInstanceKHR>(
  // //   instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
  // //   instance
  // // );

  // // for whatever reason, can't just store all values in this structure at once... 
  // vk::RawBufferStore<float3x4>(
  //   instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
  //   transform
  // );
  vk::RawBufferStore<float4>(
    instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x,
    instance.transforma
  );
  vk::RawBufferStore<float4>(
    instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(float4),
    instance.transformb
  );
  vk::RawBufferStore<float4>(
    instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(float4) + sizeof(float4),
    instance.transformc
  );


  vk::RawBufferStore<uint32_t>(
    instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(float3x4),
    instance.instanceCustomIndex24Mask8
  );

  vk::RawBufferStore<uint32_t>(
    instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(float3x4) + sizeof(uint32_t),
    instance.instanceShaderBindingTableRecordOffset24Flags8
  );

  vk::RawBufferStore<uint64_t>(
    instanceBufferPtr + sizeof(VkAccelerationStructureInstanceKHR) * DTid.x + sizeof(float3x4) + sizeof(uint32_t) + sizeof(uint32_t),
    instance.accelerationStructureReference
  );
}
