#include "gprt.h"

struct VkAccelerationStructureInstanceKHR {
  // VkTransformMatrixKHR transform;
  float3x4 transform;
  uint32_t indexAndMask;
  uint32_t recordOffsetAndFlags;
  uint64_t accelReference;
};

struct FillInstanceData {
  // input
  gprt::Buffer transforms;
  gprt::Buffer accelReferences;
  gprt::Buffer masks;
  gprt::Buffer blasOffsets;
  // ConstBufferPointer<float3x4, 4> transforms;
  // ConstBufferPointer<uint64_t, 4> accelReferences;
  // ConstBufferPointer<uint8_t, 4> masks;
  // ConstBufferPointer<uint32_t, 4> blasOffsets;

  uint32_t tlasOffset;

  // Set to 1 if masks are given. We assume 0XFF for all instances otherwise
  int useMasks;

  // Set to 1 if transforms are given. We assume identity otherwise
  int useTransforms;

  // output
  gprt::Buffer instances;
  // ConstBufferPointer<VkAccelerationStructureInstanceKHR, 4> instances;
};

[shader("compute")]
[numthreads(1, 1, 1)]
void gprtFillInstanceData(uint3 DTid: SV_DispatchThreadID, uniform FillInstanceData data) {
  VkAccelerationStructureInstanceKHR instance;

  // The instance custom index we currently assume is a value from
  // 0 -> num instances.
  uint32_t instanceCustomIndex = DTid.x;
  uint8_t mask = (data.useMasks > 0) ? gprt::load<uint8_t>(data.masks, DTid.x) : uint8_t(0XFF);
  instance.indexAndMask = (instanceCustomIndex & 0XFFFFFF) | (mask << 24);

  uint32_t recordOffset = gprt::load<uint32_t>(data.blasOffsets, DTid.x) + data.tlasOffset;
  uint8_t flags = 0; // reserved for future use
  instance.recordOffsetAndFlags = (recordOffset & 0XFFFFFF) | (flags << 24);

  instance.transform = (data.useTransforms > 0) ? gprt::load<float3x4>(data.transforms, DTid.x) : float3x4(1.0f);

  instance.accelReference = gprt::load<uint64_t>(data.accelReferences, DTid.x);

  gprt::store(data.instances, DTid.x, instance);
}