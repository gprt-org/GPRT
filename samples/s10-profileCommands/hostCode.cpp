#include <gprt.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

extern GPRTProgram profileDeviceCode;
struct Record { uint32_t *input; uint32_t *value; };

int main() {
  auto context = gprtContextCreate();
  auto module = gprtModuleCreate(context, profileDeviceCode);
  auto compute = gprtComputeCreate<uint32_t *>(context, module, "compute");
  auto raygen = gprtRayGenCreate<Record>(context, module, "raygen");
  auto computeResult = gprtHostBufferCreate<uint32_t>(context, 1);
  auto rayResult = gprtHostBufferCreate<uint32_t>(context, 1);
  auto source = gprtHostBufferCreate<uint32_t>(context, 1);
  auto copy = gprtHostBufferCreate<uint32_t>(context, 1);
  *gprtBufferGetHostPointer(source) = 43;
  gprtRayGenGetParameters(raygen)->input = gprtBufferGetDevicePointer(computeResult);
  gprtRayGenGetParameters(raygen)->value = gprtBufferGetDevicePointer(rayResult);
  gprtBuildShaderBindingTable(context);
  for (auto buffer : {computeResult, rayResult, source, copy}) gprtBufferUnmap(buffer);
  std::vector<uint64_t> input(4096);
  for (size_t i = 0; i < input.size(); ++i) input[i] = input.size() - i;
  auto keys = gprtDeviceBufferCreate<uint64_t>(context, input.size(), input.data());
  auto scratch = gprtDeviceBufferCreate<uint64_t>(context);
  float3 vertices[] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  uint3 indices[] = {{0, 1, 2}};
  auto vertexBuffer = gprtHostBufferCreate<float3>(context, 3, vertices);
  auto indexBuffer = gprtHostBufferCreate<uint3>(context, 1, indices);
  auto type = gprtGeomTypeCreate<uint32_t>(context, GPRT_TRIANGLES);
  auto geometry = gprtGeomCreate(context, type);
  gprtTrianglesSetVertices(geometry, vertexBuffer, 3);
  gprtTrianglesSetIndices(geometry, indexBuffer, 1);
  auto accel = gprtTriangleAccelCreate(context, geometry);
  GPRTBuildParams build;
  build.buildMode = GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE;
  build.allowCompaction = true;
  float total = 0;
  for (int i = 0; i < 128; ++i) {
    auto start = gprtBeginProfile(context);
    if (!start.value) throw std::runtime_error("Missing profile start event");
    if (i == 0) gprtAccelBuild(context, accel, build);
    else gprtAccelUpdate(context, accel);
    gprtComputeLaunch(compute, uint3(1), uint3(1), gprtBufferGetDevicePointer(computeResult));
    auto produced = gprtGetQueueEvent(context, GPRT_QUEUE_COMPUTE);
    gprtQueueWait(context, GPRT_QUEUE_GRAPHICS, 1, &produced);
    gprtRayGenLaunch1D(context, raygen, 1);
    gprtBufferCopy(context, source, copy, 0, 0, 1);
    gprtBufferSort(context, keys, scratch);
    GPRTEvent leaves[] = {gprtGetQueueEvent(context, GPRT_QUEUE_GRAPHICS),
                          gprtGetQueueEvent(context, GPRT_QUEUE_COMPUTE), produced, start};
    float elapsed = gprtEndProfile(context, 4, leaves);
    total += elapsed;
    if (!std::isfinite(elapsed) || elapsed <= 0) throw std::runtime_error("Invalid profile interval");
    for (auto buffer : {computeResult, rayResult, copy}) gprtBufferMap(buffer);
    if (*gprtBufferGetHostPointer(computeResult) != 41 || *gprtBufferGetHostPointer(rayResult) != 42 ||
        *gprtBufferGetHostPointer(copy) != 43) throw std::runtime_error("Incomplete profiled work");
    for (auto buffer : {computeResult, rayResult, copy}) gprtBufferUnmap(buffer);
    gprtBeginProfile(context);
    gprtBufferCopy(context, source, copy, 0, 0, 1);
    if (gprtEndProfile(context) <= 0) throw std::runtime_error("Copy-only profile was omitted");
    gprtBeginProfile(context);
    if (gprtEndProfile(context) != 0) throw std::runtime_error("Empty profile returned stale data");
  }
  std::cout << "Mean mixed-command GPU interval: " << total / 128 << " ms\n";
  gprtBeginProfile(context);
  gprtAccelCompact(context, accel);
  if (gprtEndProfile(context) <= 0) throw std::runtime_error("Compaction profile was omitted");
  // Sort-only regions must work without a compute or raygen launch.
  for (bool payload : {false, true}) {
    auto sortKeys = gprtDeviceBufferCreate<uint64_t>(context, input.size(), input.data());
    auto values = gprtDeviceBufferCreate<uint64_t>(context, input.size(), input.data());
    auto uploaded = gprtGetQueueEvent(context, GPRT_QUEUE_TRANSFER);
    gprtBeginProfile(context, 1, &uploaded);
    if (payload) gprtBufferSortPayload(context, sortKeys, values, scratch);
    else gprtBufferSort(context, sortKeys, scratch);
    auto sorted = gprtGetQueueEvent(context, GPRT_QUEUE_COMPUTE);
    float elapsed = gprtEndProfile(context, 1, &sorted);
    if (!std::isfinite(elapsed) || elapsed <= 0) throw std::runtime_error("Sort-only profile was omitted");
    std::cout << (payload ? "Payload sort: " : "Key sort: ") << elapsed << " ms\n";
    // Profile the readback separately, using the precise compute producer.
    gprtBeginProfile(context, 1, &sorted);
    gprtBufferMap(sortKeys);
    gprtBufferMap(values);
    auto readback = gprtGetQueueEvent(context, GPRT_QUEUE_TRANSFER);
    if (gprtEndProfile(context, 1, &readback) <= 0) throw std::runtime_error("Readback profile was omitted");
    for (size_t i = 0; i < input.size(); ++i) {
      if (gprtBufferGetHostPointer(sortKeys)[i] != i + 1 ||
          (payload && gprtBufferGetHostPointer(values)[i] != i + 1))
        throw std::runtime_error("Profile ended before sorting completed");
    }
    gprtBufferDestroy(sortKeys);
    gprtBufferDestroy(values);
  }
  gprtAccelDestroy(accel);
  gprtGeomDestroy(geometry);
  gprtGeomTypeDestroy(type);
  gprtBufferDestroy(vertexBuffer);
  gprtBufferDestroy(indexBuffer);
  gprtBufferDestroy(scratch);
  gprtBufferDestroy(keys);
  for (auto buffer : {computeResult, rayResult, source, copy}) gprtBufferDestroy(buffer);
  gprtRayGenDestroy(raygen);
  gprtComputeDestroy(compute);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);
}
