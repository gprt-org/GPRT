#include <gprt.h>
#include <stdexcept>

extern GPRTProgram t09_deviceCode;
struct Record { uint32_t *value; };

int main() {
  auto context = gprtContextCreate();
  auto module = gprtModuleCreate(context, t09_deviceCode);
  auto producer = gprtComputeCreate<uint32_t *>(context, module, "produce");
  auto consumer = gprtComputeCreate<uint32_t *>(context, module, "consume");
  auto raygen = gprtRayGenCreate<Record>(context, module, "raygen");
  auto buffer = gprtHostBufferCreate<uint32_t>(context, 1);
  auto pointer = gprtBufferGetDevicePointer(buffer);
  gprtRayGenGetParameters(raygen)->value = pointer;
  gprtBuildShaderBindingTable(context);
  gprtBufferUnmap(buffer);
  for (int i = 0; i < 128; ++i) {
    auto first = gprtComputeLaunch(producer, uint3(1), uint3(1), pointer);
    auto second = gprtComputeLaunchAfter(consumer, uint3(1), uint3(1), {first, 0}, pointer);
    gprtComputeSynchronize(context);
    auto graphics = gprtRayGenLaunch1D(context, raygen, 1);
    auto last = gprtComputeLaunchAfter(consumer, uint3(1), uint3(1), {0, graphics}, pointer);
    if (!(first < second && second < last)) throw std::runtime_error("Invalid compute completion values");
    gprtComputeSynchronize(context);
    gprtBufferMap(buffer);
    if (*gprtBufferGetHostPointer(buffer) != 44) throw std::runtime_error("Dependency result mismatch");
    gprtBufferUnmap(buffer);
  }
  for (auto dependency : {GPRTLaunchDependencies{UINT64_MAX, 0}, GPRTLaunchDependencies{0, UINT64_MAX}}) {
    bool rejected = false;
    try {
      gprtComputeLaunchAfter(consumer, uint3(1), uint3(1), dependency, pointer);
    } catch (const std::invalid_argument &) { rejected = true; }
    if (!rejected) throw std::runtime_error("Accepted unsubmitted dependency");
  }
  // Rejection must leave the next command-buffer slot usable.
  gprtComputeLaunch(producer, uint3(1), uint3(1), pointer);
  gprtComputeSynchronize(context);
  gprtBufferDestroy(buffer);
  gprtRayGenDestroy(raygen);
  gprtComputeDestroy(producer);
  gprtComputeDestroy(consumer);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);
}
