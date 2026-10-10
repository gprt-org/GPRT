#include <gprt.h>
#include <stdexcept>

extern GPRTProgram t14_deviceCode;

int main() {
  auto context = gprtContextCreate();
  auto module = gprtModuleCreate(context, t14_deviceCode);
  auto compute = gprtComputeCreate<uint32_t *, uint32_t>(context, module, "produce");
  auto source = gprtDeviceBufferCreate<uint32_t>(context, 32);
  for (bool hostVisible : {false, true}) {
    uint32_t initial[32];
    for (uint32_t i = 0; i < 32; ++i) initial[i] = i;
    auto destination = hostVisible ? gprtHostBufferCreate<uint32_t>(context, 32, initial)
                                   : gprtDeviceBufferCreate<uint32_t>(context, 32, initial);
    gprtBufferMap(destination);
    for (uint32_t i = 0; i < 32; ++i) {
      if (gprtBufferGetHostPointer(destination)[i] != initial[i])
        throw std::runtime_error("Buffer initialization lost contents");
    }
    gprtBufferUnmap(destination);
    for (uint32_t iteration = 0; iteration < 128; ++iteration) {
      if (iteration % 2 == 0) {
        gprtBufferMap(source);
        for (uint32_t i = 0; i < 32; ++i) gprtBufferGetHostPointer(source)[i] = iteration * 32 + i;
        gprtBufferUnmap(source);
        gprtBufferCopy(context, source, destination, 0, 0, 32);
        gprtGraphicsSynchronize(context);
      } else {
        gprtComputeLaunch(compute, uint3(32, 1, 1), uint3(1),
                          gprtBufferGetDevicePointer(destination), iteration * 32);
        gprtComputeSynchronize(context);
      }
      gprtBufferMap(destination);
      auto mapped = gprtBufferGetHostPointer(destination);
      for (uint32_t i = 0; i < 32; ++i)
        if (mapped[i] != iteration * 32 + i)
          throw std::runtime_error("Readback did not observe the GPU producer");
      mapped[0] = 0x12345678;
      gprtBufferMap(destination);
      if (gprtBufferGetHostPointer(destination) != mapped || mapped[0] != 0x12345678)
        throw std::runtime_error("Repeated mapping discarded pending host edits");
      gprtBufferUnmap(destination);
    }
    if (hostVisible) gprtBufferMap(destination);
    gprtComputeLaunch(compute, uint3(32, 1, 1), uint3(1), gprtBufferGetDevicePointer(destination), 8192u);
    gprtComputeSynchronize(context);
    gprtBufferResize(context, destination, 64, true);
    gprtBufferMap(destination);
    for (uint32_t i = 0; i < 32; ++i)
      if (gprtBufferGetHostPointer(destination)[i] != 8192u + i)
        throw std::runtime_error("Preserving resize lost completed GPU writes");
    gprtBufferUnmap(destination);
    gprtBufferDestroy(destination);
  }
  gprtBufferDestroy(source);
  gprtComputeDestroy(compute);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);
}
