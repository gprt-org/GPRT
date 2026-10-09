#include <gprt.h>
#include <stdexcept>

int main() {
  auto context = gprtContextCreate();
  auto source = gprtDeviceBufferCreate<uint32_t>(context, 32);
  auto destination = gprtDeviceBufferCreate<uint32_t>(context, 32);
  for (uint32_t iteration = 0; iteration < 128; ++iteration) {
    gprtBufferMap(source);
    for (uint32_t i = 0; i < 32; ++i) gprtBufferGetHostPointer(source)[i] = iteration * 32 + i;
    gprtBufferUnmap(source);
    gprtBufferCopy(context, source, destination, 0, 0, 32);
    gprtBufferMap(destination);
    for (uint32_t i = 0; i < 32; ++i)
      if (gprtBufferGetHostPointer(destination)[i] != iteration * 32 + i)
        throw std::runtime_error("Readback did not observe the graphics copy");
    gprtBufferUnmap(destination);
  }
  gprtBufferDestroy(destination);
  gprtBufferDestroy(source);
  gprtContextDestroy(context);
}
