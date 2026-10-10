#include <gprt.h>

// Run with Vulkan validation enabled to detect leaked or still-in-use objects.
int main() {
  for (int i = 0; i < 4; ++i) {
    auto context = gprtContextCreate();
    auto source = gprtDeviceBufferCreate<uint32_t>(context, 65536);
    auto destination = gprtDeviceBufferCreate<uint32_t>(context, 65536);
    gprtBufferCopy(context, source, destination, 0, 0, 65536);
    gprtContextDestroy(context);
  }
}
