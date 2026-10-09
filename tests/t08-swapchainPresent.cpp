#include <gprt.h>
#include <vector>

int main() {
  gprtRequestWindow(256, 256, "Swapchain presentation regression");
  auto context = gprtContextCreate();
  std::vector<uint32_t> pixels(256 * 256, 0xff0080ff);
  auto buffer = gprtDeviceBufferCreate<uint32_t>(context, pixels.size(), pixels.data());
  GPRTTextureParams params;
  params.type = GPRT_IMAGE_TYPE_2D;
  params.format = GPRT_FORMAT_R8G8B8A8_SRGB;
  params.width = 32;
  params.height = 32;
  auto texture = gprtDeviceTextureCreate<uint32_t>(context, params, pixels.data());
  for (int i = 0; i < 16; ++i) {
    gprtBufferPresent(context, buffer);
    gprtTexturePresent(context, texture);
  }
  gprtDeviceSynchronize(context);
  gprtTextureDestroy(texture);
  gprtBufferDestroy(buffer);
  gprtContextDestroy(context);
}
