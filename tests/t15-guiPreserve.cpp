#include <gprt.h>
#include <imgui.h>
#include <cmath>
#include <stdexcept>
#include <vector>

extern GPRTProgram t15_deviceCode;

int main() {
  gprtRequestWindow(64, 64, "GUI attachment regression");
  auto context = gprtContextCreate();
  auto module = gprtModuleCreate(context, t15_deviceCode);
  auto read = gprtComputeCreate<DescriptorHandle<Texture2D<float4>>, float4 *>(context, module, "readColor");
  GPRTTextureParams params;
  params.type = GPRT_IMAGE_TYPE_2D;
  params.width = params.height = 64;
  params.format = GPRT_FORMAT_R8G8B8A8_UNORM;
  std::vector<uint32_t> pixels(64 * 64, 0xff804020);
  auto color = gprtDeviceTextureCreate<uint32_t>(context, params, pixels.data());
  params.format = GPRT_FORMAT_D32_SFLOAT;
  auto depth = gprtDeviceTextureCreate<float>(context, params);
  auto result = gprtHostBufferCreate<float4>(context, 1);
  gprtGuiSetRasterAttachments(context, color, depth);
  gprtTextureClear(depth);
  uint64_t previous = 0;
  for (int frame = 0; frame < 128; ++frame) {
    gprtWindowShouldClose(context);
    ImGui::NewFrame();
    ImGui::GetBackgroundDrawList()->AddRectFilled(ImVec2(0, 0), ImVec2(8, 8), 0xffffffff);
    auto submitted = gprtGuiRasterize(context);
    if (submitted <= previous) throw std::runtime_error("GUI completion value did not advance");
    previous = submitted;
  }
  gprtGraphicsSynchronize(context);
  gprtComputeLaunch(read, uint3(1), uint3(1), gprtTextureGet2DHandle<float4>(color), gprtBufferGetDevicePointer(result));
  gprtComputeSynchronize(context);
  auto pixel = *gprtBufferGetHostPointer(result);
  if (std::abs(pixel.x - 32.f / 255.f) > 0.001f || std::abs(pixel.y - 64.f / 255.f) > 0.001f ||
      std::abs(pixel.z - 128.f / 255.f) > 0.001f || pixel.w != 1.f)
    throw std::runtime_error("GUI cleared the caller's background");
  gprtBufferDestroy(result);
  gprtComputeDestroy(read);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);
}
