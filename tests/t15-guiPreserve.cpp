#include <gprt.h>
#include <imgui.h>
#include <cmath>
#include <stdexcept>
#include <vector>

extern GPRTProgram t15_deviceCode;

int main(int argc, char **) {
  gprtRequestWindow(64, 64, "GUI attachment regression");
  auto context = gprtContextCreate();
  auto module = gprtModuleCreate(context, t15_deviceCode);
  auto read = gprtComputeCreate<DescriptorHandle<Texture2D<float4>>, DescriptorHandle<Texture2D<float>>, float4 *>(
      context, module, "readColor");
  GPRTTextureParams params;
  params.type = GPRT_IMAGE_TYPE_2D;
  params.width = params.height = 64;
  params.format = argc > 1 ? GPRT_FORMAT_R8G8B8A8_SRGB : GPRT_FORMAT_R8G8B8A8_UNORM;
  std::vector<uint32_t> pixels(64 * 64, 0xff804020);
  auto color = gprtDeviceTextureCreate<uint32_t>(context, params, pixels.data());
  params.format = GPRT_FORMAT_D32_SFLOAT;
  std::vector<float> depths(64 * 64, 0.375f);
  auto depth = gprtDeviceTextureCreate<float>(context, params, depths.data());
  auto result = gprtHostBufferCreate<float4>(context, 3);
  gprtGuiSetRasterAttachments(context, color, depth);
  uint64_t previous = 0;
  for (int frame = 0; frame < 128; ++frame) {
    if (frame == 64) gprtGuiSetRasterAttachments(context, color, depth);
    gprtWindowShouldClose(context);
    if (frame % 7 == 0) ImGui::GetIO().DisplaySize = ImVec2(0, 0);
    ImGui::NewFrame();
    ImGui::GetBackgroundDrawList()->AddRectFilled(ImVec2(0, 0), ImVec2(8, 8), 0xffffffff);
    auto submitted = gprtGuiRasterize(context);
    if (submitted <= previous) throw std::runtime_error("GUI completion value did not advance");
    previous = submitted;
  }
  gprtGraphicsSynchronize(context);
  gprtComputeLaunch(read, uint3(1), uint3(1), gprtTextureGet2DHandle<float4>(color),
                    gprtTextureGet2DHandle<float>(depth), gprtBufferGetDevicePointer(result));
  gprtComputeSynchronize(context);
  auto pixel = *gprtBufferGetHostPointer(result);
  float3 expected = argc > 1 ? float3(0.01444384f, 0.05126946f, 0.21586050f)
                             : float3(32.f / 255.f, 64.f / 255.f, 128.f / 255.f);
  if (std::abs(pixel.x - expected.x) > 0.001f || std::abs(pixel.y - expected.y) > 0.001f ||
      std::abs(pixel.z - expected.z) > 0.001f || pixel.w != 1.f)
    throw std::runtime_error("GUI cleared the caller's background");
  auto rectangle = gprtBufferGetHostPointer(result)[1];
  if (rectangle.x != 1.f || rectangle.y != 1.f || rectangle.z != 1.f || rectangle.w != 1.f ||
      gprtBufferGetHostPointer(result)[2].x != 0.375f)
    throw std::runtime_error("GUI did not draw the rectangle or preserve depth");
  gprtBufferDestroy(result);
  gprtComputeDestroy(read);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);
}
