#include <gprt.h>
#include "gprt_terminal.h"
#include <stdexcept>

// Run with GPRT_HEADLESS_SURFACE=1 and GPRT_HEADLESS_FRAME_LIMIT=3.
int main() {
  uint32_t pixels[] = {0xff000000, 0xffffffff};
  auto color = gprt::terminal::filteredColor((const uint8_t *)pixels, 2, 1, 1, 1, 0, 0);
  if (color != std::array<uint8_t, 3>{188, 188, 188})
    throw std::runtime_error("Terminal filter must average in linear space");
  gprtRequestWindow(2, 1, "Terminal regression");
  auto context = gprtContextCreate();
  if (!gprtContextIsHeadless(context)) throw std::runtime_error("Expected headless context");
  auto buffer = gprtDeviceBufferCreate<uint32_t>(context, 2, pixels);
  for (int frame = 0; frame < 3; ++frame) {
    if (gprtWindowShouldClose(context)) throw std::runtime_error("Closed before the requested frame count");
    if (gprtGetTime(context) != double(frame) / 60.0) throw std::runtime_error("Headless time is not deterministic");
    gprtBufferPresent(context, buffer);
    if (gprtWindowShouldClose(context) != (frame == 2)) throw std::runtime_error("Incorrect headless frame limit");
  }
  gprtBufferMap(buffer);
  for (int i = 0; i < 2; ++i)
    if (gprtBufferGetHostPointer(buffer)[i] != pixels[i]) throw std::runtime_error("Preview modified the framebuffer");
  auto mapped = gprtBufferGetHostPointer(buffer);
  mapped[0] = 0xff123456;
  gprtBufferPresent(context, buffer);
  if (gprtBufferGetHostPointer(buffer) != mapped || mapped[0] != 0xff123456)
    throw std::runtime_error("Preview discarded an existing mapping or host edits");
  gprtBufferUnmap(buffer);
  gprtBufferDestroy(buffer);
  gprtContextDestroy(context);
  context = gprtContextCreate();
  for (int poll = 0; poll < 4; ++poll)
    if (gprtWindowShouldClose(context) != (poll == 3)) throw std::runtime_error("Polling-only loop is unbounded");
  gprtContextDestroy(context);
  if (!gprtWindowShouldClose(nullptr)) throw std::runtime_error("Null context should close");
}
