#include <gprt.h>
#include "gprt_terminal.h"
#include <sstream>
#include <stdexcept>

void environment(const char *name, const char *value) {
#ifdef _WIN32
  _putenv_s(name, value);
#else
  if (*value) setenv(name, value, 1);
  else unsetenv(name);
#endif
}

// Run with GPRT_HEADLESS_SURFACE=1 and GPRT_HEADLESS_FRAME_LIMIT=3.
int main(int argc, char **) {
  uint32_t pixels[] = {0xff000000, 0xffffffff};
  auto color = gprt::terminal::filteredColor((const uint8_t *)pixels, 2, 1, 1, 1, 0, 0);
  if (color != std::array<uint8_t, 3>{188, 188, 188})
    throw std::runtime_error("Terminal filter must average in linear space");
  for (uint32_t value = 0; value < 256; ++value) {
    uint32_t gray = 0xff000000 | value * 0x010101;
    if (gprt::terminal::filteredColor((const uint8_t *)&gray, 1, 1, 1, 1, 0, 0) !=
        std::array<uint8_t, 3>{uint8_t(value), uint8_t(value), uint8_t(value)})
      throw std::runtime_error("Terminal filter changed an sRGB gray level");
  }
  environment("GPRT_TERMINAL_COLOR", "1");
  environment("GPRT_TERMINAL_ASCII", "0");
  const uint32_t swatches[] = {0xffff0000, 0xff00ff00, 0xff0000ff, 0xff808080, 0xff123456};
  const char *expected[] = {"255;0;0", "0;255;0", "0;0;255", "128;128;128", "18;52;86"};
  for (int i = 0; i < 5; ++i) {
    std::ostringstream output;
    auto previous = std::cout.rdbuf(output.rdbuf());
    gprt::terminal::Preview{}.present((const uint8_t *)&swatches[i], 1, 1);
    std::cout.rdbuf(previous);
    if (output.str().find(std::string("\x1b[38;2;") + expected[i] + "m\xe2\x96\x88\xe2\x96\x88") == std::string::npos)
      throw std::runtime_error("Terminal preview changed the color or block coverage");
  }
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
  auto hostBuffer = gprtHostBufferCreate<uint32_t>(context, 2, pixels);
  auto hostMapping = gprtBufferGetHostPointer(hostBuffer);
  hostMapping[0] = 0xff123456;
  gprtBufferPresent(context, hostBuffer);
  if (gprtBufferGetHostPointer(hostBuffer) != hostMapping || hostMapping[0] != 0xff123456)
    throw std::runtime_error("Preview discarded host-visible edits");
  gprtBufferDestroy(hostBuffer);
  gprtContextDestroy(context);
  context = gprtContextCreate();
  for (int poll = 0; poll < 4; ++poll)
    if (gprtWindowShouldClose(context) != (poll == 3)) throw std::runtime_error("Polling-only loop is unbounded");
  buffer = gprtDeviceBufferCreate<uint32_t>(context, 2, pixels);
  gprtBufferPresent(context, buffer);
  for (int poll = 0; poll < 4; ++poll)
    if (gprtWindowShouldClose(context) != (poll == 3)) throw std::runtime_error("Loop stopped presenting but did not close");
  gprtBufferDestroy(buffer);
  gprtContextDestroy(context);
  if (!gprtWindowShouldClose(nullptr)) throw std::runtime_error("Null context should close");
  environment("GPRT_HEADLESS_FRAME_LIMIT", "");
  context = gprtContextCreate();
  buffer = gprtDeviceBufferCreate<uint32_t>(context, 2, pixels);
  gprtBufferPresent(context, buffer);
  if (!gprtWindowShouldClose(context)) throw std::runtime_error("Default headless run must close after one frame");
  gprtBufferDestroy(buffer);
  gprtContextDestroy(context);
  if (argc > 1) {
    environment("GPRT_HEADLESS_SURFACE", "");
    context = gprtContextCreate();
    if (gprtContextIsHeadless(context)) throw std::runtime_error("Headless override changed later context defaults");
    gprtContextDestroy(context);
  }
}
