#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace gprt::terminal {

inline uint64_t environmentCount(const char *name, uint64_t fallback) {
  const char *value = std::getenv(name);
  if (!value || !*value) return fallback;
  uint64_t result = 0;
  for (; *value; ++value) {
    if (*value < '0' || *value > '9' || result > (UINT64_MAX - (*value - '0')) / 10) return fallback;
    result = result * 10 + (*value - '0');
  }
  return result ? result : fallback;
}

inline uint8_t byte(float value) { return uint8_t(std::clamp(value, 0.0f, 255.0f)); }

inline std::array<uint8_t, 3> filteredColor(const uint8_t *pixels, uint32_t width, uint32_t height,
                                         uint32_t columns, uint32_t rows, uint32_t x, uint32_t y) {
  static const std::array<float, 256> linear = [] {
    std::array<float, 256> table{};
    for (size_t i = 0; i < table.size(); ++i) {
      float value = float(i) / 255.0f;
      table[i] = value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
    }
    return table;
  }();
  double x0 = double(x) * width / columns, x1 = double(x + 1) * width / columns;
  double y0 = double(y) * height / rows, y1 = double(y + 1) * height / rows;
  double sum[3]{}, weight = 0;
  for (uint32_t sy = uint32_t(y0); sy < std::min(height, uint32_t(std::ceil(y1))); ++sy)
    for (uint32_t sx = uint32_t(x0); sx < std::min(width, uint32_t(std::ceil(x1))); ++sx) {
      double area = (std::min(x1, double(sx + 1)) - std::max(x0, double(sx))) *
                    (std::min(y1, double(sy + 1)) - std::max(y0, double(sy)));
      const uint8_t *pixel = pixels + (size_t(sy) * width + sx) * 4;
      for (int channel = 0; channel < 3; ++channel) sum[channel] += linear[pixel[2 - channel]] * area;
      weight += area;
    }
  std::array<uint8_t, 3> result{};
  for (int channel = 0; channel < 3; ++channel) {
    float value = float(sum[channel] / weight);
    float encoded = value <= 0.0031308f ? value * 12.92f : 1.055f * std::pow(value, 1.0f / 2.4f) - 0.055f;
    result[channel] = byte(encoded * 255.0f + 0.5f);
  }
  return result;
}

struct Preview {
  bool initialized = false;
  bool unicode = false;

  void present(const uint8_t *pixels, uint32_t width, uint32_t height) {
    uint32_t columns = uint32_t(std::min(uint64_t(1024), environmentCount("COLUMNS", 120)));
    uint32_t rows = uint32_t(std::min(uint64_t(1024), environmentCount("LINES", 40)));
#ifdef _WIN32
    HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO info{};
    if (GetConsoleScreenBufferInfo(console, &info)) {
      columns = info.srWindow.Right - info.srWindow.Left + 1;
      rows = info.srWindow.Bottom - info.srWindow.Top + 1;
    }
    if (!initialized) {
      DWORD mode;
      if (GetConsoleMode(console, &mode)) {
        SetConsoleMode(console, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        unicode = SetConsoleOutputCP(CP_UTF8) != 0;
      }
    }
#else
    winsize size{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0) {
      if (size.ws_col) columns = size.ws_col;
      if (size.ws_row) rows = size.ws_row;
    }
    unicode = isatty(STDOUT_FILENO) != 0;
#endif
    if (const char *ascii = std::getenv("GPRT_TERMINAL_ASCII"))
      if (std::string(ascii) == "1") unicode = false;
    columns = std::max(1u, columns / 2);
    rows = std::max(2u, rows) - 1;
    double scale = std::min(double(columns) / width, double(rows) / height);
    columns = std::max(1u, std::min(columns, uint32_t(width * scale)));
    rows = std::max(1u, std::min(rows, uint32_t(height * scale)));
    std::string output = initialized ? "\x1b[H" : "\x1b[2J\x1b[H";
    initialized = true;
    const char *blocks[] = {"\xe2\x96\x91", "\xe2\x96\x92", "\xe2\x96\x93", "\xe2\x96\x88"};
    const char *ascii[] = {".", ":", "*", "#"};
    for (uint32_t y = 0; y < rows; ++y) {
      for (uint32_t x = 0; x < columns; ++x) {
        auto rgb = filteredColor(pixels, width, height, columns, rows, x, y);
        auto brightness = byte(0.267f * rgb[0] + 0.642f * rgb[1] + 0.091f * rgb[2]);
        float average = (float(rgb[0]) + rgb[1] + rgb[2]) / 3.0f;
        for (auto &channel : rgb) channel = byte(average + (channel - average) * 1.3f);
        uint32_t color = 16 + 36 * ((rgb[0] + 25) / 51) + 6 * ((rgb[1] + 25) / 51) + ((rgb[2] + 25) / 51);
        if (rgb[0] == rgb[1] && rgb[1] == rgb[2])
          color = rgb[0] < 8 ? 16 : rgb[0] > 248 ? 231 : 232 + ((rgb[0] - 8) * 24 + 123) / 247;
        const char *glyph = unicode ? blocks[uint32_t(brightness) * 3 / 255] : ascii[uint32_t(brightness) * 3 / 255];
        output += "\x1b[38;5;" + std::to_string(color) + "m" + glyph + glyph;
      }
      output += "\x1b[0m\n";
    }
    std::cout << output << std::flush;
  }
};

} // namespace gprt::terminal
