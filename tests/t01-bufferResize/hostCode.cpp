// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gprt.h>
#include <iostream>
#include <stdexcept>

int
main(int ac, char **av) {
  // Exercise aligned growth and shrinkage without exceeding device allocation limits.
  for (bool deviceLocal : {false, true}) {
    GPRTContext context = gprtContextCreate(nullptr, 1);
    uint32_t initial[64];
    for (uint32_t i = 0; i < 64; ++i) initial[i] = i + 1;
    auto buffer = deviceLocal ? gprtDeviceBufferCreate<uint32_t>(context, 64, initial, 4096)
                              : gprtHostBufferCreate<uint32_t>(context, 64, initial, 4096);
    for (size_t count : {size_t(128), size_t(32)}) {
      gprtBufferResize(context, buffer, count, true);
      if (gprtBufferGetSize(buffer) != count * sizeof(uint32_t)
          || uint64_t(gprtBufferGetDevicePointer(buffer)) % 4096 != 0)
        throw std::runtime_error("Resized buffer has incorrect size or alignment");
      gprtBufferMap(buffer);
      auto ptr = gprtBufferGetHostPointer(buffer);
      for (size_t i = 0; i < std::min(count, size_t(64)); ++i)
        if (ptr[i] != initial[i]) throw std::runtime_error("Resized buffer lost contents");
      gprtBufferUnmap(buffer);
    }
    gprtBufferDestroy(buffer);
    gprtContextDestroy(context);
  }
  const uint32_t resizeCount = 65536;
  // Resize, but don't preserve contents (host)
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> buffer = gprtHostBufferCreate<uint32_t>(context, resizeCount / 2);

    // Act
    gprtBufferResize(context, buffer, resizeCount, false);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != resizeCount * sizeof(uint32_t))
        throw std::runtime_error("Error, buffer not properly resized!");
    }

    // Cleanup  
    gprtBufferDestroy(buffer);
    gprtContextDestroy(context);
  }

  // Resize, but don't preserve contents (host)
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> buffer = gprtDeviceBufferCreate<uint32_t>(context, resizeCount / 2);

    // Act
    gprtBufferResize(context, buffer, resizeCount, false);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != resizeCount * sizeof(uint32_t))
        throw std::runtime_error("Error, buffer not properly resized!");
    }

    // Cleanup  
    gprtBufferDestroy(buffer);
    gprtContextDestroy(context);
  }


  // Resize and preserve contents (Host pinned)
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> buffer = gprtHostBufferCreate<uint32_t>(context, resizeCount / 2);

    {
      uint32_t* ptr = gprtBufferGetHostPointer(buffer);
      for (uint32_t i = 0; i < resizeCount / 2; ++i) {
        ptr[i] = i;
      }
    }

    // Act
    gprtBufferResize(context, buffer, resizeCount, true);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != resizeCount * sizeof(uint32_t))
        throw std::runtime_error("Error, buffer not properly resized!");

      // Initial values should be preserved
      uint32_t* ptr = gprtBufferGetHostPointer(buffer);
      for (uint32_t i = 0; i < resizeCount / 2; ++i) {
        if (ptr[i] != i) {
            throw std::runtime_error("Error, buffer values not preserved!");
        }
      }
    }

    // Cleanup  
    gprtBufferDestroy(buffer);
    gprtContextDestroy(context);
  }

  // Resize and preserve contents (Device)
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> buffer = gprtDeviceBufferCreate<uint32_t>(context, resizeCount / 2);

    {
      gprtBufferMap(buffer);
      uint32_t* ptr = gprtBufferGetHostPointer(buffer);
      for (uint32_t i = 0; i < resizeCount / 2; ++i) {
        ptr[i] = i;
      }
      gprtBufferUnmap(buffer);
    }

    // Act
    gprtBufferResize(context, buffer, resizeCount, true);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != resizeCount * sizeof(uint32_t))
        throw std::runtime_error("Error, buffer not properly resized!");

      // Initial values should be preserved
      gprtBufferMap(buffer);
      uint32_t* ptr = gprtBufferGetHostPointer(buffer);
      for (uint32_t i = 0; i < resizeCount / 2; ++i) {
        if (ptr[i] != i) {
            throw std::runtime_error("Error, buffer values not preserved!");
        }
      }
      gprtBufferUnmap(buffer);
    }

    // Cleanup  
    gprtBufferDestroy(buffer);
    gprtContextDestroy(context);
  }
}
