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
  // Resize, but don't preserve contents (host)
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> buffer = gprtHostBufferCreate<uint32_t>(context, 1000000000 / 2);

    // Act
    gprtBufferResize(context, buffer, 1000000000, false);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != 1000000000 * sizeof(uint32_t)) 
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
    GPRTBufferOf<uint32_t> buffer = gprtDeviceBufferCreate<uint32_t>(context, 1000000000 / 2);

    // Act
    gprtBufferResize(context, buffer, 1000000000, false);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != 1000000000 * sizeof(uint32_t)) 
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
    GPRTBufferOf<uint32_t> buffer = gprtHostBufferCreate<uint32_t>(context, 1000000000 / 2);

    {
      uint32_t* ptr = gprtBufferGetPointer(buffer);
      for (uint32_t i = 0; i < 1000000000 / 2; ++i) {
        ptr[i] = i;
      }
    }

    // Act
    gprtBufferResize(context, buffer, 1000000000, true);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != 1000000000 * sizeof(uint32_t)) 
        throw std::runtime_error("Error, buffer not properly resized!");

      // Initial values should be preserved
      uint32_t* ptr = gprtBufferGetPointer(buffer);
      for (uint32_t i = 0; i < 1000000000 / 2; ++i) {
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
    GPRTBufferOf<uint32_t> buffer = gprtDeviceBufferCreate<uint32_t>(context, 1000000000 / 2);

    {
      gprtBufferMap(buffer);
      uint32_t* ptr = gprtBufferGetPointer(buffer);
      for (uint32_t i = 0; i < 1000000000 / 2; ++i) {
        ptr[i] = i;
      }
      gprtBufferUnmap(buffer);
    }

    // Act
    gprtBufferResize(context, buffer, 1000000000, true);

    // Assert
    {
      // Size should be correct
      if (gprtBufferGetSize(buffer) != 1000000000 * sizeof(uint32_t)) 
        throw std::runtime_error("Error, buffer not properly resized!");

      // Initial values should be preserved
      gprtBufferMap(buffer);
      uint32_t* ptr = gprtBufferGetPointer(buffer);
      for (uint32_t i = 0; i < 1000000000 / 2; ++i) {
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
