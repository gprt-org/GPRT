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

  // Full copy from A to B
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> A = gprtDeviceBufferCreate<uint32_t>(context, 100000000);
    GPRTBufferOf<uint32_t> B = gprtDeviceBufferCreate<uint32_t>(context, 100000000);

    {
      gprtBufferMap(A);
      uint32_t* ptr = gprtBufferGetPointer(A);
      for (uint32_t i = 0; i < 100000000; ++i) {
        ptr[i] = i;
      }
      gprtBufferUnmap(A);
    }

    // Act
    gprtBufferCopy(context, A, B, 0, 0, 100000000);

    // Assert
    {
      gprtBufferMap(B);
      uint32_t* ptr = gprtBufferGetPointer(B);
      for (uint32_t i = 0; i < 100000000; ++i) {
        if (ptr[i] != i) throw std::runtime_error("Error, incorrect value!");
      }
      gprtBufferUnmap(B);
    }


    // Cleanup  
    gprtBufferDestroy(A);
    gprtBufferDestroy(B);
    gprtContextDestroy(context);
  }

  // Offset partial copy 1 from A to B
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> A = gprtDeviceBufferCreate<uint32_t>(context, 100000000);
    GPRTBufferOf<uint32_t> B = gprtDeviceBufferCreate<uint32_t>(context, 100000000);

    {
      gprtBufferMap(A);
      uint32_t* ptr = gprtBufferGetPointer(A);
      for (uint32_t i = 0; i < 100000000; ++i) {
        ptr[i] = i;
      }
      gprtBufferUnmap(A);
    }

    // Act
    gprtBufferCopy(context, A, B, 1, 0, 100000000 - 1);

    // Assert
    {
      gprtBufferMap(B);
      uint32_t* ptr = gprtBufferGetPointer(B);
      for (uint32_t i = 0; i < 100000000 - 1; ++i) {
        if (ptr[i] != i + 1) throw std::runtime_error("Error, incorrect value!");
      }
      gprtBufferUnmap(B);
    }

    // Cleanup  
    gprtBufferDestroy(A);
    gprtBufferDestroy(B);
    gprtContextDestroy(context);
  }
}
