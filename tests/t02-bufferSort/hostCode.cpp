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
#include <random>

int
main(int ac, char **av) {
  // Resize, but don't preserve contents (host)
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> data = gprtDeviceBufferCreate<uint32_t>(context, 1000000);
    GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);
    
    // Seed with a real random value, if available
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<uint32_t> uniform_dist(0, 4'294'967'295);

    gprtBufferMap(data);
    uint32_t *ptr = gprtBufferGetPointer(data);
    for (uint32_t i = 0; i < 1000000; ++i) {
        ptr[i] = uniform_dist(e1);
        // std::cout<<ptr[i]<<std::endl;
    }
    gprtBufferUnmap(data);

    std::cout<<std::endl;
    
    // Act
    gprtBufferSort(context, data, scratch);

    // Assert
    {
      gprtBufferMap(data);
      uint32_t *ptr = gprtBufferGetPointer(data);
      for (uint32_t i = 1; i < 1000000; ++i) {
        std::cout<<ptr[i];
        if (ptr[i] < ptr[i - 1]) {
          std::cout << " - out of order" << std::endl;
          throw std::runtime_error("Error, found out-of-order element!");
        }
        std::cout<<std::endl;
      }
      gprtBufferUnmap(data);
    }

    // Cleanup  
    gprtBufferDestroy(data);
    gprtBufferDestroy(scratch);
    gprtContextDestroy(context);
  }
}
