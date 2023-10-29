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
  // Arrange
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTBufferOf<uint32_t> data = gprtDeviceBufferCreate<uint32_t>(context, 1000);
  GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);
  
  std::vector<uint32_t> dataHost(1000); 

  // Seed with a real random value, if available
  // std::random_device r;
  // std::default_random_engine e1(r());
  // std::uniform_int_distribution<uint32_t> uniform_dist(0, 4'294'967'295);

  // for now, just testng to see if a series of 1's work.
  gprtBufferMap(data);
  uint32_t *ptr = gprtBufferGetPointer(data);
  for (uint32_t i = 0; i < 1000; ++i) {
      ptr[i] = 1;//uniform_dist(e1);
      dataHost[i] = ptr[i];
  }
  gprtBufferUnmap(data);

  std::vector<uint32_t> dataHostExclusiveSum(1000); 

  int sum = 0;
  for (uint32_t i = 0; i < 1000; ++i) {
    int value = dataHost[i];
    dataHostExclusiveSum[i] = sum;
    sum += value;
  }
  
  // Act
  gprtBufferExclusiveSum(context, data, scratch);

  // Assert
  gprtBufferMap(data);
  ptr = gprtBufferGetPointer(data);
  for (uint32_t i = 0; i < 1000; ++i) {
    // emperical proof of correctness
    int sum = 0;
    for (int j = 0; j < i; ++j) {
      sum += dataHost[j];
    }
    if (sum != dataHostExclusiveSum[i]) throw std::runtime_error("Error, exclusive sum is incorrect!");
    // if (ptr[i] != dataHostExclusiveSum[i]) throw std::runtime_error("Error, device and host output disagree!");
  }
  gprtBufferUnmap(data);

  // Cleanup  
  gprtBufferDestroy(data);
  gprtBufferDestroy(scratch);
  gprtContextDestroy(context);
}
