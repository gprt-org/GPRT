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
#include <chrono>

int
main(int ac, char **av) {
  // 32 million int32 items
  int numItems = 32000000;

  // Arrange
  std::vector<uint32_t> dataHost(numItems, 1); 
  
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTBufferOf<uint32_t> data = gprtDeviceBufferCreate<uint32_t>(context, numItems, dataHost.data());
  GPRTBufferOf<uint32_t> exclusiveSum = gprtDeviceBufferCreate<uint32_t>(context, numItems);
  GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);
  
  std::vector<uint32_t> hostExclusiveSum(numItems); 

  // Act
  std::cout<<"Computing exclusive sum on device" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 100; ++i) {
    gprtBufferExclusiveSum(context, data, exclusiveSum, scratch);
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 100.f;

  float itemsToBillions = float(1000000000) / float(numItems);  
  float millisecondsToSeconds = float(1000) / float(1);

  duration = (duration * itemsToBillions) / millisecondsToSeconds;

  std::cout<<"Done! Billions of input items / sec: " << 1.f / duration << std::endl;

  std::cout<<"Computing exclusive sum on host" << std::endl;
  int sum = 0;
  for (uint32_t i = 0; i < numItems; ++i) {
    int value = dataHost[i];
    hostExclusiveSum[i] = sum;
    sum += value;
  }
  std::cout<<"Done!" << std::endl;

  // Assert
  std::cout<<"Verifying correctness..." << std::endl;
  bool correct = true;
  gprtBufferMap(exclusiveSum);
  uint32_t* ptr = gprtBufferGetPointer(exclusiveSum);
  for (uint32_t i = 0; i < numItems; ++i) {
    // // emperical proof of correctness
    // int sum = 0;
    // for (int j = 0; j < i; ++j) {
    //   sum += dataHost[j];
    // }
    // if (sum != hostExclusiveSum[i]) {
    //   throw std::runtime_error("Error, exclusive sum is incorrect!");
    // }
    if (ptr[i] != hostExclusiveSum[i]) {
      std::cout<<"Error. Item " << i << " is " << ptr[i] << " but should be " << hostExclusiveSum[i] << std::endl;
      correct = false;
      break;
    }
  }
  gprtBufferUnmap(exclusiveSum);

  if (!correct) {
    throw std::runtime_error("Error, device and host output disagree!");
  } else {
    std::cout<<"Results appear correct!" <<std::endl;
  }

  // Cleanup  
  gprtBufferDestroy(data);
  gprtBufferDestroy(exclusiveSum);
  gprtBufferDestroy(scratch);
  gprtContextDestroy(context);
}
