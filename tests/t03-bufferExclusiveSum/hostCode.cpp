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
  int prime = 9973;
  int numItems = 0; //1024 * 1024;//2 << 28;

  for (int round = 0; round < 100; ++round) {
    std::cout<<"\rRound " << round;
    numItems += prime;
    
    // Arrange
    std::vector<uint32_t> dataHost(numItems, 1); 
    
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> data = gprtDeviceBufferCreate<uint32_t>(context, numItems, dataHost.data());
    GPRTBufferOf<uint32_t> exclusiveSum = gprtDeviceBufferCreate<uint32_t>(context, numItems);
    GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);
    
    std::vector<uint32_t> hostExclusiveSum(numItems); 

    // Act
    // std::cout<<"Computing exclusive sum on device" << std::endl;
    uint32_t total = gprtBufferExclusiveSum(context, data, 0, numItems, exclusiveSum, scratch);
    // std::cout<<"Done! Total is " << total <<std::endl;

    // std::cout<<"Computing exclusive sum on host" << std::endl;
    int sum = 0;
    for (uint32_t i = 0; i < numItems; ++i) {
      int value = dataHost[i];
      hostExclusiveSum[i] = sum;
      sum += value;
    }
    // std::cout<<"Done!" << std::endl;

    // Assert
    // std::cout<<"Verifying correctness..." << std::endl;
    bool correct = true;
    gprtBufferMap(exclusiveSum);
    uint32_t* ptr = gprtBufferGetPointer(exclusiveSum);
    for (uint32_t i = 0; i < numItems; ++i) {
      if (ptr[i] != hostExclusiveSum[i]) {
        std::cout<<"Error. Item " << i << " is " << ptr[i] << " but should be " << hostExclusiveSum[i] << std::endl;
        correct = false;
      }
    }
    gprtBufferUnmap(exclusiveSum);

    if (total != numItems) {
      throw std::runtime_error("Error, total is incorrect!");
    }
    else if (!correct) {
      throw std::runtime_error("Error, device and host output disagree!");
    } else {
      // std::cout<<"Results appear correct!" <<std::endl;
    }

    // Cleanup  
    gprtBufferDestroy(data);
    gprtBufferDestroy(exclusiveSum);
    gprtBufferDestroy(scratch);
    gprtContextDestroy(context);
  }
  std::cout<<" - Done" << std::endl;

  numItems = prime;
  for (int round = 0; round < 100; ++round) {
    std::cout<<"\rRound " << round;
    numItems += prime;
    
    // Arrange
    // std::cout<<"Generating data..."<<std::endl;
    std::vector<int32_t> dataHost(numItems);
    for (int i = 0; i < numItems ; ++i) {
      dataHost[i] = i * ((i & 1) ? 1 : -1);
    }
    
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<int32_t> data = gprtDeviceBufferCreate<int32_t>(context, numItems, dataHost.data());
    GPRTBufferOf<int32_t> partition = gprtDeviceBufferCreate<int32_t>(context, numItems);
    GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);

    // Act
    // std::cout<<"Computing paritioning on device" << std::endl;
    uint32_t total = gprtBufferPartition(context, data, 0, numItems, true, partition, scratch);
    // std::cout<<"Done!"<<std::endl;

    // std::cout<<"Computing exclusive sum on host" << std::endl;
    std::vector<uint32_t> hostExclusiveSum(numItems); 
    int sum = 0;
    for (uint32_t i = 0; i < numItems; ++i) {
      int value = (dataHost[i] >= 0) ? 1 : 0;
      hostExclusiveSum[i] = sum;
      sum += value;
    }
    // std::cout<<"Done!" << std::endl;

    // std::cout<<"Computing host partition using exclusive sum..."<<std::endl;
    std::vector<int32_t> hostPartition = std::vector<int32_t>(numItems);
    uint32_t hostTotal = 0;
    std::vector<int32_t> devicePartition = std::vector<int32_t>(numItems);
    for (int i = 0; i < numItems; ++i) {
      if (dataHost[i] >= 0) {
        hostTotal++;
        uint32_t addr = hostExclusiveSum[i];
        hostPartition[addr] = dataHost[i];
      }
      else {
        uint32_t addr = (numItems - 1) - (i - hostExclusiveSum[i]);
        hostPartition[addr] = dataHost[i];
      }
    }

    // Assert
    // std::cout<<"Verifying correctness..." << std::endl;
    bool correct = true;
    gprtBufferMap(partition);
    int32_t* ptr = gprtBufferGetPointer(partition);
    memcpy(devicePartition.data(), ptr, numItems * sizeof(int32_t));
    for (uint32_t i = 0; i < numItems; ++i) {
      if (ptr[i] != hostPartition[i]) {
        std::cout<<"Error. Item " << i << " is " << ptr[i] << " but should be " << hostPartition[i] << std::endl;
        correct = false;
      }
    }
    gprtBufferUnmap(partition);

    if (total != hostTotal) {
      throw std::runtime_error("Error, total is incorrect!");
    }
    else if (!correct) {
      throw std::runtime_error("Error, device and host output disagree!");
    } else {
      // std::cout<<"Results appear correct!" <<std::endl;
    }

    // Cleanup  
    gprtBufferDestroy(data);
    gprtBufferDestroy(partition);
    gprtBufferDestroy(scratch);
    gprtContextDestroy(context);
  }

  std::cout<<" - Done" << std::endl;

  numItems = prime;
  for (int round = 0; round < 100; ++round) {
    std::cout<<"\rRound " << round;
    numItems += prime;
    
    // Arrange
    // std::cout<<"Generating data..."<<std::endl;
    std::vector<int32_t> dataHost(numItems);
    for (int i = 0; i < numItems ; ++i) {
      dataHost[i] = i * ((i & 1) ? 1 : -1);
    }
    
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<int32_t> data = gprtDeviceBufferCreate<int32_t>(context, numItems, dataHost.data());
    GPRTBufferOf<int32_t> partition = gprtDeviceBufferCreate<int32_t>(context, numItems);
    GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);

    // Act
    // std::cout<<"Computing selection on device" << std::endl;
    uint32_t total = gprtBufferSelect(context, data, 0, numItems, true, partition, scratch);
    // std::cout<<"Done!"<<std::endl;

    // std::cout<<"Computing exclusive sum on host" << std::endl;
    std::vector<uint32_t> hostExclusiveSum(numItems); 
    int sum = 0;
    for (uint32_t i = 0; i < numItems; ++i) {
      int value = (dataHost[i] >= 0) ? 1 : 0;
      hostExclusiveSum[i] = sum;
      sum += value;
    }
    // std::cout<<"Done!" << std::endl;

    // std::cout<<"Computing host selection using exclusive sum..."<<std::endl;
    std::vector<int32_t> hostSelection = std::vector<int32_t>(numItems);
    std::vector<int32_t> deviceSelection = std::vector<int32_t>(numItems);
    for (int i = 0; i < numItems; ++i) {
      if (dataHost[i] >= 0) {
        uint32_t addr = hostExclusiveSum[i];
        hostSelection[addr] = dataHost[i];
      }
    }

    // Assert
    // std::cout<<"Verifying correctness..." << std::endl;
    bool correct = true;
    gprtBufferMap(partition);
    int32_t* ptr = gprtBufferGetPointer(partition);
    memcpy(deviceSelection.data(), ptr, numItems * sizeof(int32_t));
    for (uint32_t i = 0; i < numItems; ++i) {
      if (ptr[i] != hostSelection[i]) {
        std::cout<<"Error. Item " << i << " is " << ptr[i] << " but should be " << hostSelection[i] << std::endl;
        correct = false;
      }
    }
    gprtBufferUnmap(partition);

    if (total != sum) {
      throw std::runtime_error("Error, total is incorrect!");
    }
    else if (!correct) {
      throw std::runtime_error("Error, device and host output disagree!");
    } else {
      // std::cout<<"Results appear correct!" <<std::endl;
    }

    // Cleanup  
    gprtBufferDestroy(data);
    gprtBufferDestroy(partition);
    gprtBufferDestroy(scratch);
    gprtContextDestroy(context);
  }
  std::cout<<" - Done" << std::endl;
}
