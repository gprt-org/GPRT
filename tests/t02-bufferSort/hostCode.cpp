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
#include <algorithm>

bool cmp(std::pair<uint32_t, uint32_t>& a,
        std::pair<uint32_t, uint32_t>& b)
{
    return a.first < b.first;
}
 

int
main(int ac, char **av) {
  // Sort Keys
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> data = gprtDeviceBufferCreate<uint32_t>(context, 100000000);
    GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);
    
    std::vector<uint32_t> dataHost(100000000); 

    // Seed with a real random value, if available
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<uint32_t> uniform_dist(0, 4'294'967'295);

    gprtBufferMap(data);
    uint32_t *ptr = gprtBufferGetPointer(data);
    for (uint32_t i = 0; i < 100000000; ++i) {
        ptr[i] = uniform_dist(e1);
        dataHost[i] = ptr[i];
    }
    gprtBufferUnmap(data);
    std::sort(dataHost.begin(), dataHost.end());
    
    // Act
    gprtBufferSort(context, data, scratch);

    // Assert
    gprtBufferMap(data);
    ptr = gprtBufferGetPointer(data);
    for (uint32_t i = 1; i < 100000000; ++i) {
      if (ptr[i] != dataHost[i]) throw std::runtime_error("Error, found element in output that does not exist in input!");
      if (i > 0 && ptr[i] < ptr[i - 1]) throw std::runtime_error("Error, found out of order element!");
    }
    gprtBufferUnmap(data);

    // Cleanup  
    gprtBufferDestroy(data);
    gprtBufferDestroy(scratch);
    gprtContextDestroy(context);
  }

  // Sort Keys and Values
  {
    // Arrange
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTBufferOf<uint32_t> keys = gprtDeviceBufferCreate<uint32_t>(context, 100000000);
    GPRTBufferOf<uint32_t> values = gprtDeviceBufferCreate<uint32_t>(context, 100000000);
    GPRTBufferOf<uint32_t> scratch = gprtDeviceBufferCreate<uint32_t>(context);
    
    std::vector<std::pair<uint32_t, uint32_t>> keyValsHost(100000000); 

    // Seed with a real random value, if available
    std::random_device r;
    std::default_random_engine e1(r());
    auto rng = std::default_random_engine { r() };
    std::uniform_int_distribution<uint32_t> uniform_dist(0, 4'294'967'295);

    // Fill a list of pairs such that no keys collide 
    for (uint32_t i = 0; i < 100000000; ++i) {
        keyValsHost[i] = {i, uniform_dist(e1)};
    }

    // shuffle the list
    std::shuffle(std::begin(keyValsHost), std::end(keyValsHost), rng);
    
    gprtBufferMap(keys);
    gprtBufferMap(values);
    uint32_t *kptr = gprtBufferGetPointer(keys);
    uint32_t *vptr = gprtBufferGetPointer(values);
    for (uint32_t i = 0; i < 100000000; ++i) {
        kptr[i] = keyValsHost[i].first;
        vptr[i] = keyValsHost[i].second;
    }
    gprtBufferUnmap(keys);
    gprtBufferUnmap(values);

    // now sort this list. Since keys are unique, after a radix sort the two lists should match
    std::sort(keyValsHost.begin(), keyValsHost.end(), cmp);
    
    // Act
    gprtBufferSortPayload(context, keys, values, scratch);

    // Assert
    gprtBufferMap(keys);
    gprtBufferMap(values);
    kptr = gprtBufferGetPointer(keys);
    vptr = gprtBufferGetPointer(values);
    for (uint32_t i = 0; i < 100000000; ++i) {
      if (kptr[i] != keyValsHost[i].first) {
        throw std::runtime_error("Error, found key in output that does not exist in input!");
      }
      if (vptr[i] != keyValsHost[i].second)  {
        throw std::runtime_error("Error, found value in output that does not exist in input!");
      }
    }
    gprtBufferUnmap(keys);
    gprtBufferUnmap(values);

    // Cleanup  
    gprtBufferDestroy(keys);
    gprtBufferDestroy(values);
    gprtBufferDestroy(scratch);
    gprtContextDestroy(context);
  }
}
