// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#ifdef GPRT_DEVICE
#include "gprt_device.hlsli"
#define alignas(alignment)

// ideally this buffer type would be a struct...
// but I'm running into a compiler bug reading one struct inside another one.
namespace gprt {
  // struct Buffer {
  //   uint64_t x;
  //   uint64_t y;
  // };
  
  typedef uint64_t2 Buffer;

  template<typename T> 
  T load(in Buffer buffer, uint64_t index) {
    return vk::RawBufferLoad<T>(buffer.x + index * sizeof(T));
  }
  template<typename T> 
  void store(in Buffer buffer, uint64_t index, in T value) {
    vk::RawBufferStore<T>(buffer.x + index * sizeof(T), value);
  }
}
#else 
#include <stdalign.h>
#include "gprt_host.h"

namespace gprt{
  typedef uint64_t2 Buffer;
}
#endif
