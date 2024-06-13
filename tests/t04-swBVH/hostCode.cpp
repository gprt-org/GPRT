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


#include "gprt_lbvh.h"

bool cmp(std::pair<uint32_t, uint32_t>& a,
        std::pair<uint32_t, uint32_t>& b)
{
    return a.first < b.first;
}

int
main(int ac, char **av) {

  // Build a point LBVH
  {
    GPRTContext context = gprtContextCreate(nullptr, 1);

    // Seed with a real random value, if available
    std::random_device r;
    std::default_random_engine e1(r());
    auto rng = std::default_random_engine { r() };
    std::uniform_real_distribution<float> uniform_dist(-1e38f, 1e38f);

    // Generate a bunch of random points
    std::vector<float3> points(42); 
    for (uint32_t i = 0; i < 42; ++i) {
        points[i] = {uniform_dist(e1), uniform_dist(e1), uniform_dist(e1)};
    }
    GPRTBufferOf<float3> pointsBuffer = gprtDeviceBufferCreate<float3>(context, 42, points.data());

    GPRTLBVH lbvh = gprtPointsLBVHCreate(context, pointsBuffer, 42);


    gprtLBVHBuild(context, lbvh);


    // Cleanup  
    gprtLBVHDestroy(lbvh);
    gprtBufferDestroy(pointsBuffer);
    gprtContextDestroy(context);
  }
}
