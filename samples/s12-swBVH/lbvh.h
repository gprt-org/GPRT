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

#include "gprt.h"

struct LBVHData {
  // Note, user must currently set global aabb before construction. 
  alignas(16) gprt::Buffer aabbs;
  alignas(8) uint32_t numPrims;

  alignas(16) gprt::Buffer positions;
  alignas(16) gprt::Buffer edges;
  alignas(16) gprt::Buffer triangles;

  alignas(16) gprt::Buffer centroids;
  alignas(16) gprt::Buffer mortonCodes;
  alignas(16) gprt::Buffer ids;
  
  // Nodes is numPrims-1 + numPrims in size. 
  // The "numPrims-1" section contains inner nodes
  // The "numPrims" section contains leaves

  // Notes is numPrims-1 + numPrims long. Each node is an int4. 
  // "X" is left, "Y" is right, "Z" is parent, and "W" is leaf or -1 if internal node.
  alignas(16) gprt::Buffer nodes;
};
