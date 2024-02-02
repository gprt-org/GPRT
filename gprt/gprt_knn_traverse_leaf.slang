// MIT License

// Copyright (c) 2023 Nathan V. Morrical

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
#pragma once

#include "gprt_knn_shared.h"

[shader("miss")]
void TraverseLeafCallable(uniform LeafRecord record, inout TraversePayload payload) {
    int leafID = payload.parentIndex;
    float3 queryOrigin = WorldRayOrigin();

    gprt::Buffer triangleLists = record.triangleLists;
    int numPrims = record.numPrims;

    // At this point, it appears that the fastest thing to do is just
    // let traversal proceed linearly at the leaves to drive upward culling.
    for (uint32_t primID = 0; primID < BRANCHING_FACTOR; ++primID) {
        uint32_t itemID = leafID * BRANCHING_FACTOR + primID;
        if (itemID >= numPrims) continue;

        float3 p = queryOrigin;

        float3 a = gprt::load<float3>(triangleLists, itemID * 3 + 0);
        float3 b = gprt::load<float3>(triangleLists, itemID * 3 + 1);

        float3 ba = b - a; float3 pa = p - a;
        float distToEdgeA = sqrt(_dot2(ba * clamp(dot(ba, pa) / _dot2(ba), 0.0f, 1.0f) - pa));
        // if (distToEdgeA > payload.closestDistance) continue;

        float3 c = gprt::load<float3>(triangleLists, itemID * 3 + 2);
        float3 cb = c - b; float3 pb = p - b;

        float distToEdgeB = sqrt(_dot2(cb * clamp(dot(cb, pb) / _dot2(cb), 0.0f, 1.0f) - pb));
        // if (distToEdgeB > payload.closestDistance) continue;

        float3 ac = a - c; float3 pc = p - c;
        float distToEdgeC = sqrt(_dot2(ac * clamp(dot(ac, pc) / _dot2(ac), 0.0f, 1.0f) - pc));
        // if (distToEdgeC > payload.closestDistance) continue;

        float3 nor = cross(ba, ac);
        float distToPlane = sqrt(dot(nor, pa) * dot(nor, pa) / _dot2(nor));
        // if (distToPlane > payload.closestDistance) continue;

        float distToEdges = min(min(distToEdgeA, distToEdgeB), distToEdgeC);

        // inside/outside test
        bool inside = (sign(dot(cross(ba, nor), pa)) +
                           sign(dot(cross(cb, nor), pb)) +
                           sign(dot(cross(ac, nor), pc)) < 2.0f);

        float dist = inside ? distToEdges : distToPlane;

        // Primitive farther than furthest
        if (dist > payload.closestDistance) continue;

        // Newly found closest primitive
        payload.closestDistance = dist;
        payload.closestPrimitive = itemID;
    }
    // HitObject hitObject = HitObject::MakeHit(/*parameters here*/);
}
