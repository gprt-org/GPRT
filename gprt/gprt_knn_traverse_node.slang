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

RayDesc PollRayDesc() {
    RayDesc ray;
    ray.Origin = WorldRayOrigin();
    ray.Direction = WorldRayDirection();
    ray.TMin = RayTMin();
    ray.TMax = RayTCurrent();
    return ray;
}

// This works, but it's slow...
[shader("miss")]
void TraverseNodeCallable(uniform NodeRecord record, inout TraversePayload payload) {
    RaytracingAccelerationStructure nullAccel;
    RayDesc query = PollRayDesc();

    int level = payload.level;
    int parentIndex = payload.parentIndex;

    int start = parentIndex * BRANCHING_FACTOR;
    var stack = intersectAndSortChildren<BRANCHING_FACTOR>(query.Origin, payload.closestDistance,
                                                            start, record.numClusters[level],
                                                            true, record.aabbs[level],
                                                            true, record.oobbs[level]);
    for (int i = 0; i < BRANCHING_FACTOR; ++i) {
        int currentIndex = stack.key(i);
        float mindist = stack.value(i);
        if (currentIndex == -1 || mindist > payload.closestDistance) return;

        payload.parentIndex = currentIndex;
        payload.level = level - 1;

        // Lots of live state currently being dumped / loaded with the above stack...
        if (level > 0) {
            HitObject hitObject = HitObject::MakeMiss(TRAVERSE_NODE_CALLABLE, query);
            hitObject.Invoke(nullAccel, hitObject, payload);
        } else {
            HitObject hitObject = HitObject::MakeMiss(TRAVERSE_LEAF_CALLABLE, query);
            hitObject.Invoke(nullAccel, hitObject, payload);
        }
    }
}
