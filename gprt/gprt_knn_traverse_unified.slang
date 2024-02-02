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

struct StackItem {
    uint32_t _key;
    float _val;
    int key() { return _key;}
    float value() { return _val;}

    static StackItem Create(uint key, float value) {
        StackItem pair;
        pair._key = key;
        pair._val = value;
        return pair;
    }

    static StackItem Create() {
        StackItem pair;
        pair._key = -1;
        pair._val = FLT_MAX;
        return pair;
    }
};

struct Stack<let N : int> {
    StackItem items[N];

    [mutating]
    void clear() {
        for (int i = 0; i < N; ++i) {
            items[i] = StackItem::Create();
        }
    };

    [mutating]
    void insert(StackItem newItem) {
        if (items[0].value() > newItem.value()) {
            items[0] = newItem;
            // [unroll]
            for (int i = 1; i < N; ++i) {
                if (items[i - 1].value() >= items[i].value()) break;
                StackItem tmp = items[i - 1]; items[i - 1] = items[i]; items[i] = tmp;
            }
        }
    }

    static Stack<N> Create<let N : int>() {
        Stack<N> list;
        list.clear();
        return list;
    };

    int key(int index) {
        return items[N - 1 - index].key();
    };

    float value(int index) {
        return items[N - 1 - index].value();
    };
};

Stack<B> intersectAndSortChildren<let B : int> (
  float3 queryOrigin, inout float closestDistance,
  int start, int maxClusters,
  bool useAABBs, in gprt::Buffer aabbs,
  bool useOOBBs, in gprt::Buffer oobbs, bool debug = false)
{
  Stack<B> H;
  H.clear();

  for (int child = 0; child < B; ++child) {
    uint32_t index = start + child;
    if (index >= maxClusters) {
      break;
    }

    float minDist = 0.f;
    float maxDist = 1e38f;

    // Upward Culling: skip superclusters father than current closest primitive
    if (useAABBs) {
      float dist = getAABBDist(queryOrigin, aabbs, index);
      minDist = max(minDist, dist);
      if (minDist > closestDistance) continue;
    }

    if (useOOBBs) {
      float dist = getOOBBDist(queryOrigin, oobbs, index);
      minDist = max(minDist, dist);
      if (minDist > closestDistance) continue;
    }

    H.insert(StackItem::Create(index, minDist));
  }

  return H;
}

void TraverseLeaf<let B : int> (NodeRecord record, inout TraversePayload payload, int index) {
    int numPrims = record.numPrims;

    // At this point, it appears that the fastest thing to do is just
    // let traversal proceed linearly at the leaves to drive upward culling.
    for (uint32_t primID = 0; primID < B; ++primID) {
        uint32_t itemID = index * B + primID;
        if (itemID >= numPrims) continue;

        float3 p = payload.Origin;

        float3x4 tri = gprt::load<float3x4>(record.triangleLists, itemID);

        float3 ba = tri[1].xyz - tri[0].xyz; float3 pa = p - tri[0].xyz;
        float3 cb = tri[2].xyz - tri[1].xyz; float3 pb = p - tri[1].xyz;
        float3 ac = tri[0].xyz - tri[2].xyz; float3 pc = p - tri[2].xyz;
        float3 nor = cross(ba, ac);

        float distToEdgeA = sqrt(_dot2(ba * clamp(dot(ba, pa) / _dot2(ba), 0.0f, 1.0f) - pa));
        float distToEdgeB = sqrt(_dot2(cb * clamp(dot(cb, pb) / _dot2(cb), 0.0f, 1.0f) - pb));
        float distToEdgeC = sqrt(_dot2(ac * clamp(dot(ac, pc) / _dot2(ac), 0.0f, 1.0f) - pc));
        
        float distToPlane = sqrt(dot(nor, pa) * dot(nor, pa) / _dot2(nor));
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
}

void TraverseTree<let N : int, let B : int>(NodeRecord record, inout TraversePayload payload, int parentIndex)
{
    int start = parentIndex * B;
    Stack<B> stack = intersectAndSortChildren<B>(payload.Origin, payload.closestDistance,
                                                 start, record.numClusters[N - 1],
                                                 true/*useAABBs*/, record.aabbs[N - 1],
                                                 true/*useOOBBs*/, record.oobbs[N - 1]);
    for (int i = 0; i < B; ++i) {
        int childIndex = stack.key(i);
        float mindist = stack.value(i);
        if (childIndex == -1 || mindist > payload.closestDistance) return;
        if (N > 1) TraverseTree<N - 1, B>(record, payload, childIndex);
        else TraverseLeaf<B>(record, payload, childIndex);
    }
}

[shader("callable")]
void TraverseNode1(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<1, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode2(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<2, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode3(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<3, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode4(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<4, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode5(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<5, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode6(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<6, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode7(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<7, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode8(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<8, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode9(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<9, BRANCHING_FACTOR>(record, payload, 0);}

[shader("callable")]
void TraverseNode10(uniform NodeRecord record, inout TraversePayload payload) {TraverseTree<10, BRANCHING_FACTOR>(record, payload, 0);}