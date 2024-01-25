/******************************************************************************
 * Exclusive Vectorized Chained Scan With Decoupled Lookback
 *
 * Variant: Raking warp-sized radix reduce scan using partitions of size equal to 
 *          maximum shared memory.
 *                    
 * Notes:   **Preprocessor macros must be manually changed for AMD**
 * 
 * Author:  Thomas Smith 8/7/2023
 *
 * Based off of Research by:
 *          Duane Merrill, Nvidia Corporation
 *          Michael Garland, Nvidia Corporation
 *          https://research.nvidia.com/publication/2016-03_single-pass-parallel-prefix-scan-decoupled-look-back
 *
 * Copyright (c) 2011, Duane Merrill.  All rights reserved.
 * Copyright (c) 2011-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the NVIDIA CORPORATION nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/
#pragma once

#include "gprt_scan_shared.h"

#define PART_VEC_SIZE       2048
#define PART_VEC_MASK       2047
#define GROUP_SIZE          512
#define THREAD_BLOCKS       256
#define PART_LOG            13
#define PART_VEC_LOG        11

#define VECTOR_MASK         3
#define VECTOR_LOG          2

#ifndef SCAN_PARTITON_SIZE 
#define SCAN_PARTITON_SIZE 8192
#endif

#define FLAG_NOT_READY  0
#define FLAG_AGGREGATE  1
#define FLAG_INCLUSIVE  2
#define FLAG_MASK       3

#define WAVE_PARTITION_SIZE   (4 * LANE_COUNT)
#define WAVE_PART_LOG         uint(log2(WAVE_PARTITION_SIZE))
#define WAVES_PER_GROUP       (SCAN_PARTITON_SIZE / (16 * LANE_COUNT))
#define LANE_LOG              uint(log2(LANE_COUNT))    
#define LANE            gtid.x
#define LANE_MASK       (LANE_COUNT - 1)
#define WAVE_INDEX      gtid.y
#define SPINE_INDEX     (((gtid.x + 1) << WAVE_PART_LOG) - 1)
#define PARTITIONS      (pc.size >> PART_LOG)
#define WAVE_PART_START (WAVE_INDEX << WAVE_PART_LOG)
#define WAVE_PART_END   (WAVE_INDEX + 1 << WAVE_PART_LOG)
#define PARTITION_START (partitionIndex << PART_VEC_LOG)

groupshared uint4 g_sharedMem[PART_VEC_SIZE];

// Using byte address buffers here for a mix of 4 wide and 1 wide stores
RWByteAddressBuffer b_input;
RWByteAddressBuffer b_output;

// Buffer of partition flags and aggregates combined, one per partition
// top 2 bits are flags, bottom 30 bits are aggregates. (for 64 bits, a 64 bit interlocked "OR" is required.)
globallycoherent RWStructuredBuffer<uint> b_state;

// A counter to schedule workgroups
globallycoherent RWStructuredBuffer<uint> b_index;

// The total aggregate value
RWStructuredBuffer<int> b_aggregate;

uint getForwardAddr(int i, int size, int exclusiveSum) {
  return exclusiveSum;
}

uint getReverseAddr(int i, int size, int exclusiveSum) {
  return (size - 1) - (i - exclusiveSum);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void InitScan(uint3 DispatchThreadID : SV_DispatchThreadID, uniform ScanConstants pc) {
  uint3 id = DispatchThreadID;
  if (id.x == 0) {
    b_index[0] = 0;
    b_aggregate[0] = 0;
  }
  // clear the flag and aggregate
  b_state[id.x] = 0;
}

void Scan<let LANE_COUNT : uint32_t>(uint3 GroupThreadID, uint3 GroupID, ScanConstants pc) {
  uint3 gtid = GroupThreadID;
  uint3 gid = GroupID;

  //Acquire the partition index
  int partitionIndex;
  if (WAVE_INDEX == 0 && LANE == 0)
    InterlockedAdd(b_index[0], 1, g_sharedMem[0].x);
  GroupMemoryBarrierWithGroupSync();
  partitionIndex = WaveReadLaneAt(g_sharedMem[0].x, 0);
  GroupMemoryBarrierWithGroupSync();

  int select = ((pc.flags & SCAN_SELECT_POSITIVE) != 0) ? 0 : 1;

  bool partitioning = ((pc.flags & SCAN_PARTITION) > 0);
  bool selecting = ((pc.flags & SCAN_SELECT) > 0);

  int4 sg0Val = int4(0,0,0,0), sg1Val = int4(0,0,0,0), sg2Val = int4(0,0,0,0), sg3Val = int4(0,0,0,0);

  // four warp-sized-radix ranking reduce scans
  const int partSize = (partitionIndex == PARTITIONS) ? (pc.size >> VECTOR_LOG) + (((pc.size & VECTOR_MASK) != 0) ? 1 : 0) - PARTITION_START : PART_VEC_SIZE;
  // printf("PartSize %d\n", partSize);
  int i = LANE + WAVE_PART_START;
  if (i < partSize)
  {
    sg0Val = reinterpret<int4>(b_input.Load4(i * sizeof(int4)));
    if (partitioning || selecting) g_sharedMem[i] = (sg0Val >> 31) == select;
    else g_sharedMem[i] = sg0Val;
    
    uint t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].y;
    g_sharedMem[i].y = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].z;
    g_sharedMem[i].z = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].w;
    g_sharedMem[i].w = t;
    g_sharedMem[i] += WavePrefixSum(g_sharedMem[i].x);
  }
          
  i += LANE_COUNT;
  if (i < partSize)
  {
    sg1Val = reinterpret<int4>(b_input.Load4(i * sizeof(int4)));
    if (partitioning || selecting) g_sharedMem[i] = (sg1Val >> 31) == select;
    else g_sharedMem[i] = sg1Val;
    
    uint t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].y;
    g_sharedMem[i].y = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].z;
    g_sharedMem[i].z = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].w;
    g_sharedMem[i].w = t;
    g_sharedMem[i] += WavePrefixSum(g_sharedMem[i].x) + WaveReadLaneAt(g_sharedMem[i - 1].x, 0);
  }
          
  i += LANE_COUNT;
  if (i < partSize)
  {
    sg2Val = reinterpret<int4>(b_input.Load4(i * sizeof(int4)));
    if (partitioning || selecting) g_sharedMem[i] = (sg2Val >> 31) == select;
    else g_sharedMem[i] = sg2Val;
    
    uint t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].y;
    g_sharedMem[i].y = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].z;
    g_sharedMem[i].z = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].w;
    g_sharedMem[i].w = t;
    g_sharedMem[i] += WavePrefixSum(g_sharedMem[i].x) + WaveReadLaneAt(g_sharedMem[i - 1].x, 0);
  }
          
  i += LANE_COUNT;
  if (i < partSize)
  {
    sg3Val = reinterpret<int4>(b_input.Load4(i * sizeof(int4)));
    if (partitioning || selecting) g_sharedMem[i] = (sg3Val >> 31) == select;
    else g_sharedMem[i] = sg3Val;
    
    uint t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].y;
    g_sharedMem[i].y = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].z;
    g_sharedMem[i].z = t;
    
    t = g_sharedMem[i].x;
    g_sharedMem[i].x += g_sharedMem[i].w;
    g_sharedMem[i].w = t;
    g_sharedMem[i] += WavePrefixSum(g_sharedMem[i].x) + WaveReadLaneAt(g_sharedMem[i - 1].x, 0);
  }
  GroupMemoryBarrierWithGroupSync();

  if (WAVE_INDEX == 0 && LANE < WAVES_PER_GROUP)
    g_sharedMem[SPINE_INDEX] += WavePrefixSum(g_sharedMem[SPINE_INDEX].x);
  GroupMemoryBarrierWithGroupSync();

  // Set flag payload
  if (WAVE_INDEX == 0 && LANE == 0)
  {
    if (partitionIndex == 0)
      InterlockedOr(b_state[partitionIndex], FLAG_INCLUSIVE ^ (g_sharedMem[PART_VEC_MASK].x << 2));
    else
      InterlockedOr(b_state[partitionIndex], FLAG_AGGREGATE ^ (g_sharedMem[PART_VEC_MASK].x << 2));
  }

  // Decoupled lookback
  uint aggregate = 0;
  if (partitionIndex != 0)
  {
    if (WAVE_INDEX == 0)
    {
      for (int k = partitionIndex - LANE - 1; 0 <= k;)
      {
        uint flagPayload = b_state[k];
        const int inclusiveIndex = WaveActiveMin(LANE + LANE_COUNT - ((flagPayload & FLAG_MASK) == FLAG_INCLUSIVE ? LANE_COUNT : 0));
        const int gapIndex = WaveActiveMin(LANE + LANE_COUNT - ((flagPayload & FLAG_MASK) == FLAG_NOT_READY ? LANE_COUNT : 0));
        if (inclusiveIndex < gapIndex)
        {
          aggregate += WaveActiveSum(LANE <= inclusiveIndex ? (flagPayload >> 2) : 0);
          if (LANE == 0)
          {
            InterlockedAdd(b_state[partitionIndex], 1 | aggregate << 2);
            g_sharedMem[PART_VEC_MASK].x = aggregate;
          }
          break;
        }
        else
        {
          if (gapIndex < LANE_COUNT)
          {
            aggregate += WaveActiveSum(LANE < gapIndex ? (flagPayload >> 2) : 0);
            k -= gapIndex;
          }
          else
          {
            aggregate += WaveActiveSum(flagPayload >> 2);
            k -= LANE_COUNT;
          }
        }
      }
    }
    GroupMemoryBarrierWithGroupSync();
        
    // Propogate aggregate values
    if ((WAVE_INDEX != 0) || (LANE != 0))
      aggregate = WaveReadLaneAt(g_sharedMem[PART_VEC_MASK].x, 1);
  }

  const uint prev = (WAVE_INDEX ? WaveReadLaneAt(g_sharedMem[LANE + WAVE_PART_START - 1].x, 0) : 0) + aggregate;
  GroupMemoryBarrierWithGroupSync();

  uint lastOffset = (pc.size - 1) % 4;

  if (i < partSize)
  {
    uint i4 = i + PARTITION_START;
    g_sharedMem[i].x = g_sharedMem[i - 1].x + (LANE != LANE_MASK ? 0 : prev - aggregate);
    int4 exclusiveSums = g_sharedMem[i] + (LANE != LANE_MASK ? prev : aggregate);
    // Scatter by exclusive sum
    if (partitioning || selecting)
      for (int j = 0; j < 4; ++j) {
        if (4 * i4 + j >= pc.size) break;
        if ((sg3Val[j] >> 31) == select)            b_output.Store<int>(getForwardAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg3Val[j]);
        else if (partitioning)                      b_output.Store<int>(getReverseAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg3Val[j]);
      }

    // Store exclusive sum
    else b_output.Store4(i4 * sizeof(uint4), reinterpret<uint4>(exclusiveSums));

    // Store final aggregate
    if (i4 == ((pc.size + 3) / 4) - 1) {
      uint lastValue;
      if (partitioning || selecting) lastValue = ((sg3Val[lastOffset] >> 31) == select);
      else                           lastValue = sg3Val[lastOffset];
      b_aggregate[0] = exclusiveSums[lastOffset] + lastValue;
    }
  }
  
  i -= LANE_COUNT;
  if (i < partSize)
  {
    uint i4 = i + PARTITION_START;
    g_sharedMem[i].x = g_sharedMem[i - 1].x;
    uint4 exclusiveSums = g_sharedMem[i] + prev;
    // Scatter by exclusive sum
    if (partitioning || selecting)
      for (int j = 0; j < 4; ++j) {
        if (4 * i4 + j >= pc.size) break;
        if ((sg2Val[j] >> 31) == select)            b_output.Store<int>(getForwardAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg2Val[j]);
        else if (partitioning)                      b_output.Store<int>(getReverseAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg2Val[j]);
      }

    // Store exclusive sum
    else b_output.Store4(i4 * sizeof(uint4), reinterpret<uint4>(exclusiveSums));

    // Store final aggregate
    if (i4 == ((pc.size + 3) / 4) - 1) {
      uint lastValue;
      if (partitioning || selecting) lastValue = ((sg2Val[lastOffset] >> 31) == select);
      else                           lastValue = sg2Val[lastOffset];
      b_aggregate[0] = exclusiveSums[lastOffset] + lastValue;
    }
  }
          
  i -= LANE_COUNT;
  if (i < partSize)
  {
    uint i4 = i + PARTITION_START;
    g_sharedMem[i].x = g_sharedMem[i - 1].x;
    uint4 exclusiveSums = g_sharedMem[i] + prev;
    // Scatter by exclusive sum
    if (partitioning || selecting)
      for (int j = 0; j < 4; ++j) {
        if (4 * i4 + j >= pc.size) break;
        if ((sg1Val[j] >> 31) == select)            b_output.Store<int>(getForwardAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg1Val[j]);
        else if (partitioning)                      b_output.Store<int>(getReverseAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg1Val[j]);
      }

    // Store exclusive sum
    else b_output.Store4(i4 * sizeof(uint4), reinterpret<uint4>(exclusiveSums));

    // Store final aggregate
    if (i4 == ((pc.size + 3) / 4) - 1) {
      uint lastValue;
      if (partitioning || selecting) lastValue = ((sg1Val[lastOffset] >> 31) == select);
      else                           lastValue = sg1Val[lastOffset];
      b_aggregate[0] = exclusiveSums[lastOffset] + lastValue;
    }
  }
          
  i -= LANE_COUNT;
  if (i < partSize)
  {
    uint i4 = i + PARTITION_START;
    g_sharedMem[i].x = LANE ? g_sharedMem[i - 1].x : 0;
    uint4 exclusiveSums = g_sharedMem[i] + prev;
    // Scatter by exclusive sum
    if (partitioning || selecting)
      for (int j = 0; j < 4; ++j) {
        if (4 * i4 + j >= pc.size) break;
        if ((sg0Val[j] >> 31) == select)            b_output.Store<int>(getForwardAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg0Val[j]);
        else if (partitioning)                      b_output.Store<int>(getReverseAddr(4 * i4 + j, pc.size, exclusiveSums[j]) * sizeof(int), sg0Val[j]);
      }

    // Store exclusive sum
    else b_output.Store4(i4 * sizeof(uint4), reinterpret<uint4>(exclusiveSums));
    
    // Store final aggregate
    if (i4 == ((pc.size + 3) / 4) - 1) {
      uint lastValue;
      if (partitioning || selecting) lastValue = ((sg0Val[lastOffset] >> 31) == select);
      else                           lastValue = sg0Val[lastOffset];
      b_aggregate[0] = exclusiveSums[lastOffset] + lastValue;
    }
  }
}

// Below are configued such that each y group ID is assigned to a different subgroup.
// Each thread in a subgroup processes 16 items using 128 byte wide loads. 
// Likewise, each subgroup processes a LANE_COUNT * 16 items. 
// Therefore, we need SCAN_PARTITION_SIZE / LANE_COUNT * 16 threads for our Y group dimension

// for 32 lane count GPUs
[shader("compute")]
[numthreads(32, (SCAN_PARTITON_SIZE / (16 * 32)), 1)]
void Scan_32(uint3 GroupThreadID: SV_GroupThreadID, uint3 GroupID: SV_GroupID, uniform ScanConstants pc) {
  Scan<32>(GroupThreadID, GroupID, pc);
}

// For 64 lane count GPUs
[shader("compute")]
[numthreads(64, (SCAN_PARTITON_SIZE / (16 * 64)), 1)]
void Scan_64(uint3 GroupThreadID: SV_GroupThreadID, uint3 GroupID: SV_GroupID, uniform ScanConstants pc) {
  Scan<64>(GroupThreadID, GroupID, pc);
}
