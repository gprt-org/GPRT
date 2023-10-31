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

#include "gprt.h"

[[vk::push_constant]] gprt::ScanConstants pc;

// for 32 threads in a wave, this is 1024
#define TG_SIZE (WAVE_SIZE * WAVE_SIZE) 

groupshared int32_t gs_WaveSums[WAVE_SIZE];

typedef gprt::ScanRecord ScanRecord;

// Note, this requires running using a multiple of WAVE_SIZE threads.
// However, input does *not* need to be a multiple of WAVE_SIZE.
GPRT_COMPUTE_PROGRAM(ExclusiveSumGroups, (ScanRecord, record), (1024,1,1)) {
  uint DTid = DispatchThreadID.x;
  uint waveIndex = GroupIndex / WAVE_SIZE;

  int32_t val = 0;
  if (DTid < pc.numItems) {
    val = gprt::load<int32_t>(pc.input, DTid + pc.inputOffset);
  }
  int32_t prefixSum = WavePrefixSum(val);

  // If we're the last lane...
  if (WaveGetLaneIndex() == (WAVE_SIZE - 1)) {
    // cache this wave's total into shared memory
    uint32_t waveSum = prefixSum + val;
    gs_WaveSums[waveIndex] = waveSum;
  }

  GroupMemoryBarrierWithGroupSync();

  // If we're the first wave...
  if (waveIndex == 0) {
    // Compute the prefix sum of the individual wave totals
    int32_t waveSum = gs_WaveSums[GroupIndex];
    int32_t prefixWaveSum = WavePrefixSum(waveSum);
    gs_WaveSums[GroupIndex] = prefixWaveSum;
  }

  GroupMemoryBarrierWithGroupSync();

  // Make current wave's prefix sum relative to prior waves
  prefixSum += gs_WaveSums[waveIndex]; 
  if (DTid < pc.numItems) {
    gprt::store<int32_t>(pc.output, DTid + pc.outputOffset, prefixSum);
  }

  // If we're the last group... 
  if (GroupIndex == 1023 || GroupIndex == (pc.numItems - 1)) {
    // cache this group's total into global memory
    uint32_t groupSum = prefixSum + val;
    gprt::store<int32_t>(pc.scratch, GroupID.x + pc.scratchOffset, groupSum);
  }
}

// Combines groups of exclusive/inclusive sums using the exclusive/inclusive
// sums of their group totals.
GPRT_COMPUTE_PROGRAM(AddGroupPartialSums, (ScanRecord, record), (1024,1,1)) {
  uint DTid = DispatchThreadID.x;
  if (DTid >= pc.numItems) return;
  uint32_t num0Groups = pc.num0Groups;
  uint32_t num1Groups = pc.num1Groups;
  uint32_t num2Groups = pc.num2Groups;
  uint32_t group0ID = DTid / 1024;
  uint32_t group1ID = group0ID / 1024;
  uint32_t group2ID = group1ID / 1024;
  int32_t group0Total = gprt::load<int32_t>(pc.scratch, num0Groups + group0ID);
  int32_t group1Total = gprt::load<int32_t>(pc.scratch, num0Groups * 2 + num1Groups + group1ID);
  int32_t group2Total = gprt::load<int32_t>(pc.scratch, num0Groups * 2 + num1Groups * 2 + num2Groups + group2ID);
  int32_t localSum = gprt::load<int32_t>(pc.output, DTid) + group0Total + group1Total + group2Total;  
  gprt::store<int32_t>(pc.output, DTid, localSum);
}