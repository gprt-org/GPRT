// ParallelSortCS.hlsl
//
// Copyright(c) 2021 Advanced Micro Devices, Inc.All rights reserved.
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


//--------------------------------------------------------------------------------------
// ParallelSort Shaders/Includes
//--------------------------------------------------------------------------------------
#define FFX_HLSL
#include "sort.h"

// [[vk::binding(/*binding*/, /*set*/)]] 

[[vk::push_constant]] FFX_ParallelSortCB rootConstData;								// Store the shift bit directly in the root signature

[[vk::binding(0, 0)]] RWStructuredBuffer<uint>	SrcBuffer		;					// The unsorted keys or scan data
[[vk::binding(2, 0)]] RWStructuredBuffer<uint>	SrcPayload		;					// The payload data
				 
[[vk::binding(0, 2)]] RWStructuredBuffer<uint>	SumTable		;					// The sum table we will write sums to
[[vk::binding(1, 2)]] RWStructuredBuffer<uint>	ReduceTable		;					// The reduced sum table we will write sums to
				 
[[vk::binding(1, 0)]] RWStructuredBuffer<uint>	DstBuffer		;					// The sorted keys or prefixed data
[[vk::binding(3, 0)]] RWStructuredBuffer<uint>	DstPayload		;					// the sorted payload data
				 
[[vk::binding(0, 1)]] RWStructuredBuffer<uint>	ScanSrc			;					// Source for Scan Data
[[vk::binding(1, 1)]] RWStructuredBuffer<uint>	ScanDst			;					// Destination for Scan Data
[[vk::binding(2, 1)]] RWStructuredBuffer<uint>	ScanScratch		;					// Scratch data for Scan
				 

groupshared uint32_t gs_FFX_PARALLELSORT_LDSSums[FFX_PARALLELSORT_THREADGROUP_SIZE];
uint32_t FFX_ParallelSort_ThreadgroupReduce(uint32_t localSum, uint32_t localID)
{
	// Do wave local reduce
	uint32_t waveReduced = WaveActiveSum(localSum);

	// First lane in a wave writes out wave reduction to LDS (this accounts for num waves per group greater than HW wave size)
	// Note that some hardware with very small HW wave sizes (i.e. <= 8) may exhibit issues with this algorithm, and have not been tested.
	uint32_t waveID = localID / WaveGetLaneCount();
	if (WaveIsFirstLane())
		gs_FFX_PARALLELSORT_LDSSums[waveID] = waveReduced;

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// First wave worth of threads sum up wave reductions
	if (!waveID)
		waveReduced = WaveActiveSum( (localID < FFX_PARALLELSORT_THREADGROUP_SIZE / WaveGetLaneCount()) ? gs_FFX_PARALLELSORT_LDSSums[localID] : 0);

	// Returned the reduced sum
	return waveReduced;
}

uint32_t FFX_ParallelSort_BlockScanPrefix(uint32_t localSum, uint32_t localID)
{
	// Do wave local scan-prefix
	uint32_t wavePrefixed = WavePrefixSum(localSum);

	// Since we are dealing with thread group sizes greater than HW wave size, we need to account for what wave we are in.
	uint32_t waveID = localID / WaveGetLaneCount();
	uint32_t laneID = WaveGetLaneIndex();

	// Last element in a wave writes out partial sum to LDS
	if (laneID == WaveGetLaneCount() - 1)
		gs_FFX_PARALLELSORT_LDSSums[waveID] = wavePrefixed + localSum;

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// First wave prefixes partial sums
	if (!waveID)
		gs_FFX_PARALLELSORT_LDSSums[localID] = WavePrefixSum(gs_FFX_PARALLELSORT_LDSSums[localID]);

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// Add the partial sums back to each wave prefix
	wavePrefixed += gs_FFX_PARALLELSORT_LDSSums[waveID];

	return wavePrefixed;
}

// FPS Count
groupshared uint32_t gs_FFX_PARALLELSORT_Histogram[FFX_PARALLELSORT_THREADGROUP_SIZE * FFX_PARALLELSORT_SORT_BIN_COUNT];
[numthreads(FFX_PARALLELSORT_THREADGROUP_SIZE, 1, 1)] [shader("compute")]
void __compute__Count(uint32_t localID : SV_GroupThreadID, uint32_t groupID : SV_GroupID)
{
	// Call the uint32_t version of the count part of the algorithm
	// FFX_ParallelSort_Count_uint(localID, groupID, CBuffer, rootConstData.CShiftBit, SrcBuffer, SumTable);  
	
	uint32_t ShiftBit = rootConstData.CShiftBit;
	// Start by clearing our local counts in LDS
	for (int32_t i = 0; i < FFX_PARALLELSORT_SORT_BIN_COUNT; i++)
		gs_FFX_PARALLELSORT_Histogram[(i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID] = 0;

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// Data is processed in blocks, and how many we process can changed based on how much data we are processing
	// versus how many thread groups we are processing with
	int32_t BlockSize = FFX_PARALLELSORT_ELEMENTS_PER_THREAD * FFX_PARALLELSORT_THREADGROUP_SIZE;

	// Figure out this thread group's index into the block data (taking into account thread groups that need to do extra reads)
	uint32_t ThreadgroupBlockStart = (BlockSize * rootConstData.NumBlocksPerThreadGroup * groupID);
	uint32_t NumBlocksToProcess = rootConstData.NumBlocksPerThreadGroup;

	if (groupID >= rootConstData.NumThreadGroups - rootConstData.NumThreadGroupsWithAdditionalBlocks)
	{
		ThreadgroupBlockStart += (groupID - (rootConstData.NumThreadGroups - rootConstData.NumThreadGroupsWithAdditionalBlocks)) * BlockSize;
		NumBlocksToProcess++;
	}

	// Get the block start index for this thread
	uint32_t BlockIndex = ThreadgroupBlockStart + localID;

	// Count value occurrence
	for (uint32_t BlockCount = 0; BlockCount < NumBlocksToProcess; BlockCount++, BlockIndex += BlockSize)
	{
		uint32_t DataIndex = BlockIndex;

		// Pre-load the key values in order to hide some of the read latency
		uint32_t srcKeys[FFX_PARALLELSORT_ELEMENTS_PER_THREAD];
		srcKeys[0] = SrcBuffer[DataIndex];
		srcKeys[1] = SrcBuffer[DataIndex + FFX_PARALLELSORT_THREADGROUP_SIZE];
		srcKeys[2] = SrcBuffer[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 2)];
		srcKeys[3] = SrcBuffer[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 3)];

		for (uint32_t i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
		{
			if (DataIndex < rootConstData.NumKeys)
			{
				uint32_t localKey = (srcKeys[i] >> ShiftBit) & 0xf;
				InterlockedAdd(gs_FFX_PARALLELSORT_Histogram[(localKey * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID], 1);
				DataIndex += FFX_PARALLELSORT_THREADGROUP_SIZE;
			}
		}
	}

	// Even though our LDS layout guarantees no collisions, our thread group size is greater than a wave
	// so we need to make sure all thread groups are done counting before we start tallying up the results
	GroupMemoryBarrierWithGroupSync();

	if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
	{
		uint32_t sum = 0;
		for (int32_t i = 0; i < FFX_PARALLELSORT_THREADGROUP_SIZE; i++)
		{
			sum += gs_FFX_PARALLELSORT_Histogram[localID * FFX_PARALLELSORT_THREADGROUP_SIZE + i];
		}
		SumTable[localID * rootConstData.NumThreadGroups + groupID] = sum;
	}
}

// FPS Reduce
[numthreads(FFX_PARALLELSORT_THREADGROUP_SIZE, 1, 1)] [shader("compute")]
void __compute__CountReduce(uint32_t localID : SV_GroupThreadID, uint32_t groupID : SV_GroupID)
{
	// Call the reduce part of the algorithm
	// FFX_ParallelSort_ReduceCount(localID, groupID, CBuffer,  SumTable, ReduceTable);

	// Figure out what bin data we are reducing
	uint32_t BinID = groupID / rootConstData.NumReduceThreadgroupPerBin;
	uint32_t BinOffset = BinID * rootConstData.NumThreadGroups;

	// Get the base index for this thread group
	uint32_t BaseIndex = (groupID % rootConstData.NumReduceThreadgroupPerBin) * FFX_PARALLELSORT_ELEMENTS_PER_THREAD * FFX_PARALLELSORT_THREADGROUP_SIZE;

	// Calculate partial sums for entries this thread reads in
	uint32_t threadgroupSum = 0;
	for (uint32_t i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; ++i)
	{
		uint32_t DataIndex = BaseIndex + (i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID;
		threadgroupSum += (DataIndex < rootConstData.NumThreadGroups) ? SumTable[BinOffset + DataIndex] : 0;
	}

	// Reduce across the entirety of the thread group
	threadgroupSum = FFX_ParallelSort_ThreadgroupReduce(threadgroupSum, localID);

	// First thread of the group writes out the reduced sum for the bin
	if (!localID)
		ReduceTable[groupID] = threadgroupSum;

	// What this will look like in the reduced table is:
	//	[ [bin0 ... bin0] [bin1 ... bin1] ... ]
}

// FPS Scan
groupshared int32_t gs_FFX_PARALLELSORT_LDS[FFX_PARALLELSORT_ELEMENTS_PER_THREAD][FFX_PARALLELSORT_THREADGROUP_SIZE];
[numthreads(FFX_PARALLELSORT_THREADGROUP_SIZE, 1, 1)] [shader("compute")]
void __compute__Scan(uint32_t localID : SV_GroupThreadID, uint32_t groupID : SV_GroupID)
{
	uint32_t BaseIndex = FFX_PARALLELSORT_ELEMENTS_PER_THREAD * FFX_PARALLELSORT_THREADGROUP_SIZE * groupID;
	// FFX_ParallelSort_ScanPrefix(rootConstData.NumScanValues, localID, groupID, 0, BaseIndex, false,
	// 							CBuffer, ScanSrc, ScanDst, ScanScratch);

	uint32_t numValuesToScan = rootConstData.NumScanValues;
	uint32_t BinOffset = 0;
	bool AddPartialSums = false;

	uint32_t i;
	// Perform coalesced loads into LDS
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
	{
		uint32_t DataIndex = BaseIndex + (i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID;

		uint32_t col = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) / FFX_PARALLELSORT_ELEMENTS_PER_THREAD;
		uint32_t row = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) % FFX_PARALLELSORT_ELEMENTS_PER_THREAD;
		gs_FFX_PARALLELSORT_LDS[row][col] = (DataIndex < numValuesToScan) ? ScanSrc[BinOffset + DataIndex] : 0;
	}

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	uint32_t threadgroupSum = 0;
	// Calculate the local scan-prefix for current thread
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
	{
		uint32_t tmp = gs_FFX_PARALLELSORT_LDS[i][localID];
		gs_FFX_PARALLELSORT_LDS[i][localID] = threadgroupSum;
		threadgroupSum += tmp;
	}

	// Scan prefix partial sums
	threadgroupSum = FFX_ParallelSort_BlockScanPrefix(threadgroupSum, localID);

	// Add reduced partial sums if requested
	uint32_t partialSum = 0;
	if (AddPartialSums)
	{
		// Partial sum additions are a little special as they are tailored to the optimal number of 
		// thread groups we ran in the beginning, so need to take that into account
		partialSum = ScanScratch[groupID];
	}

	// Add the block scanned-prefixes back in
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
		gs_FFX_PARALLELSORT_LDS[i][localID] += threadgroupSum;

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// Perform coalesced writes to scan dst
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
	{
		uint32_t DataIndex = BaseIndex + (i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID;

		uint32_t col = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) / FFX_PARALLELSORT_ELEMENTS_PER_THREAD;
		uint32_t row = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) % FFX_PARALLELSORT_ELEMENTS_PER_THREAD;

		if (DataIndex < numValuesToScan)
			ScanDst[BinOffset + DataIndex] = gs_FFX_PARALLELSORT_LDS[row][col] + partialSum;
	}
}

// // FPS ScanAdd
[numthreads(FFX_PARALLELSORT_THREADGROUP_SIZE, 1, 1)] [shader("compute")]
void __compute__ScanAdd(uint32_t localID : SV_GroupThreadID, uint32_t groupID : SV_GroupID)
{
	// When doing adds, we need to access data differently because reduce 
	// has a more specialized access pattern to match optimized count
	// Access needs to be done similarly to reduce
	// Figure out what bin data we are reducing
	uint32_t BinID = groupID / rootConstData.NumReduceThreadgroupPerBin;
	uint32_t BinOffset = BinID * rootConstData.NumThreadGroups;

	// Get the base index for this thread group
	uint32_t BaseIndex = (groupID % rootConstData.NumReduceThreadgroupPerBin) * FFX_PARALLELSORT_ELEMENTS_PER_THREAD * FFX_PARALLELSORT_THREADGROUP_SIZE;

	// FFX_ParallelSort_ScanPrefix(rootConstData.NumThreadGroups, localID, groupID, BinOffset, BaseIndex, true,
	// 							CBuffer, ScanSrc, ScanDst, ScanScratch);
	uint32_t numValuesToScan = rootConstData.NumThreadGroups;
	bool AddPartialSums = true;

	uint32_t i;
	// Perform coalesced loads into LDS
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
	{
		uint32_t DataIndex = BaseIndex + (i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID;

		uint32_t col = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) / FFX_PARALLELSORT_ELEMENTS_PER_THREAD;
		uint32_t row = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) % FFX_PARALLELSORT_ELEMENTS_PER_THREAD;
		gs_FFX_PARALLELSORT_LDS[row][col] = (DataIndex < numValuesToScan) ? ScanSrc[BinOffset + DataIndex] : 0;
	}

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	uint32_t threadgroupSum = 0;
	// Calculate the local scan-prefix for current thread
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
	{
		uint32_t tmp = gs_FFX_PARALLELSORT_LDS[i][localID];
		gs_FFX_PARALLELSORT_LDS[i][localID] = threadgroupSum;
		threadgroupSum += tmp;
	}

	// Scan prefix partial sums
	threadgroupSum = FFX_ParallelSort_BlockScanPrefix(threadgroupSum, localID);

	// Add reduced partial sums if requested
	uint32_t partialSum = 0;
	if (AddPartialSums)
	{
		// Partial sum additions are a little special as they are tailored to the optimal number of 
		// thread groups we ran in the beginning, so need to take that into account
		partialSum = ScanScratch[groupID];
	}

	// Add the block scanned-prefixes back in
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
		gs_FFX_PARALLELSORT_LDS[i][localID] += threadgroupSum;

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// Perform coalesced writes to scan dst
	for (i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
	{
		uint32_t DataIndex = BaseIndex + (i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID;

		uint32_t col = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) / FFX_PARALLELSORT_ELEMENTS_PER_THREAD;
		uint32_t row = ((i * FFX_PARALLELSORT_THREADGROUP_SIZE) + localID) % FFX_PARALLELSORT_ELEMENTS_PER_THREAD;

		if (DataIndex < numValuesToScan)
			ScanDst[BinOffset + DataIndex] = gs_FFX_PARALLELSORT_LDS[row][col] + partialSum;
	}
}

// FPS Scatter
// Offset cache to avoid loading the offsets all the time
groupshared uint32_t gs_FFX_PARALLELSORT_BinOffsetCache[FFX_PARALLELSORT_THREADGROUP_SIZE];
// Local histogram for offset calculations
groupshared uint32_t gs_FFX_PARALLELSORT_LocalHistogram[FFX_PARALLELSORT_SORT_BIN_COUNT];
// Scratch area for algorithm
groupshared uint32_t gs_FFX_PARALLELSORT_LDSScratch[FFX_PARALLELSORT_THREADGROUP_SIZE];
[numthreads(FFX_PARALLELSORT_THREADGROUP_SIZE, 1, 1)] [shader("compute")]
void __compute__Scatter(uint32_t localID : SV_GroupThreadID, uint32_t groupID : SV_GroupID)
{
	uint32_t ShiftBit = rootConstData.CShiftBit;

	// Load the sort bin threadgroup offsets into LDS for faster referencing
	if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
		gs_FFX_PARALLELSORT_BinOffsetCache[localID] = SumTable[localID * rootConstData.NumThreadGroups + groupID];

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// Data is processed in blocks, and how many we process can changed based on how much data we are processing
	// versus how many thread groups we are processing with
	int32_t BlockSize = FFX_PARALLELSORT_ELEMENTS_PER_THREAD * FFX_PARALLELSORT_THREADGROUP_SIZE;

	// Figure out this thread group's index into the block data (taking into account thread groups that need to do extra reads)
	uint32_t ThreadgroupBlockStart = (BlockSize * rootConstData.NumBlocksPerThreadGroup * groupID);
	uint32_t NumBlocksToProcess = rootConstData.NumBlocksPerThreadGroup;

	if (groupID >= rootConstData.NumThreadGroups - rootConstData.NumThreadGroupsWithAdditionalBlocks)
	{
		ThreadgroupBlockStart += (groupID - (rootConstData.NumThreadGroups - rootConstData.NumThreadGroupsWithAdditionalBlocks)) * BlockSize;
		NumBlocksToProcess++;
	}

	// Get the block start index for this thread
	uint32_t BlockIndex = ThreadgroupBlockStart + localID;

	// Count value occurences
	uint32_t newCount;
	for (int32_t BlockCount = 0; BlockCount < NumBlocksToProcess; BlockCount++, BlockIndex += BlockSize)
	{
		uint32_t DataIndex = BlockIndex;
		
		// Pre-load the key values in order to hide some of the read latency
		uint32_t srcKeys[FFX_PARALLELSORT_ELEMENTS_PER_THREAD];
		srcKeys[0] = SrcBuffer[DataIndex];
		srcKeys[1] = SrcBuffer[DataIndex + FFX_PARALLELSORT_THREADGROUP_SIZE];
		srcKeys[2] = SrcBuffer[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 2)];
		srcKeys[3] = SrcBuffer[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 3)];

		for (int32_t i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
		{
			// Clear the local histogram
			if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
				gs_FFX_PARALLELSORT_LocalHistogram[localID] = 0;

			uint32_t localKey = (DataIndex < rootConstData.NumKeys ? srcKeys[i] : 0xffffffff);

			// Sort the keys locally in LDS
			for (uint32_t bitShift = 0; bitShift < FFX_PARALLELSORT_SORT_BITS_PER_PASS; bitShift += 2)
			{
				// Figure out the keyIndex
				uint32_t keyIndex = (localKey >> ShiftBit) & 0xf;
				uint32_t bitKey = (keyIndex >> bitShift) & 0x3;

				// Create a packed histogram 
				uint32_t packedHistogram = 1U << (bitKey * 8);

				// Sum up all the packed keys (generates counted offsets up to current thread group)
				uint32_t localSum = FFX_ParallelSort_BlockScanPrefix(packedHistogram, localID);

				// Last thread stores the updated histogram counts for the thread group
				// Scratch = 0xsum3|sum2|sum1|sum0 for thread group
				if (localID == (FFX_PARALLELSORT_THREADGROUP_SIZE - 1))
					gs_FFX_PARALLELSORT_LDSScratch[0] = localSum + packedHistogram;

				// Wait for everyone to catch up
				GroupMemoryBarrierWithGroupSync();

				// Load the sums value for the thread group
				packedHistogram = gs_FFX_PARALLELSORT_LDSScratch[0];

				// Add prefix offsets for all 4 bit "keys" (packedHistogram = 0xsum2_1_0|sum1_0|sum0|0)
				packedHistogram = (packedHistogram << 8) + (packedHistogram << 16) + (packedHistogram << 24);

				// Calculate the proper offset for this thread's value
				localSum += packedHistogram;

				// Calculate target offset
				uint32_t keyOffset = (localSum >> (bitKey * 8)) & 0xff;

				// Re-arrange the keys (store, sync, load)
				gs_FFX_PARALLELSORT_LDSSums[keyOffset] = localKey;
				GroupMemoryBarrierWithGroupSync();
				localKey = gs_FFX_PARALLELSORT_LDSSums[localID];

				// Wait for everyone to catch up
				GroupMemoryBarrierWithGroupSync();

			}

			// Need to recalculate the keyIndex on this thread now that values have been copied around the thread group
			uint32_t keyIndex = (localKey >> ShiftBit) & 0xf;

			// Reconstruct histogram
			InterlockedAdd(gs_FFX_PARALLELSORT_LocalHistogram[keyIndex], 1);

			// Wait for everyone to catch up
			GroupMemoryBarrierWithGroupSync();

			// Prefix histogram
			uint32_t histogramPrefixSum = WavePrefixSum(localID < FFX_PARALLELSORT_SORT_BIN_COUNT ? gs_FFX_PARALLELSORT_LocalHistogram[localID] : 0);

			// Broadcast prefix-sum via LDS
			if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
				gs_FFX_PARALLELSORT_LDSScratch[localID] = histogramPrefixSum;

			// Get the global offset for this key out of the cache
			uint32_t globalOffset = gs_FFX_PARALLELSORT_BinOffsetCache[keyIndex];

			// Wait for everyone to catch up
			GroupMemoryBarrierWithGroupSync();

			// Get the local offset (at this point32_t the keys are all in increasing order from 0 -> num bins in localID 0 -> thread group size)
			uint32_t localOffset = localID - gs_FFX_PARALLELSORT_LDSScratch[keyIndex];

			// Write to destination
			uint32_t totalOffset = globalOffset + localOffset;

			if (totalOffset < rootConstData.NumKeys)
			{
				DstBuffer[totalOffset] = localKey;
			}

			// Wait for everyone to catch up
			GroupMemoryBarrierWithGroupSync();

			// Update the cached histogram for the next set of entries
			if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
				gs_FFX_PARALLELSORT_BinOffsetCache[localID] += gs_FFX_PARALLELSORT_LocalHistogram[localID];

			DataIndex += FFX_PARALLELSORT_THREADGROUP_SIZE;	// Increase the data offset by thread group size
		}
	}
}

void __compute__ScatterPayload(uint32_t localID : SV_GroupThreadID, uint32_t groupID : SV_GroupID)
{
	uint32_t ShiftBit = rootConstData.CShiftBit;

	// Load the sort bin threadgroup offsets into LDS for faster referencing
	if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
		gs_FFX_PARALLELSORT_BinOffsetCache[localID] = SumTable[localID * rootConstData.NumThreadGroups + groupID];

	// Wait for everyone to catch up
	GroupMemoryBarrierWithGroupSync();

	// Data is processed in blocks, and how many we process can changed based on how much data we are processing
	// versus how many thread groups we are processing with
	int32_t BlockSize = FFX_PARALLELSORT_ELEMENTS_PER_THREAD * FFX_PARALLELSORT_THREADGROUP_SIZE;

	// Figure out this thread group's index into the block data (taking into account thread groups that need to do extra reads)
	uint32_t ThreadgroupBlockStart = (BlockSize * rootConstData.NumBlocksPerThreadGroup * groupID);
	uint32_t NumBlocksToProcess = rootConstData.NumBlocksPerThreadGroup;

	if (groupID >= rootConstData.NumThreadGroups - rootConstData.NumThreadGroupsWithAdditionalBlocks)
	{
		ThreadgroupBlockStart += (groupID - (rootConstData.NumThreadGroups - rootConstData.NumThreadGroupsWithAdditionalBlocks)) * BlockSize;
		NumBlocksToProcess++;
	}

	// Get the block start index for this thread
	uint32_t BlockIndex = ThreadgroupBlockStart + localID;

	// Count value occurences
	uint32_t newCount;
	for (int32_t BlockCount = 0; BlockCount < NumBlocksToProcess; BlockCount++, BlockIndex += BlockSize)
	{
		uint32_t DataIndex = BlockIndex;
		
		// Pre-load the key values in order to hide some of the read latency
		uint32_t srcKeys[FFX_PARALLELSORT_ELEMENTS_PER_THREAD];
		srcKeys[0] = SrcBuffer[DataIndex];
		srcKeys[1] = SrcBuffer[DataIndex + FFX_PARALLELSORT_THREADGROUP_SIZE];
		srcKeys[2] = SrcBuffer[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 2)];
		srcKeys[3] = SrcBuffer[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 3)];

		// Also preload payload
		uint32_t srcValues[FFX_PARALLELSORT_ELEMENTS_PER_THREAD];
		srcValues[0] = SrcPayload[DataIndex];
		srcValues[1] = SrcPayload[DataIndex + FFX_PARALLELSORT_THREADGROUP_SIZE];
		srcValues[2] = SrcPayload[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 2)];
		srcValues[3] = SrcPayload[DataIndex + (FFX_PARALLELSORT_THREADGROUP_SIZE * 3)];

		for (int32_t i = 0; i < FFX_PARALLELSORT_ELEMENTS_PER_THREAD; i++)
		{
			// Clear the local histogram
			if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
				gs_FFX_PARALLELSORT_LocalHistogram[localID] = 0;

			uint32_t localKey = (DataIndex < rootConstData.NumKeys ? srcKeys[i] : 0xffffffff);
			uint32_t localValue = (DataIndex < rootConstData.NumKeys ? srcValues[i] : 0);

			// Sort the keys locally in LDS
			for (uint32_t bitShift = 0; bitShift < FFX_PARALLELSORT_SORT_BITS_PER_PASS; bitShift += 2)
			{
				// Figure out the keyIndex
				uint32_t keyIndex = (localKey >> ShiftBit) & 0xf;
				uint32_t bitKey = (keyIndex >> bitShift) & 0x3;

				// Create a packed histogram 
				uint32_t packedHistogram = 1U << (bitKey * 8);

				// Sum up all the packed keys (generates counted offsets up to current thread group)
				uint32_t localSum = FFX_ParallelSort_BlockScanPrefix(packedHistogram, localID);

				// Last thread stores the updated histogram counts for the thread group
				// Scratch = 0xsum3|sum2|sum1|sum0 for thread group
				if (localID == (FFX_PARALLELSORT_THREADGROUP_SIZE - 1))
					gs_FFX_PARALLELSORT_LDSScratch[0] = localSum + packedHistogram;

				// Wait for everyone to catch up
				GroupMemoryBarrierWithGroupSync();

				// Load the sums value for the thread group
				packedHistogram = gs_FFX_PARALLELSORT_LDSScratch[0];

				// Add prefix offsets for all 4 bit "keys" (packedHistogram = 0xsum2_1_0|sum1_0|sum0|0)
				packedHistogram = (packedHistogram << 8) + (packedHistogram << 16) + (packedHistogram << 24);

				// Calculate the proper offset for this thread's value
				localSum += packedHistogram;

				// Calculate target offset
				uint32_t keyOffset = (localSum >> (bitKey * 8)) & 0xff;

				// Re-arrange the keys (store, sync, load)
				gs_FFX_PARALLELSORT_LDSSums[keyOffset] = localKey;
				GroupMemoryBarrierWithGroupSync();
				localKey = gs_FFX_PARALLELSORT_LDSSums[localID];

				// Wait for everyone to catch up
				GroupMemoryBarrierWithGroupSync();

				// Re-arrange the values if we have them (store, sync, load)
				gs_FFX_PARALLELSORT_LDSSums[keyOffset] = localValue;
				GroupMemoryBarrierWithGroupSync();
				localValue = gs_FFX_PARALLELSORT_LDSSums[localID];

				// Wait for everyone to catch up
				GroupMemoryBarrierWithGroupSync();
			}

			// Need to recalculate the keyIndex on this thread now that values have been copied around the thread group
			uint32_t keyIndex = (localKey >> ShiftBit) & 0xf;

			// Reconstruct histogram
			InterlockedAdd(gs_FFX_PARALLELSORT_LocalHistogram[keyIndex], 1);

			// Wait for everyone to catch up
			GroupMemoryBarrierWithGroupSync();

			// Prefix histogram
			uint32_t histogramPrefixSum = WavePrefixSum(localID < FFX_PARALLELSORT_SORT_BIN_COUNT ? gs_FFX_PARALLELSORT_LocalHistogram[localID] : 0);

			// Broadcast prefix-sum via LDS
			if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
				gs_FFX_PARALLELSORT_LDSScratch[localID] = histogramPrefixSum;

			// Get the global offset for this key out of the cache
			uint32_t globalOffset = gs_FFX_PARALLELSORT_BinOffsetCache[keyIndex];

			// Wait for everyone to catch up
			GroupMemoryBarrierWithGroupSync();

			// Get the local offset (at this point32_t the keys are all in increasing order from 0 -> num bins in localID 0 -> thread group size)
			uint32_t localOffset = localID - gs_FFX_PARALLELSORT_LDSScratch[keyIndex];

			// Write to destination
			uint32_t totalOffset = globalOffset + localOffset;

			if (totalOffset < rootConstData.NumKeys)
			{
				DstBuffer[totalOffset] = localKey;
				DstPayload[totalOffset] = localValue;
			}

			// Wait for everyone to catch up
			GroupMemoryBarrierWithGroupSync();

			// Update the cached histogram for the next set of entries
			if (localID < FFX_PARALLELSORT_SORT_BIN_COUNT)
				gs_FFX_PARALLELSORT_BinOffsetCache[localID] += gs_FFX_PARALLELSORT_LocalHistogram[localID];

			DataIndex += FFX_PARALLELSORT_THREADGROUP_SIZE;	// Increase the data offset by thread group size
		}
	}
}
