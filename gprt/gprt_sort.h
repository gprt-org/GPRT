// Copyright (c) 2020 Advanced Micro Devices, Inc. All rights reserved.
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
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#define PARALLELSORT_SORT_BITS_PER_PASS		4
#define	PARALLELSORT_SORT_BIN_COUNT			(1 << PARALLELSORT_SORT_BITS_PER_PASS)
#define PARALLELSORT_ELEMENTS_PER_THREAD	4
#define PARALLELSORT_THREADGROUP_SIZE		128

//////////////////////////////////////////////////////////////////////////
// ParallelSort constant buffer parameters:
//
//	NumKeys								The number of keys to sort
//	Shift								How many bits to shift for this sort pass (we sort 4 bits at a time)
//	NumBlocksPerThreadGroup				How many blocks of keys each thread group needs to process
//	NumThreadGroups						How many thread groups are being run concurrently for sort
//	NumThreadGroupsWithAdditionalBlocks	How many thread groups need to process additional block data
//	NumReduceThreadgroupPerBin			How many thread groups are summed together for each reduced bin entry
//	NumScanValues						How many values to perform scan prefix (+ add) on
//////////////////////////////////////////////////////////////////////////

struct ParallelSortCB
{
    uint32_t NumKeys;
    uint32_t  NumBlocksPerThreadGroup;
    uint32_t NumThreadGroups;
    uint32_t NumThreadGroupsWithAdditionalBlocks;
    uint32_t NumReduceThreadgroupPerBin;
    uint32_t NumScanValues;

    uint32_t CShiftBit;
};

#ifndef GPRT_DEVICE
	void ParallelSort_CalculateScratchResourceSize(uint32_t MaxNumKeys, uint64_t& ScratchBufferSize, uint64_t& ReduceScratchBufferSize)
	{
		uint64_t BlockSize = PARALLELSORT_ELEMENTS_PER_THREAD * PARALLELSORT_THREADGROUP_SIZE;
		uint64_t NumBlocks = (MaxNumKeys + BlockSize - 1) / BlockSize;
		uint64_t NumReducedBlocks = (NumBlocks + BlockSize - 1) / BlockSize;

		ScratchBufferSize = PARALLELSORT_SORT_BIN_COUNT * NumBlocks * sizeof(uint32_t);
		ReduceScratchBufferSize = PARALLELSORT_SORT_BIN_COUNT * NumReducedBlocks * sizeof(uint32_t);
	}

	void ParallelSort_SetConstantAndDispatchData(uint32_t NumKeys, uint32_t MaxThreadGroups, ParallelSortCB& ConstantBuffer, uint32_t& NumThreadGroupsToRun, uint32_t& NumReducedThreadGroupsToRun)
	{
		ConstantBuffer.NumKeys = NumKeys;

		uint32_t BlockSize = PARALLELSORT_ELEMENTS_PER_THREAD * PARALLELSORT_THREADGROUP_SIZE;
		uint32_t NumBlocks = (NumKeys + BlockSize - 1) / BlockSize;

		// Figure out data distribution
		NumThreadGroupsToRun = MaxThreadGroups;
		uint32_t BlocksPerThreadGroup = (NumBlocks / NumThreadGroupsToRun);
		ConstantBuffer.NumThreadGroupsWithAdditionalBlocks = NumBlocks % NumThreadGroupsToRun;

		if (NumBlocks < NumThreadGroupsToRun)
		{
			BlocksPerThreadGroup = 1;
			NumThreadGroupsToRun = NumBlocks;
			ConstantBuffer.NumThreadGroupsWithAdditionalBlocks = 0;
		}

		ConstantBuffer.NumThreadGroups = NumThreadGroupsToRun;
		ConstantBuffer.NumBlocksPerThreadGroup = BlocksPerThreadGroup;

		// Calculate the number of thread groups to run for reduction (each thread group can process BlockSize number of entries)
		NumReducedThreadGroupsToRun = PARALLELSORT_SORT_BIN_COUNT * ((BlockSize > NumThreadGroupsToRun) ? 1 : (NumThreadGroupsToRun + BlockSize - 1) / BlockSize);
		ConstantBuffer.NumReduceThreadgroupPerBin = NumReducedThreadGroupsToRun / PARALLELSORT_SORT_BIN_COUNT;
		ConstantBuffer.NumScanValues = NumReducedThreadGroupsToRun;	// The number of reduce thread groups becomes our scan count (as each thread group writes out 1 value that needs scan prefix)
	}

	// We are using some optimizations to hide buffer load latency, so make sure anyone changing this define is made aware of that fact.
	static_assert(PARALLELSORT_ELEMENTS_PER_THREAD == 4, "ParallelSort Shaders currently explicitly rely on PARALLELSORT_ELEMENTS_PER_THREAD being set to 4 in order to optimize buffer loads. Please adjust the optimization to factor in the new define value.");
#endif // __cplusplus

