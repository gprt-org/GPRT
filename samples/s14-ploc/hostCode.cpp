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

// public GPRT API
#include <gprt.h>

// our shared data structures between host and device
#include "sharedCode.h"

// for generating meshes
#include <generator.hpp>
using namespace generator;

// A class we'll use to quickly generate meshes and bottom level trees
template <typename T> struct Mesh {
    std::vector<float3> vertices;
    std::vector<uint3> indices;
    GPRTBufferOf<float3> vertexBuffer;
    GPRTBufferOf<uint3> indexBuffer;

    float3 bbMin;
    float3 bbMax;
    
    Mesh(){};
    Mesh(GPRTContext context, T generator) {
        bbMin = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        bbMax = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        // Use the generator to generate vertices and indices
        auto vertGenerator = generator.vertices();
        auto triGenerator = generator.triangles();
        while (!vertGenerator.done()) {
            auto vertex = vertGenerator.generate();
            auto position = vertex.position;
            vertices.push_back(float3(position[0], position[1], position[2]));
            bbMin = min(bbMin, vertices.back());
            bbMax = max(bbMax, vertices.back());
            vertGenerator.next();
        }
        while (!triGenerator.done()) {
            Triangle triangle = triGenerator.generate();
            auto vertices = triangle.vertices;
            indices.push_back(uint3(vertices[0], vertices[1], vertices[2]));
            triGenerator.next();
        }

        // Upload those to the device, create the geometry
        vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(), vertices.data());
        indexBuffer = gprtDeviceBufferCreate<uint3>(context, indices.size(), indices.data());
    };

    void cleanupMesh() {
        gprtBufferDestroy(vertexBuffer);
        gprtBufferDestroy(indexBuffer);
    };
};

#define LOG(message)                                                                                                   \
  std::cout << GPRT_TERMINAL_BLUE;                                                                                     \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                                                                \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                                                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s14_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s14-ploc.png";

size_t divUp(size_t value, size_t multiple) {
  return ((value + multiple - 1) / multiple);
}


#include <functional>
#include <memory>
#include <numeric>


namespace bvh {

inline constexpr size_t get_thread_count() { return 1; }
inline constexpr size_t get_thread_id()    { return 0; }
inline void assert_not_in_parallel() {}
inline void assert_in_parallel() {}
} // namespace bvh

/// Threshold (number of nodes) under which the loops execute serially.
size_t loop_parallel_threshold = 256;
int search_radius = 14;

/// Parallel prefix sum. The parallel algorithm used in this implementation
/// needs twice the work as the naive serial version, and is thus enabled
/// only if the number of cores if greater or equal than 3.
template <typename T>
class PrefixSum {
public:
    /// Performs the prefix sum. Must be called from a parallel region.
    template <typename F = std::plus<T>>
    void sum_in_parallel(const T* input, T* output, size_t count, F f = F()) {
        bvh::assert_in_parallel();

        size_t thread_count = bvh::get_thread_count();
        size_t thread_id    = bvh::get_thread_id();

        // This algorithm is not effective when there are fewer than 2 threads.
        if (thread_count <= 2) {
            #pragma omp single
            { std::partial_sum(input, input + count, output, f); }
            return;
        }

        // Allocate temporary storage
        #pragma omp single
        {
            if (per_thread_data_size < thread_count + 1) {
                per_thread_sums = std::make_unique<T[]>(thread_count + 1);
                per_thread_data_size = thread_count + 1;
                per_thread_sums[0] = 0;
            }
        }

        T sum = T(0);

        // Compute partial sums
        #pragma omp for nowait schedule(static)
        for (size_t i = 0; i < count; ++i) {
            sum = f(sum, input[i]);
            output[i] = sum;
        }
        per_thread_sums[thread_id + 1] = sum;

        #pragma omp barrier

        // Fix the sums
        auto offset = std::accumulate(per_thread_sums.get(), per_thread_sums.get() + thread_id + 1, T(0), f);

        #pragma omp for schedule(static)
        for (size_t i = 0; i < count; ++i)
            output[i] = f(output[i], offset);
    }

private:
    std::unique_ptr<T[]> per_thread_sums;
    size_t per_thread_data_size = 0;
};

PrefixSum<uint32_t> prefix_sum;

std::pair<size_t, size_t> search_range(size_t i, size_t begin, size_t end) {
    return std::make_pair(
        i > begin + search_radius ? i - search_radius : begin,
        std::min(i + search_radius + 1, end));
}

GPRTComputeOf<PLOCCreateLeavesArgs> PLOCCreateLeaves;
GPRTComputeOf<PLOCComputeCodesArgs> PLOCComputeCodes;
GPRTComputeOf<PLOCMergeNodesArgs> PLOCMergeNodes;
GPRTComputeOf<PLOCCopyNodesArgs> PLOCCopyNodes;

std::pair<uint32_t, uint32_t> cluster(
        GPRTContext context,
        GPRTBufferOf<Node> inputBuffer,
        GPRTBufferOf<Node> outputBuffer,
        GPRTBufferOf<uint32_t> neighborsBuffer, 
        GPRTBufferOf<uint32_t> mergedIndexBuffer,
        uint32_t begin, uint32_t end,
        uint32_t previous_end)
{
    uint32_t next_begin = 0;
    uint32_t next_end   = 0;

    // Get their pointers
    Node* input = gprtBufferGetPointer(inputBuffer);
    Node* output = gprtBufferGetPointer(outputBuffer);
    uint32_t* neighbors = gprtBufferGetPointer(neighborsBuffer);
    uint32_t* merged_index = gprtBufferGetPointer(mergedIndexBuffer);

    auto thread_count = bvh::get_thread_count();
    auto thread_id    = bvh::get_thread_id();
    auto chunk_size   = (end - begin) / thread_count;
    auto chunk_begin  = begin + thread_id * chunk_size;
    auto chunk_end    = thread_id != thread_count - 1 ? chunk_begin + chunk_size : end;

    auto distances = std::make_unique<float[]>((search_radius + 1) * search_radius);
    auto distance_matrix = std::make_unique<float*[]>(search_radius + 1); // 15 x 210
    for (size_t i = 0; i <= search_radius; ++i)
        distance_matrix[i] = &distances[i * search_radius];

    // Initialize the distance matrix, which caches the distances between
    // neighboring nodes in the array. A brute force approach that recomputes the
    // distances for every neighbor can be implemented without a distance matrix,
    // but would be slower for larger search radii.
    for (size_t i = search_range(chunk_begin, begin, end).first; i < chunk_begin; ++i) {
        auto search_end = search_range(i, begin, end).second;
        for (size_t j = i + 1; j < search_end; ++j) {
            distance_matrix[chunk_begin - i][j - i - 1] = AABB::HalfArea(AABB::GetUnion(input[i].aabb, input[j].aabb));
        }
    }

    // Nearest neighbor search
    for (size_t i = chunk_begin; i < chunk_end; i++) {
        auto [search_begin, search_end] = search_range(i, begin, end);
        float best_distance = std::numeric_limits<float>::max();
        size_t best_neighbor = -1;

        // Backward search (using the previously-computed distances stored in the distance matrix)
        for (size_t j = search_begin; j < i; ++j) {
            auto distance = distance_matrix[i - j][i - j - 1];
            if (distance < best_distance) {
                best_distance = distance;
                best_neighbor = j;
            }
        }

        // Forward search (caching computed distances in the distance matrix)
        for (size_t j = i + 1; j < search_end; ++j) {
            auto distance = AABB::HalfArea(AABB::GetUnion(input[i].aabb, input[j].aabb));
            distance_matrix[0][j - i - 1] = distance;
            if (distance < best_distance) {
                best_distance = distance;
                best_neighbor = j;
            }
        }

        assert(best_neighbor != size_t(-1));
        neighbors[i] = best_neighbor;

        // Rotate the distance matrix columns
        auto last = distance_matrix[search_radius];
        std::move_backward(
            distance_matrix.get(),
            distance_matrix.get() + search_radius,
            distance_matrix.get() + search_radius + 1);
        distance_matrix[0] = last;
    }

    // #pragma omp barrier

    // Mark nodes that are the closest as merged, but keep
    // the one with lowest index to act as the parent

    // parallelize
    for (size_t i = begin; i < end; ++i) {
        auto j = neighbors[i];
        bool is_mergeable = neighbors[j] == i;
        merged_index[i] = i < j && is_mergeable ? 1 : 0;
    }

    // Perform a prefix sum to compute the insertion indices
    prefix_sum.sum_in_parallel(merged_index + begin, merged_index + begin, end - begin);   
    uint32_t merged_count   = merged_index[end - 1];
    uint32_t unmerged_count = end - begin - merged_count;
    uint32_t children_count = merged_count * 2;
    uint32_t children_begin = end - children_count;
    uint32_t unmerged_begin = end - (children_count + unmerged_count);

    // Execute below with just one thread
    {
        next_begin = unmerged_begin;
        next_end   = children_begin;
    }

    gprtBufferExclusiveSum(context, mergedIndexBuffer, )

    // Finally, merge nodes that are marked for merging and create
    // their parents using the indices computed previously.
    
    // parallelize
    // for (size_t dispatchIndex = 0; dispatchIndex < end - begin; ++dispatchIndex) {
    //     uint32_t original_i = begin + dispatchIndex;
    //     uint32_t j = neighbors[original_i];
    //     uint32_t i = neighbors[j];
    //     if (i == original_i) {
    //         if (original_i < j) {
    //             uint32_t merged_index_i = merged_index[i];
    //             uint32_t merged_index_j = merged_index[j];
    //             Node unmerged_node;
    //             Node input_i = input[i];
    //             Node input_j = input[j];
    //             uint32_t first_child = children_begin + (merged_index_i - 1) * 2;
    //             unmerged_node.aabb = AABB::GetUnion(input_j.aabb, input_i.aabb);
    //             unmerged_node.primitive_count = 0;
    //             unmerged_node.first_child_or_primitive = first_child;
    //             output[unmerged_begin + j - begin - merged_index_j] = unmerged_node;
    //             output[first_child + 0] = input_i;
    //             output[first_child + 1] = input_j;
    //         }
    //     } else {
    //         uint32_t merged_index_original_i = merged_index[original_i];
    //         Node input_original_i = input[original_i];
    //         output[unmerged_begin + original_i - begin - merged_index_original_i] = input_original_i;
    //     }
    // }
    gprtComputeLaunch(PLOCMergeNodes, {divUp(end - begin, 32), 1, 1}, {32, 1, 1}, {
        gprtBufferGetHandle(inputBuffer),
        gprtBufferGetHandle(outputBuffer),
        gprtBufferGetHandle(neighborsBuffer),
        gprtBufferGetHandle(mergedIndexBuffer),
        children_begin,
        unmerged_begin,
        begin,
        end
    });

    // // Copy the nodes of the previous level into the current array of nodes.
    // // parallelize    
    // for (size_t i = 0; i < previous_end - end; ++i)
    //     output[i + end] = input[i + end];
    gprtComputeLaunch(PLOCCopyNodes, {divUp(previous_end - end, 32), 1, 1}, {32, 1, 1}, {
        gprtBufferGetHandle(inputBuffer),
        gprtBufferGetHandle(outputBuffer),
        uint32_t(end),
        uint32_t(previous_end)
    });


    return std::make_pair(next_begin, next_end);
}

#include <iostream>
int
main(int ac, char **av) {
    gprtRequestWindow(fbSize.x, fbSize.y, "Int14 PLOC");
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, s14_deviceCode);

    // A test mesh to work with.
    Mesh<TeapotMesh> mesh(context, TeapotMesh{12});
    uint32_t numPrims = mesh.indices.size();
    uint32_t numNodes = 2 * numPrims - 1;

    auto codesBuffer = gprtHostBufferCreate<uint64_t>(context, numPrims);
    auto primIDsBuffer = gprtHostBufferCreate<uint64_t>(context, numPrims);
    auto scratch = gprtHostBufferCreate<uint8_t>(context);
    auto nodes = gprtHostBufferCreate<Node>(context, numNodes);
    auto nodesCopy = gprtHostBufferCreate<Node>(context, numNodes);
    auto neighbors = gprtHostBufferCreate<uint32_t>(context, numNodes);
    auto mergedIndex = gprtHostBufferCreate<uint32_t>(context, numNodes * 2);

    PLOCComputeCodes = gprtComputeCreate<PLOCComputeCodesArgs>(context, module, "PLOCComputeCodes");
    PLOCCreateLeaves = gprtComputeCreate<PLOCCreateLeavesArgs>(context, module, "PLOCCreateLeaves");
    PLOCMergeNodes = gprtComputeCreate<PLOCMergeNodesArgs>(context, module, "PLOCMergeNodes");
    PLOCCopyNodes = gprtComputeCreate<PLOCCopyNodesArgs>(context, module, "PLOCCopyNodes");

    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");
    GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
    RayGenData *data = gprtRayGenGetParameters(rayGen);
    data->color0 = float3(0.1f, 0.1f, 0.1f);
    data->color1 = float3(0.0f, 0.0f, 0.0f);
    data->frameBuffer = gprtBufferGetHandle(frameBuffer);

    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    // Generate SFC codes for each primitive, then sort primitive IDs by their codes
    gprtComputeLaunch(PLOCComputeCodes, {divUp(numPrims, 32), 1, 1}, {32, 1, 1}, {
        gprtBufferGetHandle(mesh.vertexBuffer), 
        gprtBufferGetHandle(mesh.indexBuffer), 
        numPrims,
        AABB{mesh.bbMin, mesh.bbMax},
        gprtBufferGetHandle(primIDsBuffer),
        gprtBufferGetHandle(codesBuffer)
    });
    gprtBufferSortPayload(context, codesBuffer, primIDsBuffer, scratch);

    size_t begin = numNodes - numPrims;
    size_t end = numNodes;
    size_t previousEnd = end;

    // Create the leaves
    gprtComputeLaunch(PLOCCreateLeaves, {divUp(numPrims, 32), 1, 1}, {32, 1, 1}, {
        gprtBufferGetHandle(mesh.vertexBuffer), 
        gprtBufferGetHandle(mesh.indexBuffer), 
        numPrims,
        gprtBufferGetHandle(primIDsBuffer),
        gprtBufferGetHandle(codesBuffer),
        gprtBufferGetHandle(nodes)
    });

    uint32_t numIterations = 0;
    while (end - begin > 1) {
        numIterations++;
        std::cout<<"begin: "<<begin<<" end: "<<end<<std::endl;

        // Cluster (for now, implementing in series)
        auto [nextBegin, nextEnd] = cluster(
            context,
            nodes,         // input
            nodesCopy,     // output
            neighbors,
            mergedIndex, 
            begin, end, 
            previousEnd
        );

        // Swap the nodes and nodesCopy buffers
        auto tmp = nodes;
        nodes = nodesCopy;
        nodesCopy = tmp;

        previousEnd = end;
        begin        = nextBegin;
        end          = nextEnd;
    }

    assert(numIterations == 43);

    // Print out the nodes
    gprtBufferMap(nodes);
    Node* nodesPtr = gprtBufferGetPointer(nodes);
    for (uint32_t i = 0; i < numNodes; ++i) {
        std::cout << "node: " << i << " primitive_count: " << nodesPtr[i].primitive_count << " first_child_or_primitive: " << nodesPtr[i].first_child_or_primitive << std::endl;
    }
    gprtBufferUnmap(nodes);
    std::cout<<"Done"<<std::endl;


    // Print out the codes
    // gprtBufferMap(codesBuffer);
    // gprtBufferMap(primIDsBuffer);
    
    // uint64_t *codes = gprtBufferGetPointer(codesBuffer);
    // uint64_t *primIDs = gprtBufferGetPointer(primIDsBuffer);

    // for (uint32_t i = 0; i < numPrims; ++i) {
    //     std::cout << "code: " << codes[i] << " primID: " << primIDs[i] << std::endl;
    //     if (i > 0 && codes[i] < codes[i - 1]) {
    //         std::cout << "ERROR: codes are not sorted!" << std::endl;
    //     }
    // }

    // gprtBufferUnmap(codesBuffer);
    // gprtBufferUnmap(primIDsBuffer);
    // std::cout<<"Done"<<std::endl;


    



// The nodes of our PLOC tree
    // 

    // The leaves of the tree are placed at the end of the array of nodes, from [end - numPrims, end)]
    // gprtBufferMap(nodes);
    // Node* nodes = gprtBufferGetPointer(nodes);
    // for (uint32_t i = 0; i < numPrims; ++i) {
    //     nodes[i].primitive_count = 1;
    //     nodes[i].first_child_or_primitive = i;
    //     nodes[i].aabb = mesh.aabbs[];
    // }
    // gprtBufferUnmap(nodes);





    gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

    LOG("executing the launch ...");
    do {
        gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, *data);
        gprtBufferPresent(context, frameBuffer);
    }
    while (!gprtWindowShouldClose(context));

    // Save final frame to an image
    gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);
    
    mesh.cleanupMesh();
    gprtBufferDestroy(frameBuffer);
    gprtRayGenDestroy(rayGen);
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
}
