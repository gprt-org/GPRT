#pragma once

#define NUM_CLUSTERS_PER_SUPERCLUSTER 8
#define NUM_PRIMS_PER_CLUSTER 8

// NOTE, struct must be synchronized with declaration in gprt_host.h
namespace gprt{

    #ifdef GPRT_DEVICE
    // ideally this buffer type would be a struct on the GPU...
    // but I'm running into a compiler bug reading one struct inside another one.
    // x stores pointer, y stores size.
    typedef uint64_t2 Buffer;
    typedef uint64_t2 Accel;
    typedef uint64_t2 Texture;
    typedef uint64_t2 Sampler;
    #else
    struct Buffer {
        uint64_t x;
        uint64_t y;
    };
    struct Accel {
        uint64_t x;
        uint64_t y;
    };
    struct Texture {
        uint64_t x;
        uint64_t y;
    };
    struct Sampler {
        uint64_t x;
        uint64_t y;
    };
    #endif

    struct NNAccel {
        // input
        alignas(4) uint32_t numPrims;
        alignas(4) uint32_t numClusters;
        alignas(4) uint32_t numSuperClusters;
        alignas(4) float maxSearchRange;

        alignas(16) gprt::Buffer points; 
        alignas(16) gprt::Buffer edges; 
        alignas(16) gprt::Buffer triangles;

        // Hilbert codes of quantized primitive centroids
        // One uint32_t per primitive
        alignas(16) gprt::Buffer codes;

        // Primitive IDs that correspond to sorted hilbert codes. 
        // One uint32_t per primitive
        alignas(16) gprt::Buffer ids;

        // Buffer containing the global AABB. Pair of two floats
        alignas(16) gprt::Buffer aabb;

        // Buffers of AABBs. Each aabb is a pair of float3.
        alignas(16) gprt::Buffer clusters;

        // Buffers of AABBs. Each AABB here contains clusters, but is also dialated by "maximum search range".
        alignas(16) gprt::Buffer superClusters;

        // An RT core tree
        alignas(16) gprt::Accel accel;

        // An LBVH tree
        alignas(16) gprt::Buffer lbvhMortonCodes;

        // Primitive IDs that correspond to sorted morton codes. 
        // One uint32_t per primitive
        alignas(16) gprt::Buffer lbvhIds;

        // numPrims-1 + numPrims long. 
        // The "numPrims-1" section contains inner nodes
        // The "numPrims" section contains leaves
        // Each node is an int4. 
        // "X" is left, "Y" is right, "Z" is parent, and "W" is leaf or -1 if internal node.
        alignas(16) gprt::Buffer lbvhNodes;

        // numPrims-1 + numPrims long. Each aabb is a pair of float3.
        alignas(16) gprt::Buffer lbvhAabbs;


        // Testing...
        alignas(16) gprt::Buffer splitClusterAABBs;
        alignas(16) gprt::Buffer splitClusterPositions; // uint16_t of split poses, one per cluster
    };

    
};