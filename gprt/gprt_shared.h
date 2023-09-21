#pragma once

// The overall results I'm leaning from the below is that, the more culling you enable
// by turning these features on, the more performance becomes dependent on how efficient
// the culling primitives are...

// Makes me wonder if bounding spheres over bounding boxes would be a better idea...

// The higher this number is, the more clusters we're going to touch
// relative to the number of primitives. 
#define BRANCHING_FACTOR 10

// Edit: nevermind... I had a bug with my previous minMaxDist function which was giving me some 
// incorrect intuition. I'm finding now that this is very helpful for the utah teapot.
// Edit edit: Downward pruning seems to fall apart when nodes are quantized. 
// #define ENABLE_DOWNAWARD_CULLING

// Enables an LBVH reference, similar to Jakob and Guthe's knn.
// Note, we use this LBVH as a top level tree for our method, so 
// this is just about if we're using RT cores or not
#define ENABLE_LBVH_REFERENCE

// Uses a quantized representation of bounding boxes
#define ENABLE_QUANTIZATION

// Uses a quantized representation for the active cluster queue
// I've bounced back and forth on if this helps or not. 
//   For untruncated queries, the ultimate bottleneck is when tree traversal fails and many prims 
//   are all approximately the same distance. There, its better to have an accurate queue for culling
// 
// #define ENABLE_QUEUE_QUANTIZATION


// Bounding boxes have some unappealing culling properties for neighbor search, namely  
// that their corners tend to be hit without the primitive itself being hit.
// Bounding balls could to better, but computing the minimum enclosing ball is a hard problem. 
// Still, brute force computing minimum enclosing balls for leaves is fast enough. 
// So, this flag uses quantized bounding balls instead of bounding boxes for the leaves.
// In theory, this should help improve culling, especially for odd configurations where
// a min and max range are selected. 
// #define ENABLE_BOUNDING_BALLS

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
        alignas(4) uint32_t numL0Clusters;
        alignas(4) uint32_t numL1Clusters;
        alignas(4) uint32_t numL2Clusters;
        alignas(4) uint32_t numL3Clusters;
        alignas(4) float maxSearchRange;

        alignas(16) gprt::Buffer points; 

        // Why are we separating these again?... 
        // They could just be "primitives"...
        alignas(16) gprt::Buffer edges; 
        alignas(16) gprt::Buffer triangles;

        // Hilbert codes of quantized primitive centroids
        // One uint64_t per primitive
        alignas(16) gprt::Buffer codes;

        // Buffer containing the global AABB. Pair of two floats
        alignas(16) gprt::Buffer aabb;

        // Buffers of bounding primitives. 
        // If bounding boxes, each aabb is a pair of float3.
        // If bounding balls, each ball is a single float4 (xyzr).
        // Clusters contain primitives, superClusters contain clusters and leaves contain superClusters. 
        // Leaves are additionally dialated by "maximum search range" when using RT cores for truncated traversal.
        alignas(16) gprt::Buffer primBounds;
        alignas(16) gprt::Buffer l0clusters;
        alignas(16) gprt::Buffer l1clusters;
        alignas(16) gprt::Buffer l2clusters;
        alignas(16) gprt::Buffer l3clusters;

        alignas(16) gprt::Buffer clusters;
         
        // 3 floats for treelet aabb min, 
        // 3 bytes for scale exponent, one unused 
        // byte   15   14    13    12    11 10 9 8   7  6  5  4  3  2  1  0
        //       [??]  [sz]  [sy]  [sx]  [  zmin  ]  [  ymin  ]  [  xmin  ]
        alignas(16) gprt::Buffer treelets;

        // If a "bounding box", 64-bit integers, 6 bytes for bounding box, 2 unused.
        // byte   8    7   6     5     4     3     2     1           
        //       [?]  [?]  [zh]  [yh]  [xh]  [zl]  [yl]  [xl]        

        // If a "bounding ball", 32-bit integers, 3 bytes for center, 1 for radius.
        // byte   4    3    2    1           
        //       [r]  [z]  [y]  [x]        
        alignas(16) gprt::Buffer children; 

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

    };

    
};