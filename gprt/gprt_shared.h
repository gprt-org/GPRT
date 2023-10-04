#pragma once

#define COLLECT_STATS

// The overall results I'm leaning from the below is that, the more culling you enable
// by turning these features on, the more performance becomes dependent on how efficient
// the culling primitives are...

// The higher this number is, the more clusters we're going to touch
// relative to the number of primitives. 
#define BRANCHING_FACTOR 10

// Edit: nevermind... I had a bug with my previous minMaxDist function which was giving me some 
// incorrect intuition. I'm finding now that this is very helpful for the utah teapot.
// Edit edit: Downward pruning seems to fall apart when nodes are quantized. The reason 
// for why is because the nearest face might differ between the quantized and unquantized boxes.
// Nearest face only matches when quantized box is just a scaling up or down of the original. 
// It's very rare that this happens, but also, the performance improvements from this aren't very good.
//
// Edit edit edit: When using OBBs, it seems like the maximal distance to the box gives good performance 
// improvements.
// More updates, the "minMaxDistance" metric doesn't work for internal OBB nodes, since internal faces aren't guaranteed
// to contain primitives. Only OBB leaves can guarantee this. So, for internal nodes, we instead use the "maxDistance"
// for downward culling.
// More more updates... So long as internal OBBs are built to tightly fit the underlying primitives, then minMaxDist is
// "free game".
#define ENABLE_DOWNAWARD_CULLING 

// Enables an LBVH reference, similar to Jakob and Guthe's knn.
// Note, we use this LBVH as a top level tree for our method, so 
// this is just about if we're using RT cores or not
#define ENABLE_LBVH_REFERENCE

// Uses a quantized representation for the active cluster queue
// I've bounced back and forth on if this helps or not. 
//   For untruncated queries, the ultimate bottleneck is when tree traversal fails and many prims 
//   are all approximately the same distance. There, its better to have an accurate queue for culling. 
//   However, I suspect the issue with culling has more to do with using AABBs over something tigher fitting
// #define ENABLE_QUEUE_QUANTIZATION


// Enables oriented bounding boxes in an attempt to improve culling performance. 
// Prior experiments seem to suggest that the minimum distance to an axis aligned bounding 
// box tends to be a poor fit for the minimum distance to the contained primitive. This
// issue seems to become more and more extreme as primitives are clustered together.
// This mode uses a singular value decomposition to compute the orientation of a bounding box 
// that best fits the underlying primitive distribution
#define ENABLE_OBBS

// Honestly I don't see much improvement by avoiding square roots. I don't think this is what the bottleneck is
// at least on my problems... Just going to take square roots for simplicity...
// #define USE_SQUARED_DISTS

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
        alignas(4) uint32_t numLeaves;
        alignas(4) uint32_t numL0Clusters;
        alignas(4) uint32_t numL1Clusters;
        alignas(4) uint32_t numL2Clusters;
        alignas(4) uint32_t numL3Clusters;
        alignas(4) uint32_t numL4Clusters;
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
        // If axis aligned bounding boxes, each is a pair of float3.
        // If bounding balls, each is a single float4 (xyzr).
        // If oriented bounding boxes, each is a triplet of float3s.
        alignas(16) gprt::Buffer leaves;
        alignas(16) gprt::Buffer l0clusters;
        alignas(16) gprt::Buffer l1clusters;
        alignas(16) gprt::Buffer l2clusters;
        alignas(16) gprt::Buffer l3clusters;
        alignas(16) gprt::Buffer l4clusters;

        // For OBBs, buffers of center points.
        alignas(16) gprt::Buffer llcenters;
        alignas(16) gprt::Buffer l0centers;
        alignas(16) gprt::Buffer l1centers;
        alignas(16) gprt::Buffer l2centers;
        alignas(16) gprt::Buffer l3centers;
        alignas(16) gprt::Buffer l4centers;

        // For OBBs, buffers of covariance matrices
        alignas(16) gprt::Buffer llcovariances;
        alignas(16) gprt::Buffer l0covariances;
        alignas(16) gprt::Buffer l1covariances;
        alignas(16) gprt::Buffer l2covariances;
        alignas(16) gprt::Buffer l3covariances;
        alignas(16) gprt::Buffer l4covariances;

        // An internal iteration parameter for construction
        alignas(4) uint32_t iteration;
         
        // 3 floats for treelet aabb min, 
        // 3 bytes for scale exponent, one unused 
        // byte   15   14    13    12    11 10 9 8   7  6  5  4  3  2  1  0
        //       [??]  [sz]  [sy]  [sx]  [  zmin  ]  [  ymin  ]  [  xmin  ]
        // alignas(16) gprt::Buffer treelets;

        // If an "axis aligned bounding box", 64-bit integers, 6 bytes for bounding box, 2 unused.
        // byte   8    7   6     5     4     3     2     1           
        //       [?]  [?]  [zh]  [yh]  [xh]  [zl]  [yl]  [xl]        

        // If a "bounding ball", 32-bit integers, 3 bytes for center, 1 for radius.
        // byte   4    3    2    1           
        //       [r]  [z]  [y]  [x]        

        // If an "oriented bounding box", we use a 128-bit integer. 
        // 6 bytes for bounding box, 2 unused. 
        // Then 20 bits for Euler rotations in X, Y, then Z. 4 bits unused.
        //        15 14 13 12 11 10 9 8   7   6   5     4     3    2    1    0
        //       [ zr ]  [ yr ]  [ xr ]   [?] [?] [zh]  [yh]  [xh]  [zl]  [yl]  [xl]
        // alignas(16) gprt::Buffer children; 

        // An RT core tree
        alignas(16) gprt::Accel accel;

        // An LBVH tree
        // alignas(16) gprt::Buffer lbvhMortonCodes;

        // Primitive IDs that correspond to sorted morton codes. 
        // One uint32_t per primitive
        // alignas(16) gprt::Buffer lbvhIds;

        // numPrims-1 + numPrims long. 
        // The "numPrims-1" section contains inner nodes
        // The "numPrims" section contains leaves
        // Each node is an int4. 
        // "X" is left, "Y" is right, "Z" is parent, and "W" is leaf or -1 if internal node.
        // alignas(16) gprt::Buffer lbvhNodes;

        // numPrims-1 + numPrims long. Each aabb is a pair of float3.
        // alignas(16) gprt::Buffer lbvhAabbs;

    };

    
};