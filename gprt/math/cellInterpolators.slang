// For all

static int GPRT_TETRA = 10;
static int GPRT_VOXEL = 11;
static int GPRT_HEXAHEDRON = 12;
static int GPRT_WEDGE = 13;
static int GPRT_PYRAMID = 14;
static int GPRT_PENTAGONAL_PRISM = 15;
static int GPRT_HEXAGONAL_PRISM = 16;

// #define GPRT_TETRA 10
// #define GPRT_VOXEL 11
// #define GPRT_HEXAHEDRON 12
// #define GPRT_WEDGE 13
// #define GPRT_PYRAMID 14
// #define GPRT_PENTAGONAL_PRISM 15
// #define GPRT_HEXAGONAL_PRISM 16

// Node positions, with "y" up and data values stored in "w"
struct Tetrahedron {
    // Bottom triangle (counter-clockwise order)
    float3 v0, v1, v2;

    // Top point
    float3 v3;
};

// Returns true if the point is within the tetrahedron. 
bool invertInterpolant(float3 P, in Tetrahedron e, inout float3 ic) {
    // Point cannot be in cell if it's volume is negative.
    float vol_all = determinant(float3x3(e.v1.xyz - e.v0.xyz, e.v2.xyz - e.v0.xyz, e.v3.xyz - e.v0.xyz)); 
    if (vol_all == 0.0) return false;

    // Determine how far the point is from each of the faces. 
    float w0 = determinant(float3x3(P - e.v1.xyz, e.v3.xyz - e.v1.xyz, e.v2.xyz - e.v1.xyz));
    if (w0 < 0.0) return false;
    float w1 = determinant(float3x3(P - e.v0.xyz, e.v2.xyz - e.v0.xyz, e.v3.xyz - e.v0.xyz));
    if (w1 < 0.0) return false;
    float w2 = determinant(float3x3(P - e.v0.xyz, e.v3.xyz - e.v0.xyz, e.v1.xyz - e.v0.xyz));
    if (w2 < 0.0) return false;
    float w3 = determinant(float3x3(P - e.v0.xyz, e.v1.xyz - e.v0.xyz, e.v2.xyz - e.v0.xyz));
    if (w3 < 0.0) return false;

    w1 *= rcp(vol_all);
    w2 *= rcp(vol_all);
    w3 *= rcp(vol_all);

    // 1 = w0 + w1 + w2 + w3, so 1 - w1 - w2 - w3 = w0
    ic = float3(w1, w2, w3);
    return true;
}

/**
 * @param ic - the isoparametric coordinates returned by inverting the tetrahedron's interpolant.
*/
float[4] isoToSupport<Tetrahedron>(float3 ic) {
    float[4] sc;
    sc[0] = 1.f - (ic.x + ic.y + ic.z);
    sc[1] = ic.x;
    sc[2] = ic.y;
    sc[3] = ic.z;
    return sc;
};
