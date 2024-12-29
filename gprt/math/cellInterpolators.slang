#pragma once
#include "gprt.h"

using namespace gprt;

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

#define NEWTON_ITERATIONS 5
#define CONVERGED_ERROR 1e-3
#define DIVERGED_ERROR  1e6

// Returns true if the point is within the tetrahedron.
bool invertInterpolant(float3 P, in Tetrahedron e, inout float3 ic) {
    float rcp_vol_all = rcp(determinant(float3x3(e.v[1] - e.v[0], e.v[2] - e.v[0], e.v[3] - e.v[0])));
    
    // Determine how far the point is from each of the faces. 
    float w0 = determinant(float3x3(P - e.v[1], e.v[3] - e.v[1], e.v[2] - e.v[1])) * rcp_vol_all;
    if (w0 < 0.0) return false;
    float w1 = determinant(float3x3(P - e.v[0], e.v[2] - e.v[0], e.v[3] - e.v[0])) * rcp_vol_all;
    if (w1 < 0.0) return false;
    float w2 = determinant(float3x3(P - e.v[0], e.v[3] - e.v[0], e.v[1] - e.v[0])) * rcp_vol_all;
    if (w2 < 0.0) return false;
    float w3 = determinant(float3x3(P - e.v[0], e.v[1] - e.v[0], e.v[2] - e.v[0])) * rcp_vol_all;
    if (w3 < 0.0) return false;

    // 1 = w0 + w1 + w2 + w3, so 1 - w1 - w2 - w3 = w0
    ic = float3(w1, w2, w3);
    return true;
};

bool invertInterpolant(float3 P, in Pyramid e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.2);

  // Iteration for Newton's method
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = Pyramid::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < 5; ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(Pyramid::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(Pyramid::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(Pyramid::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < 5; ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < 5; ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < 5; ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  // Check for containment
  return Pyramid::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in Wedge e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.25, 0.25, 0.5);

  // Iteration for Newton's method
  bool converged = false;
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = Wedge::IsoToSupport(rst);

    // If the "rst" location matches the query point, we can now use our support function values
    if (converged) break;

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(Wedge::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(Wedge::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(Wedge::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return Wedge::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in Hexahedron e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.5);

  // Iteration for Newton's method
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = Hexahedron::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(Hexahedron::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(Hexahedron::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(Hexahedron::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  // Check for containment of the final cannonical point location
  return Hexahedron::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in PentagonalPrism e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.5);
  
  // Iteration for Newton's method
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = PentagonalPrism::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(PentagonalPrism::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(PentagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(PentagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return PentagonalPrism::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in HexagonalPrism e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.5);

  // Iteration for Newton's method
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = HexagonalPrism::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(HexagonalPrism::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(HexagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(HexagonalPrism::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return HexagonalPrism::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticTetrahedron e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.5);

  // Iteration for Newton's method
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    let w = QuadraticTetrahedron::IsoToSupport(rst);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // rst-derivatives
    let dwr = fwd_diff(QuadraticTetrahedron::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(QuadraticTetrahedron::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(QuadraticTetrahedron::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
  }

  return QuadraticTetrahedron::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticPyramid e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.2);

  // Iteration for Newton's method
  bool converged = false;
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute difference in reconstructed query position vs actual query position
    float3 fcol = -P;
    let w = QuadraticPyramid::IsoToSupport(rst);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // Use this difference to compute rst derivatives...
    float3 rcol = float3(0.f);
    let dwr = fwd_diff(QuadraticPyramid::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];

    float3 scol = float3(0.f);
    let dws = fwd_diff(QuadraticPyramid::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];

    float3 tcol = float3(0.f);
    let dwt = fwd_diff(QuadraticPyramid::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants to generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;

    if (all(abs(fcol) < CONVERGED_ERROR)) converged = true;
  }

  return converged && QuadraticPyramid::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticWedge e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.2);

  // Iteration for Newton's method
  bool converged = false;
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute difference in reconstructed query position vs actual query position
    float3 fcol = -P;
    let w = QuadraticWedge::IsoToSupport(rst);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // Use this difference to compute rst derivatives...
    float3 rcol = float3(0.f);
    let dwr = fwd_diff(QuadraticWedge::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];

    float3 scol = float3(0.f);
    let dws = fwd_diff(QuadraticWedge::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];

    float3 tcol = float3(0.f);
    let dwt = fwd_diff(QuadraticWedge::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants to generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;

    if (all(abs(fcol) < CONVERGED_ERROR)) converged = true;
  }

  return converged && QuadraticWedge::IsoIsContained(rst);
}

bool invertInterpolant(float3 P, in QuadraticHexahedron e, out float3 rst) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rst = float3(0.5, 0.5, 0.2);

  // Iteration for Newton's method
  bool converged = false;
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute difference in reconstructed query position vs actual query position
    float3 fcol = -P;
    let w = QuadraticHexahedron::IsoToSupport(rst);
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) fcol += e.v[i] * w[i];

    // Use this difference to compute rst derivatives...
    float3 rcol = float3(0.f);
    let dwr = fwd_diff(QuadraticHexahedron::IsoToSupport)(diffPair(rst, float3(1.0, 0.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) rcol += e.v[i] * dwr[i];

    float3 scol = float3(0.f);
    let dws = fwd_diff(QuadraticHexahedron::IsoToSupport)(diffPair(rst, float3(0.0, 1.0, 0.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) scol += e.v[i] * dws[i];

    float3 tcol = float3(0.f);
    let dwt = fwd_diff(QuadraticHexahedron::IsoToSupport)(diffPair(rst, float3(0.0, 0.0, 1.0))).getDifferential();
    [unroll] for (int i = 0; i < e.v.getCount(); ++i) tcol += e.v[i] * dwt[i];

    // Compute determinants to generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rst.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rst.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rst.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;

    if (all(abs(fcol) < CONVERGED_ERROR)) converged = true;
  }

  return converged && QuadraticHexahedron::IsoIsContained(rst);
}