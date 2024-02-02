#pragma once
#include "gprt.h"

struct EdgeHash {
    size_t operator()(const int2& edge) const {
        size_t h1 = std::hash<int>()(edge.x);
        size_t h2 = std::hash<int>()(edge.y);
        return h1 ^ h2;
    }
};
struct EdgeEqual {
    bool operator()(const int2& edge1, const int2& edge2) const {
        int2 e1 = {std::min(edge1.x, edge1.y), std::max(edge1.x, edge1.y)};
        int2 e2 = {std::min(edge2.x, edge2.y), std::max(edge2.x, edge2.y)};
        return e1.x == e2.x && e1.y == e2.y;
    }
};

/**
 * @brief Builds a mapping from edges to triangles in a mesh.
 *
 * This function creates a map that associates each edge in the mesh with the 
 * triangles that the edge is a part of. An edge is defined as a pair of vertex 
 * indices and is treated as undirected, meaning an edge from vertex A to vertex B 
 * is considered the same as an edge from vertex B to vertex A. The function 
 * iterates over all triangles in the mesh, and for each triangle, it adds its 
 * edges to the map. If an edge is shared by multiple triangles, all those 
 * triangles will be associated with that edge in the map.
 *
 * @param triangles A std::vector<int3> containing the triangles of the mesh,
 *                  where each triangle is defined by int3 indices into the vertices array.
 * 
 * @return std::unordered_map<int2, std::vector<int>> A map where each key is an edge,
 *         defined as a pair of vertex indices, and its associated value is a vector of 
 *         integers representing the indices of the triangles that contain the edge. 
 *         The int2 type is hashed internally such that an edge is considered equal 
 *         regardless of the order of its vertices.
 */
std::unordered_map<int2, std::vector<int>, EdgeHash, EdgeEqual> buildEdgeToTriangleMap(const std::vector<int3>& triangles);

/**
 * @brief Builds a mapping from vertices to triangles in a mesh
 *
 * This function creates a map that associates each vertex in the mesh with the 
 * triangles that the vertex is a part of. The function effectively inverts the input
 * triangle to vertex mapping. 
 *
 * @param triangles A std::vector<int3> containing the triangles of the mesh,
 *                  where each triangle is defined by int3 indices into the vertices array.
 * @param numVertices The number of vertices in the mesh.
 * 
 * @return std::vector<std::vector<int>> A vector of per-vertex triangle references.
 */
std::vector<std::vector<int>> buildVertexToTriangleMap(const std::vector<int3>& triangles, int numVertices);

std::vector<uint8_t> getVertexCounts(const std::vector<int3>& triangles, int numVertices);

/**
 * @brief Computes the triangle connectivity for a given mesh.
 *
 * This function calculates the connectivity data for each triangle in a mesh. 
 * The connectivity data for a triangle consists of the indices of adjacent 
 * triangles across each of its edges. For each edge of a triangle, the function 
 * identifies the adjacent triangle (if any) that shares the same edge. If an 
 * edge is not shared with any other triangle (i.e., it's a boundary edge), 
 * the corresponding value in the connectivity data is set to -1. Ordering of these 
 * edges is such that edge 0 contains vertices 1 and 2, edge 1 contains vertices 2 and 3, 
 * and edge 2 contains vertices 0 and 1. 
 *
 * @param triangles A std::vector<Triangle> representing the triangles of the mesh,
 *                  where each Triangle consists of indices into the vertices array.
 * @param edgeToTriangleMap A std::unordered_map<Edge, std::vector<int>> mapping each
 *                          Edge to a vector of triangle indices that share the edge.
 *                          This map is used to determine the adjacent triangles for
 *                          each edge of a triangle.
 * 
 * @return std::vector<Int3> A vector where each element represents the connectivity 
 *         data for a triangle. Each Int3 element contains three indices, corresponding
 *         to the adjacent triangles across each edge of the triangle. A value of -1 
 *         indicates that the edge is a boundary edge with no neighboring triangle.
 */
std::vector<int3> computeTriangleConnectivity(const std::vector<int3>& triangles, const std::unordered_map<int2, std::vector<int>, EdgeHash, EdgeEqual>& edgeToTriangleMap);

/**
 * Computes the normal vector of a triangle defined by three vertices.
 *
 * @param v1 The first vertex of the triangle.
 * @param v2 The second vertex of the triangle.
 * @param v3 The third vertex of the triangle.
 * @return The normal vector of the triangle.
 */
float3 computeTriangleNormal(const float3& v1, const float3& v2, const float3& v3);



std::vector<float3> computeVertexNormalBounds(const std::vector<float3>& vertices, const std::vector<int3>& triangles);

std::vector<float3> computeVertexNormalCones(const std::vector<float3>& vertices, const std::vector<int3>& triangles);

/**
 * @brief Computes the normal arcs for each edge in a mesh.
 *
 * This function calculates the normal arcs for each edge in a given mesh. 
 * A normal arc for an edge is defined by the averaged normal of all the 
 * triangles sharing that edge and the maximum angle between this average 
 * normal and each of the triangle normals. The function iterates over all 
 * triangles to compute their normals and aggregates these at their edges. 
 * Then, it averages these normals and calculates the maximum angle for each 
 * edge to define the normal arc.
 *
 * @param vertices A std::vector<float3> containing the vertices of the mesh.
 * @param triangles A std::vector<int3> containing the triangles of the mesh, 
 *                  where each triangle is defined by indices into the vertices array.
 * 
 * @return std::vector<float4> A vector containing the normal arc for each 
 *         edge, where each normal arc consists of a normal vector (as xyz) and an arc 
 *         angle (as w) about the edge.
 */
std::vector<float4> computeEdgeNormalArcs(const std::vector<float3>& vertices, const std::vector<int3>& triangles);