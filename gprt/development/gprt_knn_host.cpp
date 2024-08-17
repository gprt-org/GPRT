#include "gprt_knn_host.h"
#include <iostream>

using namespace math;

float3 computeTriangleNormal(const float3& v1, const float3& v2, const float3& v3) {
    float3 edge1 = v2 - v1;
    float3 edge2 = v3 - v1;
    if (length(edge1) == 0.0f || length(edge2) == 0.0f) {
        return {0, 0, 0}; // no area
    }
    if (all(edge1 == edge2)) {
        return {0, 0, 0}; // degenerate triangle
    }
    float3 normal = cross(edge1, edge2);
    normal = normalize(normal);
    if (math::isnan(normal.x) || math::isnan(normal.y) || math::isnan(normal.z)) {
        std::cout << "NaN normal" << std::endl;
    }
    return normal;
}

std::vector<std::vector<int>> buildVertexToTriangleMap(const std::vector<int3>& triangles, size_t numVertices) {
    std::vector<std::vector<int>> vertexToTriangleMap(numVertices);

    for (int i = 0; i < triangles.size(); ++i) {
        const auto& tri = triangles[i];
        for (int j = 0; j < 3; ++j) {
            vertexToTriangleMap[tri[j]].push_back(i);
        }
    }

    return vertexToTriangleMap;
}

std::vector<uint8_t> getVertexCounts(const std::vector<int3>& triangles, int numVertices)
{
    std::vector<uint8_t> counts(numVertices, 0);
    for (const auto& tri : triangles) {
        for (int i = 0; i < 3; ++i) {
            counts[tri[i]]++;
        }
    }
    return counts;
}

std::vector<float3> computeVertexNormalBounds(const std::vector<float3>& vertices, const std::vector<int3>& triangles) {
    std::vector<std::vector<int>> vertToTri = buildVertexToTriangleMap(triangles, vertices.size());

    // Use the vertex to triangle mapping to compute an average normal for each vertex
    std::vector<float3> vertexNormalBounds(vertices.size() * 2);
    for (int i = 0; i < vertices.size(); ++i) {
        vertexNormalBounds[i * 2 + 0] = {1e38f, 1e38f, 1e38f};
        vertexNormalBounds[i * 2 + 1] = {-1e38f, -1e38f, -1e38f};
    }

    // Accumulate face normals on the vertices 
    for (int i = 0; i < triangles.size(); ++i) {
        const auto& tri = triangles[i];
        float3 normal = computeTriangleNormal(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]);
        for (int j = 0; j < 3; ++j) {
            int v = tri[j];
            vertexNormalBounds[v * 2 + 0].x = min(vertexNormalBounds[v * 2 + 0].x, normal.x);
            vertexNormalBounds[v * 2 + 0].y = min(vertexNormalBounds[v * 2 + 0].y, normal.y);
            vertexNormalBounds[v * 2 + 0].z = min(vertexNormalBounds[v * 2 + 0].z, normal.z);
            vertexNormalBounds[v * 2 + 1].x = max(vertexNormalBounds[v * 2 + 1].x, normal.x);
            vertexNormalBounds[v * 2 + 1].y = max(vertexNormalBounds[v * 2 + 1].y, normal.y);
            vertexNormalBounds[v * 2 + 1].z = max(vertexNormalBounds[v * 2 + 1].z, normal.z);
        }
    }

    // bounding range for the normal
    return vertexNormalBounds;
}

std::vector<float4> computeEdgeNormalArcs(const std::vector<float3>& vertices, const std::vector<int3>& triangles) {
    std::unordered_map<int2, std::vector<float3>, EdgeHash, EdgeEqual> edgeNormals;
    std::vector<float4> normalArcs(triangles.size() * 3);

    // Calculate normals for each triangle face and append to a list at the edges
    for (const auto& tri : triangles) {
        float3 normal = computeTriangleNormal(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]);

        int2 edges[3] = {
            int2(tri[1], tri[2]),
            int2(tri[2], tri[0]),
            int2(tri[0], tri[1])
        };

        for (int i = 0; i < 3; ++i) {
            edgeNormals[edges[i]].push_back(normal);
        }
    }

    // Average normals and compute the cone angle
    for (size_t i = 0; i < triangles.size(); ++i) {
        const auto& tri = triangles[i];
        int2 edges[3] = {
            int2(tri[1], tri[2]),
            int2(tri[2], tri[0]),
            int2(tri[0], tri[1])
        };

        // For each edge of each triangle
        for (int j = 0; j < 3; ++j) {
            const auto& edge = edges[j];
            const auto& normals = edgeNormals[edge];

            // Compute the average normal for the edge
            float3 avgNormal{0, 0, 0};
            for (const auto& normal : normals) {
                avgNormal.x += normal.x;
                avgNormal.y += normal.y;
                avgNormal.z += normal.z;
            }

            avgNormal.x /= normals.size();
            avgNormal.y /= normals.size();
            avgNormal.z /= normals.size();
            avgNormal = normalize(avgNormal);

            // Then compute the maximum angle between the average normal and each triangle normal
            float maxAngle = 0.0f;
            for (const auto& normal : normals) {
                float angle = std::acos(dot(avgNormal, normal));
                maxAngle = std::max(maxAngle, angle);
            }

            normalArcs[i * 3 + j] = {avgNormal, maxAngle};
        }
    }

    return normalArcs;
}

std::unordered_map<int2, std::vector<int>, EdgeHash, EdgeEqual> buildEdgeToTriangleMap(const std::vector<int3>& triangles) {
    std::unordered_map<int2, std::vector<int>, EdgeHash, EdgeEqual> edgeToTriangleMap;

    for (int i = 0; i < triangles.size(); ++i) {
        const auto& tri = triangles[i];
        int2 edges[3] = {
            int2(tri[1], tri[2]),
            int2(tri[2], tri[0]),
            int2(tri[0], tri[1])
        };

        for (const auto &edge : edges) {
            edgeToTriangleMap[edge].push_back(i);
        }
    }

    return edgeToTriangleMap;
}

std::vector<int3> computeTriangleConnectivity(const std::vector<int3>& triangles, const std::unordered_map<int2, std::vector<int>, EdgeHash, EdgeEqual>& edgeToTriangleMap) {
    std::vector<int3> connectivity(triangles.size(), {-1, -1, -1});
    EdgeEqual edgeEqual;

    for (int i = 0; i < triangles.size(); ++i) {
        const auto& tri = triangles[i];
        int2 edges[3] = {
            int2(tri[1], tri[2]),
            int2(tri[2], tri[0]),
            int2(tri[0], tri[1])
        };

        // Find the triangles that share an edge with this triangle
        for (int j = 0; j < 3; ++j) {
            const auto& edgeTriangles = edgeToTriangleMap.at(edges[j]);

            // For each such triangle...
            for (int k : edgeTriangles) {
                // ...if it's not the current triangle...
                if (k == i) continue;

                // then extract its edges, being careful to preserve the winding order...
                int3 otherTri = triangles[k];
                int2 otherEdges[3] = {
                    int2(otherTri[1], otherTri[2]),
                    int2(otherTri[2], otherTri[0]),
                    int2(otherTri[0], otherTri[1])
                };

                // ... and then figure out which edge it shares with the current triangle
                for (int l = 0; l < 3; ++l) {
                    if (edgeEqual(edges[j], otherEdges[l])) {
                        connectivity[i][j] = k;
                        break;
                    }
                }
            }
        }
    }

    return connectivity;
}