

#ifndef GENERATOR_TUBEMESH_HPP
#define GENERATOR_TUBEMESH_HPP

#include "CylinderMesh.hpp"
#include "FlipMesh.hpp"
#include "MergeMesh.hpp"
#include "UvFlipMesh.hpp"


namespace generator {


/// Tube (thick cylinder) centered at origin aligned along the z-axis.
/// @image html TubeMesh.svg
class TubeMesh
{
private:

	using Impl = MergeMesh<CylinderMesh, FlipMesh<UvFlipMesh<CylinderMesh>>>;
	Impl mergeMesh_;

public:

	/// @param radius The outer radius of the cylinder on the xy-plane.
	/// @param innerRadius The inner radius of the cylinder on the xy-plane.
	/// @param size Half of the length of the cylinder along the z-axis.
	/// @param slices Subdivisions around the z-axis.
	/// @param segments Subdivisions along the z-axis.
	/// @param start Counterclockwise angle around the z-axis relative to the x-axis.
	/// @param sweep Counterclockwise angle around the z-axis.
	TubeMesh(
		double radius = 1.0,
		double innerRadius = 0.75,
		double size = 1.0,
		int slices = 32,
		int segments = 8,
		double start = 0.0,
		double sweep = gml::radians(360.0)
	);

	using Triangles = typename Impl::Triangles;

	Triangles triangles() const noexcept { return mergeMesh_.triangles(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return mergeMesh_.vertices(); }

};


}

#endif
