

#ifndef GENERATOR_CYLINDERMESH_HPP
#define GENERATOR_CYLINDERMESH_HPP

#include "AxisSwapMesh.hpp"
#include "LatheMesh.hpp"
#include "LineShape.hpp"
#include "UvFlipMesh.hpp"

namespace generator {



/// Cylinder centered at origin aligned along the z-axis.
/// @image html CylinderMesh.svg
class CylinderMesh
{
private:

	using Impl = AxisSwapMesh<LatheMesh<LineShape>>;
	Impl axisSwapMesh_;

public:

	/// @param radius Radius of the cylinder along the xy-plane.
	/// @param size Half of the length of the cylinder along the z-axis.
	/// @param slices Subdivisions around the z-axis.
	/// @param segments Subdivisions along the z-axis.
	/// @param start Counterclockwise angle around the z-axis relative to the x-axis.
	/// @param sweep Counterclockwise angle around the z-axis.
	CylinderMesh(
		double radius = 1.0,
		double size = 1.0,
		int slices = 32,
		int segments = 8,
		double start = 0.0,
		double sweep = gml::radians(360.0)
	);

	using Triangles = typename Impl::Triangles;

	Triangles triangles() const noexcept { return axisSwapMesh_.triangles(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return axisSwapMesh_.vertices(); }

};


}

#endif
