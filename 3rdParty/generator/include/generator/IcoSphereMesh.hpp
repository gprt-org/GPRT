

#ifndef GENERATOR_ICOSPHEREMESH_HPP
#define GENERATOR_ICOSPHEREMESH_HPP


#include "IcosahedronMesh.hpp"
#include "SpherifyMesh.hpp"


namespace generator {

/// Icosphere aka spherical subdivided icosahedron
/// @image html IcoSphereMesh.svg
class IcoSphereMesh
{
private:

	using Impl = SpherifyMesh<IcosahedronMesh>;
	Impl spherifyMesh_;

public:

	/// @param radius The radius of the containing sphere.
	/// @param segments The number of segments per icosahedron edge. Must be >= 1.
	IcoSphereMesh(double radius = 1.0, int segments = 4);

	using Triangles = typename Impl::Triangles;

	Triangles triangles() const noexcept { return spherifyMesh_.triangles(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return spherifyMesh_.vertices(); }

};


}


#endif
