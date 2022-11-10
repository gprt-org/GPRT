

#ifndef GENERATOR_MESHVERTEX_HPP
#define GENERATOR_MESHVERTEX_HPP

#include "math.hpp"

namespace generator {


class MeshVertex {
public:

	gml::dvec3 position;

	/// Unit vector perpendicular to the surface.
	gml::dvec3 normal;

	/// UV texture coordinates
	gml::dvec2 texCoord;

	MeshVertex() noexcept :
		position{},
		normal{},
		texCoord{}
	{ }

};



}

#endif
