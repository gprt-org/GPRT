

#ifndef GENERATOR_PATHVERTEX_HPP
#define GENERATOR_PATHVERTEX_HPP

#include "math.hpp"

namespace generator {

class PathVertex {
public:

	/// Unit length vector perpendicular to the path at this point.
	/// Also the x-axis of the path coordinate system at this point.
	gml::dvec3 normal;

	gml::dvec3 position;

	/// Unit length vector parallel to the path at this point.
	/// Also the z-axis of the path at this point.
	gml::dvec3 tangent;

	double texCoord;

	PathVertex() :
		normal{},
		position{},
		tangent{},
		texCoord{}
	{ }

	/// Returns tangent x normal.
	/// Also the y-axis of the path coordinate system.
	/// See: http://mathworld.wolfram.com/BinormalVector.html
	gml::dvec3 binormal() const noexcept {
		return cross(tangent, normal);
	}

};

}

#endif
