

#ifndef GENERATOR_TRIANGLE_HPP
#define GENERATOR_TRIANGLE_HPP

#include "math.hpp"

namespace generator {



class Triangle {
public:

	/// Zero based indices of the triangle vertices in counterclockwise order.
	gml::ivec3 vertices;

	Triangle() noexcept :
		vertices{}
	{ }

	explicit Triangle(const gml::ivec3& vertices) noexcept :
		vertices{vertices}
	{ }

};

}

#endif
