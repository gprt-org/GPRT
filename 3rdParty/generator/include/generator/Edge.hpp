

#ifndef GENERATOR_EDGE_HPP
#define GENERATOR_EDGE_HPP

#include "math.hpp"

namespace generator {

class Edge {
public:

	gml::ivec2 vertices;

	Edge() noexcept :
		vertices{}
	{ }

	explicit Edge(const gml::ivec2& vertices) noexcept :
		vertices{vertices}
	{ }

};

}

#endif
