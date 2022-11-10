


#include "generator/RectangleShape.hpp"


using namespace generator;


RectangleShape::RectangleShape(
	const gml::dvec2& size,
	const gml::ivec2& segments
) :
	mergeShape_{
		{{size[0], -size[1]}, size, segments[1]},
		{size, {-size[0], size[1]}, segments[0]},
		{{-size[0], size[1]}, -size, segments[1]},
		{-size, {size[0], -size[1]}, segments[0]}
	}
{ }

