


#include "generator/LineShape.hpp"


using namespace generator;


LineShape::LineShape(
	const gml::dvec2& start,
	const gml::dvec2& end,
	int segments
) :
	parametricShape_{
		[start, end] (double t) {
			ShapeVertex vertex;

			vertex.position = start + t * (end - start);
			vertex.tangent = normalize(end - start);
			vertex.texCoord = t;

			return vertex;
		},
		segments
	}
{

}
