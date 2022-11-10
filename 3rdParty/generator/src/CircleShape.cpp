

#include "generator/CircleShape.hpp"


using namespace generator;


CircleShape::CircleShape(
	double radius,
	int segments,
	double start,
	double sweep
) :
	parametricShape_{
		[radius, start, sweep] (double t) {
			const double angle = t * sweep + start;
			const double sine = std::sin(angle);
			const double cosine = std::cos(angle);

			ShapeVertex vertex;
			vertex.position = gml::dvec2{radius * cosine, radius * sine};
			vertex.tangent = gml::dvec2{-sine, cosine};
			vertex.texCoord = t;

			return vertex;
		},
		segments
	}
{ }
