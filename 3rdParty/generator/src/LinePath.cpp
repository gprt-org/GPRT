

#include "generator/LinePath.hpp"


using namespace generator;


LinePath::LinePath(
	const gml::dvec3& start,
	const gml::dvec3& end,
	const gml::dvec3& normal,
	int segments
):
	parametricPath_{
		[start, end, normal] (double t) {
			PathVertex vertex;

			vertex.position = start + t * (end - start);
			vertex.tangent = normalize(end - start);
			vertex.normal = normal;
			vertex.texCoord = t;

			return vertex;
		},
		segments
	}
{ }

