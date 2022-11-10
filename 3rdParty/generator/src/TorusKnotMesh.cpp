


#include "generator/TorusKnotMesh.hpp"


using namespace generator;


TorusKnotMesh::TorusKnotMesh(
	int p,
	int q,
	int slices,
	int segments
) :
	extrudeMesh_{
		{0.25, slices, 0.0, gml::radians(360.0)},
		{p, q, segments}
	}
{ }

