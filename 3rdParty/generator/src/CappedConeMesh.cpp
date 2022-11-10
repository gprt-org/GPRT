


#include "generator/CappedConeMesh.hpp"


using namespace generator;


CappedConeMesh::CappedConeMesh(
	double radius,
	double size,
	int slices,
	int segments,
	int rings,
	double start,
	double sweep
) :
	mergeMesh_{
		{radius, size, slices, segments, start, sweep},
		{{{
			DiskMesh{radius, 0.0, slices, rings},
			gml::dvec3{0.0, 0.0, -size}
		}}, true, false}
	}
{ }



