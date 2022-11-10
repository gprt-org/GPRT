


#include "generator/TubeMesh.hpp"


using namespace generator;


TubeMesh::TubeMesh(
	double radius,
	double innerRadius,
	double size,
	int slices,
	int segments,
	double start,
	double sweep
) :
	mergeMesh_{
		{radius, size, slices, segments, start, sweep},
		{{{innerRadius, size, slices, segments, start, sweep}, true, false}}
	}
{ }

