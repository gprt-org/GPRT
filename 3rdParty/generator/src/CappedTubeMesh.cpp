


#include "generator/CappedTubeMesh.hpp"

using namespace generator;

using namespace generator::detail;


TubeCap::TubeCap(
	double radius,
	double innerRadius,
	double distance,
	int slices,
	int rings,
	double start,
	double sweep
) :
	translateMesh_{
		{radius, innerRadius, slices, rings, start, sweep},
		{0.0, 0.0, distance}
	}
{ }


CappedTubeMesh::CappedTubeMesh(
	double radius,
	double innerRadius,
	double size,
	int slices,
	int segments,
	int rings,
	double start,
	double sweep
) :
	mergeMesh_{
		{radius, innerRadius, size, slices, segments, start, sweep},
		{radius, innerRadius, size, slices, rings, start, sweep},
		{{radius, innerRadius, -size, slices, rings, start, sweep}},
	}
{ }

