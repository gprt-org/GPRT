


#include "generator/CappedCylinderMesh.hpp"


using namespace generator;
using namespace generator::detail;


Cap::Cap(
	double radius,
	double distance,
	int slices,
	int rings,
	double start,
	double sweep
) :
	translateMesh_{
		{radius, 0.0, slices, rings, start, sweep},
		{0.0, 0.0, distance}
	}
{ }


CappedCylinderMesh::CappedCylinderMesh(
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
		{radius, size, slices, rings, start, sweep},
		{{{radius, -size, slices, rings, start, sweep}}, true, false}
	}
{ }

