


#include "generator/SpringMesh.hpp"

using namespace generator;


SpringMesh::SpringMesh(
	double minor,
	double major,
	double size,
	int slices,
	int segments,
	double minorStart,
	double minorSweep,
	double majorStart,
	double majorSweep
) :
	extrudeMesh_{
		{minor, slices, minorStart, minorSweep},
		{major, size, segments, majorStart, majorSweep}
	}
{ }


