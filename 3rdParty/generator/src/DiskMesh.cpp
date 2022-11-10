

#include "generator/DiskMesh.hpp"


using namespace generator;


DiskMesh::DiskMesh(
	double radius,
	double innerRadius,
	int slices,
	int rings,
	double start,
	double sweep
) :
	axisSwapMesh_{
		{
			{{0.0, innerRadius}, {0.0, radius}, rings},
			{1.0, 0.0}, slices, start, sweep
		},
		Axis::Y, Axis::Z, Axis::X
	}
{ }
