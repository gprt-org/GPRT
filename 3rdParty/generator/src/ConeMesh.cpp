


#include "generator/ConeMesh.hpp"


using namespace generator;


ConeMesh::ConeMesh(
	double radius,
	double size,
	int slices,
	int segments,
	double start,
	double sweep
) :
	axisSwapMesh_{
		{
			{{size, 0.0}, {-size, radius}, segments},
			{1.0, 0.0}, slices, start, sweep
		},
		Axis::Y, Axis::Z, Axis::X
	}
{ }
