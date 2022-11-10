

#include "generator/SphereMesh.hpp"

using namespace generator;


SphereMesh::SphereMesh(
	double radius,
	int slices,
	int segments,
	double sliceStart,
	double sliceSweep,
	double segmentStart,
	double segmentSweep
) :
	axisSwapMesh_{
		{
			{radius, segments, segmentStart, segmentSweep},
			{1.0, 0.0}, slices, sliceStart, sliceSweep
		},
		Axis::Y, Axis::Z, Axis::X
	}
{ }


