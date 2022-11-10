


#include "generator/TorusMesh.hpp"


using namespace generator;



TorusMesh::TorusMesh(
	double minor,
	double major,
	int slices,
	int segments,
	double minorStart,
	double minorSweep,
	double majorStart,
	double majorSweep
) :
	axisSwapMesh_{
		{
			{
				{minor, slices, minorStart+gml::radians(90.0), minorSweep},
				{0.0, major}
			},
			{1.0, 0.0}, segments, majorStart, majorSweep
		},
		Axis::Y, Axis::Z, Axis::X
	}
{ }

