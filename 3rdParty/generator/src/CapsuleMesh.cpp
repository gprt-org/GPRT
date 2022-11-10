


#include "generator/CapsuleMesh.hpp"


using namespace generator;


CapsuleMesh::CapsuleMesh(
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
		{
			{radius, slices, rings, start, sweep, 0.0, gml::radians(90.0)},
			{0.0, 0.0, size}
		},
		{
			{
				radius, slices, rings, start, sweep,
				gml::radians(90.0), gml::radians(90.0)
			},
			{0.0, 0.0, -size}
		}
	}
{

}


