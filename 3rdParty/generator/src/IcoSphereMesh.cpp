


#include "generator/IcoSphereMesh.hpp"


using namespace generator;


IcoSphereMesh::IcoSphereMesh(double radius, int segments) :
	spherifyMesh_{{1.0, segments}, radius, 1.0}
{ }


