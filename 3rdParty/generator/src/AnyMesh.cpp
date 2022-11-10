


#include "generator/AnyMesh.hpp"


using namespace generator;



AnyMesh::Base::~Base() { }


AnyMesh::AnyMesh(const AnyMesh& that) :
	base_{that.base_->clone()}
{ }


AnyMesh& AnyMesh::operator=(const AnyMesh& that) {
	base_ = that.base_->clone();
	return *this;
}


AnyGenerator<Triangle> AnyMesh::triangles() const noexcept {
	return base_->triangles();
}


AnyGenerator<MeshVertex> AnyMesh::vertices() const noexcept {
	return base_->vertices();
}

