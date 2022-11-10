

#include "generator/AnyShape.hpp"


using namespace generator;


AnyShape::Base::~Base() { }


AnyShape::AnyShape(const AnyShape& that) :
	base_{that.base_->clone()}
{ }


AnyShape& AnyShape::operator=(const AnyShape& that) {
	base_ = that.base_->clone();
	return *this;
}


AnyGenerator<Edge> AnyShape::edges() const noexcept {
	return base_->edges();
}


AnyGenerator<ShapeVertex> AnyShape::vertices() const noexcept {
	return base_->vertices();
}


