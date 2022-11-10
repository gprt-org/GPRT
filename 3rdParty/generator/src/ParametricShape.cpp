


#include "generator/ParametricShape.hpp"


using namespace generator;


ParametricShape::Edges::Edges(const ParametricShape& shape) :
	shape_{&shape},
	i_{0}
{ }


Edge ParametricShape::Edges::generate() const {
	if (done()) throw std::out_of_range("Done!");

	return Edge{{i_, i_ + 1}};
}

bool ParametricShape::Edges::done() const noexcept {
	return i_ ==  shape_->segments_;
}

void ParametricShape::Edges::next() {
	if (done()) throw std::out_of_range("Done!");
	++i_;
}




ShapeVertex ParametricShape::Vertices::generate() const {
	if (done()) throw std::out_of_range("Done!");

	return shape_->eval_(i_ * shape_->delta_);
}



ParametricShape::Vertices::Vertices(const ParametricShape& shape) :
	shape_{&shape}, i_{0}
{ }

bool ParametricShape::Vertices::done() const noexcept {
	if (shape_->segments_ == 0) return true;
	return i_ == shape_->segments_ + 1;
}

void ParametricShape::Vertices::next() {
	if (done()) throw std::out_of_range("Done!");
	++i_;
}



ParametricShape::ParametricShape(
	std::function<ShapeVertex(double)> eval,
	int segments
) noexcept :
	eval_{std::move(eval)},
	segments_{segments},
	delta_{1.0 / segments}
{ }



ParametricShape::Edges ParametricShape::edges() const noexcept {
	return *this;
}

ParametricShape::Vertices ParametricShape::vertices() const noexcept {
	return *this;
}

