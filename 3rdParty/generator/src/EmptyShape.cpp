


#include "generator/EmptyShape.hpp"

#include <stdexcept>

using namespace generator;


EmptyShape::Edges::Edges() { }


Edge EmptyShape::Edges::generate() const {
	throw std::out_of_range("Called generate on an EmptyShape!");
}

bool EmptyShape::Edges::done() const noexcept {
	return true;
}

void EmptyShape::Edges::next() {
	throw std::out_of_range("Called next on an EmptyShape!");
}




EmptyShape::Vertices::Vertices() { }


ShapeVertex EmptyShape::Vertices::generate() const {
	throw std::out_of_range("Called generate on an EmptyShape!");
}

bool EmptyShape::Vertices::done() const noexcept {
	return true;
}

void EmptyShape::Vertices::next() {
	throw std::out_of_range("Called next on an EmptyShape!");
}



EmptyShape::Edges EmptyShape::edges() const noexcept {
	return {};
}

EmptyShape::Vertices EmptyShape::vertices() const noexcept {
	return {};
}


