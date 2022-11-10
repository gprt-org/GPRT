


#include "generator/EmptyPath.hpp"


using namespace generator;



Edge EmptyPath::Edges::generate() const {
	throw std::out_of_range("Called generate on an EmptyPath!");
}

bool EmptyPath::Edges::done() const noexcept {
	return true;
}

void EmptyPath::Edges::next() {
	throw std::out_of_range("Called next on an EmptyPath!");
}



EmptyPath::Edges::Edges() {}




PathVertex EmptyPath::Vertices::generate() const {
	throw std::out_of_range("Called generate on an EmptyPath!");
}

bool EmptyPath::Vertices::done() const noexcept {
	return true;
}

void EmptyPath::Vertices::next() {
	throw std::out_of_range("Called next on an EmptyPath!");
}


EmptyPath::Vertices::Vertices() { }


EmptyPath::EmptyPath() {}

EmptyPath::Edges EmptyPath::edges() const noexcept {
	return {};
}

EmptyPath::Vertices EmptyPath::vertices() const noexcept {
	return {};
}



