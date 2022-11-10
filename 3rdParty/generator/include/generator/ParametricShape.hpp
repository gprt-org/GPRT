

#ifndef GENERATOR_PARAMETRICSHAPE_HPP
#define GENERATOR_PARAMETRICSHAPE_HPP

#include <functional>


#include "Edge.hpp"
#include "ShapeVertex.hpp"

namespace generator {


/// A shape with values evaluated using a callback function.
class ParametricShape {
public:

	class Edges {
	public:

		Edge generate() const;
		bool done() const noexcept;
		void next();

	private:

		Edges(const ParametricShape& shape);

		const ParametricShape* shape_;

		int i_;

	friend class ParametricShape;
	};

	class Vertices {
	public:

		ShapeVertex generate() const;
		bool done() const noexcept;
		void next();

	private:

		Vertices(const ParametricShape& shape);

		const ParametricShape* shape_;

		int i_;

	friend class ParametricShape;
	};


	/// @param eval A callback that returns a ShapeVertex for a given value.
	/// @param segments The number of segments along the shape.
	/// Should be >= 1. Zero yields an empty shape.
	explicit ParametricShape(
		std::function<ShapeVertex(double)> eval,
		int segments = 16
	) noexcept;

	Edges edges() const noexcept;

	Vertices vertices() const noexcept;

private:

	std::function<ShapeVertex(double)> eval_;

	int segments_;

	double delta_;

};


}

#endif
