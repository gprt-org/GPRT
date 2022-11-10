

#ifndef GENERATOR_CIRCLESHAPE_HPP
#define GENERATOR_CIRCLESHAPE_HPP

#include "ParametricShape.hpp"

namespace generator {


/// A circle centered at origin.
/// @image html CircleShape.svg
class CircleShape
{
private:

	using Impl = ParametricShape;
	Impl parametricShape_;

public:

	/// @param radius Radius of the circle
	/// @param segments Number of subdivisions around the circle.
	/// @param start Counterclockwise angle relative to x-axis.
	/// @param sweep Counterclockwise angle.
	CircleShape(
		double radius = 1.0,
		int segments = 32,
		double start = 0.0,
		double sweep = gml::radians(360.0)
	);

	using Edges = typename Impl::Edges;

	Edges edges() const noexcept { return parametricShape_.edges(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return parametricShape_.vertices(); }

};

}

#endif
