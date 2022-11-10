

#ifndef GENERATOR_TORUSKNOTPATH_HPP
#define GENERATOR_TORUSKNOTPATH_HPP

#include "CircleShape.hpp"
#include "ExtrudeMesh.hpp"
#include "ParametricPath.hpp"

namespace generator {



/// A circle winding multiple times around.
/// @image html KnotPath.svg
class KnotPath
{
private:

	using Impl = ParametricPath;
	Impl parametricPath_;

public:

	/// @param q Times around a circle
	/// @param p Times around z axis
	/// @param segments Number of subdivisions along the path.
	KnotPath(
		int p = 2,
		int q = 3,
		int segments = 96
	);

	using Edges = typename Impl::Edges;

	Edges edges() const noexcept { return parametricPath_.edges(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return parametricPath_.vertices(); }

};

}



#endif
