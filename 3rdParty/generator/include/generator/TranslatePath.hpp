

#ifndef GENERATOR_TRANSLATEPATH_HPP
#define GENERATOR_TRANSLATEPATH_HPP

#include "math.hpp"

#include "TransformPath.hpp"

namespace generator {


/// Translates the position of each vertex by given amount.
template <typename Path>
class TranslatePath
{
private:

	using Impl = TransformPath<Path>;
	Impl transformPath_;

public:

	/// @param path Source data path.
	/// @param delta Amount to increment each vertex position.
	TranslatePath(Path path, const gml::dvec3& delta) :
		transformPath_{
			std::move(path),
			[delta] (PathVertex& value) { value.position += delta;	}
		}
	{ }

	using Edges = typename Impl::Edges;

	Edges edges() const noexcept { return transformPath_.edges(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return transformPath_.vertices(); }

};


template <typename Path>
TranslatePath<Path> translatePath(Path path, const gml::dvec3& delta) {
	return TranslatePath<Path>{std::move(path), delta};
}

}

#endif
