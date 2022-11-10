

#ifndef GENERATOR_EMPTYMESH_HPP
#define GENERATOR_EMPTYMESH_HPP

#include "Iterator.hpp"
#include "MeshVertex.hpp"
#include "Triangle.hpp"


namespace generator {


/// Empty Mesh with zero vertices and triangles.
class EmptyMesh {
public:

	class Triangles {
	public:
		Triangle generate() const;
		bool done() const noexcept;
		void next();
	private:

		Triangles();

	friend class EmptyMesh;
	};

	class Vertices {
	public:

		MeshVertex generate() const;
		bool done() const noexcept;
		void next();

	private:

		Vertices();

	friend class EmptyMesh;
	};

	EmptyMesh();

	Triangles triangles() const noexcept;

	Vertices vertices() const noexcept;

};


}

#endif
