S01 Single Triangle
================================
`Corresponding sample code can be found here <https://github.com/gprt-org/GPRT/tree/master/samples/s01-singleTriangle>`_.

An example demonstrating triangle geometry

Introduction
------------

Objective of this Sample
^^^^^^^^^^^^^^^^^^^^^^^^

Up until this point, we haven't actually used GPRT to trace any rays...
So in this example, we'll build off the previous one and use GPRT to trace rays through the lens of a pinhole camera. 
Then, we'll test for intersection against a single "hello world triangle", coloring the triangle using something called "barycentric coordinates". 
To make this process faster and easier, we'll utilize the RT cores in our ray tracing device.
Finally, we'll make this example interactive, allowing you to click and drag on the window to manipulate the camera viewing the triangle.


Background
----------

What is a Ray?
^^^^^^^^^^^^^^

Since GPRT is all about tracing rays, it's worth talking about what exactly a ray is. 
To start, tays in GPRT are represented using the following structure:

.. code-block:: hlsl

  struct RayDesc
  {
      float3 Origin;
      float  TMin;
      float3 Direction;
      float  TMax;
  };

All rays have an origin, representing where the ray starts in 3D space. 
Then, rays have a direction, telling us what orientation in 3D space the ray is traveling.
We can use this definition of a ray to define a function where we give a distance *T* along the ray, and that function returns a position along that ray.

.. code-block:: hlsl

  float3 getPositionAlongRay(RayDesc ray, float T) 
  {
    return ray.Origin + T * ray.Direction;
  }
  
Rays also have a "TMin" and a "TMax". You can think of these properties as a minimum and a maximum distance "T" that a position can be along a ray.

.. code-block:: hlsl

  float3 getPositionAlongRay(RayDesc ray, float T) 
  {
    T = clamp(T, ray.TMin, ray.TMax); // this is new!
    return ray.Origin + T * ray.Direction;
  }

Ray Primitive Intersectors
^^^^^^^^^^^^^^^^^^^^^^^^^^

As these rays travel through the world, they will hit surfaces. 
The math for where these rays intersect surfaces can be a bit complicated, but fortunately our RT cores include a built-in ray-triangle intersector, so for now we'll be using that function.

.. code-block:: hlsl

  bool rayTriangleIntersect(RayDesc ray, Triangle triangle, out hitDistance, out barycentrics);

Ray intersector functions usually look something like the above. 
We give the function our ray, as well as the primitive (in our case, a triangle). 
The function returns True if the ray hits the primitive.
If the primitive is hit, the function also returns the hit distance *T*.

Finally, for triangle intersectors, we're given something called the "barycentric coordinates" of the intersection position.
These barycentric coordinates allow us to interpolate per-vertex data on our triangle primitive. 
For example, we might want to compute the position for where our ray hit our triangle: 

.. code-block:: hlsl

  float3 getHitPosition(float2 barycentrics, Triangle triangle) 
  {
    return   triangle.v1 * barycentrics.x
           + triangle.v2 * barycentrics.y
           + triangle.v3 * (-1.0 - (barycentrics.x + barycentrics.y);
  }
  
But those same barycentrics might be used to interpolate per-vertex colors, texture coordinates, surface normals, and so on.

This process is called *Ray Tracing*.

The Ray Tracing Pipeline
^^^^^^^^^^^^^^^^^^^^^^^^

So now, say we have more than one triangle, and we want to know which triangles our ray hit.
We could do a linear traversal over all the triangles in our scene, checking potential ray triangle intersections for each triangle.
But that quickly become prohibitively expensive, since a scene that we might want to trace rays through might contain thousands if not millions of these triangles. 
And we also might have millions, if not billions of rays that we want to trace all at the same time.
So instead, we construct a hierarchical data structure, called an *Acceleration Structure*, in order to skip testing intersections against large collections of triangles. 

In GPRT, we provide real-time acceleration structure construction methods that scale to millions of triangles. 
Then, this traversal process is handled for us by our RT cores. 
To make this traversal process fast, some parts of the traversal process are handled for us, and then other parts of this traversal process we have control over. 
This pipeline is called the Ray Tracing Pipeline.



Rendering a Single Triangle
---------------------------



.. I. Introduction 
.. A. Purpose of the example 
.. B. Overview of general-purpose ray tracing toolkit

.. II. Rendering a Single Triangle
.. A. Setting up the Scene 
.. 1. Create the triangle 
.. 2. Set the camera position 
.. 3. Set the materials 

.. B. Ray Tracing 
.. 1. Trace the rays 
.. 2. Calculate the color of each triangle 

.. C. Outputting the Result 
.. 1. Save the image 
.. 2. Display the image 

.. III. Conclusion 
.. A. Summary of the example 
.. B. Benefits of using the general-purpose ray tracing toolkit



.. I. Introduction 
.. A. Definition of Ray Tracing 
.. B. Overview of RTX Ray Tracing 

.. II. How RTX Ray Tracing Works 
.. A. Step 1: Primitive Assembly 
.. B. Step 2: Ray Generation 
.. C. Step 3: Ray Tracing 
.. D. Step 4: Shading 
.. E. Step 5: Rasterization 

.. III. Benefits of RTX Ray Tracing 
.. A. Improved Visual Quality 
.. B. Increased Performance 
.. C. Reduced CPU Load 

.. IV. Conclusion 
.. A. Summary of RTX Ray Tracing 
.. B. Benefits of Ray Tracing 
