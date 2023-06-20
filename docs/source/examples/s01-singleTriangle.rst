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

After running this example, you should see an image like this appear on your 
screen and be saved as s01-singleTriangle.png in the same directory as the sample's 
executable:

.. image:: ../images/s01-singleTriangle.png

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

Acceleration Structures
^^^^^^^^^^^^^^^^^^^^^^^

In a real-world application, 3D models are composed of many such triangles, so we not only want to know *if* our ray hit *a* triangle, but also *what* triangle our ray hit. 
This process also needs to be fast and perform well for really large meshes, so we want to test our ray against only a small subset of the triangles in our model.

To do this, ray tracing frameworks construct a hierarchy over the triangles---called an *Acceleration Structure*---in order to skip testing intersections against large collections of triangles that we know won't be hit. 
Each level in this hierarchy groups more and more triangles together into boxes, and if our ray does't hit the box, then we know our ray won't hit any of the triangles or sub-boxes inside.

.. figure:: ../images/illustrations/acceleration_structures.svg
   :class: with-border

   On the left, we have a cube made of triangles. 
   In the middle, we could test our ray against each triangle of the cube, but that would be expensive. 
   Instead, on the right we use acceleration structures to skip unnecessary intersection tests.

In GPRT, we let you tap into the real-time tree building features available in modern low level ray tracing frameworks. 
These features let you build these hierarchies on the GPU over hundreds of thousands of triangles in only a millisecond or two, so if your mesh animates over time, you don't need to worry about that slowing down your app. 

The Ray Tracing Pipeline
^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: ../images/illustrations/rt_pipeline.svg
   :class: with-border

   Todo, caption this

Rendering a Single Triangle
---------------------------

In this little example, we'll build a tree over just one triangle for now. 

Shared Code
^^^^^^^^^^^
To begin, we'll again define some structures that will be shared between our host system
and our ray tracing device, which we'll declare in our *sharedCode.h* file. 

.. literalinclude:: ../../../samples/s01-singleTriangle/sharedCode.h
   :language: c++
   :lines: 23-46


Device Code
^^^^^^^^^^^
todo

Host Code
^^^^^^^^^

The first step is to create some buffers that define our geometry. 
The *vertices* represent the corners of the triangles that define our mesh in 3D space.
Then *indices* connect these vertices together. So here, since we only have one triangle, we will connect vertices 0, 1, and 2 together.

.. literalinclude:: ../../../samples/s01-singleTriangle/hostCode.cpp
   :language: c++
   :lines: 43-54

Then, we create a buffer on our ray tracing device, uploading this geometry data.

.. literalinclude:: ../../../samples/s01-singleTriangle/hostCode.cpp
   :language: c++
   :lines: 116-119

.. Then, this traversal process is handled for us by our RT cores. 
.. To make this traversal process fast, some parts of the traversal process are handled for us, and then other parts of this traversal process we have control over. 
.. This pipeline is called the Ray Tracing Pipeline.







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
