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
