S00 Ray Gen Only
================================
`Corresponding sample code can be found here <https://github.com/gprt-org/GPRT/tree/master/samples/s00-rayGenOnly>`_.

Introduction
------------

Overview of the General Purpose Raytracing Toolkit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The General Purpose Raytracing Toolkit (GPRT) is a powerful and versatile open 
source raytracing library designed to enable developers to quickly and easily 
prototype ideas involving all things high-performance ray tracing. The API is 
written using a combination of C++ and HLSL to support a wide range of features, 
including ray-object intersection, high performance tree construction, parallel 
compute kernels, and more. GPRT is also optimized for NVIDIA, AMD and Intel Arc 
architectures (with support for CPU architectures coming soon), making it 
suitable for use in a variety of applications. With its comprehensive feature 
set and flexible architecture, we believe that GPRT is an ideal choice for 
developers looking to prototype their ray tracing ideas to create stunning 3D 
graphics and highly efficient simulation codes.


Purpose of Checkerboard Test Pattern
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In this first example, we will use GPRT to create an image of a simple 
checkerboard pattern. Traditionally speaking, computer graphics libraries will 
generate a checkerboard image as a sort-of diagnostic test, to reveal any 
possible issues with a display's resolution, color accuracy, or contrast. But 
for our purposes, a checkerboard also just so happens to be a very easy pattern 
to generate, and allows us to get something up and running before diving into
more advanced concepts like acceleration structures, geometry, or ray tracing 
pipelines.

After running this example, you should see an image like this appear on your 
screen and be saved as s00-rayGenOnly.png in the same directory as the sample's 
executable:

.. image:: ../images/s00-rayGenOnly.png

Setting up a GPRT Sample
------------------------

File Structure and Purpose
^^^^^^^^^^^^^^^^^^^^^^^^^^
All of the samples in this repo follow a very similar pattern. In the 
`s00-rayGenOnly <https://github.com/gprt-org/GPRT/tree/master/samples/s00-rayGenOnly>`_
directory, we have four files: CMakeLists.txt, deviceCode.h, deviceCode.hlsl, 
and hostCode.cpp. 

The `CMakeLists.txt <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/CMakeLists.txt>`_ 
file tells our system how to compile our code into an executable or library. 

The `deviceCode.h <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/deviceCode.h>`_ 
file defines common structures shared between our raytracing device and our host 
system--for example, between our raytracing GPU and our CPU. The code in this 
file needs to be carefully written to compile with both HLSL *and* C++. 

The `deviceCode.hlsl <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/deviceCode.hlsl>`_ 
file defines all programs we would like to run in parallel on our raytracing 
device. You might also here these programs called *shaders* or *kernels*. These 
kernels are written using the HLSL programming language. 

Then finally, the `hostCode.cpp <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/hostCode.cpp>`_ 
file defines our main function on the CPU that we will use to orchestrate what 
kernels to run--or *launch*--on the device and how. This might include what 
parameters we pass to these kernels, how many threads to launch, and so on.

Configuring the Build System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Before we can use GPRT, we need to configure how to compile our code together
into an executable. Our samples do this by using *CMake*. In a typical workflow,
this CMake file would be configured as follows:

1. use the ``add_subdirectory`` command to add the gprt repository root 
directory.

2. call `embed_devicecode`, passing in our *deviceCode.hlsl* as the source file. 
Behind the scenes, this embed macro will compile our deviceCode.hlsl file into 
multiple different SPIR-V targets that can use to run our code on the GPU. 
(Note, this is subject to change as we add support for CPU backends).
This SPIR-V is then embedded into a .cpp file matching the ``OUTPUT_TARGET`` 
name---in our case, this will appear as *s00_deviceCode.cpp* in our 
build/samples/s00-rayGenOnly/ folder. 

3. call `add_executable`, passing in our *hostCode.cpp* file as the source.

4. Then finally, we call `target_link_libraries`, linking our embedded devicecode
target as well as the gprt::gprt target to our executable.

`For another minimal example, check out the CMakeLists.txt in this project
<https://github.com/gprt-org/h5m-reader/blob/main/CMakeLists.txt>`_

Creating our Checkerboard Test Pattern
--------------------------------------

.. literalinclude:: ../../../samples/s00-rayGenOnly/deviceCode.hlsl
   :language: hlsl
   :emphasize-lines: 3,6-8
   :linenos:

Shared Structures
^^^^^^^^^^^^^^^^^

Device Code
^^^^^^^^^^^

Host Code
^^^^^^^^^

Conclusion
----------

Summary of Process
^^^^^^^^^^^^^^^^^^

Benefits of Using Framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^

What's Next?
^^^^^^^^^^^^