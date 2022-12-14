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

The `sharedCode.h <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/sharedCode.h>`_ 
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
To create our checkerboard test pattern, we'll write a small program that will 
run in parallel on our ray tracing device, where we'll use individual threads to 
generate our pixel colors. 

Shared Code
^^^^^^^^^^^
To begin, we'll define some structures that will be shared between our host system
and our ray tracing device, which we'll declare in our *sharedCode.h* file. For 
this sample, we'll need two colors--given as ``float3`` values--as well as a 
framebuffer--represented using a gprt::Buffer. 

.. literalinclude:: ../../../samples/s00-rayGenOnly/sharedCode.h
   :language: c++
   :lines: 23-33

One thing that might stand out right away is the use of ``alignas(16)``. In HLSL, 
types align themselves to avoid crossing 16 byte boundaries; however, this isn't
necessarily the case in C++. Therefore, we use this alignas macro to guarantee
that the struct follows HLSL alignment rules for both C++ and HLSL so that the
struct can be shared between the two devices. 

Although it might seem intimidating at first, the rules aren't that bad to follow. 
If a type is a float3 or int3, use ``alignas(16)`` rather than ``alignas(12)``. 
Otherwise, alignas should take as input the number of bytes referenced by the 
following object's type.

Device Code
^^^^^^^^^^^

Next, we'll create a *Ray Generation Program* that will run in parallel on the device. 
As it's name suggests, the purpose of this kernel is to generate rays that we will trace
into the world. However, just to keep things simple, we won't be tracing any 
rays in this example. 


.. literalinclude:: ../../../samples/s00-rayGenOnly/deviceCode.hlsl
   :language: hlsl
   :lines: 23-43

In the code above, we declare our ray generation program using the macro 
``GPRT_RAYGEN_PROGRAM``. This macro takes as input the *name* of our program, followed
by the type and name of this kernel's "shader record". In GPRT, every program 
running on the device receives a shader record. These shader records serve as 
the parameters that are passed to this program when it executes on the device.

This function runs the same code in parallel over many different threads. We
read the thread ID using ``DispatchRaysIndex``, as well as how many threads 
were launched using ``DispatchRaysDimensions``. 

Using the first thread, we use printf to print out a message to our console. Note,
for Vulkan GPRT backends, this requires that validation layers be enabled. This 
can be done using the Vulkan Configurator tool, by using the "Debug Printf Preset".

Following that, we use the thread ID to determine which checker type are pixel
lies within. Finally, we use ``gprt::store`` to store our color into our framebuffer
at the given location.

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