S00 Ray Gen Only
================================
`Corresponding sample code can be found here <https://github.com/gprt-org/GPRT/tree/master/samples/s00-rayGenOnly>`_.

Introduction
------------

.. Overview of the General Purpose Raytracing Toolkit
.. ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. The General Purpose Raytracing Toolkit (GPRT) is a powerful and versatile open 
.. source raytracing library designed to enable developers to quickly and easily 
.. prototype ideas involving all things high-performance ray tracing. The API is 
.. written using a combination of C++ and HLSL to support a wide range of features, 
.. including ray-object intersection, high performance tree construction, parallel 
.. compute kernels, and more. GPRT is also optimized for NVIDIA, AMD and Intel Arc 
.. architectures (with support for CPU architectures coming soon), making it 
.. suitable for use in a variety of applications. With its comprehensive feature 
.. set and flexible architecture, we believe that GPRT is an ideal choice for 
.. developers looking to prototype their ray tracing ideas to create stunning 3D 
.. graphics and highly efficient simulation codes.

Welcome to the GPRT samples course! GPRT is an open source raytracing library 
that uses a combination of C++ and HLSL to allow developers to quickly prototype 
ideas involving high-performance ray tracing. It is optimized for NVIDIA, AMD and 
Intel Arc architectures, and support for CPU architectures is coming soon. By 
following these examples, you'll be able to leverage GPRT's comprehensive
feature set and create your own high performance ray tracing applications.

.. It offers a comprehensive feature set and is suitable 
.. for a variety of applications.

Purpose of Checkerboard Test Pattern
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. In this first example, we will use GPRT to create an image of a simple 
.. checkerboard pattern. Traditionally speaking, computer graphics libraries will 
.. generate a checkerboard image as a sort-of diagnostic test, to reveal any 
.. possible issues with a display's resolution, color accuracy, or contrast. But 
.. for our purposes, a checkerboard also just so happens to be a very easy pattern 
.. to generate, and allows us to get something up and running before diving into
.. more advanced concepts like acceleration structures, geometry, or ray tracing 
.. pipelines.

In this first example, we'll use GPRT to render a classic checkerboard pattern.
Traditionally, checkerboards serve as a good diagnostic test to reveal any 
possible issues with a display; but for us, it serves as an easy pattern to generate 
and allows users to get something running before moving onto more advanced concepts.

After running this example, you should see an image like this appear on your 
screen and be saved as s00-rayGenOnly.png in the same directory as the sample's 
executable:

.. image:: ../images/s00-rayGenOnly.png

Setting up a GPRT Sample
------------------------

File Structure
^^^^^^^^^^^^^^
All of the samples in this repo follow a very similar file structure. In the 
`s00-rayGenOnly <https://github.com/gprt-org/GPRT/tree/master/samples/s00-rayGenOnly>`_
directory, we have four files: CMakeLists.txt, sharedCode.h, deviceCode.hlsl, 
and hostCode.cpp. 

`CMakeLists.txt <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/CMakeLists.txt>`_ 
tells our CMake build system how to compile our code into an executable or library. 

`sharedCode.h <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/sharedCode.h>`_ 
defines common data structures shared between our ray tracing device and our host 
system. The code in this file needs to be carefully written to compile with both HLSL *and* C++. 

`deviceCode.hlsl <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/deviceCode.hlsl>`_ 
defines all programs we would like to run in parallel on our ray tracing 
device, written using the HLSL programming language. You might also see these 
programs called *shaders* or *kernels*. 

`hostCode.cpp <https://github.com/gprt-org/GPRT/blob/master/samples/s00-rayGenOnly/hostCode.cpp>`_ 
defines our main function on the CPU that we will use to orchestrate what 
kernels to run on the device and how. This might include what 
parameters we pass to these kernels, how many threads to launch, and so on.

Configuring the Build System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Before we can use GPRT, we need to configure how to compile our code together
into an executable. Our samples do this by using the *CMakeLists.txt* files. In 
a typical workflow, this CMake file would be configured as follows:

.. literalinclude:: ../../../samples/s00-rayGenOnly/CMakeLists.txt
   :language: cmake
   :lines: 23-36


The `embed_devicecode` macro compiles our *deviceCode.hlsl* into 
assembly we can run on our raytracing device. This assembly is then 
embedded into a .cpp file matching the ``OUTPUT_TARGET`` name. 

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
and our ray tracing device, which we'll declare in our *sharedCode.h* file. 

.. literalinclude:: ../../../samples/s00-rayGenOnly/sharedCode.h
   :language: c++
   :lines: 23-46

.. One thing that might stand out right away is the use of ``alignas(16)``. In HLSL, 
.. types align themselves to avoid crossing 16 byte boundaries; however, this isn't
.. necessarily the case in C++. Therefore, we use this alignas macro to guarantee
.. that the struct follows HLSL alignment rules for both C++ and HLSL so that the
.. struct can be shared between the two devices. 
.. Although it might seem intimidating at first, the rules aren't that bad to follow. 
.. If a type is a float3 or int3, use ``alignas(16)`` rather than ``alignas(12)``. 
.. Otherwise, alignas should take as input the number of bytes referenced by the 
.. following object's type.

.. The use of alignas is necessary to ensure that HLSL and C++ structs are aligned 
.. correctly and can be shared between devices. This is because C++ uses 4/8 byte 
.. alignment for 32/64bit applications, while HLSL uses 16 byte alignment. HLSL
.. also prevents variables from crossing 16-byte boundaries, and inserts padding
.. to structs to make sure this never happens. It may be intimidating, but the rules
.. for alignas are simple: use alignas(16) for float3, int3, gprt::Buffer and gprt::Accel 
.. types, and otherwise alignas should take the number of bytes of the corresponding type.

The use of alignas is necessary to ensure that HLSL and C++ structs can be shared between devices. 
HLSL requires 16 byte alignment, and prevents variables from crossing 16-byte boundaries by 
inserting padding to structs. To ensure correct alignment, alignas(16) should be used for float3, 
int3, gprt::Buffer and gprt::Accel types, and otherwise alignas should take the number of bytes 
of the corresponding type.

Device Code
^^^^^^^^^^^

Next, we'll create a *Ray Generation Program* that will run in parallel on the device. 
Normally, we use these kernels to generate rays to trace into the world. However, in this 
example we'll just be creating a checkerboard background. 


.. literalinclude:: ../../../samples/s00-rayGenOnly/deviceCode.hlsl
   :language: hlsl
   :lines: 23-43

In the code above, we declare our ray generation program using the macro 
``GPRT_RAYGEN_PROGRAM``. 
This macro is defined in `gprt_device.h <https://github.com/gprt-org/GPRT/blob/master/gprt/gprt_device.h>`_,
and it's purpose is to allow us to write multiple kernels all in the same HLSL
document. 

.. This macro takes as input the *name* of our program, followed
.. by the type and name of this kernel's "shader record".
This macro takes in the name of the kernel and the type and name of its shader record.
In GPRT, every kernel receives a shader record, which serve as the parameters 
to that kernel. 

This kernel runs the same code in parallel over many different threads. We
read the thread ID using ``DispatchRaysIndex``, as well as how many threads 
were launched using ``DispatchRaysDimensions``. 
We use the thread ID to determine which checker type are pixel
lies within, and use ``gprt::store`` to store our color into our framebuffer
at the given location.

Just like on the CPU, we can use printf to print out helpful debug messages. Note,
for Vulkan GPRT backends, this requires that validation layers be enabled, which 
can be done using the Vulkan Configurator tool by using the "Debug Printf Preset".

Host Code
^^^^^^^^^

All that's left is to write our host side code. 

We begin by requesting a window and creating a ``GPRTContext``: 

.. that we will use to show our checkerboard pattern. 
.. Then, we create our ``GPRTContext``, which under the hood initializes our underlying
.. ray tracing framework and selects the devices we'll run our kernel on.

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 58-65

After that, we create a ``GPRTModule``, which acts as a container that will hold our
compiled GPU program. 

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 71

Creating our Raytracing Pipeline
""""""""""""""""""""""""""""""""
Next, we'll setup our ray tracing pipeline. First, we provide a list of variables 
referenced in our shared ``RayGenData`` struct. Each declaration includes a *name*,
a type, and an offset. When we're done, we end this list with a "null" declaration, 
similar to a null-terminated string.

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 73-79

.. for the declaration--which we'll later use 
.. to assign values to that parameter--as well as the type of the declaration and the 
.. offset of that declaration in the shared structure. 

With this list, we can create a handle to our ray generation program: 

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 80-84

Finally, we can build our ray tracing pipeline, generating all the underlying resources required to envoke 
ray generation kernels:

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 85-86

Setting Parameters
""""""""""""""""""
Next, we can pass data back and forth between the ray tracing device in two ways: through
constant uniform values, and through buffers. Uniform values are like single global values, 
like the two ``float3`` color values used by our checkerboard. Buffers on the other hand are
allocated regions of memory which can be written to and read from by both the host and
the device.

To create buffers, we can use either ``gprtDeviceBufferCreate``,  or ``gprtHostBufferCreate``. 
As their names suggest, creating a buffer using ``gprtDeviceBufferCreate`` will allocate a 
buffer that resides on our ray tracing device, while using ``gprtHostBufferCreate`` will 
allocate a buffer that resides on our host system. For our frame buffer, we will use a 
device buffer.

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 92-98

Once the values to our ray generation shader record are ready, we can assign them to our
ray generation program using ``gprtRayGetSet`` commands. Once all parameters are set, we 
upload the values of these parameters to our device by calling ``gprtBuildShaderBindingTable``. 

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 104-109

Launching our Program
"""""""""""""""""""""
At this point, we can launch our ray generation program to create our checkerboar image. 
To do this, we call ``gprtRayGenLaunch2D``, giving our ray generation program to use as 
well as the number of threads we would like to execute concurrently. Then, to present 
our framebuffer to the screen, we call ``gprtBufferPresent``. We do this in a loop until 
the window's  "X" button has been clicked, at which point we save the framebuffer to an 
image using ``gprtBufferSaveImage``.

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 115-129

Cleaning Up
"""""""""""
When our program completes, we need to destroy all the objects we created. The 
order that these objects are destroyed is important, because some objects depend
on others. In general, we destroy our objects the reverse order that they were 
made. 

.. literalinclude:: ../../../samples/s00-rayGenOnly/hostCode.cpp
   :language: c++
   :lines: 133-138

Conclusion
----------
If you've followed along so far, congrats! Although this example is relatively simple, 
we've covered a lot of the key concepts you need to know in order to get up and running 
tracing rays. 

In the next example, we'll create a single triangle, and trace rays to intersect that triangle.
We'll be able to manipulate this triangle as well using some very simple camera math.
