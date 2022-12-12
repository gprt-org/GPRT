%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  GPRT Reference Manual
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Welcome to the manual for `GPRT <https://github.com/gprt-org/GPRT>`__, the general purpose raytracing toolkit.

Our goal is to make high performance GPU ray tracing hardware more accessible to 
a broader developer audience, whether that be on NVIDIA, AMD, or Intel ARC. 
With this toolkit, we help make your ray tracing development fast and easy 
by providing GPU-parallel acceleration structures, shader binding table 
management, and RT pipeline creation. We also provide a collection of examples 
*here* that should help you get started on all your ray tracing expeditions!


.. Getting Started
.. ===============

.. .. container:: tocdescr

..    :doc:`/hostapi`

.. .. container:: tocdescr

..    :doc:`/deviceapi`

.. .. container:: global-index-toc

..    .. toctree::
..       :caption: Getting Started
..       :maxdepth: 1

..       hostapi.rst
..       deviceapi.rst



Examples
========
.. container:: tocdescr

      .. container:: descr

         .. figure:: ./images/s00-rayGenOnly.png
            :target: examples/s00-rayGenOnly.html

         :doc:`/interface/index`
            A bare minimum GPRT example generating a checkerboard

      .. container:: descr

         .. figure:: ./images/s01-singleTriangle.png
            :target: examples/s01-singleTriangle.html

         :doc:`/editors/index`
            An example demonstrating triangle geometry

      .. Note, we'll add in the below as more samples are merged into master

      .. .. container:: descr

      ..    .. figure:: ./images/s02-instances.png
      ..       :target: examples/s02-instances.html

      ..    :doc:`/editors/index`
      ..       Multiple instances of the same triangle geometry

      .. .. container:: descr

      ..    .. figure:: ./images/s03-simpleAABB.png
      ..       :target: examples/s03-simpleAABB.html

      ..    :doc:`/editors/index`
      ..       An example demonstrating AABB geometry

      .. .. container:: descr

      ..    .. figure:: ./images/s04-computeAABBs.png
      ..       :target: examples/s04-computeAABBs.html

      ..    :doc:`/editors/index`
      ..       How to use a compute shader to generate AABBs

      .. .. container:: descr

      ..    .. figure:: ./images/s05-computeVertex.png
      ..       :target: examples/s05-computeVertex.html

      ..    :doc:`/editors/index`
      ..       How to use a compute shader to manipulate triangle vertices
      
      .. .. container:: descr

      ..    .. figure:: ./images/s06-computeTransform.png
      ..       :target: examples/s06-computeTransform.html

      ..    :doc:`/editors/index`
      ..       How to use a compute shader to manipulate instance transforms
      
      .. .. container:: descr

      ..    .. figure:: ./images/s07-multipleGeometry.png
      ..       :target: examples/s07-multipleGeometry.html

      ..    :doc:`/editors/index`
      ..       Here we combine multiple geometries into the same bottom level tree

      .. .. container:: descr

      ..    .. figure:: ./images/s08-multipleBLAS.png
      ..       :target: examples/s08-multipleBLAS.html

      ..    :doc:`/editors/index`
      ..       Shows how bottom level trees of different types can be combined

      .. .. container:: descr

      ..    .. figure:: ./images/s09-multipleTLAS.png
      ..       :target: examples/s09-multipleTLAS.html

      ..    :doc:`/editors/index`
      ..       Demonstrates multiple top level trees in the same program

Get Involved
============
GPRT is an open source project, maintained by volunteers from all over the 
place.

Please consider joining our efforts, and check out our *contributor guide here*!

