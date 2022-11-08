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
            :target: examples/rayGenOnly.html

         :doc:`/interface/index`
            A bare minimum GPRT example generating a checkerboard

      .. container:: descr

         .. figure:: ./images/s01-singleTriangle.png
            :target: examples/singleTriangle.html

         :doc:`/editors/index`
            An example demonstrating triangle geometry

      .. container:: descr

         .. figure:: ./images/s02-instances.png
            :target: examples/instances.html

         :doc:`/editors/index`
            Multiple instances of the same triangle geometry

      .. container:: descr

         .. figure:: ./images/s03-simpleAABB.png
            :target: examples/simpleAABB.html

         :doc:`/editors/index`
            An example demonstrating AABB geometry

      .. container:: descr

         .. figure:: ./images/s04-computeAABBs.png
            :target: examples/computeAABBs.html

         :doc:`/editors/index`
            How to use a compute shader to generate AABBs

      .. container:: descr

         .. figure:: ./images/s05-computeVertex.png
            :target: examples/computeVertex.html

         :doc:`/editors/index`
            How to use a compute shader to manipulate triangle vertices
..       .. container:: descr

..          .. figure:: /images/index_scene.jpg
..             :target: scene_layout/index.html

..          :doc:`/scene_layout/index`
..             Objects and their organization into scenes, view layers and collections.

..       .. container:: descr

..          .. figure:: /images/index_modeling.jpg
..             :target: modeling/index.html

..          :doc:`/modeling/index`
..             Meshes, curves, metaballs, text, modeling tools, and modifiers.

..       .. container:: descr

..          .. figure:: /images/index_painting.jpg
..             :target: sculpt_paint/index.html

..          :doc:`/sculpt_paint/index`
..             Sculpting, texture painting and vertex painting.

..       .. container:: descr

..          .. figure:: /images/index_grease-pencil.jpg
..             :target: grease_pencil/index.html

..          :doc:`/grease_pencil/index`
..             2D drawing and animation with Grease Pencil.

..       .. container:: descr

..          .. figure:: /images/index_animation.jpg
..             :target: animation/index.html

..          :doc:`/animation/index`
..             Keyframes, drivers, constraints, armatures and shape keys.

..       .. container:: descr

..          .. figure:: /images/index_physics.jpg
..             :target: physics/index.html

..          :doc:`/physics/index`
..             Physics simulations, particle systems and dynamic paint.

..       .. container:: descr

..          .. figure:: /images/index_render.jpg
..             :target: render/index.html

..          :doc:`/render/index`
..             Rendering and shading with Eevee, Cycles and Freestyle.

..       .. container:: descr

..          .. figure:: /images/index_compositing.jpg
..             :target: compositing/index.html

..          :doc:`/compositing/index`
..             Post-processing with the compositing nodes.

..       .. container:: descr

..          .. figure:: /images/index_movie-clip.jpg
..             :target: movie_clip/index.html

..          :doc:`/movie_clip/index`
..             Video motion tracking & masking.

..       .. container:: descr

..          .. figure:: /images/index_sequencer.jpg
..             :target: video_editing/index.html

..          :doc:`/video_editing/index`
..             Video editing with the sequencer.

..       .. container:: descr

..          :doc:`/files/index`
..             Data-block management and the structure of blend-files.

..       .. container:: descr

..          :doc:`/addons/index`
..             Additional functionality available as add-ons.

..       .. container:: descr

..          :doc:`/advanced/index`
..             Python scripting, how to write add-ons and a reference for command-line arguments.

..       .. container:: descr

..          :doc:`/troubleshooting/index`
..             Solving crashes, graphics issues and Python errors, recovering data and reporting bugs.

..       .. container:: descr

..          :doc:`Glossary </glossary/index>`
..             A list of terms and definitions used in Blender and this manual.

..       .. container:: descr

..          :ref:`Manual Index <genindex>`
..             A list of terms linked to the Glossary.


Get Involved
============
GPRT is an open source project, maintained by volunteers from all over the 
place.

Please consider joining our efforts, and check out our *contributor guide here*!

