GPRT Host API Complete Reference
================================

This toolkit has two sets of API, one for the "host" (typically referring to 
the CPU) and another for the "device" (normally the GPU, but really whatever
is tracing the actual rays). This file contains a complete list of all of the 
host-side functions provided by the toolkit, but for newcomers we recommend 
checking out our quick start guide *here*. 

Core Functions
------------------------------
.. doxygenfunction:: gprtContextCreate
.. doxygenfunction:: gprtContextDestroy

.. doxygenfunction:: gprtModuleCreate
.. doxygenfunction:: gprtModuleDestroy

.. doxygenfunction:: gprtBuildPipeline
.. doxygenfunction:: gprtBuildShaderBindingTable

.. doxygenfunction:: gprtBeginProfile
.. doxygenfunction:: gprtEndProfile


Buffer Functions
------------------------------
.. doxygenfunction:: gprtHostBufferCreate
.. doxygenfunction:: gprtDeviceBufferCreate
.. doxygenfunction:: gprtSharedBufferCreate
.. doxygenfunction:: gprtBufferDestroy
.. doxygenfunction:: gprtBufferMap
.. doxygenfunction:: gprtBufferUnmap
.. doxygenfunction:: gprtBufferGetPointer

Ray Generation Core Functions
------------------------------
.. doxygenfunction:: gprtRayGenCreate
.. doxygenfunction:: gprtRayGenDestroy
.. doxygenfunction:: gprtRayGenLaunch1D
.. doxygenfunction:: gprtRayGenLaunch2D
.. doxygenfunction:: gprtRayGenLaunch3D

Compute Program Core Functions
------------------------------
.. doxygenfunction:: gprtComputeCreate
.. doxygenfunction:: gprtComputeDestroy
.. doxygenfunction:: gprtComputeLaunch1D
.. doxygenfunction:: gprtComputeLaunch2D
.. doxygenfunction:: gprtComputeLaunch3D

Miss Program Core Functions
------------------------------
.. doxygenfunction:: gprtMissCreate
.. doxygenfunction:: gprtMissDestroy
.. doxygenfunction:: gprtMissSet

Hit Group Core Functions
------------------------------
.. doxygenfunction:: gprtGeomTypeCreate
.. doxygenfunction:: gprtGeomTypeDestroy
.. doxygenfunction:: gprtGeomTypeSetClosestHitProg
.. doxygenfunction:: gprtGeomTypeSetAnyHitProg
.. doxygenfunction:: gprtGeomTypeSetIntersectionProg

Geometry Functions
------------------------------
.. doxygenfunction:: gprtGeomCreate
.. doxygenfunction:: gprtGeomDestroy
.. doxygenfunction:: gprtTrianglesSetVertices
.. doxygenfunction:: gprtTrianglesSetIndices
.. doxygenfunction:: gprtAABBsSetPositions

Acceleration Structure Functions
---------------------------------
.. doxygenfunction:: gprtTrianglesAccelCreate
.. doxygenfunction:: gprtInstanceAccelCreate
.. doxygenfunction:: gprtInstanceAccelSetTransforms
.. doxygenfunction:: gprtAccelBuild
.. doxygenfunction:: gprtAccelRefit
.. doxygenfunction:: gprtAccelDestroy

Shader Record Setters
------------------------------
.. doxygenfunction:: gprtRayGenSet3f
.. doxygenfunction:: gprtRayGenSetBuffer
.. doxygenfunction:: gprtRayGenSet2i