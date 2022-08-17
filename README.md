

# VKRT (Vuklan Ray Tracer)

VKRT is a ray tracing API that wraps the Vulkan ray tracing interface.

## Linux Build Instructions

You'll need a custom version of the dxc compiler found [here](https://drive.google.com/file/d/1dF6cX5q-3tB3e5zVcZIL_Fa1W8-WRpOq/view?usp=sharing).

The Linux version currently requires shared library compilation, which can be enabled with:

```shell
-DVKRT_BUILD_SHARED=ON
```
