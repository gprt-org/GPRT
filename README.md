

# VKRT (Vuklan Ray Tracer)

VKRT is a ray tracing API that wraps the Vulkan ray tracing interface.

## Build Instructions

VKRT currently requires a custom version of the dxc compiler found [here for
Linux](https://drive.google.com/file/d/1dF6cX5q-3tB3e5zVcZIL_Fa1W8-WRpOq/view?usp=sharing)
and [here for
Windows](https://drive.google.com/file/d/1Flwpq7eKv8wVt-1F8jYt_oeSbNGRKFJG/view?usp=sharing).

VKRT used CMake for configuration. For a build directory, `bld`, in the top
directory of this repository it can be configured with:

```shell
cmake ..
```

And built with:

```shell
make all
```

## Linux-Specific Build Instructions

The Linux version currently requires shared library compilation, which can be
enabled with `-DVKRT_BUILD_SHARED=ON` in the CMake command above.