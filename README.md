

[![License](https://img.shields.io/badge/license-MIT-green)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/natevm/vkrt/actions/workflows/ci.yml/badge.svg)](https://github.com/natevm/vkrt/actions/workflows/ci.yml)

# GPRT (General Purpose Raytracing Toolkit)

GPRT is a ray tracing API that wraps the Vulkan ray tracing interface.

## Documentation
https://gprt-org.github.io/GPRT/

## Build Instructions

GPRT currently requires a custom version of the dxc compiler found [here for
Linux](https://drive.google.com/file/d/1dF6cX5q-3tB3e5zVcZIL_Fa1W8-WRpOq/view?usp=sharing)
and [here for
Windows](https://drive.google.com/file/d/1Flwpq7eKv8wVt-1F8jYt_oeSbNGRKFJG/view?usp=sharing).

GPRT used CMake for configuration. For a build directory, `bld`, in the top
directory of this repository it can be configured with

```shell
./util/download-dxc.sh /path/to/dxc/dir/
cmake .. -DDXC_BIN_DIR=/path/to/dxc/dir/bin/
```

And built with

```shell
make all
```

## Linux Build Instructions

### Dependencies

The following apt-packages should be installed:

```shell
sudo apt install xorg-dev libxinerama-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev
```

along with the [Vulkan SDK](https://vulkan.lunarg.com/doc/view/latest/linux/getting_started_ubuntu.html).
