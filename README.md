

[![License](https://img.shields.io/badge/license-MIT-green)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/natevm/vkrt/actions/workflows/ci.yml/badge.svg)](https://github.com/natevm/vkrt/actions/workflows/ci.yml)

# GPRT (General Purpose Raytracing Toolkit)

GPRT is a ray tracing API that wraps the Vulkan ray tracing interface.

## Dependencies

  - CMake
  - Vulkan SDK

## Documentation
GPRT's documentation can be found [here](https://gprt-org.github.io/GPRT/).

## Build Instructions

Install the [Vulkan SDK](https://vulkan.lunarg.com/) for your platform (version 1.3.231.1 or greater).

GPRT useds CMake for configuration. For a build directory, `build`, in the top
directory of this repository, the project can be configured with

```shell
./util/download-dxc.sh /path/to/dxc/dir/
cmake .. -DDXC_BIN_DIR=/path/to/dxc/dir/bin/
```

And built with

```shell
make all
```

## Ubuntu Dependencies

The following apt-packages should be installed:

```shell
sudo apt install xorg-dev libxinerama-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libglfw3
```

along with the [Vulkan SDK](https://vulkan.lunarg.com/doc/view/latest/linux/getting_started_ubuntu.html).
