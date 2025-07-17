# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

GPRT (General Purpose Raytracing Toolkit) is a ray tracing API that wraps the Vulkan ray tracing interface, making high-performance GPU ray tracing hardware accessible across NVIDIA, AMD, and Intel ARC platforms.

## Build Commands

### Initial Setup
```bash
# Clone with submodules (if not already done)
git clone --recursive git@github.com:gprt-org/GPRT.git

# Or update existing clone
git submodule init
git submodule update
```

### Building the Project
```bash
# Create build directory
mkdir build && cd build

# Configure (default is static library)
cmake ..

# Or configure for shared library
cmake .. -DGPRT_BUILD_SHARED=ON

# Build
cmake --build .

# Or with make on Linux/macOS
make -j$(nproc)
```

### Platform-Specific Requirements

**Ubuntu/Debian:**
```bash
sudo apt install xorg-dev libxinerama-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libglfw3
```

**All Platforms:**
- Vulkan SDK >= 1.4.321.0 (includes 'slangc')
- CMake >= 3.12
- C++17 compiler

## Code Formatting

The project uses clang-format with a custom configuration:
```bash
# Format a file
clang-format -i path/to/file.cpp

# Format all C++ files (from project root)
find gprt samples tests -name "*.cpp" -o -name "*.h" | xargs clang-format -i
```

## Documentation

Build documentation with Sphinx:
```bash
cd docs
make html
# Output will be in build/html/
```

## Architecture

### Core Components

1. **Host API** (`gprt/gprt_host.h`, `gprt/gprt.cpp`)
   - C++ interface for managing GPU resources
   - Context, module, buffer, and acceleration structure management
   - Shader binding table construction

2. **Device Code** (`gprt/*.slang`)
   - GPU shaders written in Slang
   - Ray generation, miss, hit, and compute programs
   - Built-in utilities: sorting, scanning, LBVH construction

3. **Math Library** (`gprt/math/`)
   - Vector and matrix types (float and double precision)
   - Quaternion operations
   - Derived from NVIDIA Falcor

### Programming Model

**Host-Device Separation:**
- Host code manages resources and launches kernels
- Device code implements GPU computation
- Communication via typed buffers and launch parameters

**Key Workflow:**
1. Create context: `gprtContextCreate()`
2. Compile shaders: `gprtModuleCreate(context, deviceCode)`
3. Create programs: `gprtRayGenCreate()`, `gprtMissCreate()`, etc.
4. Build acceleration structures: `gprtTrianglesCreate()`, `gprtAABBsCreate()`
5. Build SBT: `gprtBuildShaderBindingTable()`
6. Launch: `gprtRayGenLaunch2D()` or `gprtComputeLaunch()`

### Shader Compilation

Device code is compiled and embedded at build time:
- Slang files are compiled to SPIR-V
- SPIR-V bytecode is embedded in the library
- Custom CMake function: `embed_devicecode()` in `gprt/cmake/embed_devicecode.cmake`

### Sample Organization

Samples follow a progressive tutorial structure:
- `s0-*`: Ray generation basics
- `s1-*`: Miss programs
- `s2-*`: Hit programs (triangles, spheres, AABBs)
- `s3-*`: Instancing and transforms
- `s4-*`: Motion blur
- `s5-*`: Compute integration
- `s6-*`: Textures
- `s7-*`: Denoising
- `s8-*`: UI integration
- `s9-*`: Differentiable rendering

Each sample typically contains:
- `hostCode.cpp`: CPU-side setup and launch
- `deviceCode.slang`: GPU shaders
- `sharedCode.h`: Shared data structures

### Buffer Management

GPRT provides typed buffer management:
```cpp
// Create typed buffer on the device, visible to the host (when mapped)
GPRTBuffer<MyStruct> buffer = gprtDeviceBufferCreate<MyStruct>(context, count);

// Create typed buffer on the host, visible to the device
GPRTBuffer<MyStruct> buffer = gprtDeviceBufferCreate<MyStruct>(context, count);

// Map for writing to device local buffers
MyStruct* ptr = gprtBufferMap<MyStruct>(context, buffer);
// ... write data ...
gprtBufferUnmap(context, buffer);
```

### Acceleration Structures

Two-level hierarchy:
- **BLAS** (Bottom Level): Geometry (triangles, AABBs, LSS, custom)
- **TLAS** (Top Level): Instances of BLAS with transforms

Built-in support for:
- Triangle meshes
- AABBs (axis-aligned bounding boxes)
- Spheres
- LSS (linearly-swept spheres)
- Solids (finite elements for volume rendering)
- Motion blurred triangles (AABBs are still a work-in-progress)