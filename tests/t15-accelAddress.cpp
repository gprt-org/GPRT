#include <gprt.h>
#include <stdexcept>

extern GPRTProgram t15_deviceCode;
struct Parameters { const void *world; float *result; };

int main() {
  auto context = gprtContextCreate();
  auto module = gprtModuleCreate(context, t15_deviceCode);
  auto type = gprtGeomTypeCreate<void>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(type, 0, module, "hit");
  auto geom = gprtGeomCreate<void>(context, type);
  const float3 vertices[] = {{-1, -.5f, 0}, {1, -.5f, 0}, {0, .5f, 0}};
  const uint3 indices[] = {{0, 1, 2}};
  auto vertexBuffer = gprtDeviceBufferCreate<float3>(context, 3, vertices);
  auto indexBuffer = gprtDeviceBufferCreate<uint3>(context, 1, indices);
  gprtTrianglesSetVertices(geom, vertexBuffer, 3);
  gprtTrianglesSetIndices(geom, indexBuffer, 1);
  auto blas = gprtTriangleAccelCreate(context, geom);
  GPRTBuildParams options;
  options.buildMode = GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE;
  options.allowCompaction = true;
  gprtAccelBuild(context, blas, options);
  auto instance = gprtAccelGetInstance(blas);
  auto instances = gprtDeviceBufferCreate<gprt::Instance>(context, 1, &instance);
  auto tlas = gprtInstanceAccelCreate(context, 1, instances);
  gprtAccelBuild(context, tlas, options);
  auto raygen = gprtRayGenCreate<Parameters>(context, module, "trace");
  gprtMissCreate<void>(context, module, "miss");
  auto result = gprtDeviceBufferCreate<float>(context, 1);
  auto parameters = gprtRayGenGetParameters(raygen);
  parameters->result = gprtBufferGetDevicePointer(result);
  for (bool compact : {false, true}) {
    if (compact) {
      gprtAccelCompact(context, blas);
      gprtBufferMap(instances);
      *gprtBufferGetHostPointer(instances) = gprtAccelGetInstance(blas);
      gprtBufferUnmap(instances);
      gprtAccelBuild(context, tlas, options);
      gprtAccelCompact(context, tlas);
    }
    auto blasAddress = gprtAccelGetInstance(blas).accelAddress;
    parameters->world = gprtAccelGetDeviceAddress(tlas);
    gprtBuildShaderBindingTable(context);
    for (int step = 0; step < 3; ++step) {
      gprtBufferMap(vertexBuffer);
      for (int i = 0; i < 3; ++i) gprtBufferGetHostPointer(vertexBuffer)[i].z = float(step);
      gprtBufferUnmap(vertexBuffer);
      // Refit compacted trees; also rebuild full trees without growing their allocations.
      if (step == 2 && !compact) gprtAccelBuild(context, blas, options);
      else gprtAccelUpdate(context, blas);
      gprtDeviceSynchronize(context);
      gprtBufferMap(instances);
      gprtBufferGetHostPointer(instances)->transform[2].w = float(step);
      gprtBufferUnmap(instances);
      if (step == 2 && !compact) gprtAccelBuild(context, tlas, options);
      else gprtAccelUpdate(context, tlas);
      gprtDeviceSynchronize(context);
      if (gprtAccelGetInstance(blas).accelAddress != blasAddress || gprtAccelGetDeviceAddress(tlas) != parameters->world)
        throw std::runtime_error("Refit or same-size rebuild changed a cached acceleration structure address");
      gprtRayGenLaunch2D(context, raygen, 1, 1);
      gprtDeviceSynchronize(context);
      gprtBufferMap(result);
      if (*gprtBufferGetHostPointer(result) != float(2 * step + 1))
        throw std::runtime_error("Tracing through a cached address did not observe the changed geometry");
      gprtBufferUnmap(result);
    }
  }
  gprtContextDestroy(context);
}
