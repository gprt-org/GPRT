/**
 * @file gprt.slang
 * @author Nate Morrical (natemorrical@gmail.com)
 * @brief This file defines the device-side interface for the general purpose
 * raytracing toolkit.
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "math\MathConstants.slangh"

#include "gprt_shared.h"

// [[vk::binding(0, 0)]] is reserved for global "uniform" descriptor data.
[[vk::binding(0, 0)]] SamplerState samplers[];
[[vk::binding(0, 0)]] Texture1D texture1Ds[];
[[vk::binding(0, 0)]] Texture2D texture2Ds[];
[[vk::binding(0, 0)]] Texture3D texture3Ds[];
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer buffers[];

[[vk::binding(0, 0)]]
__DynamicResource<__DynamicResourceKind.Sampler> samplerHandles[];

[[vk::binding(0, 0)]]
__DynamicResource<__DynamicResourceKind.General> resourceHandles[];

export T getDescriptorFromHandle<T>(DescriptorHandle<T> handle) where T : IOpaqueDescriptor
{
  __target_switch
    {
  case spirv:
    if (T.kind == DescriptorKind.Sampler)
      return samplerHandles[((uint2) handle).x].asOpaqueDescriptor<T>();
    else
      return resourceHandles[((uint2) handle).x].asOpaqueDescriptor<T>();
  default:
    return defaultGetDescriptorFromHandle(handle);
  }
}

// Perhaps reserve binding 1 for SBT values?

// Update, we now use buffer pointers for all buffer data. This helps avoid the need for
// indirect buffer accessses through the below binding.

// globally coherent here causes memory barriers and syncs to flush data across
// the entire GPU such that other groups can see writes. Without this specifier,
// a memory barrier or sync will only flush a UAV within the current group.
// [[vk::binding(0, 5)]]

namespace gprt {
  
static float3 LessThan(float3 f, float value)
{
    return float3(
        (f.x < value) ? 1.0f : 0.0f,
        (f.y < value) ? 1.0f : 0.0f,
        (f.z < value) ? 1.0f : 0.0f);
}

//  IEC 61966-2-1:1999 standard linear to srgb definition.
float3 linear_to_srgb(float3 rgb)
{
    rgb = clamp(rgb, 0.0f, 1.0f);
    
    return lerp(
        pow(rgb * 1.055f, float3(1.f / 2.4f)) - 0.055f,
        rgb * 12.92f,
        LessThan(rgb , 0.0031308f)
    );
}

float3 srgb_to_linear(float3 srgb)
{
    // 1) Clamp to [0,1]
    srgb = clamp(srgb, 0.0f, 1.0f);

    // 2) Compute the two pieces
    float3 linear  = srgb / 12.92f;                                        // for srgb ≤ 0.04045
    float3 expPart = pow((srgb + 0.055f) / 1.055f, float3(2.4f));         // for srgb > 0.04045

    // 3) Blend based on the threshold mask
    //    LessThan(srgb,0.04045) is 1.0 where srgb<0.04045 → picks `linear`,
    //    else 0.0 → picks `expPart`.
    return lerp(expPart, linear, LessThan(srgb, 0.04045f));
}

inline uint32_t
make_8bit(const float f) {
  return min(255, max(0, int(f * 256.f)));
}

inline uint32_t
make_rgba(float3 color) {
  color = linear_to_srgb(color);
  return (make_8bit(color.x) << 0) + (make_8bit(color.y) << 8) + (make_8bit(color.z) << 16) + (0xffU << 24);
}

inline uint32_t
make_bgra(float3 color) {
  color = linear_to_srgb(color);
  return (make_8bit(color.z) << 0) + (make_8bit(color.y) << 8) + (make_8bit(color.x) << 16) + (0xffU << 24);
}

// Utility function which applies an IEC 61966-2-1:1999 linear to srgb transform, then goes to 8 bits.
inline uint32_t
make_bgra(float4 color) {
  color.rgb = linear_to_srgb(color.rgb);
  return (make_8bit(color.z) << 0) + (make_8bit(color.y) << 8) + (make_8bit(color.x) << 16) +
         (make_8bit(color.w) << 24);
}

// Make sure to use with only linear colors, not srgb!
float4
over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  if (result.a == 0.f)
    return a;   // avoid NaN
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

// struct Buffer {
//   uint64_t x;
//   uint64_t y;
// };

// T
// load<T>(in Buffer buffer, uint32_t index) {
//   return buffers[buffer.index].Load<T>(index * sizeof(T));
// }

// void
// store<T>(in Buffer buffer, uint32_t index, in T value) {
//   buffers[buffer.index].Store<T>(index * sizeof(T), value);
// }

// [[vk::ext_instruction(4447)]] RaytracingAccelerationStructure getAccelHandle(uint64_t ptr);

// RaytracingAccelerationStructure
// getAccelHandle(Accel accel) {
//   uint64_t address = accel.address;
//   return spirv_asm {
//     result: $$RaytracingAccelerationStructure = OpConvertUToAccelerationStructureKHR $address
//   };
// }

// Mainly used to teach how miss programs work, without involving closest hit or geometry complexity.
RaytracingAccelerationStructure
getNullAccelHandle() {
  uint64_t address = 0;
  return spirv_asm {
    result: $$RaytracingAccelerationStructure = OpConvertUToAccelerationStructureKHR $address
  };
}

// RWByteAddressBuffer
// getBufferHandle(Buffer buffer) {
//   return buffers[buffer.index];
// }

SamplerState
getDefaultSampler() {
  // We assume that there is a default sampler at address 0 here
  return samplers[0];
}

// Todo: update below to support pointers

// no_diff float
// atomicMin32f(no_diff in Buffer buffer, no_diff uint32_t index, inout float value) {
//   uint ret_i = asuint(buffers[buffer.index].Load<float>(index * sizeof(float)));
//   while (value < asfloat(ret_i)) {
//     uint old = ret_i;
//     buffers[buffer.index].InterlockedCompareExchange(index * sizeof(float), old, asuint(value), ret_i);
//     if (ret_i == old)
//       break;
//   }
//   return asfloat(ret_i);
// }

// // Derivative of an atomic max is discontinuous, where only an exact match between value
// // and the currently stored max is non-zero
// [BackwardDerivativeOf(atomicMin32f)]
// void
// atomicMin32f(in Buffer buffer, uint32_t index, inout DifferentialPair<float> value) {
//   float ret = buffers[buffer.index].Load<float>(index * sizeof(float));
//   value = diffPair(value.p, abs(value.p - ret) < .000001f ? ret : 0.f);
// }

// no_diff float
// atomicMax32f(no_diff in Buffer buffer, no_diff uint32_t index, inout float value) {
//   uint ret_i = asuint(buffers[buffer.index].Load<float>(index * sizeof(float)));
//   while (value > asfloat(ret_i)) {
//     uint old = ret_i;
//     buffers[buffer.index].InterlockedCompareExchange(index * sizeof(float), old, asuint(value), ret_i);
//     if (ret_i == old)
//       break;
//   }
//   return asfloat(ret_i);
// }

// // Derivative of an atomic max is discontinuous, where only an exact match between value
// // and the currently stored max is non-zero
// [BackwardDerivativeOf(atomicMax32f)]
// void
// atomicMax32f(in Buffer buffer, uint32_t index, inout DifferentialPair<float> value) {
//   float ret = buffers[buffer.index].Load<float>(index * sizeof(float));
//   value = diffPair(value.p, abs(value.p - ret) < .000001f ? ret : 0.f);
// }

// float
// atomicAdd32f(in Buffer buffer, uint32_t index, float value) {
//   uint old, newint;
//   uint ret_i = asuint(buffers[buffer.index].Load<float>(index * sizeof(float)));
//   do {
//     old = ret_i;
//     newint = asuint(asfloat(old) + value);
//     buffers[buffer.index].InterlockedCompareExchange(index * sizeof(float), old, newint, ret_i);
//   } while (ret_i != old);

//   return asfloat(ret_i);
// }

// uint32_t
// atomicAdd(in Buffer buffer, uint32_t index, uint32_t value) {
//   uint32_t old;
//   buffers[buffer.index].InterlockedAdd(index * sizeof(uint32_t), value, old);
//   return old;
// }

// uint32_t
// atomicOr(in Buffer buffer, uint32_t index, uint32_t value) {
//   uint32_t old;
//   buffers[buffer.index].InterlockedOr(index * sizeof(uint32_t), value, old);
//   return old;
// }

}

// still needs translating over
#if 0
template <typename T>
T
load(in Buffer buffer) {
  return vk::RawBufferLoad<T>(buffer.x);
}
template <typename T>
T
load(in Buffer buffer, uint64_t index) {
  return vk::RawBufferLoad<T>(buffer.x + index * sizeof(T));
}

template <typename T>
T
loadRaw(in Buffer buffer, uint64_t address) {
  return vk::RawBufferLoad<T>(buffer.x + address);
}

template <typename T>
void
store(in Buffer buffer, uint64_t index, in T value) {
  vk::RawBufferStore<T>(buffer.x + index * sizeof(T), value);
}

template <typename T>
void
storeRaw(in Buffer buffer, uint64_t address, in T value) {
  vk::RawBufferStore<T>(buffer.x + address, value);
}

// note, below  atomics are translated from
// here: https://github.com/treecode/Bonsai/blob/master/runtime/profiling/derived_atomic_functions.h

float
atomicMin32f(in Buffer buffer, uint32_t index, float value) {
  uint ret_i = asuint(buffers[buffer.y].Load<float>(index * sizeof(float)));
  while (value < asfloat(ret_i)) {
    uint old = ret_i;
    buffers[buffer.y].InterlockedCompareExchange(index * sizeof(float), old, asuint(value), ret_i);
    if (ret_i == old)
      break;
  }
  return asfloat(ret_i);
}

float3 atomicMin32f(in Buffer buffer, uint32_t index, float3 value) {
  float3 result;
  result.x = atomicMin32f(buffer, index * 3 + 0, value.x);
  result.y = atomicMin32f(buffer, index * 3 + 1, value.y);
  result.z = atomicMin32f(buffer, index * 3 + 2, value.z);
  return result;
}

float4 atomicMin32f(in Buffer buffer, uint32_t index, float4 value) {
  float4 result;
  result.x = atomicMin32f(buffer, index * 4 + 0, value.x);
  result.y = atomicMin32f(buffer, index * 4 + 1, value.y);
  result.z = atomicMin32f(buffer, index * 4 + 2, value.z);
  result.w = atomicMin32f(buffer, index * 4 + 3, value.w);
  return result;
}

float
atomicMax32f(in Buffer buffer, uint32_t index, float value) {
  uint ret_i = asuint(buffers[buffer.y].Load<float>(index * sizeof(float)));
  while (value > asfloat(ret_i)) {
    uint old = ret_i;
    buffers[buffer.y].InterlockedCompareExchange(index * sizeof(float), old, asuint(value), ret_i);
    if (ret_i == old)
      break;
  }
  return asfloat(ret_i);
}

float3 atomicMax32f(in Buffer buffer, uint32_t index, float3 value) {
  float3 result;
  result.x = atomicMax32f(buffer, index * 3 + 0, value.x);
  result.y = atomicMax32f(buffer, index * 3 + 1, value.y);
  result.z = atomicMax32f(buffer, index * 3 + 2, value.z);
  return result;
}

float4 atomicMax32f(in Buffer buffer, uint32_t index, float4 value) {
  float4 result;
  result.x = atomicMax32f(buffer, index * 4 + 0, value.x);
  result.y = atomicMax32f(buffer, index * 4 + 1, value.y);
  result.z = atomicMax32f(buffer, index * 4 + 2, value.z);
  result.w = atomicMax32f(buffer, index * 4 + 3, value.w);
  return result;
}

float
atomicAdd32f(in Buffer buffer, uint32_t index, float value) {
  uint old, newint;
  uint ret_i = asuint(buffers[buffer.y].Load<float>(index * sizeof(float)));
  do {
    old = ret_i;
    newint = asuint(asfloat(old) + value);
    buffers[buffer.y].InterlockedCompareExchange(index * sizeof(float), old, newint, ret_i);
  } while (ret_i != old);

  return asfloat(ret_i);
}

float3
atomicAdd32f(in Buffer buffer, uint32_t index, float3 value) {
  float3 result;
  result.x = atomicAdd32f(buffer, index * 3 + 0, value.x);
  result.y = atomicAdd32f(buffer, index * 3 + 1, value.y);
  result.z = atomicAdd32f(buffer, index * 3 + 2, value.z);
  return result;
}

float3x3
atomicAdd32f(in Buffer buffer, uint32_t index, float3x3 value) {
  float3x3 result;
  result._11 = atomicAdd32f(buffer, index * 9 + 0, value._11);
  result._12 = atomicAdd32f(buffer, index * 9 + 1, value._12);
  result._13 = atomicAdd32f(buffer, index * 9 + 2, value._13);

  result._21 = atomicAdd32f(buffer, index * 9 + 3, value._21);
  result._22 = atomicAdd32f(buffer, index * 9 + 4, value._22);
  result._23 = atomicAdd32f(buffer, index * 9 + 5, value._23);

  result._31 = atomicAdd32f(buffer, index * 9 + 6, value._31);
  result._32 = atomicAdd32f(buffer, index * 9 + 7, value._32);
  result._33 = atomicAdd32f(buffer, index * 9 + 8, value._33);
  return result;
}

#endif
