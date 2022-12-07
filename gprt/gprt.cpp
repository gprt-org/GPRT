// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gprt_host.h>
#include <iostream>
#include <assert.h>
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <limits>
#include <climits>
#include <algorithm>

#include <regex>

#ifdef __GNUC__
#include <execinfo.h>
#include <sys/time.h>
#include <signal.h>
#endif


#ifdef _WIN32
#define NOMINMAX
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#endif

// library for windowing
#include <GLFW/glfw3.h>

// library for image output
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#if defined(_MSC_VER)
//&& !defined(__PRETTY_FUNCTION__)
#  define __PRETTY_FUNCTION__ __FUNCTION__
#endif

#if 1
# define LOG_API_CALL() /* ignore */
#else
# define LOG_API_CALL() std::cout << "% " << __FUNCTION__ << "(...)" << std::endl;
#endif

#define LOG(message)                            \
  if (Context::logging())                       \
    std::cout                                   \
      << GPRT_TERMINAL_LIGHT_BLUE                \
      << "#gprt: "                               \
      << message                                \
      << GPRT_TERMINAL_DEFAULT << std::endl

#define LOG_OK(message)                         \
  if (Context::logging())                       \
    std::cout                                   \
      << GPRT_TERMINAL_BLUE                      \
      << "#gprt: "                               \
      << message                                \
      << GPRT_TERMINAL_DEFAULT << std::endl

namespace detail {
inline static std::string backtrace()
{
#ifdef __GNUC__
    static const int max_frames = 16;

    void* buffer[max_frames] = { 0 };
    int cnt = ::backtrace(buffer,max_frames);

    char** symbols = backtrace_symbols(buffer,cnt);

    if (symbols) {
      std::stringstream str;
      for (int n = 1; n < cnt; ++n) // skip the 1st entry (address of this function)
      {
        str << symbols[n] << '\n';
      }
      free(symbols);
      return str.str();
    }
    return "";
#else
    return "not implemented yet";
#endif
}

inline void gprtRaise_impl(std::string str)
{
  fprintf(stderr,"%s\n",str.c_str());
#ifdef WIN32
  if (IsDebuggerPresent())
    DebugBreak();
  else
    assert(false);
#else
#ifndef NDEBUG
  std::string bt = ::detail::backtrace();
  fprintf(stderr,"%s\n",bt.c_str());
#endif
  raise(SIGINT);
#endif
}
}

#define GPRT_RAISE(MSG) ::detail::gprtRaise_impl(MSG);

#define GPRT_NOTIMPLEMENTED  { std::cerr<<std::string(__PRETTY_FUNCTION__) << " not implemented" << std::endl; assert(false);};


std::string errorString(VkResult errorCode)
{
  switch (errorCode)
  {
#define STR(r) case VK_ ##r: return #r
    STR(NOT_READY);
    STR(TIMEOUT);
    STR(EVENT_SET);
    STR(EVENT_RESET);
    STR(INCOMPLETE);
    STR(ERROR_OUT_OF_HOST_MEMORY);
    STR(ERROR_OUT_OF_DEVICE_MEMORY);
    STR(ERROR_INITIALIZATION_FAILED);
    STR(ERROR_DEVICE_LOST);
    STR(ERROR_MEMORY_MAP_FAILED);
    STR(ERROR_LAYER_NOT_PRESENT);
    STR(ERROR_EXTENSION_NOT_PRESENT);
    STR(ERROR_FEATURE_NOT_PRESENT);
    STR(ERROR_INCOMPATIBLE_DRIVER);
    STR(ERROR_TOO_MANY_OBJECTS);
    STR(ERROR_FORMAT_NOT_SUPPORTED);
    STR(ERROR_SURFACE_LOST_KHR);
    STR(ERROR_NATIVE_WINDOW_IN_USE_KHR);
    STR(SUBOPTIMAL_KHR);
    STR(ERROR_OUT_OF_DATE_KHR);
    STR(ERROR_INCOMPATIBLE_DISPLAY_KHR);
    STR(ERROR_VALIDATION_FAILED_EXT);
    STR(ERROR_INVALID_SHADER_NV);
#undef STR
  default:
    return "UNKNOWN_ERROR";
  }
}

#define VK_CHECK_RESULT(f)                                        \
{                                                    \
  VkResult res = (f);                                          \
  if (res != VK_SUCCESS)                                        \
  {                                                  \
    std::cout << "Fatal : VkResult is \"" << errorString(res) << "\" in " \
      << __FILE__ << " at line " << __LINE__ << "\n"; \
    assert(res == VK_SUCCESS);                                    \
  }                                                  \
}

VKAPI_ATTR VkBool32 VKAPI_CALL debugUtilsMessengerCallback(
			VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
			VkDebugUtilsMessageTypeFlagsEXT messageType,
			const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
			void* pUserData)
{
  // Select prefix depending on flags passed to the callback
  std::string prefix("");

  if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) {
    prefix = "VERBOSE: ";
  }
  else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) {
    prefix = "INFO: ";
  }
  else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
    prefix = "WARNING: ";
  }
  else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    prefix = "ERROR: ";
  }


  // Display message to default output (console/logcat)
  std::stringstream debugMessage;
  // debugMessage << prefix << "[" << pCallbackData->messageIdNumber << "][" << pCallbackData->pMessageIdName << "] : " << pCallbackData->pMessage;
  debugMessage << pCallbackData->pMessage;

#if defined(__ANDROID__)
  if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    LOGE("%s", debugMessage.str().c_str());
  } else {
    LOGD("%s", debugMessage.str().c_str());
  }
#else
  if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    std::cerr << debugMessage.str() << "\n";
  } else {
    std::cout << debugMessage.str() << "\n";
  }
  fflush(stdout);
#endif


  // The return value of this callback controls whether the Vulkan call that caused the validation message will be aborted or not
  // We return VK_FALSE as we DON'T want Vulkan calls that cause a validation message to abort
  // If you instead want to have calls abort, pass in VK_TRUE and the function will return VK_ERROR_VALIDATION_FAILED_EXT
  return VK_FALSE;
}

// Contains definitions for internal entry points 
// (bounds programs, transform programs...)
extern std::map<std::string, std::vector<uint8_t>> gprtDeviceCode;

// forward declarations...
struct Geom; 
struct GeomType; 
struct TriangleGeom;
struct TriangleGeomType;
struct AABBGeom;
struct AABBGeomType;

namespace gprt{
  PFN_vkGetBufferDeviceAddressKHR vkGetBufferDeviceAddress;
  PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructure;
  PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructure;
  PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizes;
  PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddress;
  PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructures;
  PFN_vkBuildAccelerationStructuresKHR vkBuildAccelerationStructures;
  PFN_vkCmdTraceRaysKHR vkCmdTraceRays;
  PFN_vkGetRayTracingShaderGroupHandlesKHR vkGetRayTracingShaderGroupHandles;
  PFN_vkCreateRayTracingPipelinesKHR vkCreateRayTracingPipelines;
  
  PFN_vkCreateDebugUtilsMessengerEXT vkCreateDebugUtilsMessengerEXT;
  PFN_vkDestroyDebugUtilsMessengerEXT vkDestroyDebugUtilsMessengerEXT;
  VkDebugUtilsMessengerEXT debugUtilsMessenger;

  PFN_vkCreateDebugReportCallbackEXT vkCreateDebugReportCallbackEXT;
  PFN_vkDestroyDebugReportCallbackEXT vkDestroyDebugReportCallbackEXT;
  VkDebugReportCallbackEXT debugReportCallback;
}

struct Stage {
  // for copying transforms into the instance buffer
  // std::string fillInstanceDataEntryPoint = "gprtFillInstanceData";
  // VkPipelineLayout fillInstanceDataPipelineLayout;
  // VkShaderModule fillInstanceDataShaderModule;
  // VkPipeline fillInstanceDataPipeline;
  std::string entryPoint;
  VkPipelineLayout layout;
  VkShaderModule module;
  VkPipeline pipeline;
};

struct Module {
  // std::string program;
  GPRTProgram program;

  Module(GPRTProgram program) {
    this->program = program;
  }

  ~Module() {
  }

  std::vector<uint32_t> getBinary(std::string entryType) {
    size_t sizeOfProgram = program[entryType].size() -  1; // program is null terminated.
    std::vector<uint32_t> finalProgram(sizeOfProgram / 4);
    memcpy(finalProgram.data(), program[entryType].data(), sizeOfProgram);
    return finalProgram;
  }
};

struct Buffer {
  VkDevice device;
  VkPhysicalDeviceMemoryProperties memoryProperties;
  VkCommandBuffer commandBuffer;
  VkQueue queue;

  /** @brief Usage flags to be filled by external source at buffer creation */
  VkBufferUsageFlags usageFlags;
  
  /** @brief Memory property flags to be filled by external source at buffer creation */
  VkMemoryPropertyFlags memoryPropertyFlags;

  bool hostVisible;

  VkBuffer buffer = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkDeviceAddress address = 0;
  

  struct StagingBuffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceAddress address = 0;
  } stagingBuffer;

  VkDeviceSize size = 0;
  VkDeviceSize alignment = 0;
  void* mapped = nullptr;

  VkResult map(VkDeviceSize mapSize = VK_WHOLE_SIZE, VkDeviceSize offset = 0)
  {
    if (hostVisible) {
      if (mapped) return VK_SUCCESS;
      else return vkMapMemory(device, memory, offset, size, 0, &mapped);
    }
    else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err) GPRT_RAISE("failed to begin command buffer for buffer map! : \n" + errorString(err));

      // To do, consider allowing users to specify offsets here...
      VkBufferCopy region;
      region.srcOffset = 0;
      region.dstOffset = 0;
      region.size = size;
      vkCmdCopyBuffer(commandBuffer, buffer, stagingBuffer.buffer, 1, &region);

      err = vkEndCommandBuffer(commandBuffer);
      if (err) GPRT_RAISE("failed to end command buffer for buffer map! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, nullptr);
      if (err) GPRT_RAISE("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err) GPRT_RAISE("failed to wait for queue idle for buffer map! : \n" + errorString(err));

      // todo, transfer device data to host
      if (mapped) return VK_SUCCESS;
      else return vkMapMemory(device, stagingBuffer.memory, offset, mapSize, 0, &mapped);
    }
  }

  void unmap()
  {
    if (hostVisible) {
      if (mapped) {
        vkUnmapMemory(device, memory);
        mapped = nullptr;
      }
    }
    else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      if (err) GPRT_RAISE("failed to begin command buffer for buffer map! : \n" + errorString(err));

      // To do, consider allowing users to specify offsets here...
      VkBufferCopy region;
      region.srcOffset = 0;
      region.dstOffset = 0;
      region.size = size;
      vkCmdCopyBuffer(commandBuffer, stagingBuffer.buffer, buffer, 1, &region);

      err = vkEndCommandBuffer(commandBuffer);
      if (err) GPRT_RAISE("failed to end command buffer for buffer map! : \n" + errorString(err));
      
      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(queue, 1, &submitInfo, nullptr);
      if (err) GPRT_RAISE("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(queue);
      if (err) GPRT_RAISE("failed to wait for queue idle for buffer map! : \n" + errorString(err));
      
      // todo, transfer device data to device
      if (mapped) {
        vkUnmapMemory(device, stagingBuffer.memory);
        mapped = nullptr;
      }
    }
  }

  // VkResult bind(VkDeviceSize offset = 0)
  // {
  //   return vkBindBufferMemory(device, buffer, memory, offset);
  // }

  // void setupDescriptor(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0)
  // {
  //   descriptor.offset = offset;
  //   descriptor.buffer = buffer;
  //   descriptor.range = size;
  // }

  // void copyTo(void* data, VkDeviceSize size)
  // {
  //   assert(mapped);
  //   memcpy(mapped, data, size);
  // }

  /*! Guarantees that the host's writes are available to the device */
  VkResult flush(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0)
  {
    VkMappedMemoryRange mappedRange = {};
    mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    mappedRange.memory = memory;
    mappedRange.offset = offset;
    mappedRange.size = size;
    return vkFlushMappedMemoryRanges(device, 1, &mappedRange);
  }

  /*! Guarantees that the buffer is written to by any pending device operations */
  VkResult invalidate(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0)
  {
    VkMappedMemoryRange mappedRange = {};
    mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    mappedRange.memory = memory;
    mappedRange.offset = offset;
    mappedRange.size = size;
    return vkInvalidateMappedMemoryRanges(device, 1, &mappedRange);
  }

  VkDeviceAddress getDeviceAddress()
  {
    VkBufferDeviceAddressInfoKHR info = {};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO_KHR;
    info.buffer = buffer;
    VkDeviceAddress addr = gprt::vkGetBufferDeviceAddress(device, &info);
    return addr;
  }

  /*! Calls vkDestroy on the buffer, and frees underlying memory */
  void destroy()
  {
    if (buffer) {
      vkDestroyBuffer(device, buffer, nullptr);
      buffer = VK_NULL_HANDLE;
    }
    if (memory) {
      vkFreeMemory(device, memory, nullptr);
      memory = VK_NULL_HANDLE;
    }
    if (stagingBuffer.buffer) {
      vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
      stagingBuffer.buffer = VK_NULL_HANDLE;
    }
    if (stagingBuffer.memory) {
      vkFreeMemory(device, stagingBuffer.memory, nullptr);
      stagingBuffer.memory = VK_NULL_HANDLE;
    }

  }

  /* Default Constructor */
  Buffer() {};
  
  ~Buffer() {};

  Buffer(
    VkPhysicalDevice physicalDevice, VkDevice logicalDevice, 
    VkCommandBuffer _commandBuffer, VkQueue _queue,
    VkBufferUsageFlags _usageFlags, VkMemoryPropertyFlags _memoryPropertyFlags,
    VkDeviceSize _size, void *data = nullptr)
  {
    device = logicalDevice;
    memoryPropertyFlags = _memoryPropertyFlags;
    usageFlags = _usageFlags;
    size = _size;
    commandBuffer = _commandBuffer;
    queue = _queue;

    // Check if the buffer can be mapped to a host pointer. 
    // If the buffer isn't host visible, this is buffer and requires 
    // an additional staging buffer...
    if ((memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0) 
      hostVisible = true;
    else 
      hostVisible = false;

    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memoryProperties);

    auto getMemoryType = [this](
      uint32_t typeBits, VkMemoryPropertyFlags properties,
      VkBool32 *memTypeFound = nullptr) -> uint32_t {

      // memory type bits is a bitmask and contains one bit set for every supported memory type.
      // Bit i is set if and only if the memory type i in the memory properties is supported.
      for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; i++) {
        if ((typeBits & 1) == 1) {
          if ((memoryProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            if (memTypeFound) {
              *memTypeFound = true;
            }
            return i;
          }
        }
        typeBits >>= 1;
      }

      if (memTypeFound) {
        *memTypeFound = false;
        return 0;
      }
      else {
        GPRT_RAISE("Could not find a matching memory type");
      }
    };

    // Create the buffer handle
    VkBufferCreateInfo bufferCreateInfo {};
    bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferCreateInfo.usage = usageFlags;
    bufferCreateInfo.size = size;
    VK_CHECK_RESULT(vkCreateBuffer(logicalDevice, &bufferCreateInfo, nullptr, &buffer));

    if (!hostVisible) {
      const VkBufferUsageFlags bufferUsageFlags =
        // means we can use this buffer to transfer into another
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
        // means we can use this buffer to receive data transferred from another
        VK_BUFFER_USAGE_TRANSFER_DST_BIT
      ;

      VkBufferCreateInfo bufferCreateInfo {};
      bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
      bufferCreateInfo.usage = bufferUsageFlags;
      bufferCreateInfo.size = size;
      VK_CHECK_RESULT(vkCreateBuffer(logicalDevice, &bufferCreateInfo, nullptr, &stagingBuffer.buffer));
    }

    // Create the memory backing up the buffer handle
    VkMemoryRequirements memReqs;
    VkMemoryAllocateInfo memAllocInfo {};
    memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    vkGetBufferMemoryRequirements(logicalDevice, buffer, &memReqs);
    memAllocInfo.allocationSize = memReqs.size;
    // Find a memory type index that fits the properties of the buffer
    memAllocInfo.memoryTypeIndex = getMemoryType(memReqs.memoryTypeBits, memoryPropertyFlags);
    // If the buffer has VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT set we also
    // need to enable the appropriate flag during allocation
    VkMemoryAllocateFlagsInfoKHR allocFlagsInfo{};
    if (usageFlags & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) {
      allocFlagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO_KHR;
      allocFlagsInfo.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;
      memAllocInfo.pNext = &allocFlagsInfo;
    }
    VK_CHECK_RESULT(vkAllocateMemory(logicalDevice, &memAllocInfo, nullptr, &memory));
    alignment = memReqs.alignment;

    // Attach the memory to the buffer object
    VkResult err = vkBindBufferMemory(device, buffer, memory, /* offset */ 0);
    if (err) GPRT_RAISE("failed to bind buffer memory! : \n" + errorString(err));

    if (!hostVisible) {
      const VkMemoryPropertyFlags memoryPropertyFlags =
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | // mappable to host with vkMapMemory
        VK_MEMORY_PROPERTY_HOST_COHERENT_BIT; // means "flush" and "invalidate"  not needed

      // Create the memory backing up the staging buffer handle
      VkMemoryRequirements memReqs;
      VkMemoryAllocateInfo memAllocInfo {};
      memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
      vkGetBufferMemoryRequirements(logicalDevice, stagingBuffer.buffer, &memReqs);
      memAllocInfo.allocationSize = memReqs.size;
      // Find a memory type index that fits the properties of the buffer
      memAllocInfo.memoryTypeIndex = getMemoryType(memReqs.memoryTypeBits, memoryPropertyFlags);
      // If the buffer has VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT set we also
      // need to enable the appropriate flag during allocation
      VkMemoryAllocateFlagsInfoKHR allocFlagsInfo{};
      if (usageFlags & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) {
        allocFlagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO_KHR;
        allocFlagsInfo.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;
        memAllocInfo.pNext = &allocFlagsInfo;
      }
      VK_CHECK_RESULT(vkAllocateMemory(logicalDevice, &memAllocInfo, nullptr, &stagingBuffer.memory));
      alignment = memReqs.alignment;

      // Attach the memory to the buffer object
      VkResult err = vkBindBufferMemory(device, stagingBuffer.buffer, stagingBuffer.memory, /* offset */ 0);
      if (err) GPRT_RAISE("failed to bind staging buffer memory! : \n" + errorString(err));
    }

    // If a pointer to the buffer data has been passed, map the buffer and
    // copy over the data
    if (data != nullptr) {
      VK_CHECK_RESULT(map());
      memcpy(mapped, data, size);
      if ((memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == 0)
        flush();
      unmap();
    }

    //// Initialize a default descriptor that covers the whole buffer size
    // setupDescriptor();
    
    // means we can get this buffer's address with vkGetBufferDeviceAddress
    if ((usageFlags & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) != 0)
      address = getDeviceAddress();
  }
};

struct SBTEntry {
  // Map of the name of the variable to that variable declaration
  std::unordered_map<std::string, GPRTVarDef> vars;
};

// At the moment, we actually just use ray generation programs for compute 
// kernels. We do this instead of using actual Vulkan compute programs so that
// we can recycle the SBT records mechanism for compute IO, without 
// introducing VK descriptor sets.
struct Compute : public SBTEntry {
  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  VkDevice logicalDevice;
  std::string entryPoint;

  Compute(VkDevice  _logicalDevice,
            Module *module,
            const char* _entryPoint,
            size_t      sizeOfVarStruct,
            std::unordered_map<std::string, GPRTVarDef> _vars) : SBTEntry()
  {
    std::cout<<"Compute program is being made!"<<std::endl;

    entryPoint = std::string("__compute__") + std::string(_entryPoint);
    auto binary = module->getBinary("COMPUTE");

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);//sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data(); //(uint32_t*)binary->wordCount;//programBytes;

    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
      NULL, &shaderModule));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    // shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    shaderStage.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);
    vars = _vars;
  }
  ~Compute() {}
  void destroy() {
    std::cout<<"Compute program is being destroyed!"<<std::endl;
    vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
  }
};

struct RayGen : public SBTEntry {
  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  VkDevice logicalDevice;
  std::string entryPoint;

  RayGen(VkDevice  _logicalDevice,
          Module *module,
          const char* _entryPoint,
          size_t      sizeOfVarStruct,
          std::unordered_map<std::string, GPRTVarDef> _vars) : SBTEntry()
  {
    std::cout<<"Ray gen is being made!"<<std::endl;

    entryPoint = std::string("__raygen__") + std::string(_entryPoint);
    auto binary = module->getBinary("RAYGEN");

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);//sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data(); //(uint32_t*)binary->wordCount;//programBytes;

    VkResult err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
      NULL, &shaderModule);
    if (err) GPRT_RAISE("failed to create shader module! : \n" + errorString(err));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);
    vars = _vars;
  }
  ~RayGen() {}
  void destroy() {
    std::cout<<"Ray gen is being destroyed!"<<std::endl;
    vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
  }
};

struct Miss : public SBTEntry {
  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  VkDevice logicalDevice;
  std::string entryPoint;

  Miss(VkDevice  _logicalDevice,
            Module *module,
            const char* _entryPoint,
            size_t      sizeOfVarStruct,
            std::unordered_map<std::string, GPRTVarDef> _vars) : SBTEntry()
  {
    std::cout<<"Miss program is being made!"<<std::endl;

    entryPoint = std::string("__miss__") + std::string(_entryPoint);
    auto binary = module->getBinary("MISS");

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);//sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data(); //(uint32_t*)binary->wordCount;//programBytes;

    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
      NULL, &shaderModule));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_MISS_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);
    vars = _vars;
  }
  ~Miss() {}
  void destroy() {
    std::cout<<"Miss program is being destroyed!"<<std::endl;
    vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
  }
};

/* An abstraction for any sort of geometry type - describes the
    programs to use, and structure of the SBT records, when building
    shader binding tables (SBTs) with geometries of this type. This
    will later get subclassed into triangle geometries, user/custom
    primitive geometry types, etc */
struct GeomType : public SBTEntry {
  VkDevice logicalDevice;
  std::vector<VkPipelineShaderStageCreateInfo> closestHitShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> anyHitShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> intersectionShaderStages;
  
  std::vector<std::string> closestHitShaderEntryPoints;
  std::vector<std::string> anyHitShaderEntryPoints;
  std::vector<std::string> intersectionShaderEntryPoints;
  
  bool closestHitShadersUsed = false;
  bool intersectionShadersUsed = false;
  bool anyHitShadersUsed = false;
  
  GeomType(VkDevice  _logicalDevice,
            uint32_t numRayTypes,
            size_t      sizeOfVarStruct,
            std::unordered_map<std::string, GPRTVarDef> _vars) : SBTEntry()
  {
    std::cout<<"Geom type is being made!"<<std::endl;
    closestHitShaderStages.resize(numRayTypes, {});
    anyHitShaderStages.resize(numRayTypes, {});
    intersectionShaderStages.resize(numRayTypes, {});

    closestHitShaderEntryPoints.resize(numRayTypes, {});
    anyHitShaderEntryPoints.resize(numRayTypes, {});
    intersectionShaderEntryPoints.resize(numRayTypes, {});

    // store a reference to the logical device this module is made on
    logicalDevice = _logicalDevice;
    vars = _vars;
  }
  ~GeomType() 
  {
    std::cout<<"Geom type is being destroyed!"<<std::endl;
    // vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
  }

  virtual GPRTGeomKind getKind() {return GPRT_UNKNOWN;}
  
  void setClosestHit(int rayType,
                      Module *module,
                      const char* entryPoint) 
  {
    closestHitShadersUsed = true;
    closestHitShaderEntryPoints[rayType] = std::string("__closesthit__") + std::string(entryPoint);
    auto binary = module->getBinary("CLOSESTHIT");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
      NULL, &shaderModule));

    closestHitShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    closestHitShaderStages[rayType].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    closestHitShaderStages[rayType].module = shaderModule;
    closestHitShaderStages[rayType].pName = closestHitShaderEntryPoints[rayType].c_str();
    assert(closestHitShaderStages[rayType].module != VK_NULL_HANDLE);    
  }

  void setAnyHit(int rayType,
                      Module *module,
                      const char* entryPoint) 
  {
    anyHitShadersUsed = true;
    anyHitShaderEntryPoints[rayType] = std::string("__anyhit__") + std::string(entryPoint);
    auto binary = module->getBinary("ANYHIT");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
      NULL, &shaderModule));

    anyHitShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    anyHitShaderStages[rayType].stage = VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
    anyHitShaderStages[rayType].module = shaderModule;
    anyHitShaderStages[rayType].pName = anyHitShaderEntryPoints[rayType].c_str();
    assert(anyHitShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  void setIntersection(int rayType,
                      Module *module,
                      const char* entryPoint) 
  {
    intersectionShadersUsed = true;
    intersectionShaderEntryPoints[rayType] = std::string("__intersection__") + std::string(entryPoint);
    auto binary = module->getBinary("INTERSECTION");
    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
      NULL, &shaderModule));

    intersectionShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    intersectionShaderStages[rayType].stage = VK_SHADER_STAGE_INTERSECTION_BIT_KHR;
    intersectionShaderStages[rayType].module = shaderModule;
    intersectionShaderStages[rayType].pName = intersectionShaderEntryPoints[rayType].c_str();
    assert(intersectionShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  void destroy() {
    std::cout<<"geom type is being destroyed!"<<std::endl;
    for (uint32_t i = 0; i < closestHitShaderStages.size(); ++i) {
      if (closestHitShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, 
          closestHitShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < anyHitShaderStages.size(); ++i) {
      if (anyHitShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, 
          anyHitShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < intersectionShaderStages.size(); ++i) {
      if (intersectionShaderStages[i].module)
        vkDestroyShaderModule(logicalDevice, 
          intersectionShaderStages[i].module, nullptr);
    }
  }
  
  virtual Geom* createGeom() { return nullptr; };
};

/*! An actual geometry object with primitives - this class is still
  abstract, and will get fleshed out in its derived classes
  (AABBGeom, TriangleGeom, ...) */
struct Geom : public SBTEntry {
  Geom() : SBTEntry() {};
  ~Geom() {};

  void destroy(){}

  /*! This acts as a template that describes this geometry's variables and
      programs. */
  GeomType* geomType;
};

struct TriangleGeom : public Geom {
  struct {
    size_t count  = 0; // number of indices
    size_t stride = 0; // stride between indices
    size_t offset = 0; // offset in bytes to the first index
    size_t firstVertex = 0; // added to the index values before fetching vertices
    Buffer* buffer = nullptr;
  } index;

  struct {
    size_t count  = 0; // number of vertices
    size_t stride = 0; // stride between vertices
    size_t offset = 0; // an offset in bytes to the first vertex
    std::vector<Buffer*> buffers;
  } vertex;

  TriangleGeom(TriangleGeomType* _geomType) : Geom() {
    geomType = (GeomType*)_geomType;

    // Allocate the variables for this geometry, using our geomType vars as 
    // the template.
    std::vector<GPRTVarDecl> varDecls = getDecls(geomType->vars);
    vars = checkAndPackVariables(varDecls.data(), varDecls.size());
  };
  ~TriangleGeom() {};

  void setVertices(
    Buffer* vertices,
    size_t count,
    size_t stride,
    size_t offset) 
  {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    vertex.buffers.resize(1);
    vertex.buffers[0] = vertices;
    vertex.count = count;
    vertex.stride = stride;
    vertex.offset = offset;
  }

  void setIndices(
    Buffer* indices,
    size_t count,
    size_t stride,
    size_t offset) 
  {
    index.buffer = indices;
    index.count = count;
    index.stride = stride;
    index.offset = offset;
  }
};

struct TriangleGeomType : public GeomType {
  TriangleGeomType(
    VkDevice logicalDevice,
    uint32_t numRayTypes,
    size_t   sizeOfVarStruct,
    std::unordered_map<std::string, GPRTVarDef> vars) : 
    GeomType(logicalDevice, numRayTypes, sizeOfVarStruct, vars)
  {}
  ~TriangleGeomType() {}

  Geom* createGeom()  
  {
    return new TriangleGeom(this);
  }

  GPRTGeomKind getKind() {return GPRT_TRIANGLES;}
};

struct AABBGeom : public Geom {
  struct {
    size_t count;
    size_t stride;
    size_t offset;
    std::vector<Buffer*> buffers;
  } aabb;

  AABBGeom(AABBGeomType* _geomType) : Geom() {
    geomType = (GeomType*)_geomType;

    // Allocate the variables for this geometry, using our geomType vars as 
    // the template.
    std::vector<GPRTVarDecl> varDecls = getDecls(geomType->vars);
    vars = checkAndPackVariables(varDecls.data(), varDecls.size());
  };
  ~AABBGeom() {};

  void setAABBs(
    Buffer* aabbs,
    size_t count,
    size_t stride,
    size_t offset) 
  {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    aabb.buffers.resize(1);
    aabb.buffers[0] = aabbs;
    aabb.count = count;
    aabb.stride = stride;
    aabb.offset = offset;
  }
};

struct AABBGeomType : public GeomType {
  AABBGeomType(VkDevice  _logicalDevice,
            uint32_t numRayTypes,
            size_t      sizeOfVarStruct,
            std::unordered_map<std::string, GPRTVarDef> _vars) : 
            GeomType(_logicalDevice, numRayTypes, sizeOfVarStruct, _vars)
  {}
  ~AABBGeomType() {}
  Geom* createGeom() 
  {
    return new AABBGeom(this);
  }

  GPRTGeomKind getKind() {return GPRT_AABBS;}
};

typedef enum
{
  GPRT_UNKNOWN_ACCEL = 0x0,
  GPRT_INSTANCE_ACCEL  = 0x1,
  GPRT_TRIANGLE_ACCEL = 0x2,
  GPRT_AABB_ACCEL = 0x3,
} AccelType;

struct Accel {
  VkPhysicalDevice physicalDevice;
  VkDevice logicalDevice;
  VkCommandBuffer commandBuffer;
  VkQueue queue;
  VkDeviceAddress address = 0;
  VkAccelerationStructureKHR accelerationStructure = VK_NULL_HANDLE;

  Buffer *accelBuffer = nullptr;
  Buffer *scratchBuffer = nullptr;
  
  Accel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer commandBuffer, VkQueue queue) {
    this->physicalDevice = physicalDevice;
    this->logicalDevice = logicalDevice;
    this->commandBuffer = commandBuffer;
    this->queue = queue;
  };
  
  ~Accel() {};

  virtual void build(std::map<std::string, Stage> internalStages, std::vector<Accel*> accels, uint32_t numRayTypes) { };
  virtual void destroy() { };
  virtual AccelType getType() {return GPRT_UNKNOWN_ACCEL;}
};

struct TriangleAccel : public Accel {
  std::vector<TriangleGeom*> geometries; 

  // todo, accept this in constructor
  VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

  TriangleAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer commandBuffer, VkQueue queue,
    size_t numGeometries, TriangleGeom* geometries) : Accel(physicalDevice, logicalDevice, commandBuffer, queue) 
  {
    this->geometries.resize(numGeometries);
    memcpy(this->geometries.data(), geometries, sizeof(GPRTGeom*) * numGeometries);
  };
  
  ~TriangleAccel() {};

  AccelType getType() {return GPRT_TRIANGLE_ACCEL;}

  void build(std::map<std::string, Stage> internalStages, std::vector<Accel*> accels, uint32_t numRayTypes) {
    VkResult err;

    std::vector<VkAccelerationStructureBuildRangeInfoKHR> accelerationBuildStructureRangeInfos(geometries.size());
    std::vector<VkAccelerationStructureBuildRangeInfoKHR*> accelerationBuildStructureRangeInfoPtrs(geometries.size());
    std::vector<VkAccelerationStructureGeometryKHR> accelerationStructureGeometries(geometries.size());
    std::vector<uint32_t> maxPrimitiveCounts(geometries.size());
    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
      // geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR; 
      //   means, anyhit shader is disabled

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR; 
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on build...

      geom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
      geom.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;

      // vertex data
      geom.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
      geom.geometry.triangles.vertexData.deviceAddress = 
        geometries[gid]->vertex.buffers[0]->address + geometries[gid]->vertex.offset;
      geom.geometry.triangles.vertexStride = geometries[gid]->vertex.stride;
      geom.geometry.triangles.maxVertex = geometries[gid]->vertex.count;

      // index data
      geom.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
      // note, offset accounted for in range
      geom.geometry.triangles.indexData.deviceAddress = geometries[gid]->index.buffer->address; 
      maxPrimitiveCounts[gid] = geometries[gid]->index.count;
      
      // transform data
      // note, offset accounted for in range
      geom.geometry.triangles.transformData.hostAddress = nullptr;
      // if the above is null, then that indicates identity

      auto &geomRange = accelerationBuildStructureRangeInfos[gid];
      accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
      geomRange.primitiveCount = geometries[gid]->index.count;
      geomRange.primitiveOffset = geometries[gid]->index.offset;
      geomRange.firstVertex = geometries[gid]->index.firstVertex;
      geomRange.transformOffset = 0;
    }

    // Get size info
    VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
    accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    accelerationStructureBuildGeometryInfo.flags = flags;
    accelerationStructureBuildGeometryInfo.geometryCount = accelerationStructureGeometries.size();
    accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

    VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
    accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    gprt::vkGetAccelerationStructureBuildSizes(
      logicalDevice,
      VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
      &accelerationStructureBuildGeometryInfo,
      maxPrimitiveCounts.data(),
      &accelerationStructureBuildSizesInfo
    );
    
    if (accelBuffer && accelBuffer->size != accelerationStructureBuildSizesInfo.accelerationStructureSize)
    {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete(accelBuffer);
      accelBuffer = nullptr;
    }

    if (!accelBuffer) {
      accelBuffer = new Buffer(
        physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 
        // means we can use this buffer as a means of storing an acceleration structure
        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | 
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
        // means that this memory is stored directly on the device 
        //  (rather than the host, or in a special host/device section)
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        accelerationStructureBuildSizesInfo.accelerationStructureSize
      );

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(
        logicalDevice,
        &accelerationStructureCreateInfo, 
        nullptr,
        &accelerationStructure
      );
      if (err) GPRT_RAISE("failed to create acceleration structure for triangle accel build! : \n" + errorString(err));
    }

    if (scratchBuffer && scratchBuffer->size != accelerationStructureBuildSizesInfo.buildScratchSize)
    {
      scratchBuffer->destroy();
      delete(scratchBuffer);
      scratchBuffer = nullptr;
    }

    if (!scratchBuffer) {
      scratchBuffer = new Buffer(
        physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 
        // means that the buffer can be used in a VkDescriptorBufferInfo. // Is this required? If not, remove this...
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | 
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
        // means that this memory is stored directly on the device 
        //  (rather than the host, or in a special host/device section)
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        accelerationStructureBuildSizesInfo.buildScratchSize
      );
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    accelerationBuildGeometryInfo.dstAccelerationStructure = accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->address;

    // Build the acceleration structure on the device via a one-time command buffer submission
    // Some implementations may support acceleration structure building on the host (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands), but we prefer device builds
    // VkCommandBuffer commandBuffer = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err) GPRT_RAISE("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(
      commandBuffer,
      1,
      &accelerationBuildGeometryInfo,
      accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err) GPRT_RAISE("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

    // VkFenceCreateInfo fenceInfo {};
    // fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    // fenceInfo.flags = 0;
    // VkFence fence;
    // err = vkCreateFence(logicalDevice, &fenceInfo, nullptr, &fence);
    // if (err) GPRT_RAISE("failed to create fence for triangle accel build! : \n" + errorString(err));

    err = vkQueueSubmit(queue, 1, &submitInfo, nullptr);
    if (err) GPRT_RAISE("failed to submit to queue for triangle accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err) GPRT_RAISE("failed to wait for queue idle for triangle accel build! : \n" + errorString(err));

    // Wait for the fence to signal that command buffer has finished executing
    // err = vkWaitForFences(logicalDevice, 1, &fence, VK_TRUE, 100000000000 /*timeout*/);
    // if (err) GPRT_RAISE("failed to wait for fence for triangle accel build! : \n" + errorString(err));
    // vkDestroyFence(logicalDevice, fence, nullptr);

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);
  }

  void destroy() { 
    if (accelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
    }

    if (accelBuffer) {
      accelBuffer->destroy();
      delete accelBuffer;
      accelBuffer = nullptr;
    }

    if (scratchBuffer) {
      scratchBuffer->destroy();
      delete scratchBuffer;
      scratchBuffer = nullptr;
    }
  };
};

struct AABBAccel : public Accel {
  std::vector<AABBGeom*> geometries; 

  // todo, accept this in constructor
  VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

  AABBAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer commandBuffer, VkQueue queue,
    size_t numGeometries, AABBGeom* geometries) : Accel(physicalDevice, logicalDevice, commandBuffer, queue) 
  {
    this->geometries.resize(numGeometries);
    memcpy(this->geometries.data(), geometries, sizeof(GPRTGeom*) * numGeometries);
  };
  
  ~AABBAccel() {};

  AccelType getType() {return GPRT_AABB_ACCEL;}

  void build(std::map<std::string, Stage> internalStages, std::vector<Accel*> accels, uint32_t numRayTypes) {
    VkResult err;

    std::vector<VkAccelerationStructureBuildRangeInfoKHR> accelerationBuildStructureRangeInfos(geometries.size());
    std::vector<VkAccelerationStructureBuildRangeInfoKHR*> accelerationBuildStructureRangeInfoPtrs(geometries.size());
    std::vector<VkAccelerationStructureGeometryKHR> accelerationStructureGeometries(geometries.size());
    std::vector<uint32_t> maxPrimitiveCounts(geometries.size());

    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
      // geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR; 
      //   means, anyhit shader is disabled

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR; 
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on build...

      geom.geometryType = VK_GEOMETRY_TYPE_AABBS_KHR;

      // aabb data
      geom.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
      geom.geometry.aabbs.pNext = VK_NULL_HANDLE;
      geom.geometry.aabbs.data.deviceAddress = geometries[gid]->aabb.buffers[0]->address;
      geom.geometry.aabbs.stride = geometries[gid]->aabb.stride;

      auto &geomRange = accelerationBuildStructureRangeInfos[gid];
      accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
      geomRange.primitiveCount = geometries[gid]->aabb.count;
      geomRange.primitiveOffset = geometries[gid]->aabb.offset;
      geomRange.firstVertex = 0; // unused 
      geomRange.transformOffset = 0;

      maxPrimitiveCounts[gid] = geometries[gid]->aabb.count;
    }

    // Get size info
    VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
    accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    accelerationStructureBuildGeometryInfo.flags = flags;
    accelerationStructureBuildGeometryInfo.geometryCount = accelerationStructureGeometries.size();
    accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

    VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
    accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    gprt::vkGetAccelerationStructureBuildSizes(
      logicalDevice,
      VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
      &accelerationStructureBuildGeometryInfo,
      maxPrimitiveCounts.data(),
      &accelerationStructureBuildSizesInfo
    );

    if (accelBuffer && accelBuffer->size != accelerationStructureBuildSizesInfo.accelerationStructureSize)
    {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete(accelBuffer);
      accelBuffer = nullptr;
    }
    
    if (!accelBuffer) {
      accelBuffer = new Buffer(
        physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 
        // means we can use this buffer as a means of storing an acceleration structure
        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | 
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
        // means that this memory is stored directly on the device 
        //  (rather than the host, or in a special host/device section)
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        accelerationStructureBuildSizesInfo.accelerationStructureSize
      );

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(
        logicalDevice,
        &accelerationStructureCreateInfo, 
        nullptr,
        &accelerationStructure
      );
      if (err) GPRT_RAISE("failed to create acceleration structure for AABB accel build! : \n" + errorString(err));
    }

    if (scratchBuffer && scratchBuffer->size != accelerationStructureBuildSizesInfo.buildScratchSize)
    {
      scratchBuffer->destroy();
      delete(scratchBuffer);
      scratchBuffer = nullptr;
    }
    
    if (!scratchBuffer) {
      scratchBuffer = new Buffer(
        physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 
        // means that the buffer can be used in a VkDescriptorBufferInfo. // Is this required? If not, remove this...
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | 
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
        // means that this memory is stored directly on the device 
        //  (rather than the host, or in a special host/device section)
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        accelerationStructureBuildSizesInfo.buildScratchSize
      );
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    accelerationBuildGeometryInfo.dstAccelerationStructure = accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->address;

    // Build the acceleration structure on the device via a one-time command buffer submission
    // Some implementations may support acceleration structure building on the host (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands), but we prefer device builds
    // VkCommandBuffer commandBuffer = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err) GPRT_RAISE("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(
      commandBuffer,
      1,
      &accelerationBuildGeometryInfo,
      accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err) GPRT_RAISE("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(queue, 1, &submitInfo, nullptr);
    if (err) GPRT_RAISE("failed to submit to queue for AABB accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err) GPRT_RAISE("failed to wait for queue idle for AABB accel build! : \n" + errorString(err));

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);
  }

  void destroy() { 
    if (accelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
    }

    if (accelBuffer) {
      accelBuffer->destroy();
      delete accelBuffer;
      accelBuffer = nullptr;
    }

    if (scratchBuffer) {
      scratchBuffer->destroy();
      delete scratchBuffer;
      scratchBuffer = nullptr;
    }
  };
};

struct InstanceAccel : public Accel {
  size_t numInstances;
  std::vector<Accel*> instances; 

  // the total number of geometries referenced by this instance accel's BLASes
  size_t numGeometries = -1;

  size_t instanceOffset = -1;

  Buffer *instancesBuffer = nullptr;
  Buffer *accelAddressesBuffer = nullptr;
  Buffer *instanceOffsetsBuffer = nullptr;

  struct {
    Buffer* buffer = nullptr;
    size_t stride = 0;
    size_t offset = 0;
  } transforms;

  struct {
    Buffer* buffer = nullptr;
    // size_t stride = 0;
    // size_t offset = 0;
  } references;

  struct {
    Buffer* buffer = nullptr;
    // size_t stride = 0;
    // size_t offset = 0;
  } offsets;

  // todo, accept this in constructor
  VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

  InstanceAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer commandBuffer, VkQueue queue,
    size_t numInstances, GPRTAccel* instances) : Accel(physicalDevice, logicalDevice, commandBuffer, queue) 
  {
    this->numInstances = numInstances;

    if (instances) {
      this->instances.resize(numInstances);
      memcpy(this->instances.data(), instances, sizeof(GPRTAccel*) * numInstances);

      // count number of geometry referenced.
      size_t numGeometry = 0;
      for (uint32_t j = 0; j < this->instances.size(); ++j) {
        if (this->instances[j]->getType() == GPRT_TRIANGLE_ACCEL) {
          TriangleAccel *triangleAccel = (TriangleAccel*) this->instances[j];
          numGeometry += triangleAccel->geometries.size();
        }
        else if (this->instances[j]->getType() == GPRT_AABB_ACCEL) {
          AABBAccel *aabbAccel = (AABBAccel*) this->instances[j];
          numGeometry += aabbAccel->geometries.size();
        }
        else {
          GPRT_RAISE("Unaccounted for BLAS type!");
        }
      }
      this->numGeometries = numGeometry;
    }

    instancesBuffer = new Buffer(
      physicalDevice, logicalDevice, commandBuffer, queue,
      // I guess I need this to use these buffers as input to tree builds?
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | 
      // means we can get this buffer's address with vkGetBufferDeviceAddress
      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
      // means that this memory is stored directly on the device 
      //  (rather than the host, or in a special host/device section)
      // VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
      // VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, // temporary
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
      sizeof(VkAccelerationStructureInstanceKHR) * numInstances
    );
  };
  
  ~InstanceAccel() {};

  void setTransforms(
    Buffer* transforms,
    size_t stride,
    size_t offset
    ) 
  {
    // assuming no motion blurred triangles for now, so we assume 1 transform per instance
    this->transforms.buffer = transforms;
    this->transforms.stride = stride;
    this->transforms.offset = offset;
  }

  void setReferences(
    Buffer* references = nullptr//,
    // size_t count,
    // size_t stride,
    // size_t offset
    ) 
  {
    this->references.buffer = references;
  }

  void setOffsets(
    Buffer* offsets = nullptr//,
    // size_t count,
    // size_t stride,
    // size_t offset
    ) 
  {
    this->offsets.buffer = offsets;
  }

  void setNumGeometries(
    size_t numGeometries
  )
  {
    this->numGeometries = numGeometries;
  }

  size_t getNumGeometries() {
    if (this->numGeometries == -1) {
      GPRT_RAISE("Error, numGeometries for this instance must be set by the user!");
    }
    return this->numGeometries;
  }

  AccelType getType() {return GPRT_INSTANCE_ACCEL;}

  void build(std::map<std::string, Stage> internalStages, std::vector<Accel*> accels, uint32_t numRayTypes) {
    VkResult err;

    // Compute the instance offset for the SBT record. 
    //   The instance shader binding table record offset is the total number 
    //   of geometries referenced by all instances up until this instance tree
    //   multiplied by the number of ray types.
    uint64_t instanceShaderBindingTableRecordOffset = 0;
    for (uint32_t i = 0; i < accels.size(); ++i) {
      if (accels[i] == this) break;
      if (accels[i]->getType() == GPRT_INSTANCE_ACCEL) {
        InstanceAccel *instanceAccel = (InstanceAccel*) accels[i];
        size_t numGeometry = instanceAccel->getNumGeometries();
        instanceShaderBindingTableRecordOffset += numGeometry * numRayTypes;
      }
    }
    instanceOffset = instanceShaderBindingTableRecordOffset;
    
    // No instance addressed provided, so we need to supply our own.
    uint64_t referencesAddress;
    if (references.buffer == nullptr) {
      // delete if not big enough
      if (accelAddressesBuffer && accelAddressesBuffer->size != numInstances * sizeof(uint64_t))
      {
        accelAddressesBuffer->destroy();
        delete accelAddressesBuffer;
        accelAddressesBuffer = nullptr;
      }
      
      // make buffer if not made yet
      if (accelAddressesBuffer == nullptr) {
        accelAddressesBuffer = new Buffer(
          physicalDevice, logicalDevice, commandBuffer, queue,
          // I guess I need this to use these buffers as input to tree builds?
          VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | 
          // means we can get this buffer's address with vkGetBufferDeviceAddress
          VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
          // means that this memory is stored directly on the device 
          //  (rather than the host, or in a special host/device section)
          // VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | // temporary (doesn't work on AMD)
          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, // temporary
          sizeof(uint64_t) * numInstances
        );
      }

      // transfer over addresses
      std::vector<uint64_t> addresses(numInstances);
      for (uint32_t i = 0; i < numInstances; ++i) 
        addresses[i] = this->instances[i]->address;
      accelAddressesBuffer->map();
      memcpy(accelAddressesBuffer->mapped, addresses.data(), sizeof(uint64_t) * numInstances);
      accelAddressesBuffer->unmap();
      referencesAddress = accelAddressesBuffer->address;
    }
    // Instance acceleration structure references provided by the user
    else {
      referencesAddress = references.buffer->address;
    }

    // No instance offsets provided, so we need to supply our own.
    uint64_t instanceOffsetsAddress;
    if (offsets.buffer == nullptr) {
      // delete if not big enough
      if (instanceOffsetsBuffer && instanceOffsetsBuffer->size != numInstances * sizeof(uint64_t))
      {
        instanceOffsetsBuffer->destroy();
        delete instanceOffsetsBuffer;
        instanceOffsetsBuffer = nullptr;
      }
      
      // make buffer if not made yet
      if (instanceOffsetsBuffer == nullptr) {
        instanceOffsetsBuffer = new Buffer(
          physicalDevice, logicalDevice, commandBuffer, queue,
          // I guess I need this to use these buffers as input to tree builds?
          VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | 
          // means we can get this buffer's address with vkGetBufferDeviceAddress
          VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
          // means that this memory is stored directly on the device 
          // (rather than the host, or in a special host/device section)
          // VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | // temporary (doesn't work on AMD)
          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, // temporary
          sizeof(uint64_t) * numInstances
        );
      }

      // transfer over offsets
      std::vector<int32_t> blasOffsets(numInstances);
      int offset = 0;
      for (uint32_t i = 0; i < numInstances; ++i) {
        blasOffsets[i] = offset;

        if (this->instances[i]->getType() == GPRT_TRIANGLE_ACCEL) 
        {
          TriangleAccel* triAccel = (TriangleAccel*)this->instances[i];
          offset += triAccel->geometries.size() * numRayTypes;
        } else if (this->instances[i]->getType() == GPRT_AABB_ACCEL) {
          AABBAccel* aabbAccel = (AABBAccel*)this->instances[i];
          offset += aabbAccel->geometries.size() * numRayTypes;
        } else {
          GPRT_RAISE("Error, unknown instance type");
        }
      }
      instanceOffsetsBuffer->map();
      memcpy(instanceOffsetsBuffer->mapped, blasOffsets.data(), sizeof(int32_t) * numInstances);
      instanceOffsetsBuffer->unmap();
      instanceOffsetsAddress = instanceOffsetsBuffer->address;
    }
    // Instance acceleration structure references provided by the user
    else {
      instanceOffsetsAddress = offsets.buffer->address;
    }

    // No instance addressed provided, so we assume identity.
    uint64_t transformBufferAddress;
    if (transforms.buffer == nullptr) {
      transformBufferAddress = -1;
    } else {
      transformBufferAddress = transforms.buffer->address;
    }

    // Use a compute shader to copy transforms into instances buffer
    VkCommandBufferBeginInfo cmdBufInfo{};
    struct PushConstants {
      uint64_t instanceBufferAddr;
      uint64_t transformBufferAddr;
      uint64_t accelReferencesAddr;
      uint64_t instanceShaderBindingTableRecordOffset;
      uint64_t transformOffset;
      uint64_t transformStride;
      uint64_t instanceOffsetsBufferAddr;
      uint64_t pad[16-7];
    } pushConstants;

    pushConstants.instanceBufferAddr = instancesBuffer->address;
    pushConstants.transformBufferAddr = transformBufferAddress;
    pushConstants.accelReferencesAddr = referencesAddress;
    pushConstants.instanceShaderBindingTableRecordOffset = instanceShaderBindingTableRecordOffset;
    pushConstants.transformOffset = transforms.offset;
    pushConstants.transformStride = transforms.stride;
    pushConstants.instanceOffsetsBufferAddr = instanceOffsetsAddress;

    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    vkCmdPushConstants(commandBuffer, internalStages["gprtFillInstanceData"].layout, 
      VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushConstants), &pushConstants
    );
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, internalStages["gprtFillInstanceData"].pipeline);
    // vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, 0);
    vkCmdDispatch(commandBuffer, numInstances, 1, 1);
    err = vkEndCommandBuffer(commandBuffer);

    VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(queue, 1, &submitInfo, nullptr);
    if (err) GPRT_RAISE("failed to submit to queue for instance accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err) GPRT_RAISE("failed to wait for queue idle for instance accel build! : \n" + errorString(err));

    VkAccelerationStructureGeometryKHR accelerationStructureGeometry{};
    accelerationStructureGeometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
    accelerationStructureGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    accelerationStructureGeometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
    accelerationStructureGeometry.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    accelerationStructureGeometry.geometry.instances.arrayOfPointers = VK_FALSE;
    accelerationStructureGeometry.geometry.instances.data.deviceAddress = instancesBuffer->address;

    // Get size info
    /*
    The pSrcAccelerationStructure, dstAccelerationStructure, and mode members of pBuildInfo are ignored. 
    Any VkDeviceOrHostAddressKHR members of pBuildInfo are ignored by this command, except that the hostAddress member 
    of VkAccelerationStructureGeometryTrianglesDataKHR::transformData will be examined to check if it is NULL.*
    */
    VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
    accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    accelerationStructureBuildGeometryInfo.geometryCount = 1;
    accelerationStructureBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;

    uint32_t primitive_count = numInstances;

    VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
    accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    gprt::vkGetAccelerationStructureBuildSizes(
      logicalDevice, 
      VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
      &accelerationStructureBuildGeometryInfo,
      &primitive_count,
      &accelerationStructureBuildSizesInfo);
    

    if (accelBuffer && accelBuffer->size != accelerationStructureBuildSizesInfo.accelerationStructureSize) {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete(accelBuffer);
      accelBuffer = nullptr;
    }

    if (!accelBuffer) {
      accelBuffer = new Buffer(
        physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 
        // means we can use this buffer as a means of storing an acceleration structure
        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | 
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
        // means that this memory is stored directly on the device 
        //  (rather than the host, or in a special host/device section)
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        accelerationStructureBuildSizesInfo.accelerationStructureSize
      );

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
      err = gprt::vkCreateAccelerationStructure(
        logicalDevice,
        &accelerationStructureCreateInfo, 
        nullptr,
        &accelerationStructure
      );
      if (err) GPRT_RAISE("failed to create acceleration structure for instance accel build! : \n" + errorString(err));
    }

    if (scratchBuffer && scratchBuffer->size != accelerationStructureBuildSizesInfo.buildScratchSize)
    {
      scratchBuffer->destroy();
      delete(scratchBuffer);
      scratchBuffer = nullptr;
    }

    if (!scratchBuffer) {
      scratchBuffer = new Buffer(
        physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE,
        // means that the buffer can be used in a VkDescriptorBufferInfo. // Is this required? If not, remove this...
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | 
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, 
        // means that this memory is stored directly on the device 
        //  (rather than the host, or in a special host/device section)
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        accelerationStructureBuildSizesInfo.buildScratchSize
      );
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
    accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    accelerationBuildGeometryInfo.dstAccelerationStructure = accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = 1;
    accelerationBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;
    accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer->address;

    VkAccelerationStructureBuildRangeInfoKHR accelerationStructureBuildRangeInfo{};
    accelerationStructureBuildRangeInfo.primitiveCount = numInstances;
    accelerationStructureBuildRangeInfo.primitiveOffset = 0;
    accelerationStructureBuildRangeInfo.firstVertex = 0;
    accelerationStructureBuildRangeInfo.transformOffset = 0;
    std::vector<VkAccelerationStructureBuildRangeInfoKHR*> accelerationBuildStructureRangeInfoPtrs = { &accelerationStructureBuildRangeInfo };
    
    // // Build the acceleration structure on the device via a one-time command buffer submission
    // // Some implementations may support acceleration structure building on the host (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands), but we prefer device builds

    // VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
    if (err) GPRT_RAISE("failed to begin command buffer for instance accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(
      commandBuffer,
      1,
      &accelerationBuildGeometryInfo,
      accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(commandBuffer);
    if (err) GPRT_RAISE("failed to end command buffer for instance accel build! : \n" + errorString(err));

    // VkSubmitInfo submitInfo;
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueWaitIdle(queue);

    err = vkQueueSubmit(queue, 1, &submitInfo, nullptr);
    if (err) GPRT_RAISE("failed to submit to queue for instance accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(queue);
    if (err) GPRT_RAISE("failed to wait for queue idle for instance accel build! : \n" + errorString(err));

    // // // Wait for the fence to signal that command buffer has finished executing
    // // err = vkWaitForFences(logicalDevice, 1, &fence, VK_TRUE, 100000000000 /*timeout*/);
    // // if (err) GPRT_RAISE("failed to wait for fence for instance accel build! : \n" + errorString(err));
    // // vkDestroyFence(logicalDevice, fence, nullptr);

    VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
    accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
    accelerationDeviceAddressInfo.accelerationStructure = accelerationStructure;
    address = gprt::vkGetAccelerationStructureDeviceAddress(logicalDevice, &accelerationDeviceAddressInfo);
  }

  void destroy() { 
    if (accelerationStructure) {
      gprt::vkDestroyAccelerationStructure(logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
    }

    if (accelBuffer) {
      accelBuffer->destroy();
      delete accelBuffer;
      accelBuffer = nullptr;
    }

    if (scratchBuffer) {
      scratchBuffer->destroy();
      delete scratchBuffer;
      scratchBuffer = nullptr;
    }

    if (instancesBuffer) {
      instancesBuffer->destroy();
      delete instancesBuffer;
      instancesBuffer = nullptr;
    }

    if (accelAddressesBuffer) {
      accelAddressesBuffer->destroy();
      delete accelAddressesBuffer;
      accelAddressesBuffer = nullptr;
    }

    if (instanceOffsetsBuffer) {
      instanceOffsetsBuffer->destroy();
      delete instanceOffsetsBuffer;
      instanceOffsetsBuffer = nullptr;
    }
  };
};

/** @brief A collection of features that are requested to support before 
 * creating a GPRT context. These features might not be available on all
 * platforms.
*/
static struct RequestedFeatures {
  /** A window (VK_KHR_SURFACE, SWAPCHAIN, etc...)*/
  bool window = false;
  struct Window {
    uint32_t initialWidth;
    uint32_t initialHeight;
    std::string title;
  } windowProperties;
} requestedFeatures;

struct Context {
  VkApplicationInfo appInfo;

  // Vulkan instance, stores all per-application states
  VkInstance instance;
  std::vector<std::string> supportedInstanceExtensions;

  // optional windowing features
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  GLFWwindow* window = nullptr;
  VkExtent2D windowExtent;
  VkPresentModeKHR presentMode;
  VkSurfaceFormatKHR surfaceFormat;
  VkSurfaceCapabilitiesKHR surfaceCapabilities;
  uint32_t surfaceImageCount;
  VkSwapchainKHR swapchain = VK_NULL_HANDLE;
  std::vector<VkImage> swapchainImages;
  uint32_t currentImageIndex;
  VkSemaphore imageAvailableSemaphore = VK_NULL_HANDLE;
  VkSemaphore renderFinishedSemaphore = VK_NULL_HANDLE;
  VkFence inFlightFence = VK_NULL_HANDLE;

  // Physical device (GPU) that Vulkan will use
  VkPhysicalDevice physicalDevice;
  // Stores physical device properties (for e.g. checking device limits)
  VkPhysicalDeviceProperties deviceProperties;
  VkPhysicalDeviceRayTracingPipelinePropertiesKHR  rayTracingPipelineProperties;
  VkPhysicalDeviceAccelerationStructureFeaturesKHR accelerationStructureFeatures;
  // Stores the features available on the selected physical device (for e.g. checking if a feature is available)
  VkPhysicalDeviceFeatures deviceFeatures;
  // Stores all available memory (type) properties for the physical device
  VkPhysicalDeviceMemoryProperties deviceMemoryProperties;

  /** @brief Queue family properties of the chosen physical device */
  std::vector<VkQueueFamilyProperties> queueFamilyProperties;

  /** @brief Contains queue family indices */
  struct
  {
    uint32_t graphics;
    uint32_t compute;
    uint32_t transfer;
  } queueFamilyIndices;

  VkCommandBuffer graphicsCommandBuffer;
  VkCommandBuffer computeCommandBuffer;
  VkCommandBuffer transferCommandBuffer;

  /** @brief List of extensions supported by the chosen physical device */
  std::vector<std::string> supportedExtensions;

  /** @brief Set of physical device features to be enabled (must be set in the derived constructor) */
  VkPhysicalDeviceFeatures enabledFeatures{};
  /** @brief Set of device extensions to be enabled for this example (must be set in the derived constructor) */
  std::vector<const char*> enabledDeviceExtensions;
  std::vector<const char*> enabledInstanceExtensions;
  /** @brief Optional pNext structure for passing extension structures to device creation */
  void* deviceCreatepNextChain = nullptr;
  /** @brief Logical device, application's view of the physical device (GPU) */
  VkDevice logicalDevice;

  // Handle to the device graphics queue that command buffers are submitted to
  VkQueue graphicsQueue;
  VkQueue computeQueue;
  VkQueue transferQueue;


  // Depth buffer format (selected during Vulkan initialization)
  VkFormat depthFormat;
  // Command buffer pool
  VkCommandPool graphicsCommandPool;
  VkCommandPool computeCommandPool;
  VkCommandPool transferCommandPool;
  VkQueryPool queryPool;
  bool queryRequested = false;

  /** @brief Pipeline stages used to wait at for graphics queue submissions */
  VkPipelineStageFlags submitPipelineStages = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT; // not sure I need this one
  // Contains command buffers and semaphores to be presented to the queue
  VkSubmitInfo submitInfo;
    // Command buffers used for rendering
    // std::vector<VkCommandBuffer> drawCmdBuffers;
  // Global render pass for frame buffer writes
  VkRenderPass renderPass = VK_NULL_HANDLE;
  // List of available frame buffers (same as number of swap chain images)
  std::vector<VkFramebuffer>frameBuffers;
  // Active frame buffer index
  uint32_t currentBuffer = 0;
  // Descriptor set pool
  VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
  // List of shader modules created (stored for cleanup)
  std::vector<VkShaderModule> shaderModules;
  // Pipeline cache object
  VkPipelineCache pipelineCache;
    
  // ray tracing pipeline and layout
  VkPipeline pipeline = VK_NULL_HANDLE;
  VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;

  std::vector<Compute*> computePrograms;
  std::vector<RayGen*> raygenPrograms;
  std::vector<Miss*> missPrograms;
  std::vector<GeomType*> geomTypes;

  std::vector<Accel*> accels;

  std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups{};
  Buffer shaderBindingTable;

  uint32_t numRayTypes = 1;

  // struct InternalStages {
  //   // for copying transforms into the instance buffer
  //   std::string fillInstanceDataEntryPoint = "gprtFillInstanceData";
  //   VkPipelineLayout fillInstanceDataPipelineLayout;
  //   VkShaderModule fillInstanceDataShaderModule;
  //   VkPipeline fillInstanceDataPipeline;
  // }
  Stage fillInstanceDataStage;
  Module *internalModule = nullptr;

  /*! returns whether logging is enabled */
  inline static bool logging()
  {
#ifdef NDEBUG
  return false;
#else
  return true;
#endif
  }

  /*! returns whether validation is enabled */
  inline static bool validation()
  {
#ifdef NDEBUG
  return false;
#else
  return true;
#endif
  }

  void freeDebugCallback(VkInstance instance)
  {
    if (gprt::debugUtilsMessenger != VK_NULL_HANDLE)
    {
      gprt::vkDestroyDebugUtilsMessengerEXT(instance, gprt::debugUtilsMessenger, nullptr);
    }
  }

  size_t getNumHitRecords() {
    // The total number of geometries is the number of geometries referenced 
    // by each top level acceleration structure.
    int totalGeometries = 0;
    for (int accelID = 0; accelID < accels.size(); ++accelID) {
      Accel *accel = accels[accelID];
      if (!accel) continue;
      if (accel->getType() == GPRT_INSTANCE_ACCEL) {
        InstanceAccel* tlas = (InstanceAccel*) accel;
        totalGeometries += tlas->getNumGeometries();
      }
    }

    int numHitRecords = totalGeometries * numRayTypes;
    return numHitRecords;
  }

  Context(int32_t *requestedDeviceIDs, int numRequestedDevices) {
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "GPRT";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "GPRT";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_1;
    appInfo.pNext = VK_NULL_HANDLE;


    /// 1. Create Instance
    std::vector<const char*> instanceExtensions;// = { VK_KHR_SURFACE_EXTENSION_NAME };
    #if defined(VK_USE_PLATFORM_MACOS_MVK) && (VK_HEADER_VERSION >= 216)
      instanceExtensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
      instanceExtensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    #endif

    // Get extensions supported by the instance and store for later use
    uint32_t instExtCount = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &instExtCount, nullptr);
    if (instExtCount > 0)
    {
      std::vector<VkExtensionProperties> extensions(instExtCount);
      if (vkEnumerateInstanceExtensionProperties(nullptr, &instExtCount, &extensions.front()) == VK_SUCCESS)
      {
        for (VkExtensionProperties extension : extensions)
        {
          supportedInstanceExtensions.push_back(extension.extensionName);
        }
      }
    }

    // Enabled requested instance extensions
    if (enabledInstanceExtensions.size() > 0)
    {
      for (const char * enabledExtension : enabledInstanceExtensions)
      {
        // Output message if requested extension is not available
        if (std::find(supportedInstanceExtensions.begin(), supportedInstanceExtensions.end(), enabledExtension) == supportedInstanceExtensions.end())
        {
          std::cerr << "Enabled instance extension \"" << enabledExtension << "\" is not present at instance level\n";
        }
        instanceExtensions.push_back(enabledExtension);
      }
    }

    VkValidationFeatureEnableEXT enabled[] = {VK_VALIDATION_FEATURE_ENABLE_DEBUG_PRINTF_EXT};
    VkValidationFeaturesEXT      validationFeatures{VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT};
    validationFeatures.disabledValidationFeatureCount = 0;
    validationFeatures.enabledValidationFeatureCount  = 1;
    validationFeatures.pDisabledValidationFeatures    = nullptr;
    validationFeatures.pEnabledValidationFeatures     = enabled;

    VkInstanceCreateInfo instanceCreateInfo{};
    instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instanceCreateInfo.pApplicationInfo = &appInfo;
    //instanceCreateInfo.pNext = VK_NULL_HANDLE;
    instanceCreateInfo.pNext = &validationFeatures;

    uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions;
    if (requestedFeatures.window) {
      if (!glfwInit())
        throw std::runtime_error("Can't initialize GLFW");

      if (!glfwVulkanSupported()) {
        GPRT_RAISE("Window requested but unsupported!");
      }
      glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
      for (uint32_t i = 0; i < glfwExtensionCount; ++i) {
        instanceExtensions.push_back(glfwExtensions[i]);
      }
    }

    // Number of validation layers allowed
    instanceCreateInfo.enabledLayerCount = 0;

    #if defined(VK_USE_PLATFORM_MACOS_MVK) && (VK_HEADER_VERSION >= 216)
      instanceCreateInfo.flags = VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    #endif

    if (validation()){
      instanceExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    if (instanceExtensions.size() > 0)
    {
      instanceCreateInfo.enabledExtensionCount = (uint32_t)instanceExtensions.size();
      instanceCreateInfo.ppEnabledExtensionNames = instanceExtensions.data();
    }

    instanceCreateInfo.ppEnabledLayerNames = nullptr;
    instanceCreateInfo.enabledLayerCount = 0;

    VkResult err;

    err = vkCreateInstance(&instanceCreateInfo, nullptr, &instance);
    if (err) {
      GPRT_RAISE("failed to create instance! : \n" + errorString(err));
    }

    /// 1.5 - create a window and surface if requested
    if (requestedFeatures.window) {
      glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
      // todo, allow the window to resize and recreate swapchain
      glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE); 
      window = glfwCreateWindow(
        requestedFeatures.windowProperties.initialWidth, 
        requestedFeatures.windowProperties.initialHeight, 
        requestedFeatures.windowProperties.title.c_str(), 
        NULL, NULL);

      VkResult err = glfwCreateWindowSurface(instance, window, nullptr, &surface);
      if (err != VK_SUCCESS) {
        GPRT_RAISE("failed to create window surface! : \n" + errorString(err));
      }
      // Poll some initial event values
      glfwPollEvents();
    }

    /// 2. Select a Physical Device


    // If requested, we enable the default validation layers for debugging
    if (validation())
    {
      gprt::vkCreateDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT"));
      gprt::vkDestroyDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));

      VkDebugUtilsMessengerCreateInfoEXT debugUtilsMessengerCI{};
      debugUtilsMessengerCI.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
      debugUtilsMessengerCI.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                              VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT |
                                              VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT ;
      debugUtilsMessengerCI.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT;
      debugUtilsMessengerCI.pfnUserCallback = debugUtilsMessengerCallback;
      VkResult result = gprt::vkCreateDebugUtilsMessengerEXT(instance, &debugUtilsMessengerCI, nullptr, &gprt::debugUtilsMessenger);
      assert(result == VK_SUCCESS);
    }

    // Physical device
    uint32_t gpuCount = 0;
    // Get number of available physical devices
    VK_CHECK_RESULT(vkEnumeratePhysicalDevices(instance, &gpuCount, nullptr));
    if (gpuCount == 0) {
      GPRT_RAISE("No device with Vulkan support found : \n" + errorString(err));
    }
    // Enumerate devices
    std::vector<VkPhysicalDevice> physicalDevices(gpuCount);
    err = vkEnumeratePhysicalDevices(instance, &gpuCount, physicalDevices.data());
    if (err) {
      GPRT_RAISE("Could not enumerate physical devices : \n" + errorString(err));
    }

    // GPU selection

    auto physicalDeviceTypeString = [](VkPhysicalDeviceType type) -> std::string
    {
      switch (type)
      {
    #define STR(r) case VK_PHYSICAL_DEVICE_TYPE_ ##r: return #r
        STR(OTHER);
        STR(INTEGRATED_GPU);
        STR(DISCRETE_GPU);
        STR(VIRTUAL_GPU);
        STR(CPU);
    #undef STR
      default: return "UNKNOWN_DEVICE_TYPE";
      }
    };

    auto extensionSupported = [](std::string extension, std::vector<std::string> supportedExtensions) -> bool {
      return (std::find(supportedExtensions.begin(), supportedExtensions.end(), extension) != supportedExtensions.end());
    };

    /* function that checks if the selected physical device meets requirements */
    auto checkDeviceExtensionSupport = [](VkPhysicalDevice device, 
      std::vector<const char*> deviceExtensions) -> bool 
    {
      uint32_t extensionCount;
      vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

      std::vector<VkExtensionProperties> availableExtensions(extensionCount);
      vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

      std::set<std::string> requiredExtensions;
      for (auto &cstr : deviceExtensions) {
        requiredExtensions.insert(std::string(cstr));
      }

      for (const auto& extension : availableExtensions) {
        requiredExtensions.erase(extension.extensionName);
      }

      return requiredExtensions.empty();
    };

    // Ray tracing related extensions required
    enabledDeviceExtensions.push_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
    enabledDeviceExtensions.push_back(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME);

    // Required by VK_KHR_acceleration_structure
    enabledDeviceExtensions.push_back(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
    enabledDeviceExtensions.push_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
    enabledDeviceExtensions.push_back(VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME);

    enabledDeviceExtensions.push_back(VK_KHR_SHADER_NON_SEMANTIC_INFO_EXTENSION_NAME);

    // Required for VK_KHR_ray_tracing_pipeline
    enabledDeviceExtensions.push_back(VK_KHR_SPIRV_1_4_EXTENSION_NAME);

    enabledDeviceExtensions.push_back(VK_KHR_RAY_QUERY_EXTENSION_NAME);

    // Required by VK_KHR_spirv_1_4
    enabledDeviceExtensions.push_back(VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME);

    if (requestedFeatures.window)
    {
    	// If the device will be used for presenting to a display via a swapchain 
      // we need to request the swapchain extension
    	enabledDeviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    }

    #if defined(VK_USE_PLATFORM_MACOS_MVK) && (VK_HEADER_VERSION >= 216)
      enabledDeviceExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
    #endif

    // Select physical device to be used
    // Defaults to the first device unless specified by command line
    uint32_t selectedDevice = -1; // TODO

    std::vector<uint32_t> usableDevices;
    
    LOG("Searching for usable Vulkan physical device...");
    for (uint32_t i = 0; i < gpuCount; i++) {
      VkPhysicalDeviceProperties deviceProperties;
      vkGetPhysicalDeviceProperties(physicalDevices[i], &deviceProperties);
      std::string message = std::string("Device [") + std::to_string(i) + std::string("] : ") + std::string(deviceProperties.deviceName);
      message += std::string(", Type : ") + physicalDeviceTypeString(deviceProperties.deviceType);
      message += std::string(", API : ") + std::to_string(deviceProperties.apiVersion >> 22) + std::string(".")
               + std::to_string(((deviceProperties.apiVersion >> 12) & 0x3ff)) + std::string(".")
               + std::to_string(deviceProperties.apiVersion & 0xfff);
      LOG(message);
      
      if (checkDeviceExtensionSupport(physicalDevices[i], enabledDeviceExtensions)) {
        usableDevices.push_back(i);
      } else {
        /* Explain why we aren't using this device */
        for (const char* enabledExtension : enabledDeviceExtensions)
        {
          if (!extensionSupported(enabledExtension, supportedExtensions)) {
            LOG("Device unusable... Requested device extension \"" << 
              enabledExtension << "\" is not present.");
          }
        }
      }
    }

    if (usableDevices.size() == 0) {
      GPRT_RAISE("Unable to find physical device meeting requirements\n");
    }
    else {
      LOG("Selecting first usable device");
      selectedDevice = usableDevices[0];
    }

    physicalDevice = physicalDevices[selectedDevice];

    // Store properties (including limits), features and memory properties of the physical device
    // Device properties also contain limits and sparse properties
    vkGetPhysicalDeviceProperties(physicalDevice, &deviceProperties);

    // VkPhysicalDeviceRayTracingPipelinePropertiesKHR  rayTracingPipelineProperties{};
    rayTracingPipelineProperties = {};
    rayTracingPipelineProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
    VkPhysicalDeviceProperties2 deviceProperties2{};
    deviceProperties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    deviceProperties2.pNext = &rayTracingPipelineProperties;
    vkGetPhysicalDeviceProperties2(physicalDevice, &deviceProperties2);

    // Features should be checked by the end application before using them
    vkGetPhysicalDeviceFeatures(physicalDevice, &deviceFeatures);
    // Memory properties are used regularly for creating all kinds of buffers
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &deviceMemoryProperties);

    // Queue family properties, used for setting up requested queues upon device creation
    uint32_t queueFamilyCount;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, nullptr);
    assert(queueFamilyCount > 0);
    queueFamilyProperties.resize(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilyProperties.data());

    // Get list of supported extensions
    uint32_t devExtCount = 0;
    vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &devExtCount, nullptr);
    if (devExtCount > 0)
    {
      std::vector<VkExtensionProperties> extensions(devExtCount);
      if (vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &devExtCount, &extensions.front()) == VK_SUCCESS)
      {
        for (auto ext : extensions)
        {
          supportedExtensions.push_back(ext.extensionName);
        }
      }
    }

    // Now, create the logical device and all requested queues
    std::vector<VkDeviceQueueCreateInfo> queueCreateInfos{};
    // Get queue family indices for the requested queue family types
    // Note that the indices may overlap depending on the implementation

    auto getQueueFamilyIndex =
      [](VkQueueFlagBits queueFlags,
      std::vector<VkQueueFamilyProperties> queueFamilyProperties) -> uint32_t
      {
        // Dedicated queue for compute
        // Try to find a queue family index that supports compute but not graphics
        if (queueFlags & VK_QUEUE_COMPUTE_BIT)
          for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProperties.size()); i++)
            if ((queueFamilyProperties[i].queueFlags & queueFlags) &&
              ((queueFamilyProperties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0))
              return i;

        // Dedicated queue for transfer
        // Try to find a queue family index that supports transfer but not graphics and compute
        if (queueFlags & VK_QUEUE_TRANSFER_BIT)
          for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProperties.size()); i++)
            if ((queueFamilyProperties[i].queueFlags & queueFlags) &&
                ((queueFamilyProperties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0) &&
                ((queueFamilyProperties[i].queueFlags & VK_QUEUE_COMPUTE_BIT) == 0))
              return i;

        // For other queue types or if no separate compute queue is present, return the first one to support the requested flags
        for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProperties.size()); i++)
          if (queueFamilyProperties[i].queueFlags & queueFlags)
            return i;
        GPRT_RAISE("Could not find a matching queue family index");
      };

    const float defaultQueuePriority(0.0f);

    // Graphics queue
    // if (requestedQueueTypes & VK_QUEUE_GRAPHICS_BIT)
    {
      queueFamilyIndices.graphics = getQueueFamilyIndex(VK_QUEUE_GRAPHICS_BIT, queueFamilyProperties);
      VkDeviceQueueCreateInfo queueInfo{};
      queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
      queueInfo.queueFamilyIndex = queueFamilyIndices.graphics;
      queueInfo.queueCount = 1;
      queueInfo.pQueuePriorities = &defaultQueuePriority;
      queueCreateInfos.push_back(queueInfo);
    }
    // else
    // {
    //   queueFamilyIndices.graphics = 0;
    // }

    // Dedicated compute queue
    // if (requestedQueueTypes & VK_QUEUE_COMPUTE_BIT)
    {
      queueFamilyIndices.compute = getQueueFamilyIndex(VK_QUEUE_COMPUTE_BIT, queueFamilyProperties);
      if (queueFamilyIndices.compute != queueFamilyIndices.graphics)
      {
        // If compute family index differs, we need an additional queue create info for the compute queue
        VkDeviceQueueCreateInfo queueInfo{};
        queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueInfo.queueFamilyIndex = queueFamilyIndices.compute;
        queueInfo.queueCount = 1;
        queueInfo.pQueuePriorities = &defaultQueuePriority;
        queueCreateInfos.push_back(queueInfo);
      }
    }
    // else
    // {
    //   // Else we use the same queue
    //   queueFamilyIndices.compute = queueFamilyIndices.graphics;
    // }

    // Dedicated transfer queue
    // if (requestedQueueTypes & VK_QUEUE_TRANSFER_BIT)
    {
      queueFamilyIndices.transfer = getQueueFamilyIndex(VK_QUEUE_TRANSFER_BIT, queueFamilyProperties);
      if ((queueFamilyIndices.transfer != queueFamilyIndices.graphics) && (queueFamilyIndices.transfer != queueFamilyIndices.compute))
      {
        // If compute family index differs, we need an additional queue create info for the compute queue
        VkDeviceQueueCreateInfo queueInfo{};
        queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueInfo.queueFamilyIndex = queueFamilyIndices.transfer;
        queueInfo.queueCount = 1;
        queueInfo.pQueuePriorities = &defaultQueuePriority;
        queueCreateInfos.push_back(queueInfo);
      }
    }
    // else
    // {
    //   // Else we use the same queue
    //   queueFamilyIndices.transfer = queueFamilyIndices.graphics;
    // }

    /// 3. Create the logical device representation
    VkPhysicalDeviceBufferDeviceAddressFeatures bufferDeviceAddressFeatures = {};
    bufferDeviceAddressFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
    bufferDeviceAddressFeatures.bufferDeviceAddress = VK_TRUE;
    bufferDeviceAddressFeatures.bufferDeviceAddressCaptureReplay = VK_FALSE;
    bufferDeviceAddressFeatures.bufferDeviceAddressMultiDevice = VK_FALSE;
    bufferDeviceAddressFeatures.pNext = nullptr;

    VkPhysicalDeviceAccelerationStructureFeaturesKHR accelerationStructureFeatures{};
    accelerationStructureFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
    accelerationStructureFeatures.pNext = &bufferDeviceAddressFeatures;

    VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtPipelineFeatures{};
    rtPipelineFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
    rtPipelineFeatures.rayTracingPipeline = true;
    rtPipelineFeatures.pNext = &accelerationStructureFeatures;

    VkPhysicalDeviceRayQueryFeaturesKHR rtQueryFeatures{};
    rtQueryFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
    rtQueryFeatures.rayQuery = VK_TRUE;
    rtQueryFeatures.pNext = &rtPipelineFeatures;

    VkPhysicalDeviceFeatures2 deviceFeatures2{};
    deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    deviceFeatures2.pNext = &rtQueryFeatures;
    vkGetPhysicalDeviceFeatures2(physicalDevice, &deviceFeatures2);



    VkDeviceCreateInfo deviceCreateInfo = {};
    deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    deviceCreateInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());;
    deviceCreateInfo.pQueueCreateInfos = queueCreateInfos.data();
    deviceCreateInfo.pEnabledFeatures =nullptr; //  &enabledFeatures; // TODO, remove or update enabledFeatures
    deviceCreateInfo.pNext = &deviceFeatures2;

    // If a pNext(Chain) has been passed, we need to add it to the device creation info
    // VkPhysicalDeviceFeatures2 physicalDeviceFeatures2{};
    // if (pNextChain) {
    //   physicalDeviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    //   physicalDeviceFeatures2.features = enabledFeatures;
    //   physicalDeviceFeatures2.pNext = pNextChain;
    //   deviceCreateInfo.pEnabledFeatures = nullptr;
    //   deviceCreateInfo.pNext = &physicalDeviceFeatures2;
    // }

    // // Enable the debug marker extension if it is present (likely meaning a debugging tool is present)
    // if (extensionSupported(VK_EXT_DEBUG_MARKER_EXTENSION_NAME))
    // {
    //   enabledDeviceExtensions.push_back(VK_EXT_DEBUG_MARKER_EXTENSION_NAME);
    //   enableDebugMarkers = true;
    // }

    

    if (enabledDeviceExtensions.size() > 0)
    {
      deviceCreateInfo.enabledExtensionCount = (uint32_t)enabledDeviceExtensions.size();
      deviceCreateInfo.ppEnabledExtensionNames = enabledDeviceExtensions.data();
    }

    // this->enabledFeatures = enabledFeatures;
    err = vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &logicalDevice);
    if (err) {
      GPRT_RAISE("Could not create logical devices : \n" + errorString(err));
    }

    // Get the ray tracing and acceleration structure related function pointers required by this sample
    gprt::vkGetBufferDeviceAddress = reinterpret_cast<PFN_vkGetBufferDeviceAddressKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetBufferDeviceAddressKHR"));
    gprt::vkCmdBuildAccelerationStructures = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCmdBuildAccelerationStructuresKHR"));
    gprt::vkBuildAccelerationStructures = reinterpret_cast<PFN_vkBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(logicalDevice, "vkBuildAccelerationStructuresKHR"));
    gprt::vkCreateAccelerationStructure = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCreateAccelerationStructureKHR"));
    gprt::vkDestroyAccelerationStructure = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(vkGetDeviceProcAddr(logicalDevice, "vkDestroyAccelerationStructureKHR"));
    gprt::vkGetAccelerationStructureBuildSizes = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetAccelerationStructureBuildSizesKHR"));
    gprt::vkGetAccelerationStructureDeviceAddress = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetAccelerationStructureDeviceAddressKHR"));
    gprt::vkCmdTraceRays = reinterpret_cast<PFN_vkCmdTraceRaysKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCmdTraceRaysKHR"));
    gprt::vkGetRayTracingShaderGroupHandles = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetRayTracingShaderGroupHandlesKHR"));
    gprt::vkCreateRayTracingPipelines = reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCreateRayTracingPipelinesKHR"));

    auto createCommandPool = [&](
      uint32_t queueFamilyIndex,
      VkCommandPoolCreateFlags createFlags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT)
      -> VkCommandPool
    {
      VkCommandPoolCreateInfo cmdPoolInfo = {};
      cmdPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
      cmdPoolInfo.queueFamilyIndex = queueFamilyIndex;
      cmdPoolInfo.flags = createFlags;
      VkCommandPool cmdPool;
      VK_CHECK_RESULT(vkCreateCommandPool(logicalDevice, &cmdPoolInfo, nullptr, &cmdPool));
      return cmdPool;
    };

    /// 4. Create Command Pools
    graphicsCommandPool = createCommandPool(queueFamilyIndices.graphics);
    computeCommandPool = createCommandPool(queueFamilyIndices.compute);
    transferCommandPool = createCommandPool(queueFamilyIndices.transfer);

    /// 4.5 Create a query pool to measure performance
    VkQueryPoolCreateInfo queryPoolCreateInfo{};
    queryPoolCreateInfo.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    queryPoolCreateInfo.pNext = nullptr; // Optional
    queryPoolCreateInfo.flags = 0; // Reserved for future use, must be 0!

    queryPoolCreateInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
    queryPoolCreateInfo.queryCount = 2; // for now, just two queries, a before and an after.

    err = vkCreateQueryPool(logicalDevice, &queryPoolCreateInfo, nullptr, &queryPool);
    if (err) GPRT_RAISE("Failed to create time query pool!");

    /// 5. Allocate command buffers
    VkCommandBufferAllocateInfo cmdBufAllocateInfo{};
    cmdBufAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmdBufAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdBufAllocateInfo.commandBufferCount = 1;

    cmdBufAllocateInfo.commandPool = graphicsCommandPool;
    err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &graphicsCommandBuffer);
    if (err) GPRT_RAISE("Could not create graphics command buffer : \n" + errorString(err));

    cmdBufAllocateInfo.commandPool = computeCommandPool;
    err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &computeCommandBuffer);
    if (err) GPRT_RAISE("Could not create compute command buffer : \n" + errorString(err));

    cmdBufAllocateInfo.commandPool = transferCommandPool;
    err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &transferCommandBuffer);
    if (err) GPRT_RAISE("Could not create transfer command buffer : \n" + errorString(err));

    /// 6. Get queue handles
    vkGetDeviceQueue(logicalDevice, queueFamilyIndices.graphics, 0, &graphicsQueue);
    vkGetDeviceQueue(logicalDevice, queueFamilyIndices.compute, 0, &computeQueue);
    vkGetDeviceQueue(logicalDevice, queueFamilyIndices.transfer, 0, &transferQueue);

    // 7. Create a module for internal device entry points
    internalModule = new Module(gprtDeviceCode);
    setupInternalStages(internalModule);

    // Swapchain semaphores and fences
    if (requestedFeatures.window) {
      VkSemaphoreCreateInfo semaphoreInfo{};
      semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

      VkFenceCreateInfo fenceInfo{};
      fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

      if (vkCreateSemaphore(logicalDevice, &semaphoreInfo, nullptr, &imageAvailableSemaphore) != VK_SUCCESS ||
        vkCreateSemaphore(logicalDevice, &semaphoreInfo, nullptr, &renderFinishedSemaphore) != VK_SUCCESS ||
        vkCreateFence(logicalDevice, &fenceInfo, nullptr, &inFlightFence) != VK_SUCCESS) {
        GPRT_RAISE("Failed to create swapchain semaphores");
      }
    }

    // Swapchain setup
    if (requestedFeatures.window) {
      VkSurfaceCapabilitiesKHR surfaceCapabilities;
      vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &surfaceCapabilities);

      // SRGB is a commonly supported surface format. It's also efficient to 
      // write to, so we assume this format.
      // Note, we don't use RGBA since NVIDIA doesn't support this...
      surfaceFormat.format = VK_FORMAT_B8G8R8A8_SRGB;
      surfaceFormat.colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
      bool surfaceFormatFound = false;
      std::vector<VkSurfaceFormatKHR> formats;
      uint32_t formatCount;
      vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);
      if (formatCount != 0) {
          formats.resize(formatCount);
          vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, formats.data());
      }
      for (uint32_t i = 0; i < formatCount; ++i)
        if (formats[i].format == surfaceFormat.format 
          && formats[i].colorSpace == surfaceFormat.colorSpace ) surfaceFormatFound = true;
      if (!surfaceFormatFound) 
        GPRT_RAISE("Error, unable to find RGBA8 SRGB surface format...");

      // For now, we assume the user wants FIFO, which is similar to vsync.
      // All vulkan implementations must support FIFO per-spec, so it's 
      // something we can depend on.
      presentMode = VK_PRESENT_MODE_FIFO_KHR;
      bool presentModeFound = false;
      std::vector<VkPresentModeKHR> presentModes;
      uint32_t presentModeCount;
      vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, nullptr);
      if (presentModeCount != 0) {
          presentModes.resize(presentModeCount);
          vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, presentModes.data());
      }
      for (uint32_t i = 0; i < presentModeCount; ++i)
        if (presentModes[i] == presentMode) presentModeFound = true;
      if (!presentModeFound) 
        GPRT_RAISE("Error, unable to find vsync present mode...");

      // Window extent is the number of pixels truly used by the surface. For 
      // high dps displays (like on mac) the window extent might be larger
      // than the screen coordinate size.
      if (surfaceCapabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        windowExtent = surfaceCapabilities.currentExtent;
      } else {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        VkExtent2D actualExtent = {
            static_cast<uint32_t>(width),
            static_cast<uint32_t>(height)
        };
        actualExtent.width = std::clamp(actualExtent.width, surfaceCapabilities.minImageExtent.width, surfaceCapabilities.maxImageExtent.width);
        actualExtent.height = std::clamp(actualExtent.height, surfaceCapabilities.minImageExtent.height, surfaceCapabilities.maxImageExtent.height);
        windowExtent = actualExtent;
      }

      // We request one more than the minimum number of images in the swapchain,
      // since choosing exactly the minimum can cause stalls. We also make sure
      // to not request more than the maximum number of images.
      surfaceImageCount = surfaceCapabilities.minImageCount + 1;
      if (surfaceCapabilities.maxImageCount > 0 && surfaceImageCount > surfaceCapabilities.maxImageCount) {
        surfaceImageCount = surfaceCapabilities.maxImageCount;
      }

      // Now, we have enough information to create our swapchain.
      VkSwapchainCreateInfoKHR createInfo{};
      createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
      createInfo.surface = surface;
      createInfo.minImageCount = surfaceImageCount;
      createInfo.imageFormat = surfaceFormat.format;
      createInfo.imageColorSpace = surfaceFormat.colorSpace;
      createInfo.imageExtent = windowExtent;
      createInfo.imageArrayLayers = 1; // might be 2 for stereoscopic 3D images like VR
      createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT; // might need to change this...
      // currently assuming graphics and present queue are the same...
      createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE; 
      createInfo.queueFamilyIndexCount = 0; // optional if exclusive
      createInfo.pQueueFamilyIndices = nullptr; // optional if exclusive
      // could use this to flip the image vertically if we want
      createInfo.preTransform = surfaceCapabilities.currentTransform; 
      // means, we don't composite this window with other OS windows.
      createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR; 
      createInfo.presentMode = presentMode;
      // clipped here means we don't care about color of obscured pixels, like 
      // those covered by other OS windows.
      createInfo.clipped = VK_TRUE;
      // Since the swapchain can go out of date, we need to supply the out of 
      // date swapchain here.
      createInfo.oldSwapchain = swapchain;

      // Create the swapchain
      err = vkCreateSwapchainKHR(logicalDevice, &createInfo, nullptr, &swapchain);
      if (err != VK_SUCCESS) {
        GPRT_RAISE("Error, failed to create swap chain : \n" + errorString(err));
      }

      // Now, receive the swapchain images 
      vkGetSwapchainImagesKHR(logicalDevice, swapchain, &surfaceImageCount, nullptr);
      swapchainImages.resize(surfaceImageCount);
      vkGetSwapchainImagesKHR(logicalDevice, swapchain, &surfaceImageCount, swapchainImages.data());

      /* Transition all images to presentable */
      for (uint32_t i = 0; i < swapchainImages.size(); ++i) {
        transitionImageLayout(swapchainImages[i], surfaceFormat.format, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
      }

      // And acquire the first image to use
      vkAcquireNextImageKHR(logicalDevice, swapchain, UINT64_MAX, 
        imageAvailableSemaphore, VK_NULL_HANDLE, &currentImageIndex);
    }
  };

  void destroy() {

    if (imageAvailableSemaphore) vkDestroySemaphore(logicalDevice, imageAvailableSemaphore, nullptr);
    if (renderFinishedSemaphore) vkDestroySemaphore(logicalDevice, renderFinishedSemaphore, nullptr);
    if (inFlightFence) vkDestroyFence(logicalDevice, inFlightFence, nullptr);

    if (swapchain) {
      vkDestroySwapchainKHR(logicalDevice, swapchain, nullptr);
    }
    if (window) {
      glfwDestroyWindow(window);
      glfwTerminate();
    }
    if (surface)
      vkDestroySurfaceKHR(instance, surface, nullptr);
    
    if (pipelineLayout)
      vkDestroyPipelineLayout(logicalDevice, pipelineLayout, nullptr);
    if (pipeline)
      vkDestroyPipeline(logicalDevice, pipeline, nullptr);
    
    if (fillInstanceDataStage.layout)
      vkDestroyPipelineLayout(logicalDevice, fillInstanceDataStage.layout, nullptr);
    if (fillInstanceDataStage.pipeline)
      vkDestroyPipeline(logicalDevice, fillInstanceDataStage.pipeline, nullptr);
    if (fillInstanceDataStage.module)
      vkDestroyShaderModule(logicalDevice, fillInstanceDataStage.module, nullptr);

    shaderBindingTable.destroy();
    vkFreeCommandBuffers(logicalDevice, graphicsCommandPool, 1, &graphicsCommandBuffer);
    vkFreeCommandBuffers(logicalDevice, computeCommandPool, 1, &computeCommandBuffer);
    vkFreeCommandBuffers(logicalDevice, transferCommandPool, 1, &transferCommandBuffer);
    vkDestroyCommandPool(logicalDevice, graphicsCommandPool, nullptr);
    vkDestroyCommandPool(logicalDevice, computeCommandPool, nullptr);
    vkDestroyCommandPool(logicalDevice, transferCommandPool, nullptr);
    vkDestroyQueryPool(logicalDevice, queryPool, nullptr);
    vkDestroyDevice(logicalDevice, nullptr);

    freeDebugCallback(instance);
    vkDestroyInstance(instance, nullptr);
  }

  ~Context() {};

  void buildSBT() {
    auto alignedSize = [](uint32_t value, uint32_t alignment) -> uint32_t {
      return (value + alignment - 1) & ~(alignment - 1);
    };

    auto align_to = [](uint64_t val, uint64_t align) -> uint64_t {
      return ((val + align - 1) / align) * align;
    };

    const uint32_t handleSize = rayTracingPipelineProperties.shaderGroupHandleSize;
    const uint32_t maxGroupSize = rayTracingPipelineProperties.maxShaderGroupStride;
    const uint32_t groupAlignment = rayTracingPipelineProperties.shaderGroupHandleAlignment;
    const uint32_t maxShaderRecordStride = rayTracingPipelineProperties.maxShaderGroupStride;

    // for the moment, just assume the max group size
    const uint32_t recordSize = alignedSize(std::min(maxGroupSize, uint32_t(4096)), groupAlignment);

    const uint32_t groupCount = static_cast<uint32_t>(shaderGroups.size());
    const uint32_t sbtSize = groupCount * handleSize;

    std::vector<uint8_t> shaderHandleStorage(sbtSize);
    VkResult err = gprt::vkGetRayTracingShaderGroupHandles(logicalDevice, pipeline, 0, groupCount, sbtSize, shaderHandleStorage.data());
    if (err) GPRT_RAISE("failed to get ray tracing shader group handles! : \n" + errorString(err));

    const VkBufferUsageFlags bufferUsageFlags = 
      // means we can use this buffer as a SBT
      VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | 
      // means we can get this buffer's address with vkGetBufferDeviceAddress
      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    const VkMemoryPropertyFlags memoryUsageFlags = 
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | // mappable to host with vkMapMemory
      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT; // means "flush" and "invalidate"  not needed

    // std::cout<<"Todo, get some smarter memory allocation working..." <<std::endl;

    size_t numComputes = computePrograms.size();
    size_t numRayGens = raygenPrograms.size();
    size_t numMissProgs = missPrograms.size();
    size_t numHitRecords = getNumHitRecords();
    
    size_t numRecords = numComputes + numRayGens + numMissProgs + numHitRecords;

    if (shaderBindingTable.size != recordSize * numRecords) {
      shaderBindingTable.destroy();
    }
    if (shaderBindingTable.buffer == VK_NULL_HANDLE) {
      shaderBindingTable = Buffer(physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 
        bufferUsageFlags, memoryUsageFlags, recordSize * numRecords);
    }
    shaderBindingTable.map();
    uint8_t* mapped = ((uint8_t*) (shaderBindingTable.mapped));

    // Compute records
    if (computePrograms.size() > 0) {
      for (uint32_t idx = 0; idx < computePrograms.size(); ++idx) {
        size_t recordStride = recordSize;
        size_t handleStride = handleSize;
        
        // First, copy handle
        size_t recordOffset = recordStride * idx;
        size_t handleOffset = handleStride * idx;
        memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

        // Then, copy params following handle
        recordOffset = recordOffset + handleSize;
        uint8_t* params = mapped + recordOffset;
        Compute *compute = computePrograms[idx];
        for (auto &var : compute->vars) {
          size_t varOffset = var.second.decl.offset;
          size_t varSize = getSize(var.second.decl.type);
          memcpy(params + varOffset, var.second.data, varSize);
        }
      }
    }

    // Raygen records
    if (raygenPrograms.size() > 0) {
      for (uint32_t idx = 0; idx < raygenPrograms.size(); ++idx) {
        size_t recordStride = recordSize;
        size_t handleStride = handleSize;
        
        // First, copy handle
        size_t recordOffset = recordStride * idx + recordStride * numComputes;
        size_t handleOffset = handleStride * idx + handleStride * numComputes;
        memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

        // Then, copy params following handle
        recordOffset = recordOffset + handleSize;
        uint8_t* params = mapped + recordOffset;
        RayGen *raygen = raygenPrograms[idx];
        for (auto &var : raygen->vars) {
          size_t varOffset = var.second.decl.offset;
          size_t varSize = getSize(var.second.decl.type);
          memcpy(params + varOffset, var.second.data, varSize);
        }
      }
    }

    // Miss records
    if (missPrograms.size() > 0) {
      for (uint32_t idx = 0; idx < missPrograms.size(); ++idx) {
        size_t recordStride = recordSize;
        size_t handleStride = handleSize;

        // First, copy handle
        size_t recordOffset = recordStride * idx + recordStride * numRayGens + recordStride * numComputes;
        size_t handleOffset = handleStride * idx + handleStride * numRayGens + handleStride * numComputes;
        memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

        // Then, copy params following handle
        recordOffset = recordOffset + handleSize;
        uint8_t* params = mapped + recordOffset;
        Miss *miss = missPrograms[idx];
        for (auto &var : miss->vars) {
          size_t varOffset = var.second.decl.offset;
          size_t varSize = getSize(var.second.decl.type);
          memcpy(params + varOffset, var.second.data, varSize);
        }
      }
    }

    // Hit records
    if (numHitRecords > 0)
    {
      // Go over all TLAS by order they were created
      for (int tlasID = 0; tlasID < accels.size(); ++tlasID) { 
        Accel *tlas = accels[tlasID];
        if (!tlas) continue;
        if (tlas->getType() == GPRT_INSTANCE_ACCEL) {
          InstanceAccel *instanceAccel = (InstanceAccel*) tlas;
          // this is an issue, because if instances can be set on device, we don't
          // have a list of instances we can iterate through and copy the SBT data...
          // So, if we have a bunch of instances set by reference on device, 
          // we need to eventually do something smarter here...
          size_t geomIDOffset = 0;
          for (int blasID = 0; blasID < instanceAccel->instances.size(); ++blasID) {
            
            Accel *blas = instanceAccel->instances[blasID];
            if (blas->getType() == GPRT_TRIANGLE_ACCEL) {
              TriangleAccel *triAccel = (TriangleAccel*) blas;

              for (int geomID = 0; geomID < triAccel->geometries.size(); ++geomID) {
                auto &geom = triAccel->geometries[geomID];

                for (int rayType = 0; rayType < numRayTypes; ++rayType) {
                  size_t recordStride = recordSize;
                  size_t handleStride = handleSize;

                  // First, copy handle
                  // Account for all prior instance's geometries and for prior BLAS's geometry
                  size_t instanceOffset = instanceAccel->instanceOffset + geomIDOffset;
                  size_t recordOffset = recordStride * (rayType + numRayTypes * geomID + instanceOffset) + recordStride * (numComputes + numRayGens + numMissProgs);
                  size_t handleOffset = handleStride * (rayType + numRayTypes * geomID + instanceOffset) + handleStride * (numComputes + numRayGens + numMissProgs);
                  memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);
                  
                  // Then, copy params following handle
                  recordOffset = recordOffset + handleSize;
                  uint8_t* params = mapped + recordOffset;
                  for (auto &var : geom->vars) {
                    size_t varOffset = var.second.decl.offset;
                    size_t varSize = getSize(var.second.decl.type);
                    memcpy(params + varOffset, var.second.data, varSize);
                  }
                }
              }
              geomIDOffset += triAccel->geometries.size();
            }

            else if (blas->getType() == GPRT_AABB_ACCEL) {
              AABBAccel *aabbAccel = (AABBAccel*) blas;

              for (int geomID = 0; geomID < aabbAccel->geometries.size(); ++geomID) {
                auto &geom = aabbAccel->geometries[geomID];

                for (int rayType = 0; rayType < numRayTypes; ++rayType) {
                  size_t recordStride = recordSize;
                  size_t handleStride = handleSize;

                  // First, copy handle
                  // Account for all prior instance's geometries and for prior BLAS's geometry
                  size_t instanceOffset = instanceAccel->instanceOffset + geomIDOffset;
                  size_t recordOffset = recordStride * (rayType + numRayTypes * geomID + instanceOffset) + recordStride * (numComputes + numRayGens + numMissProgs);
                  size_t handleOffset = handleStride * (rayType + numRayTypes * geomID + instanceOffset) + handleStride * (numComputes + numRayGens + numMissProgs);
                  memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);
                  
                  // Then, copy params following handle
                  recordOffset = recordOffset + handleSize;
                  uint8_t* params = mapped + recordOffset;
                  for (auto &var : geom->vars) {
                    size_t varOffset = var.second.decl.offset;
                    size_t varSize = getSize(var.second.decl.type);
                    memcpy(params + varOffset, var.second.data, varSize);
                  }
                }
              }
              geomIDOffset += aabbAccel->geometries.size();
            }
            
            else {
              GPRT_RAISE("Unaccounted for BLAS type!");
            }
          }
        }
      }
    }
  }

  // void buildPrograms()
  // {
  //   // At the moment, we don't actually build our programs here. 
  // }

  void buildPipeline()
  {

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.size = 128;
    pushConstantRange.offset = 0;
    pushConstantRange.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR 
                                  | VK_SHADER_STAGE_ANY_HIT_BIT_KHR 
                                  | VK_SHADER_STAGE_INTERSECTION_BIT_KHR 
                                  | VK_SHADER_STAGE_MISS_BIT_KHR 
                                  | VK_SHADER_STAGE_RAYGEN_BIT_KHR;

    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
    pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutCreateInfo.setLayoutCount = 0;    // if ever we use descriptors
    pipelineLayoutCreateInfo.pSetLayouts = nullptr; // if ever we use descriptors

    VkPipelineLayoutCreateInfo pipelineLayoutCI{};
    pipelineLayoutCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    // pipelineLayoutCI.setLayoutCount = 1;
    // pipelineLayoutCI.pSetLayouts = &descriptorSetLayout;
    pipelineLayoutCI.setLayoutCount = 0;
    pipelineLayoutCI.pSetLayouts = nullptr;
    pipelineLayoutCI.pushConstantRangeCount = 1;
    pipelineLayoutCI.pPushConstantRanges = &pushConstantRange;

    if (pipelineLayout != VK_NULL_HANDLE) {
      vkDestroyPipelineLayout(logicalDevice, pipelineLayout, nullptr);
      pipelineLayout = VK_NULL_HANDLE;
    }
    VK_CHECK_RESULT(vkCreatePipelineLayout(logicalDevice, &pipelineLayoutCI,
      nullptr, &pipelineLayout));

    /*
      Setup ray tracing shader groups
    */
    std::vector<VkPipelineShaderStageCreateInfo> shaderStages;
    shaderGroups.clear();

    // Compute program groups
    {
      for (auto compute : computePrograms) {
        shaderStages.push_back(compute->shaderStage);
        VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
        shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        shaderGroup.generalShader = static_cast<uint32_t>(shaderStages.size()) - 1;
        shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
        shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
        shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;
        shaderGroups.push_back(shaderGroup);
      }
    }

    // Ray generation groups
    {
      for (auto raygen : raygenPrograms) {
        shaderStages.push_back(raygen->shaderStage);
        VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
        shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        shaderGroup.generalShader = static_cast<uint32_t>(shaderStages.size()) - 1;
        shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
        shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
        shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;
        shaderGroups.push_back(shaderGroup);
      }
    }

    // Miss group
    {
      for (auto miss : missPrograms) {
        shaderStages.push_back(miss->shaderStage);
        VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
        shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        shaderGroup.generalShader = static_cast<uint32_t>(shaderStages.size()) - 1;
        shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
        shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
        shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;
        shaderGroups.push_back(shaderGroup);
      }
    }

    // Hit groups

    // Go over all TLAS by order they were created
    for (int tlasID = 0; tlasID < accels.size(); ++tlasID) {
      Accel *tlas = accels[tlasID];
      if (!tlas) continue;
      if (tlas->getType() == GPRT_INSTANCE_ACCEL) {
        // Iterate over all BLAS stored in the TLAS
        InstanceAccel *instanceAccel = (InstanceAccel*) tlas;
        for (int blasID = 0; blasID < instanceAccel->instances.size(); ++blasID) {
          Accel *blas = instanceAccel->instances[blasID];
          // Handle different BLAS types...
          if (blas->getType() == GPRT_TRIANGLE_ACCEL) {
            TriangleAccel *triAccel = (TriangleAccel*) blas;
            // Add a record for every geometry-raytype permutation
            for (int geomID = 0; geomID < triAccel->geometries.size(); ++geomID) {
              auto &geom = triAccel->geometries[geomID];
              for (int rayType = 0; rayType < numRayTypes; ++rayType) {
                VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
                shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
                shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;

                // init all to unused
                shaderGroup.generalShader = VK_SHADER_UNUSED_KHR;
                shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
                shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
                shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;

                // populate hit group programs using geometry type
                if (geom->geomType->closestHitShadersUsed) {
                  shaderStages.push_back(geom->geomType->closestHitShaderStages[rayType]);
                  shaderGroup.closestHitShader = static_cast<uint32_t>(shaderStages.size()) - 1;
                }

                if (geom->geomType->anyHitShadersUsed) {
                  shaderStages.push_back(geom->geomType->anyHitShaderStages[rayType]);
                  shaderGroup.anyHitShader = static_cast<uint32_t>(shaderStages.size()) - 1;
                }

                if (geom->geomType->intersectionShadersUsed) {
                  shaderStages.push_back(geom->geomType->intersectionShaderStages[rayType]);
                  shaderGroup.intersectionShader = static_cast<uint32_t>(shaderStages.size()) - 1;
                }
                shaderGroups.push_back(shaderGroup);
              }
            }
          }
          else if (blas->getType() == GPRT_AABB_ACCEL) {
            AABBAccel *aabbAccel = (AABBAccel*) blas;
            // Add a record for every geometry-raytype permutation
            for (int geomID = 0; geomID < aabbAccel->geometries.size(); ++geomID) {
              auto &geom = aabbAccel->geometries[geomID];
              for (int rayType = 0; rayType < numRayTypes; ++rayType) {
                VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
                shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
                shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR;

                // init all to unused
                shaderGroup.generalShader = VK_SHADER_UNUSED_KHR;
                shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
                shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
                shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;

                // populate hit group programs using geometry type
                if (geom->geomType->closestHitShadersUsed) {
                  shaderStages.push_back(geom->geomType->closestHitShaderStages[rayType]);
                  shaderGroup.closestHitShader = static_cast<uint32_t>(shaderStages.size()) - 1;
                }

                if (geom->geomType->anyHitShadersUsed) {
                  shaderStages.push_back(geom->geomType->anyHitShaderStages[rayType]);
                  shaderGroup.anyHitShader = static_cast<uint32_t>(shaderStages.size()) - 1;
                }

                if (geom->geomType->intersectionShadersUsed) {
                  shaderStages.push_back(geom->geomType->intersectionShaderStages[rayType]);
                  shaderGroup.intersectionShader = static_cast<uint32_t>(shaderStages.size()) - 1;
                }
                shaderGroups.push_back(shaderGroup);
              }
            }
          }
          else {
            GPRT_RAISE("Unaccounted for BLAS type!");
          }
        }
      }
    }

    /*
      Create the ray tracing pipeline
    */
    VkRayTracingPipelineCreateInfoKHR rayTracingPipelineCI{};
    rayTracingPipelineCI.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
    rayTracingPipelineCI.stageCount = static_cast<uint32_t>(shaderStages.size());
    rayTracingPipelineCI.pStages = shaderStages.data();
    rayTracingPipelineCI.groupCount = static_cast<uint32_t>(shaderGroups.size());
    rayTracingPipelineCI.pGroups = shaderGroups.data();
    rayTracingPipelineCI.maxPipelineRayRecursionDepth = 1; // WHA!?
    rayTracingPipelineCI.layout = pipelineLayout;

    if (pipeline != VK_NULL_HANDLE) {
      vkDestroyPipeline(logicalDevice, pipeline, nullptr);
      pipeline = VK_NULL_HANDLE;
    }
    VkResult err = gprt::vkCreateRayTracingPipelines(logicalDevice,
      VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &rayTracingPipelineCI,
      nullptr, &pipeline
    );
    if (err) {
      GPRT_RAISE("failed to create ray tracing pipeline! : \n" + errorString(err));
    }
  }

  void setupInternalStages(Module *module) {
    fillInstanceDataStage.entryPoint = "gprtFillInstanceData";

    // todo, consider refactoring this into a more official "Compute" shader object

    VkResult err;
    // currently not using cache.
    VkPipelineCache cache = VK_NULL_HANDLE;

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.size = 128;
    pushConstantRange.offset = 0;
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
    pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutCreateInfo.setLayoutCount = 0;    // if ever we use descriptors
    pipelineLayoutCreateInfo.pSetLayouts = nullptr; // if ever we use descriptors
    pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
    pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

    err = vkCreatePipelineLayout(logicalDevice, 
      &pipelineLayoutCreateInfo, nullptr, &fillInstanceDataStage.layout);

    std::string entryPoint = std::string("__compute__") + std::string(fillInstanceDataStage.entryPoint);
    auto binary = module->getBinary("COMPUTE");

    VkShaderModuleCreateInfo moduleCreateInfo = {};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
      NULL, &fillInstanceDataStage.module);

    VkPipelineShaderStageCreateInfo shaderStage = {};
    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    shaderStage.module = fillInstanceDataStage.module; 
    shaderStage.pName = entryPoint.c_str();

    VkComputePipelineCreateInfo computePipelineCreateInfo = {};
    computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    computePipelineCreateInfo.layout = fillInstanceDataStage.layout;
    computePipelineCreateInfo.flags = 0;
    computePipelineCreateInfo.stage = shaderStage;

    // At this point, create all internal compute pipelines as well.
    err = vkCreateComputePipelines(logicalDevice, 
      cache, 1, &computePipelineCreateInfo, nullptr, &fillInstanceDataStage.pipeline);
    //todo, destroy the above stuff
  }

  
  VkCommandBuffer beginSingleTimeCommands(VkCommandPool pool) {
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandPool = pool;
    allocInfo.commandBufferCount = 1;

    VkCommandBuffer commandBuffer;
    vkAllocateCommandBuffers(logicalDevice, &allocInfo, &commandBuffer);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    return commandBuffer;
  }

  void endSingleTimeCommands(VkCommandBuffer commandBuffer, VkCommandPool pool, VkQueue queue) {
      vkEndCommandBuffer(commandBuffer);

      VkSubmitInfo submitInfo{};
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &commandBuffer;

      vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
      vkQueueWaitIdle(queue);

      vkFreeCommandBuffers(logicalDevice, pool, 1, &commandBuffer);
  }

  void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout) {
    VkCommandBuffer commandBuffer = beginSingleTimeCommands(graphicsCommandPool);

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    
    // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents of the original 
    // image. I'm assuming this is ok.
    barrier.oldLayout = oldLayout;

    // The new layout for the image
    barrier.newLayout = newLayout;

    // If we're transferring which queue owns this image, we need to set these.
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    // specify the image that is affected and the specific parts of that image.
    barrier.image = image;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    VkPipelineStageFlags sourceStage;
    VkPipelineStageFlags destinationStage;

    if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR) {
        sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        barrier.srcAccessMask = 0;
        destinationStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
    }
    else if (oldLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
        sourceStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;

        destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        barrier.dstAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
    } else {
        throw std::invalid_argument("unsupported layout transition!");
    }

    vkCmdPipelineBarrier(
        commandBuffer,
        sourceStage, destinationStage,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrier
    );

    endSingleTimeCommands(commandBuffer, graphicsCommandPool, graphicsQueue);
  }
};


GPRT_API void gprtRequestWindow(
  uint32_t initialWidth, 
  uint32_t initialHeight, 
  const char *title)
{
  LOG_API_CALL();
  requestedFeatures.window = true;
  requestedFeatures.windowProperties.initialWidth = initialWidth;
  requestedFeatures.windowProperties.initialHeight = initialHeight;
  requestedFeatures.windowProperties.title = std::string(title);
}

GPRT_API bool gprtWindowShouldClose(GPRTContext _context) {
  LOG_API_CALL();
  Context *context = (Context*)_context;
  if (!requestedFeatures.window) return true;
  
  glfwPollEvents();
  return glfwWindowShouldClose(context->window);
}

GPRT_API void gprtGetCursorPos(GPRTContext _context, 
  double * xpos, double * ypos)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  if (!requestedFeatures.window) return;

  glfwGetCursorPos(context->window, xpos, ypos);
}

GPRT_API int gprtGetMouseButton(GPRTContext _context,
  int button)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  if (!requestedFeatures.window) return GPRT_RELEASE;

  return glfwGetMouseButton(context->window, button);
}

GPRT_API void gprtBufferPresent(GPRTContext _context, GPRTBuffer _buffer) {
  LOG_API_CALL();
  if (!requestedFeatures.window) return;
  Context *context = (Context*)_context;
  Buffer *buffer = (Buffer*)_buffer;
  
  VkPresentInfoKHR presentInfo{};
  presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

  // VkSemaphore signalSemaphores[] = {renderFinishedSemaphore};
  // submitInfo.signalSemaphoreCount = 1;
  // submitInfo.pSignalSemaphores = signalSemaphores;

  VkCommandBuffer commandBuffer = context->beginSingleTimeCommands(context->graphicsCommandPool);

  // transition image layout from PRESENT_SRC to TRANSFER_DST
  {
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    
    // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents of the original 
    // image. I'm assuming this is ok.
    barrier.oldLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    // The new layout for the image
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

    // If we're transferring which queue owns this image, we need to set these.
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    // specify the image that is affected and the specific parts of that image.
    barrier.image = context->swapchainImages[context->currentImageIndex];
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    VkPipelineStageFlags sourceStage;
    VkPipelineStageFlags destinationStage;

    sourceStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;

    destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dstAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;

    vkCmdPipelineBarrier(
        commandBuffer,
        sourceStage, destinationStage,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrier
    );
  }

  // now do the transfer
  {
    VkBufferImageCopy region{};
    region.bufferOffset = 0;
    // if 0, vulkan assumes buffer memory is tightly packed
    region.bufferRowLength = 0; 
    region.bufferImageHeight = 0; 

    region.imageOffset.x = 0;
    region.imageOffset.y = 0;
    region.imageOffset.z = 0;

    region.imageExtent.width = context->windowExtent.width;
    region.imageExtent.height = context->windowExtent.height;
    region.imageExtent.depth = 1;

    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    
    vkCmdCopyBufferToImage(commandBuffer, buffer->buffer, context->swapchainImages[context->currentImageIndex], 
      VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
  }

  // now go from TRANSFER_DST back to PRESENT_SRC
  {
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    
    // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents of the original 
    // image. I'm assuming this is ok.
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

    // The new layout for the image
    barrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    // If we're transferring which queue owns this image, we need to set these.
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    // specify the image that is affected and the specific parts of that image.
    barrier.image = context->swapchainImages[context->currentImageIndex];
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    VkPipelineStageFlags sourceStage;
    VkPipelineStageFlags destinationStage;

    sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;

    destinationStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;

    vkCmdPipelineBarrier(
        commandBuffer,
        sourceStage, destinationStage,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrier
    );
  }

  context->endSingleTimeCommands(commandBuffer, context->graphicsCommandPool, context->graphicsQueue);

  presentInfo.waitSemaphoreCount = 0;
  presentInfo.pWaitSemaphores = VK_NULL_HANDLE;

  VkSwapchainKHR swapchains[] = {context->swapchain};
  presentInfo.swapchainCount = 1;
  presentInfo.pSwapchains = swapchains;
  presentInfo.pImageIndices = &context->currentImageIndex;

  presentInfo.pResults = nullptr;

  // currently throwing an error because the images given by the swapchain don't 
  // have a defined layout...
  VkResult err1 = vkQueuePresentKHR(context->graphicsQueue, &presentInfo);


  VkResult err2 = vkAcquireNextImageKHR(context->logicalDevice, context->swapchain, UINT64_MAX, 
        VK_NULL_HANDLE, context->inFlightFence, &context->currentImageIndex);
  vkWaitForFences(context->logicalDevice,1, &context->inFlightFence, true, UINT_MAX);
  vkResetFences(context->logicalDevice, 1, &context->inFlightFence);
}

GPRT_API GPRTContext gprtContextCreate(int32_t *requestedDeviceIDs,
                                    int      numRequestedDevices)
{
  LOG_API_CALL();
  Context *context = new Context(requestedDeviceIDs, numRequestedDevices);
  LOG("context created...");
  return (GPRTContext)context;
}

GPRT_API void gprtContextDestroy(GPRTContext _context)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  context->destroy();
  delete context;
  LOG("context destroyed...");
}

GPRT_API GPRTModule gprtModuleCreate(GPRTContext _context, GPRTProgram spvCode)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  Module *module = new Module(spvCode);
  LOG("module created...");
  return (GPRTModule)module;
}

GPRT_API void gprtModuleDestroy(GPRTModule _module)
{
  LOG_API_CALL();
  Module *module = (Module*)_module;
  delete module;
  LOG("module destroyed...");
}

GPRT_API GPRTGeom
gprtGeomCreate(GPRTContext  _context,
              GPRTGeomType _geomType)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  GeomType *geomType = (GeomType*)_geomType;

  // depending on what the geomType is, we'll use this inherited "createGeom"
  // function to construct the appropriate geometry
  Geom *geometry = geomType->createGeom();
  return (GPRTGeom)geometry;
  LOG("geometry created...");
}

GPRT_API void 
gprtGeomDestroy(GPRTGeom _geometry)
{
  LOG_API_CALL();
  Geom *geometry = (Geom*)_geometry;
  geometry->destroy();
  delete geometry;
  LOG("geometry destroyed...");
}

// ==================================================================
// "Triangles" functions
// ==================================================================
GPRT_API void gprtTrianglesSetVertices(GPRTGeom _triangles,
                                      GPRTBuffer _vertices,
                                      size_t count,
                                      size_t stride,
                                      size_t offset)
{
  LOG_API_CALL();
  TriangleGeom *triangles = (TriangleGeom*)_triangles;
  Buffer *vertices = (Buffer*)_vertices;
  triangles->setVertices(vertices, count, stride, offset);
  LOG("Setting triangle vertices...");
}
// GPRT_API void gprtTrianglesSetMotionVertices(GPRTGeom triangles,
//                                            /*! number of vertex arrays
//                                                passed here, the first
//                                                of those is for t=0,
//                                                thelast for t=1,
//                                                everything is linearly
//                                                interpolated
//                                                in-between */
//                                            size_t    numKeys,
//                                            GPRTBuffer *vertexArrays,
//                                            size_t count,
//                                            size_t stride,
//                                            size_t offset)
// {
//   GPRT_NOTIMPLEMENTED;
// }

GPRT_API void gprtTrianglesSetIndices(GPRTGeom _triangles,
                                     GPRTBuffer _indices,
                                     size_t count,
                                     size_t stride,
                                     size_t offset)
{
  LOG_API_CALL();
  TriangleGeom *triangles = (TriangleGeom*)_triangles;
  Buffer *indices = (Buffer*)_indices;
  triangles->setIndices(indices, count, stride, offset);
  LOG("Setting triangle indices...");
}

void gprtAABBsSetPositions(GPRTGeom _aabbs, 
                           GPRTBuffer _positions,
                           size_t count,
                           size_t stride,
                           size_t offset)
{
  LOG_API_CALL();
  AABBGeom *aabbs = (AABBGeom*)_aabbs;
  Buffer *positions = (Buffer*)_positions;
  aabbs->setAABBs(positions, count, stride, offset);
  LOG("Setting AABB positions...");
}

GPRT_API GPRTRayGen
gprtRayGenCreate(GPRTContext _context,
                 GPRTModule  _module,
                 const char  *programName,
                 size_t       sizeOfVarStruct,
                 GPRTVarDecl *vars,
                 int          numVars)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  Module *module = (Module*)_module;

  RayGen *raygen = new RayGen(
    context->logicalDevice, module, programName,
    sizeOfVarStruct, checkAndPackVariables(vars, numVars));

  context->raygenPrograms.push_back(raygen);

  LOG("raygen created...");
  return (GPRTRayGen)raygen;
}

GPRT_API void
gprtRayGenDestroy(GPRTRayGen _rayGen)
{
  LOG_API_CALL();
  RayGen *rayGen = (RayGen*)_rayGen;
  rayGen->destroy();
  delete rayGen;
  LOG("raygen destroyed...");
}

GPRT_API GPRTCompute
gprtComputeCreate(GPRTContext _context,
                 GPRTModule  _module,
                 const char  *programName,
                 size_t       sizeOfVarStruct,
                 GPRTVarDecl *vars,
                 int          numVars)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  Module *module = (Module*)_module;

  Compute *compute = new Compute(
    context->logicalDevice, module, programName,
    sizeOfVarStruct, checkAndPackVariables(vars, numVars));

  context->computePrograms.push_back(compute);

  LOG("compute created...");
  return (GPRTCompute)compute;
}

GPRT_API void
gprtComputeDestroy(GPRTCompute _compute)
{
  LOG_API_CALL();
  Compute *compute = (Compute*)_compute;
  compute->destroy();
  delete compute;
  LOG("compute destroyed...");
}

GPRT_API GPRTMiss
gprtMissCreate(GPRTContext _context,
                   GPRTModule  _module,
                   const char  *programName,
                   size_t       sizeOfVarStruct,
                   GPRTVarDecl *vars,
                   int          numVars)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  Module *module = (Module*)_module;

  Miss *missProg = new Miss(
    context->logicalDevice, module, programName,
    sizeOfVarStruct, checkAndPackVariables(vars, numVars));

  context->missPrograms.push_back(missProg);

  LOG("miss program created...");
  return (GPRTMiss)missProg;
}


/*! sets the given miss program for the given ray type */
GPRT_API void
gprtMissSet(GPRTContext  _context,
               int rayType,
               GPRTMiss _missToUse)
{
  GPRT_NOTIMPLEMENTED;
}

GPRT_API void
gprtMissDestroy(GPRTMiss _miss)
{
  LOG_API_CALL();
  Miss *missProg = (Miss*)_miss;
  missProg->destroy();
  delete missProg;
  LOG("miss program destroyed...");
}

GPRT_API GPRTGeomType
gprtGeomTypeCreate(GPRTContext  _context,
                   GPRTGeomKind kind,
                   size_t       sizeOfVarStruct,
                   GPRTVarDecl  *vars,
                   int          numVars)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;

  GeomType *geomType = nullptr;

  switch(kind) {
    case GPRT_TRIANGLES:
      geomType = new TriangleGeomType(
        context->logicalDevice, context->numRayTypes,
        sizeOfVarStruct, checkAndPackVariables(vars, numVars));
        break;
    case GPRT_AABBS:
      geomType = new AABBGeomType(
        context->logicalDevice, context->numRayTypes,
        sizeOfVarStruct, checkAndPackVariables(vars, numVars));
        break;
    default:
      GPRT_NOTIMPLEMENTED;
      break;
  }

  context->geomTypes.push_back(geomType);

  LOG("geom type created...");
  return (GPRTGeomType)geomType;
}

GPRT_API void 
gprtGeomTypeDestroy(GPRTGeomType _geomType)
{
  LOG_API_CALL();
  GeomType *geomType = (GeomType*)_geomType;
  geomType->destroy();
  delete geomType;
  LOG("geom type destroyed...");
}

GPRT_API void
gprtGeomTypeSetClosestHitProg(GPRTGeomType _geomType,
                          int rayType,
                          GPRTModule _module,
                          const char *progName)
{
  LOG_API_CALL();
  GeomType *geomType = (GeomType*)_geomType;
  Module *module = (Module*)_module;

  geomType->setClosestHit(rayType, module, progName);
  LOG("assigning closest hit program to geom type...");
}

GPRT_API void
gprtGeomTypeSetAnyHitProg(GPRTGeomType _geomType,
                          int rayType,
                          GPRTModule _module,
                          const char *progName)
{
  LOG_API_CALL();
  GeomType *geomType = (GeomType*)_geomType;
  Module *module = (Module*)_module;

  geomType->setAnyHit(rayType, module, progName);
  LOG("assigning any hit program to geom type...");
}

GPRT_API void
gprtGeomTypeSetIntersectionProg(GPRTGeomType _geomType,
                          int rayType,
                          GPRTModule _module,
                          const char *progName)
{
  LOG_API_CALL();
  GeomType *geomType = (GeomType*)_geomType;
  Module *module = (Module*)_module;

  geomType->setIntersection(rayType, module, progName);
  LOG("assigning intersect program to geom type...");
}

GPRT_API GPRTBuffer
gprtHostBufferCreate(GPRTContext _context, GPRTDataType type, size_t count, const void* init)
{
  LOG_API_CALL();
  const VkBufferUsageFlags bufferUsageFlags =
    // means we can get this buffer's address with vkGetBufferDeviceAddress
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
    // I guess I need this to use these buffers as input to tree builds?
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | 
    // means we can use this buffer to transfer into another
    VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
    // means we can use this buffer to receive data transferred from another
    VK_BUFFER_USAGE_TRANSFER_DST_BIT
  ;
  const VkMemoryPropertyFlags memoryUsageFlags =
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | // mappable to host with vkMapMemory
    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT; // means "flush" and "invalidate"  not needed

  Context *context = (Context*)_context;
  Buffer *buffer = new Buffer(
    context->physicalDevice, context->logicalDevice, 
    context->graphicsCommandBuffer, context->graphicsQueue,
    bufferUsageFlags, memoryUsageFlags,
    getSize(type) * count
  );

  // Pin the buffer to the host
  buffer->map();
  
  if (init) {
    void* mapped = buffer->mapped;
    memcpy(mapped, init, getSize(type) * count);
    buffer->flush();
    buffer->invalidate();
  }
  LOG("buffer created");
  return (GPRTBuffer)buffer;
}

GPRT_API GPRTBuffer
gprtDeviceBufferCreate(GPRTContext _context, GPRTDataType type, size_t count, const void* init)
{
  LOG_API_CALL();
  const VkBufferUsageFlags bufferUsageFlags =
    // means we can get this buffer's address with vkGetBufferDeviceAddress
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
    // I guess I need this to use these buffers as input to tree builds?
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | 
    // means we can use this buffer to transfer into another
    VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
    // means we can use this buffer to receive data transferred from another
    VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags =
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT; // means most efficient for device access

  Context *context = (Context*)_context;
  Buffer *buffer = new Buffer(
    context->physicalDevice, context->logicalDevice,
    context->graphicsCommandBuffer, context->graphicsQueue,
    bufferUsageFlags, memoryUsageFlags,
    getSize(type) * count
  );
  
  if (init) {    
    buffer->map();
    void* mapped = buffer->mapped;
    memcpy(mapped, init, getSize(type) * count);
    buffer->unmap();
  }
  LOG("buffer created");
  return (GPRTBuffer)buffer;
}

GPRT_API GPRTBuffer
gprtSharedBufferCreate(GPRTContext _context, GPRTDataType type, size_t count, const void* init)
{
  LOG_API_CALL();
  const VkBufferUsageFlags bufferUsageFlags =
    // means we can get this buffer's address with vkGetBufferDeviceAddress
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
    // I guess I need this to use these buffers as input to tree builds?
    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | 
    // means we can use this buffer to transfer into another
    VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
    // means we can use this buffer to receive data transferred from another
    VK_BUFFER_USAGE_TRANSFER_DST_BIT
  ;
  const VkMemoryPropertyFlags memoryUsageFlags =
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | // mappable to host with vkMapMemory
    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |  // means "flush" and "invalidate"  not needed
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT; // means most efficient for device access

  Context *context = (Context*)_context;
  Buffer *buffer = new Buffer(
    context->physicalDevice, context->logicalDevice, 
    context->graphicsCommandBuffer, context->graphicsQueue,
    bufferUsageFlags, memoryUsageFlags,
    getSize(type) * count
  );

  // Pin the buffer to the host
  buffer->map();
  
  if (init) {
    void* mapped = buffer->mapped;
    memcpy(mapped, init, getSize(type) * count);
    buffer->flush();
    buffer->invalidate();
  }
  LOG("buffer created");
  return (GPRTBuffer)buffer;
}

GPRT_API void
gprtBufferDestroy(GPRTBuffer _buffer)
{
  LOG_API_CALL();
  Buffer *buffer = (Buffer*)_buffer;
  buffer->destroy();
  delete buffer;
  LOG("buffer destroyed");
}

GPRT_API void *
gprtBufferGetPointer(GPRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  Buffer *buffer = (Buffer*)_buffer;
  return buffer->mapped;
}

GPRT_API void
gprtBufferMap(GPRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  Buffer *buffer = (Buffer*)_buffer;
  buffer->map();
}

GPRT_API void
gprtBufferUnmap(GPRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  Buffer *buffer = (Buffer*)_buffer;
  buffer->unmap();
}

GPRT_API void gprtBufferSaveImage(GPRTBuffer _buffer, 
  uint32_t width, uint32_t height, const char *imageName)
{
  LOG_API_CALL();
  Buffer *buffer = (Buffer*)_buffer;

  // Keep track of whether the buffer was mapped before this call
  bool mapped = true;
  if (buffer->mapped == nullptr) mapped = false;

  // If not mapped currently, map it
  if (!mapped)
    buffer->map();

  const uint32_t *fb = (const uint32_t*) buffer->mapped;
  stbi_flip_vertically_on_write(1);
  stbi_write_png(imageName,width,height,4,
                 fb,(uint32_t)(width)*sizeof(uint32_t));
  
  // Return mapped to previous state
  if (!mapped)
    buffer->unmap();
}

GPRT_API void gprtBuildPipeline(GPRTContext _context)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  context->buildPipeline();
  LOG("programs built...");
}

GPRT_API GPRTAccel
gprtAABBAccelCreate(GPRTContext _context,
                       size_t       numGeometries,
                       GPRTGeom    *arrayOfChildGeoms,
                       unsigned int flags)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  AABBAccel *accel = new 
    AABBAccel(
      context->physicalDevice, context->logicalDevice, 
      context->graphicsCommandBuffer, context->graphicsQueue, 
      numGeometries, (AABBGeom*)arrayOfChildGeoms);
  context->accels.push_back(accel);
  return (GPRTAccel)accel;
}

GPRT_API GPRTAccel
gprtTrianglesAccelCreate(GPRTContext _context,
                            size_t     numGeometries,
                            GPRTGeom   *arrayOfChildGeoms,
                            unsigned int flags)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  TriangleAccel *accel = new 
    TriangleAccel(
      context->physicalDevice, context->logicalDevice, 
      context->graphicsCommandBuffer, context->graphicsQueue, 
      numGeometries, (TriangleGeom*)arrayOfChildGeoms);
  context->accels.push_back(accel);
  return (GPRTAccel)accel;
}

GPRT_API GPRTAccel
gprtCurvesAccelCreate(GPRTContext context,
                         size_t     numCurveGeometries,
                         GPRTGeom   *curveGeometries,
                         unsigned int flags)
{
  GPRT_NOTIMPLEMENTED;
  return nullptr;
}

GPRT_API GPRTAccel
gprtInstanceAccelCreate(GPRTContext _context,
                        size_t numAccels,
                        GPRTAccel *arrayOfAccels,
                        unsigned int flags)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  InstanceAccel *accel = new 
    InstanceAccel(
      context->physicalDevice, context->logicalDevice, 
      context->graphicsCommandBuffer, context->graphicsQueue, 
      numAccels, arrayOfAccels);
  context->accels.push_back(accel);
  return (GPRTAccel)accel;
}

GPRT_API void 
gprtInstanceAccelSet3x4Transforms(GPRTAccel instanceAccel,
                                  GPRTBuffer transforms)
{
  gprtInstanceAccelSetTransforms(instanceAccel, transforms, sizeof(float3x4), 0);
}

GPRT_API void 
gprtInstanceAccelSet4x4Transforms(GPRTAccel instanceAccel,
                                  GPRTBuffer transforms)
{
  gprtInstanceAccelSetTransforms(instanceAccel, transforms, sizeof(float4x4), 0);
}

GPRT_API void 
gprtInstanceAccelSetTransforms(GPRTAccel instanceAccel,
                               GPRTBuffer _transforms,
                               size_t stride,
                               size_t offset
                               )
{
  LOG_API_CALL();
  InstanceAccel *accel = (InstanceAccel*)instanceAccel;
  Buffer *transforms = (Buffer*)_transforms;
  accel->setTransforms(transforms, stride, offset);
  LOG("Setting instance accel transforms...");
}

GPRT_API void 
gprtInstanceAccelSetReferences(GPRTAccel instanceAccel,
                               GPRTBuffer _references//,
                               // size_t offset, // maybe I can support these too?
                               // size_t stride  // maybe I can support these too?
                               )
{
  LOG_API_CALL();
  InstanceAccel *accel = (InstanceAccel*)instanceAccel;
  Buffer *references = (Buffer*)_references;
  accel->setReferences(references);
  LOG("Setting instance accel references...");
}

GPRT_API void 
gprtInstanceAccelSetNumGeometries(GPRTAccel instanceAccel, 
                                  size_t numGeometries)
{
  LOG_API_CALL();
  InstanceAccel *accel = (InstanceAccel*)instanceAccel;
  accel->setNumGeometries(numGeometries);
  LOG("Setting instance accel references...");
}

GPRT_API void
gprtAccelDestroy(GPRTAccel _accel)
{
  LOG_API_CALL();
  Accel *accel = (Accel*)_accel;
  accel->destroy();
  delete accel;
  LOG("accel destroyed");
}

GPRT_API void gprtAccelBuild(GPRTContext _context, GPRTAccel _accel)
{
  Accel *accel = (Accel*)_accel;
  Context *context = (Context*)_context;
  accel->build({
    {"gprtFillInstanceData", context->fillInstanceDataStage}},
    context->accels, context->numRayTypes
  );
}

GPRT_API void gprtAccelRefit(GPRTContext _context, GPRTAccel accel)
{
  GPRT_NOTIMPLEMENTED;
}

GPRT_API uint64_t gprtAccelGetReference(GPRTAccel _accel)
{
  Accel *accel = (Accel*)_accel;
  return uint64_t(accel->address);
}

GPRT_API void gprtBuildShaderBindingTable(GPRTContext _context,
                           GPRTBuildSBTFlags flags)
{
  LOG_API_CALL();
  Context *context = (Context*)_context;
  context->buildSBT();
}

GPRT_API void
gprtRayGenLaunch1D(GPRTContext _context, GPRTRayGen _rayGen, int dims_x)
{
  LOG_API_CALL();
  gprtRayGenLaunch2D(_context, _rayGen,dims_x,1);
}

GPRT_API void
gprtRayGenLaunch2D(GPRTContext _context, GPRTRayGen _rayGen, int dims_x, int dims_y)
{
  LOG_API_CALL();
  gprtRayGenLaunch3D(_context, _rayGen,dims_x,dims_y,1);
}

GPRT_API void
gprtRayGenLaunch3D(GPRTContext _context, GPRTRayGen _rayGen, int dims_x, int dims_y, int dims_z)
{
  LOG_API_CALL();
  assert(_rayGen);

  Context *context = (Context*)_context;
  RayGen *raygen = (RayGen*)_rayGen;
  VkResult err;

  VkCommandBufferBeginInfo cmdBufInfo{};
  cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);

  if (context->queryRequested) {
    vkCmdResetQueryPool(context->graphicsCommandBuffer,  context->queryPool, 0, 2);
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, context->queryPool, 0);
  }

  vkCmdBindPipeline(
    context->graphicsCommandBuffer,
    VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
    context->pipeline);

  struct PushConstants {
    uint64_t pad[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  } pushConstants;
  vkCmdPushConstants(context->graphicsCommandBuffer,  context->pipelineLayout, 
    VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR 
    | VK_SHADER_STAGE_ANY_HIT_BIT_KHR 
    | VK_SHADER_STAGE_INTERSECTION_BIT_KHR 
    | VK_SHADER_STAGE_MISS_BIT_KHR 
    | VK_SHADER_STAGE_RAYGEN_BIT_KHR, 
    0, sizeof(PushConstants), &pushConstants
  );

  auto getBufferDeviceAddress = [](VkDevice device, VkBuffer buffer) -> uint64_t
	{
		VkBufferDeviceAddressInfoKHR bufferDeviceAI{};
		bufferDeviceAI.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
		bufferDeviceAI.buffer = buffer;
		return gprt::vkGetBufferDeviceAddress(device, &bufferDeviceAI);
	};

  auto alignedSize = [](uint32_t value, uint32_t alignment) -> uint32_t
  {
    return (value + alignment - 1) & ~(alignment - 1);
  };

  const uint32_t handleSize = context->rayTracingPipelineProperties.shaderGroupHandleSize;
  const uint32_t maxGroupSize = context->rayTracingPipelineProperties.maxShaderGroupStride;
  const uint32_t groupAlignment = context->rayTracingPipelineProperties.shaderGroupHandleAlignment;
  const uint32_t maxShaderRecordStride = context->rayTracingPipelineProperties.maxShaderGroupStride;

  // for the moment, just assume the max group size
  const uint32_t recordSize = alignedSize(std::min(maxGroupSize, uint32_t(4096)), groupAlignment);
  uint64_t baseAddr = getBufferDeviceAddress(
    context->logicalDevice, context->shaderBindingTable.buffer);

  // find raygen in current list of raygens
  int computeOffset = context->computePrograms.size() * recordSize;
  int raygenOffset = 0;
  for (int i = 0; i < context->raygenPrograms.size(); ++i) {
    if (context->raygenPrograms[i] == raygen) {
      raygenOffset = i * recordSize;
      break;
    }
  }

  VkStridedDeviceAddressRegionKHR raygenShaderSbtEntry{};
  raygenShaderSbtEntry.deviceAddress = baseAddr + raygenOffset + computeOffset;
  raygenShaderSbtEntry.stride = recordSize;
  raygenShaderSbtEntry.size = raygenShaderSbtEntry.stride; // for raygen, can only be one. this needs to be the same as stride.
  // * context->raygenPrograms.size();

  VkStridedDeviceAddressRegionKHR missShaderSbtEntry{};
  if (context->missPrograms.size() > 0) {
    missShaderSbtEntry.deviceAddress = baseAddr + recordSize * context->raygenPrograms.size() + recordSize * context->computePrograms.size() ;
    missShaderSbtEntry.stride = recordSize;
    missShaderSbtEntry.size = missShaderSbtEntry.stride * context->missPrograms.size();
  }
  VkStridedDeviceAddressRegionKHR hitShaderSbtEntry{};
  size_t numHitRecords = context->getNumHitRecords();
  if (numHitRecords > 0) {
    hitShaderSbtEntry.deviceAddress = baseAddr + recordSize * (context->computePrograms.size() + context->raygenPrograms.size() + context->missPrograms.size());
    hitShaderSbtEntry.stride = recordSize;
    hitShaderSbtEntry.size = hitShaderSbtEntry.stride * numHitRecords;
  }

  VkStridedDeviceAddressRegionKHR callableShaderSbtEntry{}; // empty
  callableShaderSbtEntry.deviceAddress = 0;
  // callableShaderSbtEntry.stride = handleSizeAligned;
  // callableShaderSbtEntry.size = handleSizeAligned;

  gprt::vkCmdTraceRays(
    context->graphicsCommandBuffer,
    &raygenShaderSbtEntry,
    &missShaderSbtEntry,
    &hitShaderSbtEntry,
    &callableShaderSbtEntry,
    dims_x,
    dims_y,
    dims_z);
  
  if (context->queryRequested)
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, context->queryPool, 1);

  err = vkEndCommandBuffer(context->graphicsCommandBuffer);
  if (err) GPRT_RAISE("failed to end command buffer! : \n" + errorString(err));

  VkSubmitInfo submitInfo;
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.pNext = NULL;
  submitInfo.waitSemaphoreCount = 0;
  submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
  submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
  submitInfo.signalSemaphoreCount = 0;
  submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

  err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
  if (err) GPRT_RAISE("failed to submit to queue! : \n" + errorString(err));

  err = vkQueueWaitIdle(context->graphicsQueue);
  if (err) GPRT_RAISE("failed to wait for queue idle! : \n" + errorString(err));
}

GPRT_API void
gprtComputeLaunch1D(GPRTContext _context, GPRTCompute _compute, int dims_x)
{
  LOG_API_CALL();
  gprtComputeLaunch2D(_context, _compute,dims_x,1);
}

GPRT_API void
gprtComputeLaunch2D(GPRTContext _context, GPRTCompute _compute, int dims_x, int dims_y)
{
  LOG_API_CALL();
  gprtComputeLaunch3D(_context, _compute,dims_x,dims_y,1);
}

GPRT_API void
gprtComputeLaunch3D(GPRTContext _context, GPRTCompute _compute, int dims_x, int dims_y, int dims_z)
{
  LOG_API_CALL();
  assert(_compute);

  Context *context = (Context*)_context;
  Compute *compute = (Compute*)_compute;
  VkResult err;

  VkCommandBufferBeginInfo cmdBufInfo{};
  cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);

  if (context->queryRequested) {
    vkCmdResetQueryPool(context->graphicsCommandBuffer,  context->queryPool, 0, 2);
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, context->queryPool, 0);
  }
  
  vkCmdBindPipeline(
    context->graphicsCommandBuffer,
    VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
    context->pipeline);

  struct PushConstants {
    uint64_t pad[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  } pushConstants;
  vkCmdPushConstants(context->graphicsCommandBuffer,  context->pipelineLayout, 
    VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR 
    | VK_SHADER_STAGE_ANY_HIT_BIT_KHR 
    | VK_SHADER_STAGE_INTERSECTION_BIT_KHR 
    | VK_SHADER_STAGE_MISS_BIT_KHR 
    | VK_SHADER_STAGE_RAYGEN_BIT_KHR, // todo, replace eventually with compute.
    0, sizeof(PushConstants), &pushConstants
  );

  auto getBufferDeviceAddress = [](VkDevice device, VkBuffer buffer) -> uint64_t
	{
		VkBufferDeviceAddressInfoKHR bufferDeviceAI{};
		bufferDeviceAI.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
		bufferDeviceAI.buffer = buffer;
		return gprt::vkGetBufferDeviceAddress(device, &bufferDeviceAI);
	};

  auto alignedSize = [](uint32_t value, uint32_t alignment) -> uint32_t
  {
    return (value + alignment - 1) & ~(alignment - 1);
  };

  const uint32_t handleSize = context->rayTracingPipelineProperties.shaderGroupHandleSize;
  const uint32_t maxGroupSize = context->rayTracingPipelineProperties.maxShaderGroupStride;
  const uint32_t groupAlignment = context->rayTracingPipelineProperties.shaderGroupHandleAlignment;
  const uint32_t maxShaderRecordStride = context->rayTracingPipelineProperties.maxShaderGroupStride;

  // for the moment, just assume the max group size
  const uint32_t recordSize = alignedSize(std::min(maxGroupSize, uint32_t(4096)), groupAlignment);
  uint64_t baseAddr = getBufferDeviceAddress(
    context->logicalDevice, context->shaderBindingTable.buffer);

  // find compute program in current list of compute programs
  int computeOffset = 0;
  for (int i = 0; i < context->computePrograms.size(); ++i) {
    if (context->computePrograms[i] == compute) {
      computeOffset = i * recordSize;
      break;
    }
  }

  VkStridedDeviceAddressRegionKHR raygenShaderSbtEntry{};
  raygenShaderSbtEntry.deviceAddress = baseAddr + computeOffset;
  raygenShaderSbtEntry.stride = recordSize;
  raygenShaderSbtEntry.size = raygenShaderSbtEntry.stride; // can only be one.

  VkStridedDeviceAddressRegionKHR missShaderSbtEntry{};
  VkStridedDeviceAddressRegionKHR hitShaderSbtEntry{};
  VkStridedDeviceAddressRegionKHR callableShaderSbtEntry{};
  
  gprt::vkCmdTraceRays(
    context->graphicsCommandBuffer,
    &raygenShaderSbtEntry,
    &missShaderSbtEntry,
    &hitShaderSbtEntry,
    &callableShaderSbtEntry,
    dims_x,
    dims_y,
    dims_z);

  if (context->queryRequested)
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, context->queryPool, 1);

  err = vkEndCommandBuffer(context->graphicsCommandBuffer);
  if (err) GPRT_RAISE("failed to end command buffer! : \n" + errorString(err));

  VkSubmitInfo submitInfo;
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.pNext = NULL;
  submitInfo.waitSemaphoreCount = 0;
  submitInfo.pWaitSemaphores = nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
  submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
  submitInfo.signalSemaphoreCount = 0;
  submitInfo.pSignalSemaphores = nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]};

  err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
  if (err) GPRT_RAISE("failed to submit to queue! : \n" + errorString(err));

  err = vkQueueWaitIdle(context->graphicsQueue);
  if (err) GPRT_RAISE("failed to wait for queue idle! : \n" + errorString(err));
}

GPRT_API void gprtBeginProfile(GPRTContext _context)
{
  LOG_API_CALL();
  assert(_context);
  Context *context = (Context*)_context;
  context->queryRequested = true;  
}

GPRT_API float gprtEndProfile(GPRTContext _context)
{
  LOG_API_CALL();
  assert(_context);
  Context *context = (Context*)_context;

  if (context->queryRequested != true) GPRT_RAISE("Requested profile data without calling gprtBeginProfile");
  context->queryRequested = false;  

  uint64_t buffer[2];
  VkResult result = vkGetQueryPoolResults(context->logicalDevice, context->queryPool, 0, 2, sizeof(uint64_t) * 2, buffer, sizeof(uint64_t), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
  if (result != VK_SUCCESS) GPRT_RAISE("Failed to receive query results!");
  uint64_t timeResults = buffer[1] - buffer[0];
  float time = float(timeResults) / context->deviceProperties.limits.timestampPeriod;
  return time;
}

std::pair<size_t, void*> gprtGetVariable(
  SBTEntry *entry, std::string name, GPRTDataType type
) {
  auto found = entry->vars.find(std::string(name));

  // 1. Figure out if the variable "name" exists 
  assert(found != entry->vars.end());
  
  // 2. Assert the types match 
  assert(found->second.decl.type == type);

  std::pair<size_t, void*> variable;

  // 3. Get the expected size for this variable. 
  variable.first = getSize(found->second.decl.type);

  // 4. Get the pointer to the SBT. 
  variable.second = found->second.data;

  return variable;
}

#ifdef __cplusplus
// ------------------------------------------------------------------
// setters for variables of type "bool" (bools only on c++)
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1b(GPRTCompute _compute, const char *name, bool x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL);
  bool val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2b(GPRTCompute _compute, const char *name, bool x, bool y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  bool val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3b(GPRTCompute _compute, const char *name, bool x, bool y, bool z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  bool val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4b(GPRTCompute _compute, const char *name, bool x, bool y, bool z, bool w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  bool val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2bv(GPRTCompute _compute, const char *name, const bool *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3bv(GPRTCompute _compute, const char *name, const bool *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4bv(GPRTCompute _compute, const char *name, const bool *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1b(GPRTRayGen _raygen, const char *name, bool x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL);
  bool val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2b(GPRTRayGen _raygen, const char *name, bool x, bool y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  bool val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3b(GPRTRayGen _raygen, const char *name, bool x, bool y, bool z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  bool val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4b(GPRTRayGen _raygen, const char *name, bool x, bool y, bool z, bool w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  bool val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2bv(GPRTRayGen _raygen, const char *name, const bool *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3bv(GPRTRayGen _raygen, const char *name, const bool *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4bv(GPRTRayGen _raygen, const char *name, const bool *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1b(GPRTMiss _miss, const char *name, bool x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL);
  bool val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2b(GPRTMiss _miss, const char *name, bool x, bool y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  bool val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3b(GPRTMiss _miss, const char *name, bool x, bool y, bool z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  bool val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4b(GPRTMiss _miss, const char *name, bool x, bool y, bool z, bool w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  bool val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2bv(GPRTMiss _miss, const char *name, const bool *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3bv(GPRTMiss _miss, const char *name, const bool *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4bv(GPRTMiss _miss, const char *name, const bool *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1b(GPRTGeom _geom, const char *name, bool x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL);
  bool val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2b(GPRTGeom _geom, const char *name, bool x, bool y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  bool val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3b(GPRTGeom _geom, const char *name, bool x, bool y, bool z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  bool val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4b(GPRTGeom _geom, const char *name, bool x, bool y, bool z, bool w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  bool val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2bv(GPRTGeom _geom, const char *name, const bool *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3bv(GPRTGeom _geom, const char *name, const bool *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4bv(GPRTGeom _geom, const char *name, const bool *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  memcpy(var.second, (void*)val, var.first);
}

// // setters for variables on "Params"s
// GPRT_API void gprtParamsSet1b(OWLParams var, const char *name, bool val);
// GPRT_API void gprtParamsSet2b(OWLParams var, const char *name, bool x, bool y);
// GPRT_API void gprtParamsSet3b(OWLParams var, const char *name, bool x, bool y, bool z);
// GPRT_API void gprtParamsSet4b(OWLParams var, const char *name, bool x, bool y, bool z, bool w);
// GPRT_API void gprtParamsSet2bv(OWLParams var, const char *name, const bool *val);
// GPRT_API void gprtParamsSet3bv(OWLParams var, const char *name, const bool *val);
// GPRT_API void gprtParamsSet4bv(OWLParams var, const char *name, const bool *val);
#endif

// ------------------------------------------------------------------
// setters for variables of type "char"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1c(GPRTCompute _compute, const char *name, int8_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T);
  int8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2c(GPRTCompute _compute, const char *name, int8_t x, int8_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  int8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3c(GPRTCompute _compute, const char *name, int8_t x, int8_t y, int8_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  int8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4c(GPRTCompute _compute, const char *name, int8_t x, int8_t y, int8_t z, int8_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  int8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2cv(GPRTCompute _compute, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3cv(GPRTCompute _compute, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4cv(GPRTCompute _compute, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1c(GPRTRayGen _raygen, const char *name, int8_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T);
  int8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2c(GPRTRayGen _raygen, const char *name, int8_t x, int8_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  int8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3c(GPRTRayGen _raygen, const char *name, int8_t x, int8_t y, int8_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  int8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4c(GPRTRayGen _raygen, const char *name, int8_t x, int8_t y, int8_t z, int8_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  int8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2cv(GPRTRayGen _raygen, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3cv(GPRTRayGen _raygen, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4cv(GPRTRayGen _raygen, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1c(GPRTMiss _miss, const char *name, int8_t x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T);
  int8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2c(GPRTMiss _miss, const char *name, int8_t x, int8_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  int8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3c(GPRTMiss _miss, const char *name, int8_t x, int8_t y, int8_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  int8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4c(GPRTMiss _miss, const char *name, int8_t x, int8_t y, int8_t z, int8_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  int8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2cv(GPRTMiss _miss, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3cv(GPRTMiss _miss, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4cv(GPRTMiss _miss, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1c(GPRTGeom _geom, const char *name, int8_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T);
  int8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2c(GPRTGeom _geom, const char *name, int8_t x, int8_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  int8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3c(GPRTGeom _geom, const char *name, int8_t x, int8_t y, int8_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  int8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4c(GPRTGeom _geom, const char *name, int8_t x, int8_t y, int8_t z, int8_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  int8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2cv(GPRTGeom _geom, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3cv(GPRTGeom _geom, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4cv(GPRTGeom _geom, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1c(OWLParams obj, const char *name, char val);
// GPRT_API void gprtParamsSet2c(OWLParams obj, const char *name, char x, char y);
// GPRT_API void gprtParamsSet3c(OWLParams obj, const char *name, char x, char y, char z);
// GPRT_API void gprtParamsSet4c(OWLParams obj, const char *name, char x, char y, char z, char w);
// GPRT_API void gprtParamsSet2cv(OWLParams obj, const char *name, const char *val);
// GPRT_API void gprtParamsSet3cv(OWLParams obj, const char *name, const char *val);
// GPRT_API void gprtParamsSet4cv(OWLParams obj, const char *name, const char *val);

// ------------------------------------------------------------------
// setters for variables of type "uint8_t"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1uc(GPRTCompute _compute, const char *name, uint8_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T);
  uint8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2uc(GPRTCompute _compute, const char *name, uint8_t x, uint8_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  uint8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3uc(GPRTCompute _compute, const char *name, uint8_t x, uint8_t y, uint8_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  uint8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4uc(GPRTCompute _compute, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  uint8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2ucv(GPRTCompute _compute, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3ucv(GPRTCompute _compute, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4ucv(GPRTCompute _compute, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1uc(GPRTRayGen _raygen, const char *name, uint8_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T);
  uint8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2uc(GPRTRayGen _raygen, const char *name, uint8_t x, uint8_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  uint8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3uc(GPRTRayGen _raygen, const char *name, uint8_t x, uint8_t y, uint8_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  uint8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4uc(GPRTRayGen _raygen, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  uint8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ucv(GPRTRayGen _raygen, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3ucv(GPRTRayGen _raygen, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4ucv(GPRTRayGen _raygen, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1uc(GPRTMiss _miss, const char *name, uint8_t x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T);
  uint8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2uc(GPRTMiss _miss, const char *name, uint8_t x, uint8_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  uint8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3uc(GPRTMiss _miss, const char *name, uint8_t x, uint8_t y, uint8_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  uint8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4uc(GPRTMiss _miss, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  uint8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2ucv(GPRTMiss _miss, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3ucv(GPRTMiss _miss, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4ucv(GPRTMiss _miss, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1uc(GPRTGeom _geom, const char *name, uint8_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T);
  uint8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2uc(GPRTGeom _geom, const char *name, uint8_t x, uint8_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  uint8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3uc(GPRTGeom _geom, const char *name, uint8_t x, uint8_t y, uint8_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  uint8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4uc(GPRTGeom _geom, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  uint8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ucv(GPRTGeom _geom, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3ucv(GPRTGeom _geom, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4ucv(GPRTGeom _geom, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1uc(OWLParams obj, const char *name, uint8_t val);
// GPRT_API void gprtParamsSet2uc(OWLParams obj, const char *name, uint8_t x, uint8_t y);
// GPRT_API void gprtParamsSet3uc(OWLParams obj, const char *name, uint8_t x, uint8_t y, uint8_t z);
// GPRT_API void gprtParamsSet4uc(OWLParams obj, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w);
// GPRT_API void gprtParamsSet2ucv(OWLParams obj, const char *name, const uint8_t *val);
// GPRT_API void gprtParamsSet3ucv(OWLParams obj, const char *name, const uint8_t *val);
// GPRT_API void gprtParamsSet4ucv(OWLParams obj, const char *name, const uint8_t *val);

// ------------------------------------------------------------------
// setters for variables of type "int16_t"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1s(GPRTCompute _compute, const char *name, int16_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T);
  int16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2s(GPRTCompute _compute, const char *name, int16_t x, int16_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  int16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3s(GPRTCompute _compute, const char *name, int16_t x, int16_t y, int16_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  int16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4s(GPRTCompute _compute, const char *name, int16_t x, int16_t y, int16_t z, int16_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  int16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2sv(GPRTCompute _compute, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3sv(GPRTCompute _compute, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4sv(GPRTCompute _compute, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1s(GPRTRayGen _raygen, const char *name, int16_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T);
  int16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2s(GPRTRayGen _raygen, const char *name, int16_t x, int16_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  int16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3s(GPRTRayGen _raygen, const char *name, int16_t x, int16_t y, int16_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  int16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4s(GPRTRayGen _raygen, const char *name, int16_t x, int16_t y, int16_t z, int16_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  int16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2sv(GPRTRayGen _raygen, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3sv(GPRTRayGen _raygen, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4sv(GPRTRayGen _raygen, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1s(GPRTMiss _miss, const char *name, int16_t val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T);
  memcpy(var.second, (void*)&val, var.first);
}

GPRT_API void gprtMissSet2s(GPRTMiss _miss, const char *name, int16_t x, int16_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  int16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3s(GPRTMiss _miss, const char *name, int16_t x, int16_t y, int16_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  int16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4s(GPRTMiss _miss, const char *name, int16_t x, int16_t y, int16_t z, int16_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  int16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2sv(GPRTMiss _miss, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3sv(GPRTMiss _miss, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4sv(GPRTMiss _miss, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1s(GPRTGeom _geom, const char *name, int16_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T);
  int16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2s(GPRTGeom _geom, const char *name, int16_t x, int16_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  int16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3s(GPRTGeom _geom, const char *name, int16_t x, int16_t y, int16_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  int16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4s(GPRTGeom _geom, const char *name, int16_t x, int16_t y, int16_t z, int16_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  int16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2sv(GPRTGeom _geom, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3sv(GPRTGeom _geom, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4sv(GPRTGeom _geom, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1s(OWLParams obj, const char *name, int16_t val);
// GPRT_API void gprtParamsSet2s(OWLParams obj, const char *name, int16_t x, int16_t y);
// GPRT_API void gprtParamsSet3s(OWLParams obj, const char *name, int16_t x, int16_t y, int16_t z);
// GPRT_API void gprtParamsSet4s(OWLParams obj, const char *name, int16_t x, int16_t y, int16_t z, int16_t w);
// GPRT_API void gprtParamsSet2sv(OWLParams obj, const char *name, const int16_t *val);
// GPRT_API void gprtParamsSet3sv(OWLParams obj, const char *name, const int16_t *val);
// GPRT_API void gprtParamsSet4sv(OWLParams obj, const char *name, const int16_t *val);

// ------------------------------------------------------------------
// setters for variables of type "uint16_t"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1us(GPRTCompute _compute, const char *name, uint16_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T);
  uint16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2us(GPRTCompute _compute, const char *name, uint16_t x, uint16_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  uint16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3us(GPRTCompute _compute, const char *name, uint16_t x, uint16_t y, uint16_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  uint16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4us(GPRTCompute _compute, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  uint16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2usv(GPRTCompute _compute, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3usv(GPRTCompute _compute, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4usv(GPRTCompute _compute, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1us(GPRTRayGen _raygen, const char *name, uint16_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T);
  uint16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2us(GPRTRayGen _raygen, const char *name, uint16_t x, uint16_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  uint16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3us(GPRTRayGen _raygen, const char *name, uint16_t x, uint16_t y, uint16_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  uint16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4us(GPRTRayGen _raygen, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  uint16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2usv(GPRTRayGen _raygen, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3usv(GPRTRayGen _raygen, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4usv(GPRTRayGen _raygen, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1us(GPRTMiss _miss, const char *name, uint16_t x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T);
  uint16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2us(GPRTMiss _miss, const char *name, uint16_t x, uint16_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  uint16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3us(GPRTMiss _miss, const char *name, uint16_t x, uint16_t y, uint16_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  uint16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4us(GPRTMiss _miss, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  uint16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2usv(GPRTMiss _miss, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3usv(GPRTMiss _miss, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4usv(GPRTMiss _miss, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1us(GPRTGeom _geom, const char *name, uint16_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T);
  uint16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2us(GPRTGeom _geom, const char *name, uint16_t x, uint16_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  uint16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3us(GPRTGeom _geom, const char *name, uint16_t x, uint16_t y, uint16_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  uint16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4us(GPRTGeom _geom, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  uint16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2usv(GPRTGeom _geom, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3usv(GPRTGeom _geom, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4usv(GPRTGeom _geom, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1us(OWLParams obj, const char *name, uint16_t val);
// GPRT_API void gprtParamsSet2us(OWLParams obj, const char *name, uint16_t x, uint16_t y);
// GPRT_API void gprtParamsSet3us(OWLParams obj, const char *name, uint16_t x, uint16_t y, uint16_t z);
// GPRT_API void gprtParamsSet4us(OWLParams obj, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w);
// GPRT_API void gprtParamsSet2usv(OWLParams obj, const char *name, const uint16_t *val);
// GPRT_API void gprtParamsSet3usv(OWLParams obj, const char *name, const uint16_t *val);
// GPRT_API void gprtParamsSet4usv(OWLParams obj, const char *name, const uint16_t *val);

// ------------------------------------------------------------------
// setters for variables of type "int"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1i(GPRTCompute _compute, const char *name, int32_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T);
  int32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2i(GPRTCompute _compute, const char *name, int32_t x, int32_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  int32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3i(GPRTCompute _compute, const char *name, int32_t x, int32_t y, int32_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  int32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4i(GPRTCompute _compute, const char *name, int32_t x, int32_t y, int32_t z, int32_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  int32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2iv(GPRTCompute _compute, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3iv(GPRTCompute _compute, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4iv(GPRTCompute _compute, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1i(GPRTRayGen _raygen, const char *name, int32_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T);
  int32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2i(GPRTRayGen _raygen, const char *name, int32_t x, int32_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  int32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3i(GPRTRayGen _raygen, const char *name, int32_t x, int32_t y, int32_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  int32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4i(GPRTRayGen _raygen, const char *name, int32_t x, int32_t y, int32_t z, int32_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  int32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2iv(GPRTRayGen _raygen, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3iv(GPRTRayGen _raygen, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4iv(GPRTRayGen _raygen, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1i(GPRTMiss _miss, const char *name, int32_t x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T);
  int32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2i(GPRTMiss _miss, const char *name, int32_t x, int32_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  int32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3i(GPRTMiss _miss, const char *name, int32_t x, int32_t y, int32_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  int32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4i(GPRTMiss _miss, const char *name, int32_t x, int32_t y, int32_t z, int32_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  int32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2iv(GPRTMiss _miss, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3iv(GPRTMiss _miss, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4iv(GPRTMiss _miss, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1i(GPRTGeom _geom, const char *name, int32_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T);
  int32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2i(GPRTGeom _geom, const char *name, int32_t x, int32_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  int32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3i(GPRTGeom _geom, const char *name, int32_t x, int32_t y, int32_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  int32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4i(GPRTGeom _geom, const char *name, int32_t x, int32_t y, int32_t z, int32_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  int32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2iv(GPRTGeom _geom, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3iv(GPRTGeom _geom, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4iv(GPRTGeom _geom, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1i(OWLParams obj, const char *name, int32_t val);
// GPRT_API void gprtParamsSet2i(OWLParams obj, const char *name, int32_t x, int32_t y);
// GPRT_API void gprtParamsSet3i(OWLParams obj, const char *name, int32_t x, int32_t y, int32_t z);
// GPRT_API void gprtParamsSet4i(OWLParams obj, const char *name, int32_t x, int32_t y, int32_t z, int32_t w);
// GPRT_API void gprtParamsSet2iv(OWLParams obj, const char *name, const int32_t *val);
// GPRT_API void gprtParamsSet3iv(OWLParams obj, const char *name, const int32_t *val);
// GPRT_API void gprtParamsSet4iv(OWLParams obj, const char *name, const int32_t *val);

// ------------------------------------------------------------------
// setters for variables of type "uint32_t"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1ui(GPRTCompute _compute, const char *name, uint32_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T);
  uint32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2ui(GPRTCompute _compute, const char *name, uint32_t x, uint32_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  uint32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3ui(GPRTCompute _compute, const char *name, uint32_t x, uint32_t y, uint32_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  uint32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4ui(GPRTCompute _compute, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  uint32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2uiv(GPRTCompute _compute, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3uiv(GPRTCompute _compute, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4uiv(GPRTCompute _compute, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1ui(GPRTRayGen _raygen, const char *name, uint32_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T);
  uint32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ui(GPRTRayGen _raygen, const char *name, uint32_t x, uint32_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  uint32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3ui(GPRTRayGen _raygen, const char *name, uint32_t x, uint32_t y, uint32_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  uint32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4ui(GPRTRayGen _raygen, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  uint32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2uiv(GPRTRayGen _raygen, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3uiv(GPRTRayGen _raygen, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4uiv(GPRTRayGen _raygen, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1ui(GPRTMiss _miss, const char *name, uint32_t x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T);
  uint32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2ui(GPRTMiss _miss, const char *name, uint32_t x, uint32_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  uint32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3ui(GPRTMiss _miss, const char *name, uint32_t x, uint32_t y, uint32_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  uint32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4ui(GPRTMiss _miss, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  uint32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2uiv(GPRTMiss _miss, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3uiv(GPRTMiss _miss, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4uiv(GPRTMiss _miss, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1ui(GPRTGeom _geom, const char *name, uint32_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T);
  uint32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ui(GPRTGeom _geom, const char *name, uint32_t x, uint32_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  uint32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3ui(GPRTGeom _geom, const char *name, uint32_t x, uint32_t y, uint32_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  uint32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4ui(GPRTGeom _geom, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  uint32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2uiv(GPRTGeom _geom, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3uiv(GPRTGeom _geom, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4uiv(GPRTGeom _geom, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1ui(OWLParams obj, const char *name, uint32_t val);
// GPRT_API void gprtParamsSet2ui(OWLParams obj, const char *name, uint32_t x, uint32_t y);
// GPRT_API void gprtParamsSet3ui(OWLParams obj, const char *name, uint32_t x, uint32_t y, uint32_t z);
// GPRT_API void gprtParamsSet4ui(OWLParams obj, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w);
// GPRT_API void gprtParamsSet2uiv(OWLParams obj, const char *name, const uint32_t *val);
// GPRT_API void gprtParamsSet3uiv(OWLParams obj, const char *name, const uint32_t *val);
// GPRT_API void gprtParamsSet4uiv(OWLParams obj, const char *name, const uint32_t *val);

// ------------------------------------------------------------------
// setters for variables of type "float"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1f(GPRTCompute _compute, const char *name, float x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT);
  float val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2f(GPRTCompute _compute, const char *name, float x, float y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  float val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3f(GPRTCompute _compute, const char *name, float x, float y, float z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  float val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4f(GPRTCompute _compute, const char *name, float x, float y, float z, float w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  float val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2fv(GPRTCompute _compute, const char *name, const float *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3fv(GPRTCompute _compute, const char *name, const float *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4fv(GPRTCompute _compute, const char *name, const float *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1f(GPRTRayGen _raygen, const char *name, float x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT);
  float val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2f(GPRTRayGen _raygen, const char *name, float x, float y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  float val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3f(GPRTRayGen _raygen, const char *name, float x, float y, float z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  float val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4f(GPRTRayGen _raygen, const char *name, float x, float y, float z, float w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  float val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2fv(GPRTRayGen _raygen, const char *name, const float *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3fv(GPRTRayGen _raygen, const char *name, const float *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4fv(GPRTRayGen _raygen, const char *name, const float *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1f(GPRTMiss _miss, const char *name, float x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT);
  float val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2f(GPRTMiss _miss, const char *name, float x, float y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  float val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3f(GPRTMiss _miss, const char *name, float x, float y, float z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  float val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4f(GPRTMiss _miss, const char *name, float x, float y, float z, float w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  float val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2fv(GPRTMiss _miss, const char *name, const float *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3fv(GPRTMiss _miss, const char *name, const float *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4fv(GPRTMiss _miss, const char *name, const float *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1f(GPRTGeom _geom, const char *name, float x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT);
  float val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2f(GPRTGeom _geom, const char *name, float x, float y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  float val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3f(GPRTGeom _geom, const char *name, float x, float y, float z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  float val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4f(GPRTGeom _geom, const char *name, float x, float y, float z, float w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  float val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2fv(GPRTGeom _geom, const char *name, const float *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3fv(GPRTGeom _geom, const char *name, const float *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4fv(GPRTGeom _geom, const char *name, const float *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1f(OWLParams obj, const char *name, float val);
// GPRT_API void gprtParamsSet2f(OWLParams obj, const char *name, float x, float y);
// GPRT_API void gprtParamsSet3f(OWLParams obj, const char *name, float x, float y, float z);
// GPRT_API void gprtParamsSet4f(OWLParams obj, const char *name, float x, float y, float z, float w);
// GPRT_API void gprtParamsSet2fv(OWLParams obj, const char *name, const float *val);
// GPRT_API void gprtParamsSet3fv(OWLParams obj, const char *name, const float *val);
// GPRT_API void gprtParamsSet4fv(OWLParams obj, const char *name, const float *val);

// ------------------------------------------------------------------
// setters for variables of type "double"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1d(GPRTCompute _compute, const char *name, double x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE);
  double val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2d(GPRTCompute _compute, const char *name, double x, double y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  double val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3d(GPRTCompute _compute, const char *name, double x, double y, double z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  double val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4d(GPRTCompute _compute, const char *name, double x, double y, double z, double w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  double val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2dv(GPRTCompute _compute, const char *name, const double *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3dv(GPRTCompute _compute, const char *name, const double *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4dv(GPRTCompute _compute, const char *name, const double *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1d(GPRTRayGen _raygen, const char *name, double x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE);
  double val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2d(GPRTRayGen _raygen, const char *name, double x, double y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  double val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3d(GPRTRayGen _raygen, const char *name, double x, double y, double z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  double val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4d(GPRTRayGen _raygen, const char *name, double x, double y, double z, double w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  double val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2dv(GPRTRayGen _raygen, const char *name, const double *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3dv(GPRTRayGen _raygen, const char *name, const double *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4dv(GPRTRayGen _raygen, const char *name, const double *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1d(GPRTMiss _miss, const char *name, double x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE);
  double val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2d(GPRTMiss _miss, const char *name, double x, double y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  double val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3d(GPRTMiss _miss, const char *name, double x, double y, double z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  double val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4d(GPRTMiss _miss, const char *name, double x, double y, double z, double w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  double val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2dv(GPRTMiss _miss, const char *name, const double *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3dv(GPRTMiss _miss, const char *name, const double *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4dv(GPRTMiss _miss, const char *name, const double *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1d(GPRTGeom _geom, const char *name, double x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE);
  double val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2d(GPRTGeom _geom, const char *name, double x, double y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  double val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3d(GPRTGeom _geom, const char *name, double x, double y, double z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  double val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4d(GPRTGeom _geom, const char *name, double x, double y, double z, double w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  double val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2dv(GPRTGeom _geom, const char *name, const double *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3dv(GPRTGeom _geom, const char *name, const double *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4dv(GPRTGeom _geom, const char *name, const double *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1d(OWLParams obj, const char *name, double val);
// GPRT_API void gprtParamsSet2d(OWLParams obj, const char *name, double x, double y);
// GPRT_API void gprtParamsSet3d(OWLParams obj, const char *name, double x, double y, double z);
// GPRT_API void gprtParamsSet4d(OWLParams obj, const char *name, double x, double y, double z, double w);
// GPRT_API void gprtParamsSet2dv(OWLParams obj, const char *name, const double *val);
// GPRT_API void gprtParamsSet3dv(OWLParams obj, const char *name, const double *val);
// GPRT_API void gprtParamsSet4dv(OWLParams obj, const char *name, const double *val);

// ------------------------------------------------------------------
// setters for variables of type "int64_t"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1l(GPRTCompute _compute, const char *name, int64_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T);
  int64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2l(GPRTCompute _compute, const char *name, int64_t x, int64_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  int64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3l(GPRTCompute _compute, const char *name, int64_t x, int64_t y, int64_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  int64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4l(GPRTCompute _compute, const char *name, int64_t x, int64_t y, int64_t z, int64_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  int64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2lv(GPRTCompute _compute, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3lv(GPRTCompute _compute, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4lv(GPRTCompute _compute, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1l(GPRTRayGen _raygen, const char *name, int64_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T);
  int64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2l(GPRTRayGen _raygen, const char *name, int64_t x, int64_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  int64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3l(GPRTRayGen _raygen, const char *name, int64_t x, int64_t y, int64_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  int64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4l(GPRTRayGen _raygen, const char *name, int64_t x, int64_t y, int64_t z, int64_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  int64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2lv(GPRTRayGen _raygen, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3lv(GPRTRayGen _raygen, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4lv(GPRTRayGen _raygen, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1l(GPRTMiss _miss, const char *name, int64_t x)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T);
  int64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2l(GPRTMiss _miss, const char *name, int64_t x, int64_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  int64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3l(GPRTMiss _miss, const char *name, int64_t x, int64_t y, int64_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  int64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4l(GPRTMiss _miss, const char *name, int64_t x, int64_t y, int64_t z, int64_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  int64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2lv(GPRTMiss _miss, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3lv(GPRTMiss _miss, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4lv(GPRTMiss _miss, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1l(GPRTGeom _geom, const char *name, int64_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T);
  int64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2l(GPRTGeom _geom, const char *name, int64_t x, int64_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  int64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3l(GPRTGeom _geom, const char *name, int64_t x, int64_t y, int64_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  int64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4l(GPRTGeom _geom, const char *name, int64_t x, int64_t y, int64_t z, int64_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  int64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2lv(GPRTGeom _geom, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3lv(GPRTGeom _geom, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4lv(GPRTGeom _geom, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1l(OWLParams obj, const char *name, int64_t val);
// GPRT_API void gprtParamsSet2l(OWLParams obj, const char *name, int64_t x, int64_t y);
// GPRT_API void gprtParamsSet3l(OWLParams obj, const char *name, int64_t x, int64_t y, int64_t z);
// GPRT_API void gprtParamsSet4l(OWLParams obj, const char *name, int64_t x, int64_t y, int64_t z, int64_t w);
// GPRT_API void gprtParamsSet2lv(OWLParams obj, const char *name, const int64_t *val);
// GPRT_API void gprtParamsSet3lv(OWLParams obj, const char *name, const int64_t *val);
// GPRT_API void gprtParamsSet4lv(OWLParams obj, const char *name, const int64_t *val);

// ------------------------------------------------------------------
// setters for variables of type "uint64_t"
// ------------------------------------------------------------------

// setters for variables on "Compute"s
GPRT_API void gprtComputeSet1ul(GPRTCompute _compute, const char *name, uint64_t x) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T);
  uint64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2ul(GPRTCompute _compute, const char *name, uint64_t x, uint64_t y) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  uint64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet3ul(GPRTCompute _compute, const char *name, uint64_t x, uint64_t y, uint64_t z) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  uint64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet4ul(GPRTCompute _compute, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  uint64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtComputeSet2ulv(GPRTCompute _compute, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet3ulv(GPRTCompute _compute, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtComputeSet4ulv(GPRTCompute _compute, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  Compute *entry = (Compute*)_compute;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1ul(GPRTRayGen _raygen, const char *name, uint64_t x) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T);
  uint64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ul(GPRTRayGen _raygen, const char *name, uint64_t x, uint64_t y) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  uint64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3ul(GPRTRayGen _raygen, const char *name, uint64_t x, uint64_t y, uint64_t z) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  uint64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4ul(GPRTRayGen _raygen, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  uint64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ulv(GPRTRayGen _raygen, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet3ulv(GPRTRayGen _raygen, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtRayGenSet4ulv(GPRTRayGen _raygen, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  RayGen *entry = (RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  memcpy(var.second, (void*)val, var.first);
}

// setters for variables on "MissProg"s
GPRT_API void gprtMissSet1ul(GPRTMiss _miss, const char *name, uint64_t val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet2ul(GPRTMiss _miss, const char *name, uint64_t x, uint64_t y)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  uint64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet3ul(GPRTMiss _miss, const char *name, uint64_t x, uint64_t y, uint64_t z)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  uint64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet4ul(GPRTMiss _miss, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  uint64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissSet2ulv(GPRTMiss _miss, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet3ulv(GPRTMiss _miss, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtMissSet4ulv(GPRTMiss _miss, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  Miss *entry = (Miss*)_miss;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1ul(GPRTGeom _geom, const char *name, uint64_t x)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T);
  uint64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ul(GPRTGeom _geom, const char *name, uint64_t x, uint64_t y)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  uint64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3ul(GPRTGeom _geom, const char *name, uint64_t x, uint64_t y, uint64_t z)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  uint64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4ul(GPRTGeom _geom, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  uint64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ulv(GPRTGeom _geom, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet3ulv(GPRTGeom _geom, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  memcpy(var.second, (void*)val, var.first);
}

GPRT_API void gprtGeomSet4ulv(GPRTGeom _geom, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  Geom *entry = (Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  memcpy(var.second, (void*)val, var.first);
}


// setters for variables on "Params"s
// GPRT_API void gprtParamsSet1ul(OWLParams obj, const char *name, uint64_t val);
// GPRT_API void gprtParamsSet2ul(OWLParams obj, const char *name, uint64_t x, uint64_t y);
// GPRT_API void gprtParamsSet3ul(OWLParams obj, const char *name, uint64_t x, uint64_t y, uint64_t z);
// GPRT_API void gprtParamsSet4ul(OWLParams obj, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w);
// GPRT_API void gprtParamsSet2ulv(OWLParams obj, const char *name, const uint64_t *val);
// GPRT_API void gprtParamsSet3ulv(OWLParams obj, const char *name, const uint64_t *val);
// GPRT_API void gprtParamsSet4ulv(OWLParams obj, const char *name, const uint64_t *val);





























// ------------------------------------------------------------------
// setters for "meta" types
// ------------------------------------------------------------------

// setters for variables on "Compute"s
// GPRT_API void gprtComputeSetTexture(GPRTCompute _compute, const char *name, GPRTTexture val) 
// GPRT_API void gprtComputeSetPointer(GPRTCompute _compute, const char *name, const void *val) 
GPRT_API void gprtComputeSetBuffer(GPRTCompute _rayGen, const char *name, GPRTBuffer _val)
{
  LOG_API_CALL();
  Compute *raygen = (Compute*)_rayGen;
  assert(raygen);

  Buffer *val = (Buffer*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists (Maybe through a dictionary?)
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // The found variable must be a buffer
  assert(raygen->vars[name].decl.type == GPRT_BUFFER || 
         raygen->vars[name].decl.type == GPRT_BUFPTR);

  // Buffer pointers are 64 bits
  size_t size = sizeof(uint64_t);

  // 3. Assign the value to that variable
  VkDeviceAddress addr = val->address;
  memcpy(raygen->vars[name].data, &addr, size);
}

GPRT_API void gprtComputeSetAccel(GPRTCompute _compute, const char *name, GPRTAccel _val)
{
  LOG_API_CALL();
  Compute *raygen = (Compute*)_compute;
  assert(raygen);

  Accel *val = (Accel*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // The found variable must be an acceleration structure
  assert(raygen->vars[name].decl.type == GPRT_ACCEL);

  // Acceleration structure pointers are 64 bits
  size_t size = sizeof(uint64_t);

  // 3. Assign the value to that variable
  VkDeviceAddress addr = val->address;
  memcpy(raygen->vars[name].data, &addr, size);
}

GPRT_API void gprtComputeSetRaw(GPRTCompute _compute, const char *name, const void *val)
{
  LOG_API_CALL();
  Compute *raygen = (Compute*)_compute;
  assert(raygen);

  // 1. Figure out if the variable "name" exists (Maybe through a dictionary?)
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // 2. Get the expected size for this variable
  size_t size = getSize(raygen->vars[name].decl.type);

  // 3. Assign the value to that variable
  memcpy(raygen->vars[name].data, val, size);
}

// setters for variables on "RayGen"s
// GPRT_API void gprtRayGenSetTexture(GPRTRayGen _raygen, const char *name, GPRTTexture val) 
// GPRT_API void gprtRayGenSetPointer(GPRTRayGen _raygen, const char *name, const void *val) 
GPRT_API void gprtRayGenSetBuffer(GPRTRayGen _raygen, const char *name, GPRTBuffer _val)
{
  LOG_API_CALL();
  RayGen *raygen = (RayGen*)_raygen;
  assert(raygen);

  Buffer *val = (Buffer*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists (Maybe through a dictionary?)
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // The found variable must be a buffer
  assert(raygen->vars[name].decl.type == GPRT_BUFFER || 
         raygen->vars[name].decl.type == GPRT_BUFFER_POINTER);

  if (raygen->vars[name].decl.type == GPRT_BUFFER_POINTER) {
    // Buffer pointers are 64 bits
    size_t size = sizeof(uint64_t);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(raygen->vars[name].data, &addr, size);
  } else {
    gprt::Buffer buffer;
    buffer.x = val->address;
    buffer.y = val->size;

    size_t size = sizeof(uint64_t2);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(raygen->vars[name].data, &buffer, size);
  }
}

GPRT_API void gprtRayGenSetAccel(GPRTRayGen _raygen, const char *name, GPRTAccel _val)
{
  LOG_API_CALL();
  RayGen *raygen = (RayGen*)_raygen;
  assert(raygen);

  Accel *val = (Accel*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // The found variable must be an acceleration structure
  assert(raygen->vars[name].decl.type == GPRT_ACCEL ||
         raygen->vars[name].decl.type == GPRT_ACCEL_POINTER);

  if (raygen->vars[name].decl.type == GPRT_ACCEL_POINTER) {
    // Acceleration structure pointers are 64 bits
    size_t size = sizeof(uint64_t);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(raygen->vars[name].data, &addr, size);
  } else {
    gprt::Accel accel;
    accel.x = val->address;
    accel.y = 0; // todo

    size_t size = sizeof(uint64_t2);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(raygen->vars[name].data, &accel, size);
  }
}

GPRT_API void gprtRayGenSetRaw(GPRTRayGen _rayGen, const char *name, const void *val)
{
  LOG_API_CALL();
  RayGen *raygen = (RayGen*)_rayGen;
  assert(raygen);

  // 1. Figure out if the variable "name" exists (Maybe through a dictionary?)
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // 2. Get the expected size for this variable
  size_t size = getSize(raygen->vars[name].decl.type);

  // 3. Assign the value to that variable
  memcpy(raygen->vars[name].data, val, size);
}

// // setters for variables on "Geom"s
// GPRT_API void gprtGeomSetTexture(GPRTGeom _geom, const char *name, GPRTTexture val);
// GPRT_API void gprtGeomSetPointer(GPRTGeom _geom, const char *name, const void *val);
GPRT_API void gprtGeomSetBuffer(GPRTGeom _geom, const char *name, GPRTBuffer _val) 
{
  LOG_API_CALL();
  Geom *geom = (Geom*)_geom;
  assert(geom);

  Buffer *val = (Buffer*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(geom->vars.find(std::string(name)) != geom->vars.end());

  // The found variable must be a buffer
  assert(geom->vars[name].decl.type == GPRT_BUFFER || 
         geom->vars[name].decl.type == GPRT_BUFFER_POINTER);

  if (geom->vars[name].decl.type == GPRT_BUFFER_POINTER) {
    // Buffer pointers are 64 bits
    size_t size = sizeof(uint64_t);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(geom->vars[name].data, &addr, size);
  } else {
    gprt::Buffer buffer;
    buffer.x = val->address;
    buffer.y = val->size;

    size_t size = sizeof(uint64_t2);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(geom->vars[name].data, &buffer, size);
  }
}

GPRT_API void gprtGeomSetAccel(GPRTGeom _geom, const char *name, GPRTAccel _val)
{
  LOG_API_CALL();
  Geom *geom = (Geom*)_geom;
  assert(geom);

  Accel *val = (Accel*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(geom->vars.find(std::string(name)) != geom->vars.end());

  // The found variable must be an acceleration structure
  assert(geom->vars[name].decl.type == GPRT_ACCEL ||
         geom->vars[name].decl.type == GPRT_ACCEL_POINTER);

  if (geom->vars[name].decl.type == GPRT_ACCEL_POINTER) {
    // Acceleration structure pointers are 64 bits
    size_t size = sizeof(uint64_t);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(geom->vars[name].data, &addr, size);
  } else {
    gprt::Accel accel;
    accel.x = val->address;
    accel.y = 0; // todo

    size_t size = sizeof(uint64_t2);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(geom->vars[name].data, &accel, size);
  }
}

GPRT_API void gprtGeomSetRaw(GPRTGeom _geom, const char *name, const void *val)
{
  LOG_API_CALL();
  Geom *geom = (Geom*)_geom;
  assert(geom);

  // 1. Figure out if the variable "name" exists
  assert(geom->vars.find(std::string(name)) != geom->vars.end());

  // 2. Get the expected size for this variable
  size_t size = getSize(geom->vars[name].decl.type);

  // 3. Assign the value to that variable
  memcpy(geom->vars[name].data, val, size);
}

// // setters for variables on "Params"s
// GPRT_API void gprtParamsSetTexture(GPRTParams obj, const char *name, GPRTTexture val);
// GPRT_API void gprtParamsSetPointer(GPRTParams obj, const char *name, const void *val);
// GPRT_API void gprtParamsSetBuffer(GPRTParams obj, const char *name, GPRTBuffer val);
// GPRT_API void gprtParamsSetAccel(GPRTParams obj, const char *name, GPRTAccel val);
// GPRT_API void gprtParamsSetRaw(GPRTParams obj, const char *name, const void *val);

// setters for variables on "MissProg"s
// GPRT_API void gprtMissSetTexture(GPRTMiss _miss, const char *name, GPRTTexture val)

// GPRT_API void gprtMissSetPointer(GPRTMiss _miss, const char *name, const void *val);
GPRT_API void gprtMissSetBuffer(GPRTMiss _miss, const char *name, GPRTBuffer _val)
{
  LOG_API_CALL();
  Miss *miss = (Miss*)_miss;
  assert(miss);

  Buffer *val = (Buffer*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(miss->vars.find(std::string(name)) != miss->vars.end());

  // The found variable must be a buffer
  assert(miss->vars[name].decl.type == GPRT_BUFFER || 
         miss->vars[name].decl.type == GPRT_BUFFER_POINTER);

  if (miss->vars[name].decl.type == GPRT_BUFFER_POINTER) {
    // Buffer pointers are 64 bits
    size_t size = sizeof(uint64_t);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(miss->vars[name].data, &addr, size);
  } else {
    gprt::Buffer buffer;
    buffer.x = val->address;
    buffer.y = val->size;

    size_t size = sizeof(uint64_t2);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(miss->vars[name].data, &buffer, size);
  }
}

GPRT_API void gprtMissSetAccel(GPRTMiss _miss, const char *name, GPRTAccel _val)
{
  LOG_API_CALL();
  Miss *miss = (Miss*)_miss;
  assert(miss);

  Accel *val = (Accel*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(miss->vars.find(std::string(name)) != miss->vars.end());

  assert(miss->vars[name].decl.type == GPRT_ACCEL ||
         miss->vars[name].decl.type == GPRT_ACCEL_POINTER);

  if (miss->vars[name].decl.type == GPRT_ACCEL_POINTER) {
    // Acceleration structure pointers are 64 bits
    size_t size = sizeof(uint64_t);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(miss->vars[name].data, &addr, size);
  } else {
    gprt::Accel accel;
    accel.x = val->address;
    accel.y = 0; // todo

    size_t size = sizeof(uint64_t2);

    // 3. Assign the value to that variable
    VkDeviceAddress addr = val->address;
    memcpy(miss->vars[name].data, &accel, size);
  }
}

GPRT_API void gprtMissSetRaw(GPRTMiss _miss, const char *name, const void *val)
{
  LOG_API_CALL();
  Miss *missProg = (Miss*)_miss;
  assert(missProg);

  // 1. Figure out if the variable "name" exists
  assert(missProg->vars.find(std::string(name)) != missProg->vars.end());

  // 2. Get the expected size for this variable
  size_t size = getSize(missProg->vars[name].decl.type);

  // 3. Assign the value to that variable
  memcpy(missProg->vars[name].data, val, size);
}
