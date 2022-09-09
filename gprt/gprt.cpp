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

#include <regex>

#ifdef __GNUC__
#include <execinfo.h>
#include <sys/time.h>
#include <signal.h>
#endif

#ifdef _WIN32
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
  if (gprt::Context::logging())                       \
    std::cout                                   \
      << GPRT_TERMINAL_LIGHT_BLUE                \
      << "#gprt: "                               \
      << message                                \
      << GPRT_TERMINAL_DEFAULT << std::endl

#define LOG_OK(message)                         \
  if (gprt::Context::logging())                       \
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
  debugMessage << prefix << "[" << pCallbackData->messageIdNumber << "][" << pCallbackData->pMessageIdName << "] : " << pCallbackData->pMessage;

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

namespace gprt {
  // forward declarations...
  struct Geom; 
  struct GeomType; 
  struct TrianglesGeom;
  struct TrianglesGeomType;
  struct UserGeom;
  struct UserGeomType;

  PFN_vkGetBufferDeviceAddressKHR vkGetBufferDeviceAddressKHR;
  PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructureKHR;
  PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructureKHR;
  PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizesKHR;
  PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddressKHR;
  PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructuresKHR;
  PFN_vkBuildAccelerationStructuresKHR vkBuildAccelerationStructuresKHR;
  PFN_vkCmdTraceRaysKHR vkCmdTraceRaysKHR;
  PFN_vkGetRayTracingShaderGroupHandlesKHR vkGetRayTracingShaderGroupHandlesKHR;
  PFN_vkCreateRayTracingPipelinesKHR vkCreateRayTracingPipelinesKHR;

  PFN_vkCreateDebugUtilsMessengerEXT vkCreateDebugUtilsMessengerEXT;
  PFN_vkDestroyDebugUtilsMessengerEXT vkDestroyDebugUtilsMessengerEXT;
  VkDebugUtilsMessengerEXT debugUtilsMessenger;

  PFN_vkCreateDebugReportCallbackEXT vkCreateDebugReportCallbackEXT;
  PFN_vkDestroyDebugReportCallbackEXT vkDestroyDebugReportCallbackEXT;
  VkDebugReportCallbackEXT debugReportCallback;

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
    std::map<std::string, std::vector<uint8_t>> program;

    Module(std::map<std::string, std::vector<uint8_t>> program) {
      this->program = program;
    }

    ~Module() {
    }

    std::vector<uint32_t> getBinary(std::string entryType, std::string entryPoint) {
      size_t sizeOfProgram = program[entryType].size() -  1; // program is null terminated.
      std::vector<uint32_t> finalProgram(sizeOfProgram / 4);
      memcpy(finalProgram.data(), program[entryType].data(), sizeOfProgram);
      return finalProgram;


      // std::regex re("( *)(OpEntryPoint )(.*? )([%][A-Za-z]*)( \"[A-Za-z]*\" )(.*)");
      // std::smatch match;

      // std::string text;

      // spv_text spvText = nullptr;
      // spv_diagnostic spvTextDiagnostic = nullptr;
      // std::vector<uint8_t> prog = program[entryType];
      // spvBinaryToText(spvContext, (uint32_t*)prog.data(), prog.size() / 4, SPV_BINARY_TO_TEXT_OPTION_NONE, &spvText, &spvTextDiagnostic);
      // text = std::string(spvText->str);
      // spvTextDestroy(spvText);

      // std::string singleEntryPointProgram;
      // while (std::regex_search(text, match, re))
      // {
      //   std::string line = match.str(0);
      //   std::string otherEntryPoint = match.str(4);

      //   if (match.str(4) == (std::string("%") + std::string(entryPoint)) ) {
      //     singleEntryPointProgram += match.prefix().str() + match.str(0);
      //     text = match.suffix().str();
      //   }
      //   else {
      //     // Remove the other entry points. Currently, SPIRV doesn't support
      //     // multiple entry points in combination with debug printf
      //     singleEntryPointProgram += match.prefix();
      //     text = match.suffix().str();

      //     // Remove the OpName %entrypoint "entrypoint" line
      //     std::regex OpNameRE("( *)(OpName )(" + otherEntryPoint + ")( \"[A-Za-z]*\")");
      //     std::smatch OpNameMatch;
      //     std::string subtext = text;
      //     if (!std::regex_search(subtext, OpNameMatch, OpNameRE)) throw std::runtime_error("Error editing SPIRV");
      //     else {
      //       // std::cout<<"Found OpName: "<< OpNameMatch.str(0) << " ... removing..."<<std::endl;
      //       text = OpNameMatch.prefix().str() + OpNameMatch.suffix().str();
      //     }

      //     // Remove the entrypoint itself.
      //     // found by %entrypoint = ... until the first occurance of OpFunctionEnd
      //     std::regex OpEntryPointRE("( *)(" + otherEntryPoint + " =[^]*?OpFunctionEnd)");
      //     std::smatch OpEntryPointMatch;
      //     subtext = text;
      //     if (!std::regex_search(subtext, OpEntryPointMatch, OpEntryPointRE)) throw std::runtime_error("Error editing SPIRV");
      //     else {
      //       // std::cout<<"Found Entry Point: "<< OpEntryPointMatch.str(0) << " ... removing..."<<std::endl;
      //       text = OpEntryPointMatch.prefix().str() + OpEntryPointMatch.suffix().str();
      //     }
      //   }
      // }
      // singleEntryPointProgram += text;
      
      // // Now, assemble the IR
      // spv_binary binary = nullptr;
      // spv_diagnostic diagnostic = nullptr;
      // spvTextToBinary(spvContext, singleEntryPointProgram.c_str(), singleEntryPointProgram.size(), &binary, &diagnostic);
      // spvValidateBinary(spvContext, binary->code, binary->wordCount, &diagnostic);
      // if (diagnostic) {
      //   spvDiagnosticPrint(diagnostic);
      //   spvDiagnosticDestroy(diagnostic);
      // }

      // std::vector<uint32_t> finalBinary(binary->wordCount);
      // memcpy(finalBinary.data(), binary->code, finalBinary.size() * sizeof(uint32_t));
      // spvBinaryDestroy(binary);
      // return finalBinary;
    }

    // void releaseBinary(spv_binary binary) {
    //   spvBinaryDestroy(binary);
    // }
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
      VkDeviceAddress addr = vkGetBufferDeviceAddressKHR(device, &info);
      return addr;
    }

    /*! Calls vkDestroy on the buffer, and frees underlying memory */
    void destroy()
    {
      if (buffer)
        vkDestroyBuffer(device, buffer, nullptr);
      if (memory)
        vkFreeMemory(device, memory, nullptr);
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
          throw std::runtime_error("Could not find a matching memory type");
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
      if (err) throw std::runtime_error("failed to bind buffer memory! : \n" + errorString(err));

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
        if (err) throw std::runtime_error("failed to bind staging buffer memory! : \n" + errorString(err));
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
 
  struct ComputeProg : public SBTEntry {
    VkShaderModule shaderModule;
    VkPipelineShaderStageCreateInfo shaderStage{};
    VkShaderModuleCreateInfo moduleCreateInfo{};
    VkDevice logicalDevice;

    ComputeProg(VkDevice  _logicalDevice,
             Module *module,
             const char* _entryPoint,
             size_t      sizeOfVarStruct,
             std::unordered_map<std::string, GPRTVarDef> _vars) : SBTEntry()
    {
      std::cout<<"Compute program is being made!"<<std::endl;

      std::string entryPoint = std::string("__compute__") + std::string(_entryPoint);
      auto binary = module->getBinary("COMPUTE", entryPoint.c_str());

      // store a reference to the logical device this module is made on
      logicalDevice = _logicalDevice;

      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);//sizeOfProgramBytes;
      moduleCreateInfo.pCode = binary.data(); //(uint32_t*)binary->wordCount;//programBytes;

      VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
        NULL, &shaderModule));

      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shaderStage.module = shaderModule;
      shaderStage.pName = entryPoint.c_str();
      assert(shaderStage.module != VK_NULL_HANDLE);

      // module->releaseBinary(binary);

      vars = _vars;
    }
    ~ComputeProg() {}
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
      auto binary = module->getBinary("RAYGEN", entryPoint);

      // store a reference to the logical device this module is made on
      logicalDevice = _logicalDevice;

      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);//sizeOfProgramBytes;
      moduleCreateInfo.pCode = binary.data(); //(uint32_t*)binary->wordCount;//programBytes;

      VkResult err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
        NULL, &shaderModule);
      if (err) throw std::runtime_error("failed to create shader module! : \n" + errorString(err));

      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
      shaderStage.module = shaderModule;
      shaderStage.pName = entryPoint.c_str();
      assert(shaderStage.module != VK_NULL_HANDLE);

      // module->releaseBinary(binary);

      vars = _vars;
    }
    ~RayGen() {}
    void destroy() {
      std::cout<<"Ray gen is being destroyed!"<<std::endl;
      vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
    }
  };

  struct MissProg : public SBTEntry {
    VkShaderModule shaderModule;
    VkPipelineShaderStageCreateInfo shaderStage{};
    VkShaderModuleCreateInfo moduleCreateInfo{};
    VkDevice logicalDevice;
    std::string entryPoint;

    MissProg(VkDevice  _logicalDevice,
             Module *module,
             const char* _entryPoint,
             size_t      sizeOfVarStruct,
             std::unordered_map<std::string, GPRTVarDef> _vars) : SBTEntry()
    {
      std::cout<<"Miss program is being made!"<<std::endl;

      entryPoint = std::string("__miss__") + std::string(_entryPoint);
      auto binary = module->getBinary("MISS", entryPoint.c_str());

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

      // module->releaseBinary(binary);

      vars = _vars;
    }
    ~MissProg() {}
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
    std::string closestHitEntryPoint;
    
    GeomType(VkDevice  _logicalDevice,
             uint32_t numRayTypes,
             size_t      sizeOfVarStruct,
             std::unordered_map<std::string, GPRTVarDef> _vars) : SBTEntry()
    {
      std::cout<<"Geom type is being made!"<<std::endl;
      closestHitShaderStages.resize(numRayTypes, {});

      // store a reference to the logical device this module is made on
      logicalDevice = _logicalDevice;
      vars = _vars;
    }
    ~GeomType() 
    {
      std::cout<<"Geom type is being destroyed!"<<std::endl;
      // vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
    }
    
    void setClosestHit(int rayType,
                       Module *module,
                       const char* _entryPoint) 
    {
      closestHitEntryPoint = std::string("__closesthit__") + std::string(_entryPoint);
      auto binary = module->getBinary("CLOSESTHIT", closestHitEntryPoint);
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
      closestHitShaderStages[rayType].pName = closestHitEntryPoint.c_str();
      assert(closestHitShaderStages[rayType].module != VK_NULL_HANDLE);

      // module->releaseBinary(binary);
    }

    void setAnyHit(int rayType,
                       Module *module,
                       const char* entryPoint) 
    {
      std::cout<<"TODO! Create a shader module for this any hit program"<<std::endl;
    }

    void setIntersectProg(int rayType,
                       Module *module,
                       const char* entryPoint) 
    {
      std::cout<<"TODO! Create a shader module for this intersect program"<<std::endl;
    }

    void setBoundsProg(Module *module,
                       const char* entryPoint) 
    {
      std::cout<<"TODO! Create a shader module for this bounds program"<<std::endl;
    }

    void destroy() {
      std::cout<<"geom type is being destroyed!"<<std::endl;
      for (uint32_t i = 0; i < closestHitShaderStages.size(); ++i) {
        if (closestHitShaderStages[i].module)
          vkDestroyShaderModule(logicalDevice, 
            closestHitShaderStages[i].module, nullptr);
      }
    }
    
    virtual Geom* createGeom() { return nullptr; };
  };

  /*! An actual geometry object with primitives - this class is still
    abstract, and will get fleshed out in its derived classes
    (UserGeom, TrianglesGeom, ...) */
  struct Geom : public SBTEntry {
    Geom() : SBTEntry() {};
    ~Geom() {};

    void destroy(){}

    /*! This acts as a template that describes this geometry's variables and
        programs. */
    GeomType* geomType;
  };

  struct TrianglesGeom : public Geom {
    struct {
      size_t count  = 0; // number of indices
      size_t stride = 0; // stride between indices
      size_t offset = 0; // offset in bytes to the first index
      size_t firstVertex = 0; // added to the index values before fetching vertices
      gprt::Buffer* buffer = nullptr;
    } index;

    struct {
      size_t count  = 0; // number of vertices
      size_t stride = 0; // stride between vertices
      size_t offset = 0; // an offset in bytes to the first vertex
      std::vector<gprt::Buffer*> buffers;
    } vertex;

    TrianglesGeom(TrianglesGeomType* _geomType) : Geom() {
      geomType = (GeomType*)_geomType;

      // Allocate the variables for this geometry, using our geomType vars as 
      // the template.
      std::vector<GPRTVarDecl> varDecls = getDecls(geomType->vars);
      vars = checkAndPackVariables(varDecls.data(), varDecls.size());
    };
    ~TrianglesGeom() {};

    void setVertices(
      gprt::Buffer* vertices,
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
      gprt::Buffer* indices,
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

  struct TrianglesGeomType : public GeomType {
    TrianglesGeomType(
      VkDevice logicalDevice,
      uint32_t numRayTypes,
      size_t   sizeOfVarStruct,
      std::unordered_map<std::string, GPRTVarDef> vars) : 
      GeomType(logicalDevice, numRayTypes, sizeOfVarStruct, vars)
    {}
    ~TrianglesGeomType() {}

    Geom* createGeom()  
    {
      return new TrianglesGeom(this);
    }
  };

  struct UserGeom : public Geom {
    UserGeom(UserGeomType* _geomType) : Geom() {
      geomType = (GeomType*)_geomType;
    };
    ~UserGeom() {};
  };

  struct UserGeomType : public GeomType {
    UserGeomType(VkDevice  _logicalDevice,
             uint32_t numRayTypes,
             size_t      sizeOfVarStruct,
             std::unordered_map<std::string, GPRTVarDef> _vars) : 
             GeomType(_logicalDevice, numRayTypes, sizeOfVarStruct, _vars)
    {}
    ~UserGeomType() {}
    Geom* createGeom() 
    {
      return new UserGeom(this);
    }
  };

  typedef enum
  {
    GPRT_UNKNOWN_ACCEL = 0x0,
    GPRT_TRIANGLES_ACCEL = 0x1,
    GPRT_INSTANCE_ACCEL  = 0x2,
  } AccelType;

  struct Accel {
    VkPhysicalDevice physicalDevice;
    VkDevice logicalDevice;
    VkCommandBuffer commandBuffer;
    VkQueue queue;
    VkDeviceAddress address = 0;
    VkAccelerationStructureKHR accelerationStructure = VK_NULL_HANDLE;

    gprt::Buffer *accelBuffer = nullptr;
    gprt::Buffer *scratchBuffer = nullptr;
    
    Accel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer commandBuffer, VkQueue queue) {
      this->physicalDevice = physicalDevice;
      this->logicalDevice = logicalDevice;
      this->commandBuffer = commandBuffer;
      this->queue = queue;
    };
    
    ~Accel() {};

    virtual void build(std::map<std::string, Stage> internalStages) { };
    virtual void destroy() { };
    virtual AccelType getType() {return GPRT_UNKNOWN_ACCEL;}
  };

  struct TrianglesAccel : public Accel {
    std::vector<TrianglesGeom*> geometries; 
    
    struct {
      gprt::Buffer* buffer = nullptr;
      // size_t stride = 0;
      // size_t offset = 0;
    } transforms;

    // todo, accept this in constructor
    VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

    TrianglesAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer commandBuffer, VkQueue queue,
      size_t numGeometries, TrianglesGeom* geometries) : Accel(physicalDevice, logicalDevice, commandBuffer, queue) 
    {
      this->geometries.resize(numGeometries);
      memcpy(this->geometries.data(), geometries, sizeof(GPRTGeom*) * numGeometries);
    };
    
    ~TrianglesAccel() {};

    void setTransforms(
      gprt::Buffer* transforms//,
      // size_t count,
      // size_t stride,
      // size_t offset
      ) 
    {
      // assuming no motion blurred triangles for now, so we assume 1 transform per instance
      this->transforms.buffer = transforms;
    }

    AccelType getType() {return GPRT_TRIANGLES_ACCEL;}

    void build(std::map<std::string, Stage> internalStages) {
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
        geom.geometry.triangles.transformData.deviceAddress = transforms.buffer->address;
        // if the above is null, then that indicates identity

        auto &geomRange = accelerationBuildStructureRangeInfos[gid];
        accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
        geomRange.primitiveCount = geometries[gid]->index.count;
        geomRange.primitiveOffset = geometries[gid]->index.offset;
        geomRange.firstVertex = geometries[gid]->index.firstVertex;
        geomRange.transformOffset = gid * 12 * sizeof(float); // might change this later...
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
      vkGetAccelerationStructureBuildSizesKHR(
        logicalDevice,
        VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
        &accelerationStructureBuildGeometryInfo,
        maxPrimitiveCounts.data(),
        &accelerationStructureBuildSizesInfo
      );
      
      accelBuffer = new gprt::Buffer(
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

      scratchBuffer = new gprt::Buffer(
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

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
      accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      err = vkCreateAccelerationStructureKHR(
        logicalDevice,
        &accelerationStructureCreateInfo, 
        nullptr,
        &accelerationStructure
      );
      if (err) GPRT_RAISE("failed to create acceleration structure for triangle accel build! : \n" + errorString(err));

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

      vkCmdBuildAccelerationStructuresKHR(
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
      address = gprt::vkGetAccelerationStructureDeviceAddressKHR(logicalDevice, &accelerationDeviceAddressInfo);
    }
  
    void destroy() { 
      if (accelerationStructure) {
        vkDestroyAccelerationStructureKHR(logicalDevice, accelerationStructure, nullptr);
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
    std::vector<Accel*> instances; 

    gprt::Buffer *instancesBuffer = nullptr;
    gprt::Buffer *accelAddressesBuffer = nullptr;

    struct {
      gprt::Buffer* buffer = nullptr;
      // size_t stride = 0;
      // size_t offset = 0;
    } transforms;
    
    // todo, accept this in constructor
    VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

    InstanceAccel(VkPhysicalDevice physicalDevice, VkDevice logicalDevice, VkCommandBuffer commandBuffer, VkQueue queue,
      size_t numInstances, GPRTAccel* instances) : Accel(physicalDevice, logicalDevice, commandBuffer, queue) 
    {
      this->instances.resize(numInstances);
      memcpy(this->instances.data(), instances, sizeof(GPRTAccel*) * numInstances);

      accelAddressesBuffer = new gprt::Buffer(
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

      std::vector<uint64_t> addresses(numInstances);
      for (uint32_t i = 0; i < numInstances; ++i) 
        addresses[i] = this->instances[i]->address;
      accelAddressesBuffer->map();
      memcpy(accelAddressesBuffer->mapped, addresses.data(), sizeof(uint64_t) * numInstances);
    };
    
    ~InstanceAccel() {};

    void setTransforms(
      gprt::Buffer* transforms//,
      // size_t count,
      // size_t stride,
      // size_t offset
      ) 
    {
      // assuming no motion blurred triangles for now, so we assume 1 transform per instance
      this->transforms.buffer = transforms;
    }

    AccelType getType() {return GPRT_INSTANCE_ACCEL;}

    void build(std::map<std::string, Stage> internalStages) {
      VkResult err;

      // todo, transfer instance transforms into instances buffer

      instancesBuffer = new gprt::Buffer(
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
        sizeof(VkAccelerationStructureInstanceKHR) * instances.size()
      );


      // Use a compute shader to copy transforms into instances buffer
      
      VkCommandBufferBeginInfo cmdBufInfo{};
      struct PushContants {
        uint64_t instanceBufferAddr;
        uint64_t transformBufferAddr;
        uint64_t accelReferencesAddr;
      } pushConstants;

      pushConstants.instanceBufferAddr = instancesBuffer->address;
      pushConstants.transformBufferAddr = transforms.buffer->address;
      pushConstants.accelReferencesAddr = accelAddressesBuffer->address;

      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      vkCmdPushConstants(commandBuffer, internalStages["gprtFillInstanceData"].layout, 
        VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushContants), &pushConstants
      );
      vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, internalStages["gprtFillInstanceData"].pipeline);
      // vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, 0);
      vkCmdDispatch(commandBuffer, instances.size(), 1, 1);
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

      VkAccelerationStructureInstanceKHR instance{};
      // struct TMP {
      //   uint32_t a;
      //   uint32_t b;
      //   uint32_t c;
      // };
      // TMP tmp;
      instancesBuffer->map();
      memcpy(&instance, instancesBuffer->mapped, sizeof(VkAccelerationStructureInstanceKHR));

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

      uint32_t primitive_count = 1;

      VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
      accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
      vkGetAccelerationStructureBuildSizesKHR(
        logicalDevice, 
        VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
        &accelerationStructureBuildGeometryInfo,
        &primitive_count,
        &accelerationStructureBuildSizesInfo);
      
      accelBuffer = new gprt::Buffer(
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
      err = vkCreateAccelerationStructureKHR(
        logicalDevice,
        &accelerationStructureCreateInfo, 
        nullptr,
        &accelerationStructure
      );
      if (err) GPRT_RAISE("failed to create acceleration structure for instance accel build! : \n" + errorString(err));

      scratchBuffer = new gprt::Buffer(
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
      accelerationStructureBuildRangeInfo.primitiveCount = 1;
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

      vkCmdBuildAccelerationStructuresKHR(
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
      address = gprt::vkGetAccelerationStructureDeviceAddressKHR(logicalDevice, &accelerationDeviceAddressInfo);
    }

    void destroy() { 
      if (accelerationStructure) {
        vkDestroyAccelerationStructureKHR(logicalDevice, accelerationStructure, nullptr);
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
    };
  };

  struct Context {
    VkApplicationInfo appInfo;

    // Vulkan instance, stores all per-application states
    VkInstance instance;
    std::vector<std::string> supportedInstanceExtensions;
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
    // Wraps the swap chain to present images (framebuffers) to the windowing system
    // VulkanSwapChain swapChain; // I kinda want to avoid using Vulkan's swapchain system...
    // Synchronization semaphores
    struct {
      // Swap chain image presentation
      VkSemaphore presentComplete;
      // Command buffer submission and execution
      VkSemaphore renderComplete;
    } semaphores;
    std::vector<VkFence> waitFences;

    // ray tracing pipeline and layout
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;

    std::vector<RayGen*> raygenPrograms;
    std::vector<MissProg*> missPrograms;
    std::vector<GeomType*> geomTypes;

    std::vector<Accel*> accels;

    std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups{};
    gprt::Buffer shaderBindingTable;

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
			if (debugUtilsMessenger != VK_NULL_HANDLE)
			{
				vkDestroyDebugUtilsMessengerEXT(instance, debugUtilsMessenger, nullptr);
			}
		}

    size_t getNumHitRecords() {
      int totalGeometries = 0;
      for (int accelID = 0; accelID < accels.size(); ++accelID) {
        Accel *accel = accels[accelID];
        if (!accel) continue;
        if (accel->getType() == GPRT_INSTANCE_ACCEL) continue;
        if (accel->getType() == GPRT_TRIANGLES_ACCEL) {
          TrianglesAccel *triAccel = (TrianglesAccel*) accel;
          totalGeometries += triAccel->geometries.size();
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

      // uint32_t glfwExtensionCount = 0;
      // const char** glfwExtensions;
      // glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
      // instanceCreateInfo.enabledExtensionCount = glfwExtensionCount;
      // instanceCreateInfo.ppEnabledExtensionNames = glfwExtensions;

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

      // The VK_LAYER_KHRONOS_validation contains all current validation functionality.
      // Note that on Android this layer requires at least NDK r20
      const char* validationLayerName = "VK_LAYER_KHRONOS_validation";
      if (validation())
      {
        // Check if this layer is available at instance level
        uint32_t instanceLayerCount;
        vkEnumerateInstanceLayerProperties(&instanceLayerCount, nullptr);
        std::vector<VkLayerProperties> instanceLayerProperties(instanceLayerCount);
        vkEnumerateInstanceLayerProperties(&instanceLayerCount, instanceLayerProperties.data());
        bool validationLayerPresent = false;
        for (VkLayerProperties layer : instanceLayerProperties) {
          if (strcmp(layer.layerName, validationLayerName) == 0) {
            validationLayerPresent = true;
            break;
          }
        }
        if (validationLayerPresent) {
          instanceCreateInfo.ppEnabledLayerNames = &validationLayerName;
          instanceCreateInfo.enabledLayerCount = 1;
        } else {
          std::cerr << "Validation layer VK_LAYER_KHRONOS_validation not present, validation is disabled";
        }
      }

      VkResult err;

      err = vkCreateInstance(&instanceCreateInfo, nullptr, &instance);
      if (err) {
        throw std::runtime_error("failed to create instance! : \n" + errorString(err));
      }

      // err = createDebugUtilsMessenger(instance,
      //     info.debug_callback,
      //     info.debug_message_severity,
      //     info.debug_message_type,
      //     &instance.debug_messenger,
      //     info.allocation_callbacks);
      // if (err) {
      //   throw std::runtime_error("failed to debug messenger callback! : \n" + errorString(err));
      // }

      /// 2. Select a Physical Device


      // If requested, we enable the default validation layers for debugging
      if (validation())
      {


        vkCreateDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT"));
        vkDestroyDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));

        VkDebugUtilsMessengerCreateInfoEXT debugUtilsMessengerCI{};
        debugUtilsMessengerCI.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        debugUtilsMessengerCI.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                                VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT |
                                                VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT ;
        debugUtilsMessengerCI.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT;
        debugUtilsMessengerCI.pfnUserCallback = debugUtilsMessengerCallback;
        VkResult result = vkCreateDebugUtilsMessengerEXT(instance, &debugUtilsMessengerCI, nullptr, &debugUtilsMessenger);
        assert(result == VK_SUCCESS);



        // vkCreateDebugReportCallbackEXT = reinterpret_cast<PFN_vkCreateDebugReportCallbackEXT>(vkGetInstanceProcAddr(instance, "vkCreateDebugReportCallbackEXT"));
        // vkDestroyDebugReportCallbackEXT = reinterpret_cast<PFN_vkDestroyDebugReportCallbackEXT>(vkGetInstanceProcAddr(instance, "vkDestroyDebugReportCallbackEXT"));

        // The report flags determine what type of messages for the layers will be displayed
        // For validating (debugging) an application the error and warning bits should suffice
        // VkDebugReportFlagsEXT debugReportFlags = VK_DEBUG_REPORT_ERROR_BIT_EXT | VK_DEBUG_REPORT_WARNING_BIT_EXT | VK_DEBUG_REPORT_INFORMATION_BIT_EXT;
        // Additional flags include performance info, loader and layer debug messages, etc.
        // vks::debug::setupDebugging(instance, debugReportFlags, VK_NULL_HANDLE);

        // VkDebugReportCallbackCreateInfoEXT debugReportCI{};
        // debugReportCI.sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT;
        // debugReportCI.flags = debugReportFlags;
        // debugReportCI.pfnCallback = debugReportCallback;
        // debugReportCI.pUserData = nullptr;

        // // debugReportCI.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT;
        // result = vkCreateDebugReportCallbackEXT(instance, &debugReportCI, nullptr, &debugReportCallback);
        // assert(result == VK_SUCCESS);
      }

      // Physical device
      uint32_t gpuCount = 0;
      // Get number of available physical devices
      VK_CHECK_RESULT(vkEnumeratePhysicalDevices(instance, &gpuCount, nullptr));
      if (gpuCount == 0) {
        throw std::runtime_error("No device with Vulkan support found : \n" + errorString(err));
      }
      // Enumerate devices
      std::vector<VkPhysicalDevice> physicalDevices(gpuCount);
      err = vkEnumeratePhysicalDevices(instance, &gpuCount, physicalDevices.data());
      if (err) {
        throw std::runtime_error("Could not enumerate physical devices : \n" + errorString(err));
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

      // Select physical device to be used
      // Defaults to the first device unless specified by command line
      uint32_t selectedDevice = 0; // TODO
      
      std::cout << "Available Vulkan devices" << "\n";
      for (uint32_t i = 0; i < gpuCount; i++) {
        VkPhysicalDeviceProperties deviceProperties;
        vkGetPhysicalDeviceProperties(physicalDevices[i], &deviceProperties);
        std::cout << "Device [" << i << "] : " << deviceProperties.deviceName << std::endl;
        std::cout << " Type: " << physicalDeviceTypeString(deviceProperties.deviceType) << "\n";
        std::cout << " API: " << (deviceProperties.apiVersion >> 22) << "."
          << ((deviceProperties.apiVersion >> 12) & 0x3ff) << "."
          << (deviceProperties.apiVersion & 0xfff) << "\n";
        
        if (deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)
          selectedDevice = i;

        // Get ray tracing pipeline properties
        VkPhysicalDeviceRayTracingPipelinePropertiesKHR  rayTracingPipelineProperties{};
        rayTracingPipelineProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
        VkPhysicalDeviceProperties2 deviceProperties2{};
        deviceProperties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
        deviceProperties2.pNext = &rayTracingPipelineProperties;
        vkGetPhysicalDeviceProperties2(physicalDevices[i], &deviceProperties2);

        // Get acceleration structure properties
        VkPhysicalDeviceAccelerationStructureFeaturesKHR accelerationStructureFeatures{};
        accelerationStructureFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
        VkPhysicalDeviceFeatures2 deviceFeatures2{};
        deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
        deviceFeatures2.pNext = &accelerationStructureFeatures;
        vkGetPhysicalDeviceFeatures2(physicalDevices[i], &deviceFeatures2);
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
          throw std::runtime_error("Could not find a matching queue family index");
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

      // if (useSwapChain)
      // {
      // 	// If the device will be used for presenting to a display via a swapchain we need to request the swapchain extension
      // 	enabledDeviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
      // }

      #if defined(VK_USE_PLATFORM_MACOS_MVK) && (VK_HEADER_VERSION >= 216)
        enabledDeviceExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
      #endif

      VkPhysicalDeviceBufferDeviceAddressFeatures bufferDeviceAddressFeatures = {};
      bufferDeviceAddressFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
      bufferDeviceAddressFeatures.bufferDeviceAddress = VK_TRUE;
      bufferDeviceAddressFeatures.bufferDeviceAddressCaptureReplay = VK_FALSE;
      bufferDeviceAddressFeatures.bufferDeviceAddressMultiDevice = VK_FALSE;

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

      auto extensionSupported = [](std::string extension, std::vector<std::string> supportedExtensions) -> bool {
        return (std::find(supportedExtensions.begin(), supportedExtensions.end(), extension) != supportedExtensions.end());
      };

      if (enabledDeviceExtensions.size() > 0)
      {
        for (const char* enabledExtension : enabledDeviceExtensions)
        {
          if (!extensionSupported(enabledExtension, supportedExtensions)) {
            std::cerr << "Enabled device extension \"" << enabledExtension << "\" is not present at device level\n";
          }
        }

        deviceCreateInfo.enabledExtensionCount = (uint32_t)enabledDeviceExtensions.size();
        deviceCreateInfo.ppEnabledExtensionNames = enabledDeviceExtensions.data();
      }

      // this->enabledFeatures = enabledFeatures;
      err = vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &logicalDevice);
      if (err) {
        throw std::runtime_error("Could not create logical devices : \n" + errorString(err));
      }

      // Get the ray tracing and acceleration structure related function pointers required by this sample
      vkGetBufferDeviceAddressKHR = reinterpret_cast<PFN_vkGetBufferDeviceAddressKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetBufferDeviceAddressKHR"));
      vkCmdBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCmdBuildAccelerationStructuresKHR"));
      vkBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(logicalDevice, "vkBuildAccelerationStructuresKHR"));
      vkCreateAccelerationStructureKHR = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCreateAccelerationStructureKHR"));
      vkDestroyAccelerationStructureKHR = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(vkGetDeviceProcAddr(logicalDevice, "vkDestroyAccelerationStructureKHR"));
      vkGetAccelerationStructureBuildSizesKHR = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetAccelerationStructureBuildSizesKHR"));
      vkGetAccelerationStructureDeviceAddressKHR = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetAccelerationStructureDeviceAddressKHR"));
      vkCmdTraceRaysKHR = reinterpret_cast<PFN_vkCmdTraceRaysKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCmdTraceRaysKHR"));
      vkGetRayTracingShaderGroupHandlesKHR = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(vkGetDeviceProcAddr(logicalDevice, "vkGetRayTracingShaderGroupHandlesKHR"));
      vkCreateRayTracingPipelinesKHR = reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCreateRayTracingPipelinesKHR"));

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

      /// 5. Allocate command buffers
      VkCommandBufferAllocateInfo cmdBufAllocateInfo{};
      cmdBufAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
      cmdBufAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
      cmdBufAllocateInfo.commandBufferCount = 1;

      cmdBufAllocateInfo.commandPool = graphicsCommandPool;
      err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &graphicsCommandBuffer);
      if (err) throw std::runtime_error("Could not create graphics command buffer : \n" + errorString(err));

      cmdBufAllocateInfo.commandPool = computeCommandPool;
      err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &computeCommandBuffer);
      if (err) throw std::runtime_error("Could not create compute command buffer : \n" + errorString(err));

      cmdBufAllocateInfo.commandPool = transferCommandPool;
      err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &transferCommandBuffer);
      if (err) throw std::runtime_error("Could not create transfer command buffer : \n" + errorString(err));

      /// 6. Get queue handles
      vkGetDeviceQueue(logicalDevice, queueFamilyIndices.graphics, 0, &graphicsQueue);
      vkGetDeviceQueue(logicalDevice, queueFamilyIndices.compute, 0, &computeQueue);
      vkGetDeviceQueue(logicalDevice, queueFamilyIndices.transfer, 0, &transferQueue);

      // 7. Create a module for internal device entry points
      internalModule = new Module(gprtDeviceCode);
      setupInternalStages(internalModule);
    };

    void destroy() {
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
      const uint32_t recordSize = alignedSize(maxGroupSize, groupAlignment);

      const uint32_t groupCount = static_cast<uint32_t>(shaderGroups.size());
      const uint32_t sbtSize = groupCount * handleSize;

      std::vector<uint8_t> shaderHandleStorage(sbtSize);
      VkResult err = vkGetRayTracingShaderGroupHandlesKHR(logicalDevice, pipeline, 0, groupCount, sbtSize, shaderHandleStorage.data());
      if (err) throw std::runtime_error("failed to get ray tracing shader group handles! : \n" + errorString(err));

      const VkBufferUsageFlags bufferUsageFlags = 
        // means we can use this buffer as a SBT
        VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | 
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
      const VkMemoryPropertyFlags memoryUsageFlags = 
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | // mappable to host with vkMapMemory
        VK_MEMORY_PROPERTY_HOST_COHERENT_BIT; // means "flush" and "invalidate"  not needed


      // std::cout<<"Todo, get some smarter memory allocation working..." <<std::endl;

      size_t numRayGens = raygenPrograms.size();
      size_t numMissProgs = missPrograms.size();
      size_t numHitRecords = getNumHitRecords();
      size_t numRecords = numRayGens + numMissProgs + numHitRecords;

      if (shaderBindingTable.size != recordSize * numRecords) {
        shaderBindingTable.destroy();
      }
      if (shaderBindingTable.buffer == VK_NULL_HANDLE) {
        shaderBindingTable = Buffer(physicalDevice, logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 
          bufferUsageFlags, memoryUsageFlags, recordSize * numRecords);
      }
      shaderBindingTable.map();
      uint8_t* mapped = ((uint8_t*) (shaderBindingTable.mapped));

      // Raygen records
      if (raygenPrograms.size() > 0) {
        for (uint32_t idx = 0; idx < raygenPrograms.size(); ++idx) {
          size_t recordStride = recordSize;
          size_t handleStride = handleSize;
          
          // First, copy handle
          size_t recordOffset = recordStride * idx;
          size_t handleOffset = handleStride * idx;
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
          size_t recordOffset = recordStride * idx + recordStride * numRayGens;
          size_t handleOffset = handleStride * idx + handleStride * numRayGens;
          memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

          // Then, copy params following handle
          recordOffset = recordOffset + handleSize;
          uint8_t* params = mapped + recordOffset;
          MissProg *missprog = missPrograms[idx];
          for (auto &var : missprog->vars) {
            size_t varOffset = var.second.decl.offset;
            size_t varSize = getSize(var.second.decl.type);
            memcpy(params + varOffset, var.second.data, varSize);
          }
        }
      }

      // Hit records
      if (numHitRecords > 0)
      {
        for (int accelID = 0; accelID < accels.size(); ++accelID) {
          Accel *accel = accels[accelID];
          if (!accel) continue;
          if (accel->getType() == GPRT_INSTANCE_ACCEL) continue;
          if (accel->getType() == GPRT_TRIANGLES_ACCEL) {
            TrianglesAccel *triAccel = (TrianglesAccel*) accel;

            
            for (int geomID = 0; geomID < triAccel->geometries.size(); ++geomID) {
              auto &geom = triAccel->geometries[geomID];

              for (int rayType = 0; rayType < numRayTypes; ++rayType) {
                size_t recordStride = recordSize;
                size_t handleStride = handleSize;

                // First, copy handle
                size_t instanceOffset = 0; // TODO
                size_t recordOffset = recordStride * (rayType + numRayTypes * geomID + instanceOffset) + recordStride * (numRayGens + numMissProgs);
                size_t handleOffset = handleStride * (rayType + numRayTypes * geomID + instanceOffset) + handleStride * (numRayGens + numMissProgs);
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
          }
        }
      }


      // {
      //     TrianglesAccel *triAccel = (TrianglesAccel*) accel;

      //     const size_t sbtOffset = 0 ; // triAccel->sbtOffset; //?????
      //     std::cout<<"TODO, compute correct sbt offset for a triangle accel!"<<std::endl;
      //     for (int geomID = 0; geomID < triAccel->geometries.size(); ++geomID) {
      //       for (int rayTypeID = 0; rayTypeID < numRayTypes; ++rayTypeID) {

      //         size_t offset = stride * idx + handleSize /* params start after shader identifier */ ;
      //         uint8_t* params = ((uint8_t*) (raygenShaderBindingTable.mapped)) + offset;

      //         int HG_stride = ;
      //         int R_stride = numRayTypes;
      //         int R_offset = rayTypeID;
      //         int G_ID = geomID;
      //         int I_offset = ;
      //         void* HG = hitShaderBindingTable.mapped + (HG_stride * (R_offset + R_stride * G_ID + I_offset));

      //       }
      //     }
      //   }


      // if (hitgroupPrograms.size() > 0) 
      {
        
        
        
        

        // TODO: hit programs...
        
      }
    }

    void buildPrograms()
    {
      // At the moment, we don't actually build our programs here. 
    }

    void buildPipeline()
    {
      VkPipelineLayoutCreateInfo pipelineLayoutCI{};
      pipelineLayoutCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
      // pipelineLayoutCI.setLayoutCount = 1;
      // pipelineLayoutCI.pSetLayouts = &descriptorSetLayout;
      pipelineLayoutCI.setLayoutCount = 0;
      pipelineLayoutCI.pSetLayouts = nullptr;
      VK_CHECK_RESULT(vkCreatePipelineLayout(logicalDevice, &pipelineLayoutCI,
        nullptr, &pipelineLayout));

      /*
        Setup ray tracing shader groups
      */
      std::vector<VkPipelineShaderStageCreateInfo> shaderStages;

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
        for (auto missprog : missPrograms) {
          shaderStages.push_back(missprog->shaderStage);
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

      // Closest hit group
      {
        for (auto geomType : geomTypes) {
          for (uint32_t rayType = 0; rayType < numRayTypes; ++rayType) {
            VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
            shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
            shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR; // or VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR
            
            // init all to unused
            shaderGroup.generalShader = VK_SHADER_UNUSED_KHR;
            shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
            shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
            shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;

            if (geomType->closestHitShaderStages.size() > 0) {
              shaderStages.push_back(geomType->closestHitShaderStages[rayType]);
              shaderGroup.closestHitShader = static_cast<uint32_t>(shaderStages.size()) - 1;
            }

            if (geomType->anyHitShaderStages.size() > 0) {
              shaderStages.push_back(geomType->anyHitShaderStages[rayType]);
              shaderGroup.anyHitShader = static_cast<uint32_t>(shaderStages.size()) - 1;
            }

            if (geomType->intersectionShaderStages.size() > 0) {
              shaderStages.push_back(geomType->intersectionShaderStages[rayType]);
              shaderGroup.intersectionShader = static_cast<uint32_t>(shaderStages.size()) - 1;
            }
            shaderGroups.push_back(shaderGroup);
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

      VkResult err = vkCreateRayTracingPipelinesKHR(logicalDevice,
        VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &rayTracingPipelineCI,
        nullptr, &pipeline
      );
      if (err) {
        throw std::runtime_error("failed to create ray tracing pipeline! : \n" + errorString(err));
      }
    }
  
    void setupInternalStages(gprt::Module *module) {
      fillInstanceDataStage.entryPoint = "gprtFillInstanceData";

      // todo, consider refactoring this into a more official "Compute" shader object

      VkResult err;
      // currently not using cache.
      VkPipelineCache cache = VK_NULL_HANDLE;

      VkPushConstantRange pushConstantRange = {};
      pushConstantRange.size = 3 * sizeof(uint64_t);
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
      auto binary = module->getBinary("COMPUTE", entryPoint.c_str());

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

      // module->releaseBinary(binary);

      //todo, destroy the above stuff
    }
  };
}


GPRT_API GPRTContext gprtContextCreate(int32_t *requestedDeviceIDs,
                                    int      numRequestedDevices)
{
  LOG_API_CALL();
  gprt::Context *context = new gprt::Context(requestedDeviceIDs, numRequestedDevices);
  LOG("context created...");
  return (GPRTContext)context;
}

GPRT_API void gprtContextDestroy(GPRTContext _context)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  context->destroy();
  delete context;
  LOG("context destroyed...");
}

GPRT_API GPRTModule gprtModuleCreate(GPRTContext _context, std::map<std::string, std::vector<uint8_t>> spvCode)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  gprt::Module *module = new gprt::Module(spvCode);
  LOG("module created...");
  return (GPRTModule)module;
}

GPRT_API void gprtModuleDestroy(GPRTModule _module)
{
  LOG_API_CALL();
  gprt::Module *module = (gprt::Module*)_module;
  delete module;
  LOG("module destroyed...");
}

GPRT_API GPRTGeom
gprtGeomCreate(GPRTContext  _context,
              GPRTGeomType _geomType)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  gprt::GeomType *geomType = (gprt::GeomType*)_geomType;

  // depending on what the geomType is, we'll use this inherited "createGeom"
  // function to construct the appropriate geometry
  gprt::Geom *geometry = geomType->createGeom();
  return (GPRTGeom)geometry;
  LOG("geometry created...");
}

GPRT_API void 
gprtGeomDestroy(GPRTGeom _geometry)
{
  LOG_API_CALL();
  gprt::Geom *geometry = (gprt::Geom*)_geometry;
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
  gprt::TrianglesGeom *triangles = (gprt::TrianglesGeom*)_triangles;
  gprt::Buffer *vertices = (gprt::Buffer*)_vertices;
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
  gprt::TrianglesGeom *triangles = (gprt::TrianglesGeom*)_triangles;
  gprt::Buffer *indices = (gprt::Buffer*)_indices;
  triangles->setIndices(indices, count, stride, offset);
  LOG("Setting triangle indices...");
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
  gprt::Context *context = (gprt::Context*)_context;
  gprt::Module *module = (gprt::Module*)_module;

  gprt::RayGen *raygen = new gprt::RayGen(
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
  gprt::RayGen *rayGen = (gprt::RayGen*)_rayGen;
  rayGen->destroy();
  delete rayGen;
  LOG("raygen destroyed...");
}

GPRT_API GPRTMissProg
gprtMissProgCreate(GPRTContext _context,
                   GPRTModule  _module,
                   const char  *programName,
                   size_t       sizeOfVarStruct,
                   GPRTVarDecl *vars,
                   int          numVars)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  gprt::Module *module = (gprt::Module*)_module;

  gprt::MissProg *missProg = new gprt::MissProg(
    context->logicalDevice, module, programName,
    sizeOfVarStruct, checkAndPackVariables(vars, numVars));

  context->missPrograms.push_back(missProg);

  LOG("miss program created...");
  return (GPRTMissProg)missProg;
}


/*! sets the given miss program for the given ray type */
GPRT_API void
gprtMissProgSet(GPRTContext  _context,
               int rayType,
               GPRTMissProg _missProgToUse)
{
  GPRT_NOTIMPLEMENTED;
}

GPRT_API void
gprtMissProgDestroy(GPRTMissProg _missProg)
{
  LOG_API_CALL();
  gprt::MissProg *missProg = (gprt::MissProg*)_missProg;
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
  gprt::Context *context = (gprt::Context*)_context;

  gprt::GeomType *geomType = nullptr;

  switch(kind) {
    case GPRT_TRIANGLES:
      geomType = new gprt::TrianglesGeomType(
        context->logicalDevice, context->numRayTypes,
        sizeOfVarStruct, checkAndPackVariables(vars, numVars));
        break;
    case GPRT_USER:
      geomType = new gprt::UserGeomType(
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
  gprt::GeomType *geomType = (gprt::GeomType*)_geomType;
  geomType->destroy();
  delete geomType;
  LOG("geom type destroyed...");
}

GPRT_API void
gprtGeomTypeSetClosestHit(GPRTGeomType _geomType,
                          int rayType,
                          GPRTModule _module,
                          const char *progName)
{
  LOG_API_CALL();
  gprt::GeomType *geomType = (gprt::GeomType*)_geomType;
  gprt::Module *module = (gprt::Module*)_module;

  geomType->setClosestHit(rayType, module, progName);
  LOG("assigning closest hit program to geom type...");
}

GPRT_API void
gprtGeomTypeSetAnyHit(GPRTGeomType _geomType,
                          int rayType,
                          GPRTModule _module,
                          const char *progName)
{
  LOG_API_CALL();
  gprt::GeomType *geomType = (gprt::GeomType*)_geomType;
  gprt::Module *module = (gprt::Module*)_module;

  geomType->setAnyHit(rayType, module, progName);
  LOG("assigning any hit program to geom type...");
}

GPRT_API void
gprtGeomTypeSetIntersectProg(GPRTGeomType _geomType,
                          int rayType,
                          GPRTModule _module,
                          const char *progName)
{
  LOG_API_CALL();
  gprt::GeomType *geomType = (gprt::GeomType*)_geomType;
  gprt::Module *module = (gprt::Module*)_module;

  geomType->setIntersectProg(rayType, module, progName);
  LOG("assigning intersect program to geom type...");
}

GPRT_API void
gprtGeomTypeSetBoundsProg(GPRTGeomType _geomType,
                          GPRTModule _module,
                          const char *progName)
{
  LOG_API_CALL();
  gprt::GeomType *geomType = (gprt::GeomType*)_geomType;
  gprt::Module *module = (gprt::Module*)_module;

  geomType->setBoundsProg(module, progName);
  LOG("assigning bounds program to geom type...");
}

GPRT_API GPRTBuffer
gprtHostPinnedBufferCreate(GPRTContext _context, GPRTDataType type, size_t count, const void* init)
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

  gprt::Context *context = (gprt::Context*)_context;
  gprt::Buffer *buffer = new gprt::Buffer(
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
  std::cout<<"Todo, remove host visible bit... substitute for some staging mechanism..."<<std::endl;
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

  gprt::Context *context = (gprt::Context*)_context;
  gprt::Buffer *buffer = new gprt::Buffer(
    context->physicalDevice, context->logicalDevice,
    context->graphicsCommandBuffer, context->graphicsQueue,
    bufferUsageFlags, memoryUsageFlags,
    getSize(type) * count
  );
  
  if (init) {    
    // NOTE, this mapping mechanism wont work for large buffers...
    std::cout<<"WARNING: in gprtDeviceBufferCreate, need to replace map functionality with a staging function."<<std::endl;
    buffer->map();
    void* mapped = buffer->mapped;
    memcpy(mapped, init, getSize(type) * count);
    buffer->unmap();
  }
  LOG("buffer created");
  return (GPRTBuffer)buffer;
}

GPRT_API void
gprtBufferDestroy(GPRTBuffer _buffer)
{
  LOG_API_CALL();
  gprt::Buffer *buffer = (gprt::Buffer*)_buffer;
  buffer->destroy();
  delete buffer;
  LOG("buffer destroyed");
}

GPRT_API void *
gprtBufferGetPointer(GPRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  gprt::Buffer *buffer = (gprt::Buffer*)_buffer;
  return buffer->mapped;
}

GPRT_API void
gprtBufferMap(GPRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  gprt::Buffer *buffer = (gprt::Buffer*)_buffer;
  buffer->map();
}

GPRT_API void
gprtBufferUnmap(GPRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  gprt::Buffer *buffer = (gprt::Buffer*)_buffer;
  buffer->unmap();
}



GPRT_API void gprtBuildPrograms(GPRTContext _context)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  context->buildPrograms();
  LOG("programs built...");
}

GPRT_API GPRTAccel
gprtAABBAccelCreate(GPRTContext context,
                       size_t       numGeometries,
                       GPRTGeom    *arrayOfChildGeoms,
                       unsigned int flags)
{
  GPRT_NOTIMPLEMENTED;
  return nullptr;
}

GPRT_API GPRTAccel
gprtTrianglesAccelCreate(GPRTContext _context,
                            size_t     numGeometries,
                            GPRTGeom   *arrayOfChildGeoms,
                            unsigned int flags)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  gprt::TrianglesAccel *accel = new 
    gprt::TrianglesAccel(
      context->physicalDevice, context->logicalDevice, 
      context->graphicsCommandBuffer, context->graphicsQueue, 
      numGeometries, (gprt::TrianglesGeom*)arrayOfChildGeoms);
  context->accels.push_back(accel);
  return (GPRTAccel)accel;
}

GPRT_API void gprtTrianglesAccelSetTransforms(GPRTAccel _trianglesAccel,
                                             GPRTBuffer _transforms)
{
  LOG_API_CALL();
  gprt::TrianglesAccel *accel = (gprt::TrianglesAccel*)_trianglesAccel;
  gprt::Buffer *transforms = (gprt::Buffer*)_transforms;
  accel->setTransforms(transforms);
  LOG("Setting triangle accel transforms...");
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
  gprt::Context *context = (gprt::Context*)_context;
  gprt::InstanceAccel *accel = new 
    gprt::InstanceAccel(
      context->physicalDevice, context->logicalDevice, 
      context->graphicsCommandBuffer, context->graphicsQueue, 
      numAccels, arrayOfAccels);
  context->accels.push_back(accel);
  return (GPRTAccel)accel;
}

GPRT_API void 
gprtInstanceAccelSetTransforms(GPRTAccel instanceAccel,
                               GPRTBuffer _transforms//,
                               // size_t offset, // maybe I can support these too?
                               // size_t stride  // maybe I can support these too?
                               )
{
  LOG_API_CALL();
  gprt::InstanceAccel *accel = (gprt::InstanceAccel*)instanceAccel;
  gprt::Buffer *transforms = (gprt::Buffer*)_transforms;
  accel->setTransforms(transforms);
  LOG("Setting instance accel transforms...");
}

GPRT_API void
gprtAccelDestroy(GPRTAccel _accel)
{
  LOG_API_CALL();
  gprt::Accel *accel = (gprt::Accel*)_accel;
  accel->destroy();
  delete accel;
  LOG("accel destroyed");
}

GPRT_API void gprtAccelBuild(GPRTContext _context, GPRTAccel _accel)
{
  gprt::Accel *accel = (gprt::Accel*)_accel;
  gprt::Context *context = (gprt::Context*)_context;
  accel->build({
    {"gprtFillInstanceData", context->fillInstanceDataStage}
  });
}

GPRT_API void gprtAccelRefit(GPRTContext _context, GPRTAccel accel)
{
  GPRT_NOTIMPLEMENTED;
}

GPRT_API void gprtBuildPipeline(GPRTContext _context)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  context->buildPipeline();
  LOG("pipeline created...");
}

GPRT_API void gprtBuildSBT(GPRTContext _context,
                           GPRTBuildSBTFlags flags)
{
  LOG_API_CALL();
  gprt::Context *context = (gprt::Context*)_context;
  context->buildSBT();
  LOG("SBT created...");
}

/*! Executes a ray tracing pipeline with the given raygen program.
  This call will block until the raygen program returns. */
GPRT_API void
gprtRayGenLaunch2D(GPRTContext _context, GPRTRayGen _rayGen, int dims_x, int dims_y)
{
  LOG_API_CALL();
  gprtRayGenLaunch3D(_context, _rayGen,dims_x,dims_y,1);
}

/*! 3D-launch variant of \see gprtRayGenLaunch2D */
GPRT_API void
gprtRayGenLaunch3D(GPRTContext _context, GPRTRayGen _rayGen, int dims_x, int dims_y, int dims_z)
{
  LOG_API_CALL();
  assert(_rayGen);

  gprt::Context *context = (gprt::Context*)_context;
  gprt::RayGen *raygen = (gprt::RayGen*)_rayGen;
  VkResult err;

  VkCommandBufferBeginInfo cmdBufInfo{};
  cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);

  vkCmdBindPipeline(
    context->graphicsCommandBuffer,
    VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
    context->pipeline);

  auto getBufferDeviceAddress = [](VkDevice device, VkBuffer buffer) -> uint64_t
	{
		VkBufferDeviceAddressInfoKHR bufferDeviceAI{};
		bufferDeviceAI.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
		bufferDeviceAI.buffer = buffer;
		return gprt::vkGetBufferDeviceAddressKHR(device, &bufferDeviceAI);
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
  const uint32_t recordSize = alignedSize(maxGroupSize, groupAlignment);
  uint64_t baseAddr = getBufferDeviceAddress(
    context->logicalDevice, context->shaderBindingTable.buffer);

  VkStridedDeviceAddressRegionKHR raygenShaderSbtEntry{};
  raygenShaderSbtEntry.deviceAddress = baseAddr;
  raygenShaderSbtEntry.stride = recordSize;
  raygenShaderSbtEntry.size = raygenShaderSbtEntry.stride * context->raygenPrograms.size();

  VkStridedDeviceAddressRegionKHR missShaderSbtEntry{};
  if (context->missPrograms.size() > 0) {
    missShaderSbtEntry.deviceAddress = baseAddr + recordSize * context->raygenPrograms.size();
    missShaderSbtEntry.stride = recordSize;
    missShaderSbtEntry.size = missShaderSbtEntry.stride * context->missPrograms.size();
  }
  VkStridedDeviceAddressRegionKHR hitShaderSbtEntry{};
  size_t numHitRecords = context->getNumHitRecords();
  if (numHitRecords > 0) {
    hitShaderSbtEntry.deviceAddress = baseAddr + recordSize * (context->raygenPrograms.size() + context->missPrograms.size());
    hitShaderSbtEntry.stride = recordSize;
    hitShaderSbtEntry.size = hitShaderSbtEntry.stride * numHitRecords;
  }

  VkStridedDeviceAddressRegionKHR callableShaderSbtEntry{}; // empty
  callableShaderSbtEntry.deviceAddress = 0;
  // callableShaderSbtEntry.stride = handleSizeAligned;
  // callableShaderSbtEntry.size = handleSizeAligned;

  gprt::vkCmdTraceRaysKHR(
    context->graphicsCommandBuffer,
    &raygenShaderSbtEntry,
    &missShaderSbtEntry,
    &hitShaderSbtEntry,
    &callableShaderSbtEntry,
    dims_x,
    dims_y,
    dims_z);

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

std::pair<size_t, void*> gprtGetVariable(
  gprt::SBTEntry *entry, std::string name, GPRTDataType type
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1b(GPRTRayGen _raygen, const char *name, bool x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL);
  bool val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2b(GPRTRayGen _raygen, const char *name, bool x, bool y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  bool val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3b(GPRTRayGen _raygen, const char *name, bool x, bool y, bool z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  bool val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4b(GPRTRayGen _raygen, const char *name, bool x, bool y, bool z, bool w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  bool val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2bv(GPRTRayGen _raygen, const char *name, const bool *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3bv(GPRTRayGen _raygen, const char *name, const bool *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4bv(GPRTRayGen _raygen, const char *name, const bool *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1b(GPRTMissProg _missprog, const char *name, bool x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL);
  bool val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2b(GPRTMissProg _missprog, const char *name, bool x, bool y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  bool val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3b(GPRTMissProg _missprog, const char *name, bool x, bool y, bool z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  bool val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4b(GPRTMissProg _missprog, const char *name, bool x, bool y, bool z, bool w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  bool val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2bv(GPRTMissProg _missprog, const char *name, const bool *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3bv(GPRTMissProg _missprog, const char *name, const bool *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4bv(GPRTMissProg _missprog, const char *name, const bool *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1b(GPRTGeom _geom, const char *name, bool x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL);
  bool val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2b(GPRTGeom _geom, const char *name, bool x, bool y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  bool val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3b(GPRTGeom _geom, const char *name, bool x, bool y, bool z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  bool val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4b(GPRTGeom _geom, const char *name, bool x, bool y, bool z, bool w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  bool val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2bv(GPRTGeom _geom, const char *name, const bool *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3bv(GPRTGeom _geom, const char *name, const bool *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4bv(GPRTGeom _geom, const char *name, const bool *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_BOOL4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1c(GPRTRayGen _raygen, const char *name, int8_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T);
  int8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2c(GPRTRayGen _raygen, const char *name, int8_t x, int8_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  int8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3c(GPRTRayGen _raygen, const char *name, int8_t x, int8_t y, int8_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  int8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4c(GPRTRayGen _raygen, const char *name, int8_t x, int8_t y, int8_t z, int8_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  int8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2cv(GPRTRayGen _raygen, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3cv(GPRTRayGen _raygen, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4cv(GPRTRayGen _raygen, const char *name, const int8_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1c(GPRTMissProg _missprog, const char *name, int8_t x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T);
  int8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2c(GPRTMissProg _missprog, const char *name, int8_t x, int8_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  int8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3c(GPRTMissProg _missprog, const char *name, int8_t x, int8_t y, int8_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  int8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4c(GPRTMissProg _missprog, const char *name, int8_t x, int8_t y, int8_t z, int8_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  int8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2cv(GPRTMissProg _missprog, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3cv(GPRTMissProg _missprog, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4cv(GPRTMissProg _missprog, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1c(GPRTGeom _geom, const char *name, int8_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T);
  int8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2c(GPRTGeom _geom, const char *name, int8_t x, int8_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  int8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3c(GPRTGeom _geom, const char *name, int8_t x, int8_t y, int8_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  int8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4c(GPRTGeom _geom, const char *name, int8_t x, int8_t y, int8_t z, int8_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  int8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2cv(GPRTGeom _geom, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3cv(GPRTGeom _geom, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4cv(GPRTGeom _geom, const char *name, const int8_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT8_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1uc(GPRTRayGen _raygen, const char *name, uint8_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T);
  uint8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2uc(GPRTRayGen _raygen, const char *name, uint8_t x, uint8_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  uint8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3uc(GPRTRayGen _raygen, const char *name, uint8_t x, uint8_t y, uint8_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  uint8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4uc(GPRTRayGen _raygen, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  uint8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ucv(GPRTRayGen _raygen, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3ucv(GPRTRayGen _raygen, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4ucv(GPRTRayGen _raygen, const char *name, const uint8_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1uc(GPRTMissProg _missprog, const char *name, uint8_t x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T);
  uint8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2uc(GPRTMissProg _missprog, const char *name, uint8_t x, uint8_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  uint8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3uc(GPRTMissProg _missprog, const char *name, uint8_t x, uint8_t y, uint8_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  uint8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4uc(GPRTMissProg _missprog, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  uint8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2ucv(GPRTMissProg _missprog, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3ucv(GPRTMissProg _missprog, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4ucv(GPRTMissProg _missprog, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1uc(GPRTGeom _geom, const char *name, uint8_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T);
  uint8_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2uc(GPRTGeom _geom, const char *name, uint8_t x, uint8_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  uint8_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3uc(GPRTGeom _geom, const char *name, uint8_t x, uint8_t y, uint8_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  uint8_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4uc(GPRTGeom _geom, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  uint8_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ucv(GPRTGeom _geom, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3ucv(GPRTGeom _geom, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4ucv(GPRTGeom _geom, const char *name, const uint8_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT8_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1s(GPRTRayGen _raygen, const char *name, int16_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T);
  int16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2s(GPRTRayGen _raygen, const char *name, int16_t x, int16_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  int16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3s(GPRTRayGen _raygen, const char *name, int16_t x, int16_t y, int16_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  int16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4s(GPRTRayGen _raygen, const char *name, int16_t x, int16_t y, int16_t z, int16_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  int16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2sv(GPRTRayGen _raygen, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3sv(GPRTRayGen _raygen, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4sv(GPRTRayGen _raygen, const char *name, const int16_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1s(GPRTMissProg _missprog, const char *name, int16_t val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2s(GPRTMissProg _missprog, const char *name, int16_t x, int16_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  int16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3s(GPRTMissProg _missprog, const char *name, int16_t x, int16_t y, int16_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  int16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4s(GPRTMissProg _missprog, const char *name, int16_t x, int16_t y, int16_t z, int16_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  int16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2sv(GPRTMissProg _missprog, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3sv(GPRTMissProg _missprog, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4sv(GPRTMissProg _missprog, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1s(GPRTGeom _geom, const char *name, int16_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T);
  int16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2s(GPRTGeom _geom, const char *name, int16_t x, int16_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  int16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3s(GPRTGeom _geom, const char *name, int16_t x, int16_t y, int16_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  int16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4s(GPRTGeom _geom, const char *name, int16_t x, int16_t y, int16_t z, int16_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  int16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2sv(GPRTGeom _geom, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3sv(GPRTGeom _geom, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4sv(GPRTGeom _geom, const char *name, const int16_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT16_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1us(GPRTRayGen _raygen, const char *name, uint16_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T);
  uint16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2us(GPRTRayGen _raygen, const char *name, uint16_t x, uint16_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  uint16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3us(GPRTRayGen _raygen, const char *name, uint16_t x, uint16_t y, uint16_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  uint16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4us(GPRTRayGen _raygen, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  uint16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2usv(GPRTRayGen _raygen, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3usv(GPRTRayGen _raygen, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4usv(GPRTRayGen _raygen, const char *name, const uint16_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1us(GPRTMissProg _missprog, const char *name, uint16_t x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T);
  uint16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2us(GPRTMissProg _missprog, const char *name, uint16_t x, uint16_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  uint16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3us(GPRTMissProg _missprog, const char *name, uint16_t x, uint16_t y, uint16_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  uint16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4us(GPRTMissProg _missprog, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  uint16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2usv(GPRTMissProg _missprog, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3usv(GPRTMissProg _missprog, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4usv(GPRTMissProg _missprog, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1us(GPRTGeom _geom, const char *name, uint16_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T);
  uint16_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2us(GPRTGeom _geom, const char *name, uint16_t x, uint16_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  uint16_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3us(GPRTGeom _geom, const char *name, uint16_t x, uint16_t y, uint16_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  uint16_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4us(GPRTGeom _geom, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  uint16_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2usv(GPRTGeom _geom, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3usv(GPRTGeom _geom, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4usv(GPRTGeom _geom, const char *name, const uint16_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT16_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1i(GPRTRayGen _raygen, const char *name, int32_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T);
  int32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2i(GPRTRayGen _raygen, const char *name, int32_t x, int32_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  int32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3i(GPRTRayGen _raygen, const char *name, int32_t x, int32_t y, int32_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  int32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4i(GPRTRayGen _raygen, const char *name, int32_t x, int32_t y, int32_t z, int32_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  int32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2iv(GPRTRayGen _raygen, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3iv(GPRTRayGen _raygen, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4iv(GPRTRayGen _raygen, const char *name, const int32_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1i(GPRTMissProg _missprog, const char *name, int32_t x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T);
  int32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2i(GPRTMissProg _missprog, const char *name, int32_t x, int32_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  int32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3i(GPRTMissProg _missprog, const char *name, int32_t x, int32_t y, int32_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  int32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4i(GPRTMissProg _missprog, const char *name, int32_t x, int32_t y, int32_t z, int32_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  int32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2iv(GPRTMissProg _missprog, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3iv(GPRTMissProg _missprog, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4iv(GPRTMissProg _missprog, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1i(GPRTGeom _geom, const char *name, int32_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T);
  int32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2i(GPRTGeom _geom, const char *name, int32_t x, int32_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  int32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3i(GPRTGeom _geom, const char *name, int32_t x, int32_t y, int32_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  int32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4i(GPRTGeom _geom, const char *name, int32_t x, int32_t y, int32_t z, int32_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  int32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2iv(GPRTGeom _geom, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3iv(GPRTGeom _geom, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4iv(GPRTGeom _geom, const char *name, const int32_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT32_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1ui(GPRTRayGen _raygen, const char *name, uint32_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T);
  uint32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ui(GPRTRayGen _raygen, const char *name, uint32_t x, uint32_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  uint32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3ui(GPRTRayGen _raygen, const char *name, uint32_t x, uint32_t y, uint32_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  uint32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4ui(GPRTRayGen _raygen, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  uint32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2uiv(GPRTRayGen _raygen, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3uiv(GPRTRayGen _raygen, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4uiv(GPRTRayGen _raygen, const char *name, const uint32_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1ui(GPRTMissProg _missprog, const char *name, uint32_t x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T);
  uint32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2ui(GPRTMissProg _missprog, const char *name, uint32_t x, uint32_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  uint32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3ui(GPRTMissProg _missprog, const char *name, uint32_t x, uint32_t y, uint32_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  uint32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4ui(GPRTMissProg _missprog, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  uint32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2uiv(GPRTMissProg _missprog, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3uiv(GPRTMissProg _missprog, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4uiv(GPRTMissProg _missprog, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1ui(GPRTGeom _geom, const char *name, uint32_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T);
  uint32_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ui(GPRTGeom _geom, const char *name, uint32_t x, uint32_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  uint32_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3ui(GPRTGeom _geom, const char *name, uint32_t x, uint32_t y, uint32_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  uint32_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4ui(GPRTGeom _geom, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  uint32_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2uiv(GPRTGeom _geom, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3uiv(GPRTGeom _geom, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4uiv(GPRTGeom _geom, const char *name, const uint32_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT32_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1f(GPRTRayGen _raygen, const char *name, float x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT);
  float val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2f(GPRTRayGen _raygen, const char *name, float x, float y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  float val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3f(GPRTRayGen _raygen, const char *name, float x, float y, float z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  float val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4f(GPRTRayGen _raygen, const char *name, float x, float y, float z, float w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  float val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2fv(GPRTRayGen _raygen, const char *name, const float *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3fv(GPRTRayGen _raygen, const char *name, const float *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4fv(GPRTRayGen _raygen, const char *name, const float *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1f(GPRTMissProg _missprog, const char *name, float x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT);
  float val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2f(GPRTMissProg _missprog, const char *name, float x, float y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  float val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3f(GPRTMissProg _missprog, const char *name, float x, float y, float z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  float val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4f(GPRTMissProg _missprog, const char *name, float x, float y, float z, float w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  float val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2fv(GPRTMissProg _missprog, const char *name, const float *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3fv(GPRTMissProg _missprog, const char *name, const float *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4fv(GPRTMissProg _missprog, const char *name, const float *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1f(GPRTGeom _geom, const char *name, float x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT);
  float val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2f(GPRTGeom _geom, const char *name, float x, float y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  float val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3f(GPRTGeom _geom, const char *name, float x, float y, float z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  float val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4f(GPRTGeom _geom, const char *name, float x, float y, float z, float w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  float val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2fv(GPRTGeom _geom, const char *name, const float *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3fv(GPRTGeom _geom, const char *name, const float *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4fv(GPRTGeom _geom, const char *name, const float *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_FLOAT4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1d(GPRTRayGen _raygen, const char *name, double x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE);
  double val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2d(GPRTRayGen _raygen, const char *name, double x, double y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  double val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3d(GPRTRayGen _raygen, const char *name, double x, double y, double z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  double val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4d(GPRTRayGen _raygen, const char *name, double x, double y, double z, double w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  double val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2dv(GPRTRayGen _raygen, const char *name, const double *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3dv(GPRTRayGen _raygen, const char *name, const double *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4dv(GPRTRayGen _raygen, const char *name, const double *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1d(GPRTMissProg _missprog, const char *name, double x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE);
  double val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2d(GPRTMissProg _missprog, const char *name, double x, double y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  double val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3d(GPRTMissProg _missprog, const char *name, double x, double y, double z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  double val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4d(GPRTMissProg _missprog, const char *name, double x, double y, double z, double w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  double val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2dv(GPRTMissProg _missprog, const char *name, const double *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3dv(GPRTMissProg _missprog, const char *name, const double *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4dv(GPRTMissProg _missprog, const char *name, const double *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1d(GPRTGeom _geom, const char *name, double x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE);
  double val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2d(GPRTGeom _geom, const char *name, double x, double y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  double val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3d(GPRTGeom _geom, const char *name, double x, double y, double z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  double val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4d(GPRTGeom _geom, const char *name, double x, double y, double z, double w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  double val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2dv(GPRTGeom _geom, const char *name, const double *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3dv(GPRTGeom _geom, const char *name, const double *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4dv(GPRTGeom _geom, const char *name, const double *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_DOUBLE4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1l(GPRTRayGen _raygen, const char *name, int64_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T);
  int64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2l(GPRTRayGen _raygen, const char *name, int64_t x, int64_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  int64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3l(GPRTRayGen _raygen, const char *name, int64_t x, int64_t y, int64_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  int64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4l(GPRTRayGen _raygen, const char *name, int64_t x, int64_t y, int64_t z, int64_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  int64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2lv(GPRTRayGen _raygen, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3lv(GPRTRayGen _raygen, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4lv(GPRTRayGen _raygen, const char *name, const int64_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1l(GPRTMissProg _missprog, const char *name, int64_t x)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T);
  int64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2l(GPRTMissProg _missprog, const char *name, int64_t x, int64_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  int64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3l(GPRTMissProg _missprog, const char *name, int64_t x, int64_t y, int64_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  int64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4l(GPRTMissProg _missprog, const char *name, int64_t x, int64_t y, int64_t z, int64_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  int64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2lv(GPRTMissProg _missprog, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3lv(GPRTMissProg _missprog, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4lv(GPRTMissProg _missprog, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1l(GPRTGeom _geom, const char *name, int64_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T);
  int64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2l(GPRTGeom _geom, const char *name, int64_t x, int64_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  int64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3l(GPRTGeom _geom, const char *name, int64_t x, int64_t y, int64_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  int64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4l(GPRTGeom _geom, const char *name, int64_t x, int64_t y, int64_t z, int64_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  int64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2lv(GPRTGeom _geom, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3lv(GPRTGeom _geom, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4lv(GPRTGeom _geom, const char *name, const int64_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_INT64_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
GPRT_API void gprtRayGenSet1ul(GPRTRayGen _raygen, const char *name, uint64_t x) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T);
  uint64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ul(GPRTRayGen _raygen, const char *name, uint64_t x, uint64_t y) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  uint64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3ul(GPRTRayGen _raygen, const char *name, uint64_t x, uint64_t y, uint64_t z) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  uint64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4ul(GPRTRayGen _raygen, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  uint64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet2ulv(GPRTRayGen _raygen, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet3ulv(GPRTRayGen _raygen, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtRayGenSet4ulv(GPRTRayGen _raygen, const char *name, const uint64_t *val) 
{
  LOG_API_CALL();
  gprt::RayGen *entry = (gprt::RayGen*)_raygen;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  memcpy(var.second, &val, var.first);
}

// setters for variables on "MissProg"s
GPRT_API void gprtMissProgSet1ul(GPRTMissProg _missprog, const char *name, uint64_t val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2ul(GPRTMissProg _missprog, const char *name, uint64_t x, uint64_t y)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  uint64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3ul(GPRTMissProg _missprog, const char *name, uint64_t x, uint64_t y, uint64_t z)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  uint64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4ul(GPRTMissProg _missprog, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  uint64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet2ulv(GPRTMissProg _missprog, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet3ulv(GPRTMissProg _missprog, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtMissProgSet4ulv(GPRTMissProg _missprog, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  gprt::MissProg *entry = (gprt::MissProg*)_missprog;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  memcpy(var.second, &val, var.first);
}


// setters for variables on "Geom"s
GPRT_API void gprtGeomSet1ul(GPRTGeom _geom, const char *name, uint64_t x)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T);
  uint64_t val[] = {x};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ul(GPRTGeom _geom, const char *name, uint64_t x, uint64_t y)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  uint64_t val[] = {x, y};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3ul(GPRTGeom _geom, const char *name, uint64_t x, uint64_t y, uint64_t z)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  uint64_t val[] = {x, y, z};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4ul(GPRTGeom _geom, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  uint64_t val[] = {x, y, z, w};
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet2ulv(GPRTGeom _geom, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T2);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet3ulv(GPRTGeom _geom, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T3);
  memcpy(var.second, &val, var.first);
}

GPRT_API void gprtGeomSet4ulv(GPRTGeom _geom, const char *name, const uint64_t *val)
{
  LOG_API_CALL();
  gprt::Geom *entry = (gprt::Geom*)_geom;
  assert(entry);
  auto var = gprtGetVariable(entry, name, GPRT_UINT64_T4);
  memcpy(var.second, &val, var.first);
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

// setters for variables on "RayGen"s
// GPRT_API void gprtRayGenSetTexture(GPRTRayGen _raygen, const char *name, GPRTTexture val) 
// GPRT_API void gprtRayGenSetPointer(GPRTRayGen _raygen, const char *name, const void *val) 
GPRT_API void gprtRayGenSetBuffer(GPRTRayGen _rayGen, const char *name, GPRTBuffer _val)
{
  LOG_API_CALL();
  gprt::RayGen *raygen = (gprt::RayGen*)_rayGen;
  assert(raygen);

  gprt::Buffer *val = (gprt::Buffer*)_val;
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

GPRT_API void gprtRayGenSetAccel(GPRTRayGen _raygen, const char *name, GPRTAccel _val)
{
  LOG_API_CALL();
  gprt::RayGen *raygen = (gprt::RayGen*)_raygen;
  assert(raygen);

  gprt::Accel *val = (gprt::Accel*)_val;
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

GPRT_API void gprtRayGenSetRaw(GPRTRayGen _rayGen, const char *name, const void *val)
{
  LOG_API_CALL();
  gprt::RayGen *raygen = (gprt::RayGen*)_rayGen;
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
  gprt::Geom *geom = (gprt::Geom*)_geom;
  assert(geom);

  gprt::Buffer *val = (gprt::Buffer*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(geom->vars.find(std::string(name)) != geom->vars.end());

  // The found variable must be a buffer
  assert(geom->vars[name].decl.type == GPRT_BUFFER || 
         geom->vars[name].decl.type == GPRT_BUFPTR);

  // Buffer pointers are 64 bits
  size_t size = sizeof(uint64_t);

  // 3. Assign the value to that variable
  VkDeviceAddress addr = val->address;
  memcpy(geom->vars[name].data, &addr, size);
}

GPRT_API void gprtGeomSetAccel(GPRTGeom _geom, const char *name, GPRTAccel _val)
{
  LOG_API_CALL();
  gprt::Geom *geom = (gprt::Geom*)_geom;
  assert(geom);

  gprt::Accel *val = (gprt::Accel*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(geom->vars.find(std::string(name)) != geom->vars.end());

  // The found variable must be an acceleration structure
  assert(geom->vars[name].decl.type == GPRT_ACCEL);

  // Acceleration structure pointers are 64 bits
  size_t size = sizeof(uint64_t);

  // 3. Assign the value to that variable
  VkDeviceAddress addr = val->address;
  memcpy(geom->vars[name].data, &addr, size);
}

GPRT_API void gprtGeomSetRaw(GPRTGeom _geom, const char *name, const void *val)
{
  LOG_API_CALL();
  gprt::Geom *geom = (gprt::Geom*)_geom;
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
// GPRT_API void gprtMissProgSetTexture(GPRTMissProg _missprog, const char *name, GPRTTexture val)

// GPRT_API void gprtMissProgSetPointer(GPRTMissProg _missprog, const char *name, const void *val);
GPRT_API void gprtMissProgSetBuffer(GPRTMissProg _missProg, const char *name, GPRTBuffer _val)
{
  LOG_API_CALL();
  gprt::MissProg *missprog = (gprt::MissProg*)_missProg;
  assert(missprog);

  gprt::Buffer *val = (gprt::Buffer*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(missprog->vars.find(std::string(name)) != missprog->vars.end());

  // The found variable must be a buffer
  assert(missprog->vars[name].decl.type == GPRT_BUFFER || 
         missprog->vars[name].decl.type == GPRT_BUFPTR);

  // Buffer pointers are 64 bits
  size_t size = sizeof(uint64_t);

  // 3. Assign the value to that variable
  VkDeviceAddress addr = val->address;
  memcpy(missprog->vars[name].data, &addr, size);
}

GPRT_API void gprtMissProgSetAccel(GPRTMissProg _missprog, const char *name, GPRTAccel _val)
{
  LOG_API_CALL();
  gprt::MissProg *missprog = (gprt::MissProg*)_missprog;
  assert(missprog);

  gprt::Accel *val = (gprt::Accel*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists
  assert(missprog->vars.find(std::string(name)) != missprog->vars.end());

  // The found variable must be an acceleration structure
  assert(missprog->vars[name].decl.type == GPRT_ACCEL);

  // Acceleration structure pointers are 64 bits
  size_t size = sizeof(uint64_t);

  // 3. Assign the value to that variable
  VkDeviceAddress addr = val->address;
  memcpy(missprog->vars[name].data, &addr, size);
}

GPRT_API void gprtMissProgSetRaw(GPRTMissProg _missProg, const char *name, const void *val)
{
  LOG_API_CALL();
  gprt::MissProg *missProg = (gprt::MissProg*)_missProg;
  assert(missProg);

  // 1. Figure out if the variable "name" exists
  assert(missProg->vars.find(std::string(name)) != missProg->vars.end());

  // 2. Get the expected size for this variable
  size_t size = getSize(missProg->vars[name].decl.type);

  // 3. Assign the value to that variable
  memcpy(missProg->vars[name].data, val, size);
}
