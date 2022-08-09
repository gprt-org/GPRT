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

#include <vkrt_host.h>
#include <iostream>
#include <vector>
#include <assert.h>
#include <fstream>
#include <sstream>
#include <unordered_map>

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
  if (vkrt::Context::logging())                       \
    std::cout                                   \
      << VKRT_TERMINAL_LIGHT_BLUE                \
      << "#vkrt: "                               \
      << message                                \
      << VKRT_TERMINAL_DEFAULT << std::endl

#define LOG_OK(message)                         \
  if (vkrt::Context::logging())                       \
    std::cout                                   \
      << VKRT_TERMINAL_BLUE                      \
      << "#vkrt: "                               \
      << message                                \
      << VKRT_TERMINAL_DEFAULT << std::endl

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

inline void vkrtRaise_impl(std::string str)
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

#define VKRT_RAISE(MSG) ::detail::vkrtRaise_impl(MSG);

#define VKRT_NOTIMPLEMENTED  { std::cerr<<std::string(__PRETTY_FUNCTION__) << " not implemented" << std::endl; assert(false);};


#include "3rdParty/SPIRV-Tools/include/spirv-tools/libspirv.h"

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

namespace vkrt {

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

  struct Module {
    spv_context spvContext;
    std::string program;

    Module(const char* spvCode) {
      program = std::string(spvCode);
      spvContext = spvContextCreate(SPV_ENV_UNIVERSAL_1_4);
    }

    ~Module() {
      spvContextDestroy(spvContext);
    }

    spv_binary getBinary(std::string entryPoint) {
      std::regex re("( *)(OpEntryPoint )(.*? )([%][A-Za-z]*)( \"[A-Za-z]*\" )(.*)");
      std::smatch match;
      std::string text = program;

      std::string newProgram;
      while (std::regex_search(text, match, re))
      {
        std::string line = match.str(0);
        std::string otherEntryPoint = match.str(4);

        if (match.str(4) == (std::string("%") + std::string(entryPoint)) ) {
          newProgram += match.prefix().str() + match.str(0);
          text = match.suffix().str();
        }
        else {
          // Remove the other entry points. Currently, SPIRV doesn't support
          // multiple entry points in combination with debug printf
          newProgram += match.prefix();
          text = match.suffix().str();

          // Remove the OpName %entrypoint "entrypoint" line
          std::regex OpNameRE("( *)(OpName )(" + otherEntryPoint + ")( \"[A-Za-z]*\")");
          std::smatch OpNameMatch;
          std::string subtext = text;
          if (!std::regex_search(subtext, OpNameMatch, OpNameRE)) throw std::runtime_error("Error editing SPIRV");
          else {
            // std::cout<<"Found OpName: "<< OpNameMatch.str(0) << " ... removing..."<<std::endl;
            text = OpNameMatch.prefix().str() + OpNameMatch.suffix().str();
          }

          // Remove the entrypoint itself.
          // found by %entrypoint = ... until the first occurance of OpFunctionEnd
          std::regex OpEntryPointRE("( *)(" + otherEntryPoint + " =[^]*?OpFunctionEnd)");
          std::smatch OpEntryPointMatch;
          subtext = text;
          if (!std::regex_search(subtext, OpEntryPointMatch, OpEntryPointRE)) throw std::runtime_error("Error editing SPIRV");
          else {
            // std::cout<<"Found Entry Point: "<< OpEntryPointMatch.str(0) << " ... removing..."<<std::endl;
            text = OpEntryPointMatch.prefix().str() + OpEntryPointMatch.suffix().str();
          }
        }
      }
      newProgram += text;

      // std::cout<<"final program " << std::endl;
      // std::cout<<newProgram<<std::endl;

      // Now, assemble the IR
      spv_binary binary = nullptr;
      spv_diagnostic diagnostic = nullptr;
      spvTextToBinary(spvContext, newProgram.c_str(), newProgram.size(), &binary, &diagnostic);
      spvValidateBinary(spvContext, binary->code, binary->wordCount, &diagnostic);
      if (diagnostic) {
        spvDiagnosticPrint(diagnostic);
        spvDiagnosticDestroy(diagnostic);
      }

      return binary;
    }

    void releaseBinary(spv_binary binary) {
      spvBinaryDestroy(binary);
    }
  };

  struct Buffer {
    VkDevice device;
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDescriptorBufferInfo descriptor;
    VkDeviceSize size = 0;
    VkDeviceSize alignment = 0;
    void* mapped = nullptr;
    /** @brief Usage flags to be filled by external source at buffer creation (to query at some later point) */
    VkBufferUsageFlags usageFlags;
    /** @brief Memory property flags to be filled by external source at buffer creation (to query at some later point) */
    VkMemoryPropertyFlags memoryPropertyFlags;

    VkPhysicalDeviceMemoryProperties memoryProperties;

    VkDeviceAddress address = 0;

    VkResult map(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0)
    {
      return vkMapMemory(device, memory, offset, size, 0, &mapped);
    }
    void unmap()
    {
      if (mapped)
      {
        vkUnmapMemory(device, memory);
        mapped = nullptr;
      }
    }
    VkResult bind(VkDeviceSize offset = 0)
    {
      return vkBindBufferMemory(device, buffer, memory, offset);
    }
    void setupDescriptor(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0)
    {
      descriptor.offset = offset;
      descriptor.buffer = buffer;
      descriptor.range = size;
    }
    void copyTo(void* data, VkDeviceSize size)
    {
      assert(mapped);
      memcpy(mapped, data, size);
    }

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
      {
        vkDestroyBuffer(device, buffer, nullptr);
      }
      if (memory)
      {
        vkFreeMemory(device, memory, nullptr);
      }
    }

    /* Default Constructor */
    Buffer() {};

    Buffer(
      VkPhysicalDevice physicalDevice, VkDevice logicalDevice,
      VkBufferUsageFlags _usageFlags, VkMemoryPropertyFlags _memoryPropertyFlags,
      VkDeviceSize _size, void *data = nullptr)
    {
      device = logicalDevice;
      memoryPropertyFlags = _memoryPropertyFlags;
      size = _size;
      usageFlags = _usageFlags;
      memoryPropertyFlags = _memoryPropertyFlags;

      vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memoryProperties);

      auto getMemoryType = [this](
        uint32_t typeBits, VkMemoryPropertyFlags properties,
        VkBool32 *memTypeFound = nullptr) -> uint32_t {
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

      // If a pointer to the buffer data has been passed, map the buffer and
      // copy over the data
      if (data != nullptr) {
        VK_CHECK_RESULT(map());
        memcpy(mapped, data, size);
        if ((memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == 0)
          flush();
        unmap();
      }

      // Initialize a default descriptor that covers the whole buffer size
      setupDescriptor();

      // Attach the memory to the buffer object
      VkResult err = bind();
      if (err) throw std::runtime_error("failed to create buffer! : \n" + errorString(err));

      // means we can get this buffer's address with vkGetBufferDeviceAddress
      if ((usageFlags & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) != 0)
        address = getDeviceAddress();
    }
  };

  struct SBTEntry {
    // Map of the name of the variable to that variable declaration
    std::unordered_map<std::string, VKRTVarDef> vars;
  };

  struct RayGen : public SBTEntry {
    VkShaderModule shaderModule;
    VkPipelineShaderStageCreateInfo shaderStage{};
    VkShaderModuleCreateInfo moduleCreateInfo{};
    VkDevice logicalDevice;

    RayGen(VkDevice  _logicalDevice,
           Module *module,
           const char* entryPoint,
           size_t      sizeOfVarStruct,
           std::unordered_map<std::string, VKRTVarDef> _vars) : SBTEntry()
    {
      std::cout<<"Ray gen is being made!"<<std::endl;

      spv_binary binary = module->getBinary(entryPoint);

      // store a reference to the logical device this module is made on
      logicalDevice = _logicalDevice;

      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary->wordCount * sizeof(uint32_t);//sizeOfProgramBytes;
      moduleCreateInfo.pCode = binary->code; //(uint32_t*)binary->wordCount;//programBytes;

      VkResult err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
        NULL, &shaderModule);
      if (err) throw std::runtime_error("failed to create shader module! : \n" + errorString(err));

      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
      shaderStage.module = shaderModule;
      shaderStage.pName = entryPoint;
      assert(shaderStage.module != VK_NULL_HANDLE);

      module->releaseBinary(binary);

      vars = _vars;
    }
    ~RayGen() {
      std::cout<<"Ray gen is being destroyed!"<<std::endl;
      vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
    }
  };

  struct MissProg : public SBTEntry {
    VkShaderModule shaderModule;
    VkPipelineShaderStageCreateInfo shaderStage{};
    VkShaderModuleCreateInfo moduleCreateInfo{};
    VkDevice logicalDevice;

    MissProg(VkDevice  _logicalDevice,
             Module *module,
             const char* entryPoint,
             size_t      sizeOfVarStruct,
             std::unordered_map<std::string, VKRTVarDef> _vars) : SBTEntry()
    {
      std::cout<<"Miss program is being made!"<<std::endl;

      spv_binary binary = module->getBinary(entryPoint);

      // store a reference to the logical device this module is made on
      logicalDevice = _logicalDevice;

      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary->wordCount * sizeof(uint32_t);//sizeOfProgramBytes;
      moduleCreateInfo.pCode = binary->code; //(uint32_t*)binary->wordCount;//programBytes;

      VK_CHECK_RESULT(vkCreateShaderModule(logicalDevice, &moduleCreateInfo,
        NULL, &shaderModule));

      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_MISS_BIT_KHR;
      shaderStage.module = shaderModule;
      shaderStage.pName = entryPoint;
      assert(shaderStage.module != VK_NULL_HANDLE);

      module->releaseBinary(binary);

      vars = _vars;
    }
    ~MissProg() {
      std::cout<<"Miss program is being destroyed!"<<std::endl;
      vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
    }
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
    VkPipeline pipeline;
    VkPipelineLayout pipelineLayout;

    std::vector<RayGen*> raygenPrograms;
    std::vector<MissProg*> missPrograms;

    std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups{};
    vkrt::Buffer raygenShaderBindingTable;
    vkrt::Buffer missShaderBindingTable;
    vkrt::Buffer hitShaderBindingTable;

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

    std::unordered_map<std::string, VKRTVarDef> checkAndPackVariables(const VKRTVarDecl *vars,
                                                   int               numVars)
    {
      if (vars == nullptr && (numVars == 0 || numVars == -1))
        return {};

      // *copy* the vardecls here, so we can catch any potential memory
      // *access errors early

      assert(vars);
      if (numVars == -1) {
        // using -1 as count value for a variable list means the list is
        // null-terminated... so just count it
        for (numVars = 0; vars[numVars].name != nullptr; numVars++);
      }
      std::unordered_map<std::string, VKRTVarDef> varDefs;
      for (int i=0;i<numVars;i++) {
        assert(vars[i].name != nullptr);
        varDefs[vars[i].name].decl = vars[i];

        // allocate depending on the size of the variable...
        varDefs[vars[i].name].data = malloc(getSize(vars[i].type));
      }
      return varDefs;
    }

    Context(int32_t *requestedDeviceIDs, int numRequestedDevices) {
      appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
      appInfo.pApplicationName = "VKRT";
      appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
      appInfo.pEngineName = "VKRT";
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

      VkPhysicalDeviceFeatures2 deviceFeatures2{};
      deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
      deviceFeatures2.pNext = &rtPipelineFeatures;
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
    };

    ~Context() {

      if (pipelineLayout)
        vkDestroyPipelineLayout(logicalDevice, pipelineLayout, nullptr);
      if (pipeline)
        vkDestroyPipeline(logicalDevice, pipeline, nullptr);

      vkFreeCommandBuffers(logicalDevice, graphicsCommandPool, 1, &graphicsCommandBuffer);
      vkFreeCommandBuffers(logicalDevice, computeCommandPool, 1, &computeCommandBuffer);
      vkFreeCommandBuffers(logicalDevice, transferCommandPool, 1, &transferCommandBuffer);
      vkDestroyCommandPool(logicalDevice, graphicsCommandPool, nullptr);
      vkDestroyCommandPool(logicalDevice, computeCommandPool, nullptr);
      vkDestroyCommandPool(logicalDevice, transferCommandPool, nullptr);
      vkDestroyDevice(logicalDevice, nullptr);
      vkDestroyInstance(instance, nullptr);
    };

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

      const VkBufferUsageFlags bufferUsageFlags = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
      const VkMemoryPropertyFlags memoryUsageFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;


      // std::cout<<"Todo, get some smarter memory allocation working..." <<std::endl;

      int numRayGens = raygenPrograms.size();
      int numMissProgs = missPrograms.size();
      int numRayTypes = 1;

      // Raygen records
      if (raygenShaderBindingTable.size != recordSize * raygenPrograms.size()) {
        raygenShaderBindingTable.destroy();
      }
      if (raygenShaderBindingTable.buffer == VK_NULL_HANDLE) {
        raygenShaderBindingTable = Buffer(physicalDevice, logicalDevice,
        bufferUsageFlags, memoryUsageFlags, recordSize * raygenPrograms.size());
      }
      raygenShaderBindingTable.map();
      memcpy(raygenShaderBindingTable.mapped, shaderHandleStorage.data(), handleSize);
      for (uint32_t idx = 0; idx < raygenPrograms.size(); ++idx) {
        RayGen *raygen = raygenPrograms[idx];
        size_t stride = recordSize;
        size_t offset = stride * idx + handleSize /* params start after shader identifier */ ;
        uint8_t* params = ((uint8_t*) (raygenShaderBindingTable.mapped)) + offset;
        for (auto &var : raygen->vars) {
          size_t varOffset = var.second.decl.offset;
          size_t varSize = getSize(var.second.decl.type);
          std::cout<<"Embedding " << var.first << " into raygen SBT record. Size: " << varSize << " Offset: " << varOffset << std::endl;
          memcpy(params + varOffset, var.second.data, varSize);
        }
      }
      raygenShaderBindingTable.unmap();

      // Miss records
      if (missShaderBindingTable.size != recordSize * missPrograms.size()) {
        missShaderBindingTable.destroy();
      }
      if (missShaderBindingTable.buffer == VK_NULL_HANDLE) {
        missShaderBindingTable = Buffer(physicalDevice, logicalDevice,
          bufferUsageFlags, memoryUsageFlags, recordSize * missPrograms.size());
      }
      missShaderBindingTable.map();
      memcpy(missShaderBindingTable.mapped,
        shaderHandleStorage.data() + handleSize * numRayGens,
        handleSize);
      for (uint32_t idx = 0; idx < missPrograms.size(); ++idx) {
        MissProg *missprog = missPrograms[idx];
        size_t stride = recordSize;
        size_t offset = stride * idx + handleSize /* params start after shader identifier */ ;
        uint8_t* params = ((uint8_t*) (missShaderBindingTable.mapped)) + offset;
        for (auto &var : missprog->vars) {
          size_t varOffset = var.second.decl.offset;
          size_t varSize = getSize(var.second.decl.type);
          std::cout<<"Embedding " << var.first << " into missprog SBT record. Size: " << varSize << " Offset: " << varOffset << std::endl;
          memcpy(params + varOffset, var.second.data, varSize);
        }
      }
      missShaderBindingTable.unmap();

      // Hit records
      if (hitShaderBindingTable.size != recordSize * 1 /*TODO!*/) {
        hitShaderBindingTable.destroy();
      }
      if (hitShaderBindingTable.buffer == VK_NULL_HANDLE) {
        hitShaderBindingTable = Buffer(physicalDevice, logicalDevice, bufferUsageFlags, memoryUsageFlags, recordSize);
      }
      hitShaderBindingTable.map();
      memcpy(hitShaderBindingTable.mapped,
          shaderHandleStorage.data() + handleSize * (numRayGens + numMissProgs),
          handleSize);

      // TODO: hit programs...
      hitShaderBindingTable.unmap();
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

      // auto loadShader = [context](std::string fileName, VkShaderStageFlagBits stage)-> VkPipelineShaderStageCreateInfo
      // {

      //   auto loadShader = [](const char *fileName, VkDevice device) -> VkShaderModule
      // 	{
      // 		std::ifstream is(fileName, std::ios::binary | std::ios::in | std::ios::ate);

      // 		if (is.is_open())
      // 		{
      // 			size_t size = is.tellg();
      // 			is.seekg(0, std::ios::beg);
      // 			char* shaderCode = new char[size];
      // 			is.read(shaderCode, size);
      // 			is.close();

      // 			assert(size > 0);

      // 			VkShaderModule shaderModule;
      // 			VkShaderModuleCreateInfo moduleCreateInfo{};
      // 			moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      // 			moduleCreateInfo.codeSize = size;
      // 			moduleCreateInfo.pCode = (uint32_t*)shaderCode;

      // 			VK_CHECK_RESULT(vkCreateShaderModule(device, &moduleCreateInfo, NULL, &shaderModule));

      // 			delete[] shaderCode;

      // 			return shaderModule;
      // 		}
      // 		else
      // 		{
      // 			std::cerr << "Error: Could not open shader file \"" << fileName << "\"" << "\n";
      // 			return VK_NULL_HANDLE;
      // 		}
      // 	};

      //   VkPipelineShaderStageCreateInfo shaderStage = {};
      //   shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      //   shaderStage.stage = stage;
      //   shaderStage.module = loadShader(fileName.c_str(), context->logicalDevice);
      //   shaderStage.pName = "main";
      //   assert(shaderStage.module != VK_NULL_HANDLE);
      //   context->shaderModules.push_back(shaderStage.module);
      //   return shaderStage;
      // };

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
  };
}


VKRT_API VKRTContext vkrtContextCreate(int32_t *requestedDeviceIDs,
                                    int      numRequestedDevices)
{
  LOG_API_CALL();
  vkrt::Context *context = new vkrt::Context(requestedDeviceIDs, numRequestedDevices);
  LOG("context created...");
  return (VKRTContext)context;
}

VKRT_API void vkrtContextDestroy(VKRTContext _context)
{
  LOG_API_CALL();
  vkrt::Context *context = (vkrt::Context*)_context;
  // context->releaseAll();
  delete context;
  LOG("context destroyed...");
}

VKRT_API VKRTModule vkrtModuleCreate(VKRTContext _context, const char* spvCode)
{
  LOG_API_CALL();

  vkrt::Context *context = (vkrt::Context*)_context;

  vkrt::Module *module = new vkrt::Module(spvCode);

  LOG("module created...");
  return (VKRTModule)module;
}

VKRT_API VKRTRayGen
vkrtRayGenCreate(VKRTContext _context,
                 VKRTModule  _module,
                 const char  *programName,
                 size_t       sizeOfVarStruct,
                 VKRTVarDecl *vars,
                 int          numVars)
{
  LOG_API_CALL();
  vkrt::Context *context = (vkrt::Context*)_context;
  vkrt::Module *module = (vkrt::Module*)_module;

  vkrt::RayGen *raygen = new vkrt::RayGen(
    context->logicalDevice, module, programName,
    sizeOfVarStruct, context->checkAndPackVariables(vars, numVars));

  context->raygenPrograms.push_back(raygen);

  LOG("raygen created...");
  return (VKRTRayGen)raygen;
}

VKRT_API void
vkrtRayGenRelease(VKRTRayGen _rayGen)
{
  LOG_API_CALL();
  vkrt::RayGen *rayGen = (vkrt::RayGen*)_rayGen;
  delete rayGen;
  LOG("raygen destroyed...");
}

VKRT_API VKRTMissProg
vkrtMissProgCreate(VKRTContext _context,
                   VKRTModule  _module,
                   const char  *programName,
                   size_t       sizeOfVarStruct,
                   VKRTVarDecl *vars,
                   int          numVars)
{
  LOG_API_CALL();
  vkrt::Context *context = (vkrt::Context*)_context;
  vkrt::Module *module = (vkrt::Module*)_module;

  vkrt::MissProg *missProg = new vkrt::MissProg(
    context->logicalDevice, module, programName,
    sizeOfVarStruct, context->checkAndPackVariables(vars, numVars));

  context->missPrograms.push_back(missProg);

  LOG("miss program created...");
  return (VKRTMissProg)missProg;
}


/*! sets the given miss program for the given ray type */
VKRT_API void
vkrtMissProgSet(VKRTContext  _context,
               int rayType,
               VKRTMissProg missProgToUse)
{
  VKRT_NOTIMPLEMENTED;
}

VKRT_API void
vkrtMissProgRelease(VKRTMissProg _missProg)
{
  LOG_API_CALL();
  vkrt::MissProg *missProg = (vkrt::MissProg*)_missProg;
  delete missProg;
  LOG("miss program destroyed...");
}

VKRT_API VKRTBuffer
vkrtHostPinnedBufferCreate(VKRTContext _context, VKRTDataType type, size_t count)
{
  LOG_API_CALL();
  const VkBufferUsageFlags bufferUsageFlags =
    // means we can get this buffer's address with vkGetBufferDeviceAddress
    VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags =
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | // mappable to host with vkMapMemory
    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT; // means "flush" and "invalidate"  not needed

  vkrt::Context *context = (vkrt::Context*)_context;
  vkrt::Buffer *buffer = new vkrt::Buffer(
    context->physicalDevice, context->logicalDevice,
    bufferUsageFlags, memoryUsageFlags,
    getSize(type) * count
  );

  // Pin the buffer to the host
  buffer->map();
  
  LOG("buffer created");
  return (VKRTBuffer)buffer;
}

VKRT_API void
vkrtBufferRelease(VKRTBuffer _buffer)
{
  LOG_API_CALL();
  vkrt::Buffer *buffer = (vkrt::Buffer*)_buffer;
  buffer->destroy();
  LOG("buffer released");
}

VKRT_API void *
vkrtBufferGetPointer(VKRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  vkrt::Buffer *buffer = (vkrt::Buffer*)_buffer;
  return buffer->mapped;
}

VKRT_API void
vkrtBufferMap(VKRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  vkrt::Buffer *buffer = (vkrt::Buffer*)_buffer;
  buffer->map();
}

VKRT_API void
vkrtBufferUnmap(VKRTBuffer _buffer, int deviceID)
{
  LOG_API_CALL();
  vkrt::Buffer *buffer = (vkrt::Buffer*)_buffer;
  buffer->unmap();
}

VKRT_API void vkrtBuildPrograms(VKRTContext _context)
{
  VKRT_NOTIMPLEMENTED;
}



VKRT_API void vkrtBuildPipeline(VKRTContext _context)
{
  LOG_API_CALL();
  vkrt::Context *context = (vkrt::Context*)_context;
  context->buildPipeline();
  LOG("pipeline created...");
}

VKRT_API void vkrtBuildSBT(VKRTContext _context,
                           VKRTBuildSBTFlags flags)
{
  LOG_API_CALL();
  vkrt::Context *context = (vkrt::Context*)_context;
  context->buildSBT();
  LOG("SBT created...");
}

/*! Executes a ray tracing pipeline with the given raygen program.
  This call will block until the raygen program returns. */
VKRT_API void
vkrtRayGenLaunch2D(VKRTContext _context, VKRTRayGen _rayGen, int dims_x, int dims_y)
{
  LOG_API_CALL();
  vkrtRayGenLaunch3D(_context, _rayGen,dims_x,dims_y,1);
}

/*! 3D-launch variant of \see vkrtRayGenLaunch2D */
VKRT_API void
vkrtRayGenLaunch3D(VKRTContext _context, VKRTRayGen _rayGen, int dims_x, int dims_y, int dims_z)
{
  LOG_API_CALL();
  assert(_rayGen);

  vkrt::Context *context = (vkrt::Context*)_context;
  vkrt::RayGen *raygen = (vkrt::RayGen*)_rayGen;
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
		return vkrt::vkGetBufferDeviceAddressKHR(device, &bufferDeviceAI);
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

  VkStridedDeviceAddressRegionKHR raygenShaderSbtEntry{};
  raygenShaderSbtEntry.deviceAddress = getBufferDeviceAddress(
    context->logicalDevice, context->raygenShaderBindingTable.buffer);
  raygenShaderSbtEntry.stride = recordSize;
  raygenShaderSbtEntry.size = raygenShaderSbtEntry.stride * 1; // only one

  VkStridedDeviceAddressRegionKHR missShaderSbtEntry{};
  missShaderSbtEntry.deviceAddress = getBufferDeviceAddress(
    context->logicalDevice, context->missShaderBindingTable.buffer);
  missShaderSbtEntry.stride = recordSize;
  missShaderSbtEntry.size = missShaderSbtEntry.stride * 1; // only one

  VkStridedDeviceAddressRegionKHR hitShaderSbtEntry{};
  hitShaderSbtEntry.deviceAddress = getBufferDeviceAddress(
    context->logicalDevice, context->hitShaderBindingTable.buffer);
  hitShaderSbtEntry.stride = recordSize;
  hitShaderSbtEntry.size = hitShaderSbtEntry.stride * 1; // only one

  VkStridedDeviceAddressRegionKHR callableShaderSbtEntry{}; // empty
  // callableShaderSbtEntry.stride = handleSizeAligned;
  // callableShaderSbtEntry.size = handleSizeAligned;

  vkrt::vkCmdTraceRaysKHR(
    context->graphicsCommandBuffer,
    &raygenShaderSbtEntry,
    &missShaderSbtEntry,
    &hitShaderSbtEntry,
    &callableShaderSbtEntry,
    dims_x,
    dims_y,
    dims_z);

  err = vkEndCommandBuffer(context->graphicsCommandBuffer);
  if (err) VKRT_RAISE("failed to end command buffer! : \n" + errorString(err));

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
  if (err) VKRT_RAISE("failed to submit to queue! : \n" + errorString(err));

  err = vkQueueWaitIdle(context->graphicsQueue);
  if (err) VKRT_RAISE("failed to wait for queue idle! : \n" + errorString(err));
}




// ------------------------------------------------------------------
// setters for "meta" types
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
// VKRT_API void vkrtRayGenSetTexture(VKRTRayGen raygen, const char *name, VKRTTexture val);
// VKRT_API void vkrtRayGenSetPointer(VKRTRayGen raygen, const char *name, const void *val);
VKRT_API void vkrtRayGenSetBuffer(VKRTRayGen _rayGen, const char *name, VKRTBuffer _val)
{
  LOG_API_CALL();
  vkrt::RayGen *raygen = (vkrt::RayGen*)_rayGen;
  assert(raygen);

  vkrt::Buffer *val = (vkrt::Buffer*)_val;
  assert(val);

  // 1. Figure out if the variable "name" exists (Maybe through a dictionary?)
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // The found variable must be a buffer
  assert(raygen->vars[name].decl.type == VKRT_BUFFER);

  // Buffer pointers are 64 bits
  size_t size = sizeof(uint64_t);

  // 3. Assign the value to that variable
  VkDeviceAddress addr = val->address;
  memcpy(raygen->vars[name].data, &addr, size);
}

// VKRT_API void vkrtRayGenSetGroup(VKRTRayGen raygen, const char *name, VKRTGroup val);
VKRT_API void vkrtRayGenSetRaw(VKRTRayGen _rayGen, const char *name, const void *val)
{
  LOG_API_CALL();
  vkrt::RayGen *raygen = (vkrt::RayGen*)_rayGen;
  assert(raygen);

  // 1. Figure out if the variable "name" exists (Maybe through a dictionary?)
  assert(raygen->vars.find(std::string(name)) != raygen->vars.end());

  // 2. Get the expected size for this variable
  size_t size = getSize(raygen->vars[name].decl.type);

  // 3. Assign the value to that variable
  memcpy(raygen->vars[name].data, val, size);
}

// // setters for variables on "Geom"s
// VKRT_API void vkrtGeomSetTexture(VKRTGeom obj, const char *name, VKRTTexture val);
// VKRT_API void vkrtGeomSetPointer(VKRTGeom obj, const char *name, const void *val);
// VKRT_API void vkrtGeomSetBuffer(VKRTGeom obj, const char *name, VKRTBuffer val);
// VKRT_API void vkrtGeomSetGroup(VKRTGeom obj, const char *name, VKRTGroup val);
// VKRT_API void vkrtGeomSetRaw(VKRTGeom obj, const char *name, const void *val);

// // setters for variables on "Params"s
// VKRT_API void vkrtParamsSetTexture(VKRTParams obj, const char *name, VKRTTexture val);
// VKRT_API void vkrtParamsSetPointer(VKRTParams obj, const char *name, const void *val);
// VKRT_API void vkrtParamsSetBuffer(VKRTParams obj, const char *name, VKRTBuffer val);
// VKRT_API void vkrtParamsSetGroup(VKRTParams obj, const char *name, VKRTGroup val);
// VKRT_API void vkrtParamsSetRaw(VKRTParams obj, const char *name, const void *val);

// setters for variables on "MissProg"s
// VKRT_API void vkrtMissProgSetTexture(VKRTMissProg missprog, const char *name, VKRTTexture val);
// VKRT_API void vkrtMissProgSetPointer(VKRTMissProg missprog, const char *name, const void *val);
// VKRT_API void vkrtMissProgSetBuffer(VKRTMissProg missprog, const char *name, VKRTBuffer val);
// VKRT_API void vkrtMissProgSetGroup(VKRTMissProg missprog, const char *name, VKRTGroup val);
VKRT_API void vkrtMissProgSetRaw(VKRTMissProg _missProg, const char *name, const void *val)
{
  LOG_API_CALL();
  vkrt::MissProg *missProg = (vkrt::MissProg*)_missProg;
  assert(missProg);

  // 1. Figure out if the variable "name" exists (Maybe through a dictionary?)
  assert(missProg->vars.find(std::string(name)) != missProg->vars.end());

  // 2. Get the expected size for this variable
  size_t size = getSize(missProg->vars[name].decl.type);

  // 3. Assign the value to that variable
  memcpy(missProg->vars[name].data, val, size);
}
