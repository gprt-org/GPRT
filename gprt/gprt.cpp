// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <assert.h>
#include <climits>
#include <fstream>
#include <gprt_host.h>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>

#include <regex>

#ifdef __GNUC__
#include <execinfo.h>
#include <signal.h>
#include <sys/time.h>

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

// For SPIRV reflection
#include "spirv_reflect.h"

// library for image output
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

// For user interface
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

#define VENDOR_ID_AMD 0x1002
#define VENDOR_ID_ImgTec 0x1010
#define VENDOR_ID_NVIDIA 0x10DE
#define VENDOR_ID_ARM 0x13B5
#define VENDOR_ID_Qualcomm 0x5143
#define VENDOR_ID_INTEL 0x8086
static inline std::string getVendorString(uint32_t vendorID) {
  switch(vendorID) {
    case VENDOR_ID_AMD: return "AMD";
    case VENDOR_ID_ImgTec: return "ImgTec";
    case VENDOR_ID_NVIDIA: return "NVIDIA";
    case VENDOR_ID_ARM: return "ARM";
    case VENDOR_ID_Qualcomm: return "Qualcomm";
    case VENDOR_ID_INTEL: return "INTEL";
    default: return std::to_string(vendorID);
  };
}

// For advanced vulkan memory allocation
#define VMA_VULKAN_VERSION 1004000   // Vulkan 1.4
#define VMA_IMPLEMENTATION
// #define VMA_DEBUG_LOG printf
#include "vma/vk_mem_alloc.h"

// For FFX radix sort
#include "gprt_sort.h"

// For software fallbacks for various primitive types
#include "gprt_fallbacks.h"

// For DLSS
#include "nvsdk_ngx_vk.h"
#include "nvsdk_ngx_helpers.h"
#include "nvsdk_ngx_helpers_vk.h"

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

  uint32_t numRayTypes = 1;

  // On AMD, might require RADV driver...
  uint32_t rayRecursionDepth = 1;

  uint32_t maxRayPayloadSize = 32;

  // Setting to 4, to allow for RSTW attributes
  uint32_t maxRayHitAttributeSize = 4;
  
  uint32_t internalAdditionalSize = 128;
  uint32_t raygenRecordSize = 1024;
  
  uint32_t hitRecordSize = 256;
  uint32_t missRecordSize = 256;
  uint32_t callableRecordSize = 256;

  uint32_t maxDescriptorCount = 256;

  // Not all GPUs support the RT pipeline.
  // Currently we assume this is true, but down the road would be nice to make optional.
  bool rayTracingPipeline = true;

  /** Ray queries enable inline ray tracing.
   * Not supported by some platforms like the A100, so requesting is important. */
  bool rayQueries = false;

  bool invocationReordering = false;
  bool linearSweptSpheres = true;

  bool debugPrintf = true;

  // An abstraction over DLSS RR, FSR4, etc
  struct AIDenoiserProperties {
    bool requested = false;
    GPRTDenoiseFlags flags = GPRT_DENOISE_FLAGS_NONE; // mirrors the create flags of DLSS
    uint32_t outputWidth;
    uint32_t outputHeight;
  } aiDenoiser;

  /*! returns whether logging is enabled */
  inline static bool logging() {
    return requestedFeatures.debugPrintf;
  }
} requestedFeatures;

#if defined(_MSC_VER)
//&& !defined(__PRETTY_FUNCTION__)
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

// For error handling
namespace detail {
inline static std::string
backtrace() {
#ifdef __GNUC__
  static const int max_frames = 16;

  void *buffer[max_frames] = {0};
  int cnt = ::backtrace(buffer, max_frames);

  char **symbols = backtrace_symbols(buffer, cnt);

  if (symbols) {
    std::stringstream str;
    for (int n = 1; n < cnt; ++n)   // skip the 1st entry (address of this function)
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

inline void
gprtRaise_impl(std::string str) {
  fprintf(stderr, "%s\n", str.c_str());
#ifdef WIN32
  if (IsDebuggerPresent())
    DebugBreak();
  else
    assert(false);
#else
#ifndef NDEBUG
  std::string bt = ::detail::backtrace();
  fprintf(stderr, "%s\n", bt.c_str());
#endif
  raise(SIGINT);
#endif
}
}   // namespace detail

#define GPRT_RAISE(MSG) ::detail::gprtRaise_impl(MSG);

#define GPRT_NOTIMPLEMENTED                                                                                            \
  {                                                                                                                    \
    std::cerr << std::string(__PRETTY_FUNCTION__) << " not implemented" << std::endl;                                  \
    assert(false);                                                                                                     \
  };

#if 1
#define LOG_API_CALL() /* ignore */
#else
#define LOG_API_CALL() std::cout << "% " << __FUNCTION__ << "(...)" << std::endl;
#endif

#define LOG_VERBOSE(message)                                                                                           \
  if (RequestedFeatures::logging())                                                                                    \
  std::cout << GPRT_TERMINAL_CYAN << "#gprt verbose:  " << message << GPRT_TERMINAL_DEFAULT << std::endl

#define LOG_INFO(message)                                                                                              \
  if (RequestedFeatures::logging())                                                                                    \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE << "#gprt info:  " << message << GPRT_TERMINAL_DEFAULT << std::endl

#define LOG_PRINTF(message) std::cout << GPRT_TERMINAL_GREEN << message << GPRT_TERMINAL_DEFAULT;

#define LOG_WARNING(message)                                                                                           \
  if (RequestedFeatures::logging())                                                                                    \
  std::cout << GPRT_TERMINAL_YELLOW << "#gprt warn:  " << message << GPRT_TERMINAL_DEFAULT << std::endl

#define LOG_ERROR(message)                                                                                             \
  {                                                                                                                    \
    if (RequestedFeatures::logging())                                                                                  \
      std::cout << GPRT_TERMINAL_RED << "#gprt error: " << message << GPRT_TERMINAL_DEFAULT << std::endl;              \
    GPRT_RAISE(message)                                                                                                \
  }

std::string
errorString(VkResult errorCode) {
  switch (errorCode) {
#define STR(r)                                                                                                         \
  case VK_##r:                                                                                                         \
    return #r
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
    STR(ERROR_OUT_OF_POOL_MEMORY);
#undef STR
  default:
    return "UNKNOWN_ERROR";
  }
}

#define VK_CHECK_RESULT(f)                                                                                             \
  {                                                                                                                    \
    VkResult res = (f);                                                                                                \
    if (res != VK_SUCCESS) {                                                                                           \
      std::cout << "Fatal : VkResult is \"" << errorString(res) << "\" in " << __FILE__ << " at line " << __LINE__     \
                << "\n";                                                                                               \
      assert(res == VK_SUCCESS);                                                                                       \
    }                                                                                                                  \
  }

VKAPI_ATTR VkBool32 VKAPI_CALL
debugUtilsMessengerCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                            VkDebugUtilsMessageTypeFlagsEXT messageType,
                            const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData) {
  if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) {
    LOG_VERBOSE(pCallbackData->pMessage);
  } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) {
    // Check to see if it's a GPU printf
    std::string message = std::string(pCallbackData->pMessage);
    if (message.find("MessageID =") != std::string::npos) {
      // Find the position of the first "|" after the MessageID
      size_t pos = message.find("|", message.find("MessageID ="));
      if (pos != std::string::npos) {
        // Skip the prefix up to the second "|"
        message = message.substr(pos + 2);   // +2 to account for "| "
      }
      std::cout << message;   // Expect the user to include \n
    } else {
      LOG_INFO(pCallbackData->pMessage);
    }
  } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
    LOG_WARNING(pCallbackData->pMessage);
  } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    LOG_ERROR(pCallbackData->pMessage);
  }

  // The return value of this callback controls whether the Vulkan call that
  // caused the validation message will be aborted or not We return VK_FALSE as
  // we DON'T want Vulkan calls that cause a validation message to abort If you
  // instead want to have calls abort, pass in VK_TRUE and the function will
  // return VK_ERROR_VALIDATION_FAILED_EXT
  return VK_FALSE;
}

// Contains definitions for internal entry points
// extern std::vector<uint8_t> scanDeviceCode;
extern GPRTProgram sortDeviceCode;
extern GPRTProgram fallbacksDeviceCode;

// forward declarations...
struct Context;
struct Geom;
struct Accel;
struct Texture;
struct RayGen;
struct Miss;
struct Callable;
struct Compute;
struct Module;
struct ImageResource;
struct Buffer;
struct GeomType;
struct TriangleGeom;
struct TriangleGeomType;
struct SphereGeom;
struct SphereGeomType;
struct LSSGeom;
struct LSSGeomType;
struct SolidGeom;
struct SolidGeomType;
struct AABBGeom;
struct AABBGeomType;
struct NNPointGeom;
struct NNPointGeomType;
struct NNEdgeGeom;
struct NNEdgeGeomType;
struct NNTriangleGeom;
struct NNTriangleGeomType;

namespace gprt {
PFN_vkGetBufferDeviceAddressKHR vkGetBufferDeviceAddress;
PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructure;
PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructure;
PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizes;
PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddress;
PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructures;
PFN_vkCmdCopyAccelerationStructureKHR vkCmdCopyAccelerationStructure;
PFN_vkBuildAccelerationStructuresKHR vkBuildAccelerationStructures;
PFN_vkCopyAccelerationStructureKHR vkCopyAccelerationStructure;
PFN_vkCmdTraceRaysKHR vkCmdTraceRays;
PFN_vkGetRayTracingShaderGroupHandlesKHR vkGetRayTracingShaderGroupHandles;
PFN_vkCreateRayTracingPipelinesKHR vkCreateRayTracingPipelines;
PFN_vkCmdWriteAccelerationStructuresPropertiesKHR vkCmdWriteAccelerationStructuresProperties;

PFN_vkCreateDebugUtilsMessengerEXT vkCreateDebugUtilsMessengerEXT;
PFN_vkDestroyDebugUtilsMessengerEXT vkDestroyDebugUtilsMessengerEXT;
VkDebugUtilsMessengerEXT debugUtilsMessenger;

// Note, the following were deprecated and shouldn't be used
// PFN_vkCreateDebugReportCallbackEXT vkCreateDebugReportCallbackEXT;
// PFN_vkDestroyDebugReportCallbackEXT vkDestroyDebugReportCallbackEXT;
// VkDebugReportCallbackEXT debugReportCallback;
}   // namespace gprt

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

struct Context {
  // For convenience, an opaque handle to the context
  GPRTContext context = (GPRTContext) this;

  VkApplicationInfo appInfo;

  // Vulkan instance, stores all per-application states
  VkInstance instance;
  std::vector<VkLayerProperties> supportedInstanceLayers;
  std::vector<std::string> supportedInstanceLayerNames;
  std::vector<VkExtensionProperties> supportedInstanceExtensions;
  std::vector<std::string> supportedInstanceExtensionNames;

  // optional windowing features
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  GLFWwindow *window = nullptr;
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

  struct AIDenoising {
    const uint64_t kAppID = 231313132;
    NVSDK_NGX_Parameter* ngxParameters = nullptr;
    NVSDK_NGX_Handle* ngxHandle = nullptr;

    struct OptimalSettings
    {
        float sharpness;
        uint2 optimalRenderSize;
        uint2 minRenderSize;
        uint2 maxRenderSize;
    } optimalSettings;

    std::string resultToString(NVSDK_NGX_Result result)
    {
        char buf[1024];
        snprintf(buf, sizeof(buf), "(code: 0x%08x, info: %ls)", result, GetNGXResultAsString(result));
        buf[sizeof(buf) - 1] = '\0';
        return std::string(buf);
    }
  } aiDenoising;

  struct ImGuiData {
    uint32_t width = -1;
    uint32_t height = -1;
    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkFramebuffer frameBuffer = VK_NULL_HANDLE;
    Texture *colorAttachment = nullptr;
    Texture *depthAttachment = nullptr;
  } imgui;

  // Physical device (GPU) that Vulkan will use
  VkPhysicalDevice physicalDevice;
  // Stores physical device properties (for e.g. checking device limits)
  VkPhysicalDeviceProperties deviceProperties;
  VkPhysicalDeviceProperties2 deviceProperties2;
  VkPhysicalDeviceSubgroupProperties subgroupProperties;
  VkPhysicalDeviceRayTracingPipelinePropertiesKHR rayTracingPipelineProperties;
  VkPhysicalDeviceAccelerationStructureFeaturesKHR accelerationStructureFeatures;
  // Stores the features available on the selected physical device (for e.g.
  // checking if a feature is available)
  VkPhysicalDeviceFeatures deviceFeatures;
  VkPhysicalDeviceFeatures2 deviceFeatures2;
  VkPhysicalDeviceVulkan11Features deviceVulkan11Features;
  VkPhysicalDeviceVulkan12Features deviceVulkan12Features;
  VkPhysicalDeviceShaderAtomicFloatFeaturesEXT atomicFloatFeatures;
  VkPhysicalDeviceRayTracingInvocationReorderFeaturesNV invocationReorderFeatures;
  #ifdef VK_NV_ray_tracing_linear_swept_spheres
  VkPhysicalDeviceRayTracingLinearSweptSpheresFeaturesNV linearSweptSpheresFeatures;
  #endif
  VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtPipelineFeatures;
  VkPhysicalDeviceRayQueryFeaturesKHR rtQueryFeatures;
  VkPhysicalDeviceMutableDescriptorTypeFeaturesEXT mutableDescriptorFeatures;
  VkPhysicalDeviceShaderClockFeaturesKHR shaderClockFeatures;

  // Stores all available memory (type) properties for the physical device
  VkPhysicalDeviceMemoryProperties deviceMemoryProperties;

  // For handling vulkan memory allocation
  VmaAllocator allocator;

  /** @brief Queue family properties of the chosen physical device */
  std::vector<VkQueueFamilyProperties> queueFamilyProperties;

  /** @brief Contains queue family indices */
  struct {
    uint32_t graphics;
    uint32_t compute;
    uint32_t transfer;
  } queueFamilyIndices;

  VkCommandBuffer graphicsCommandBuffer;
  VkCommandBuffer computeCommandBuffer;
  VkCommandBuffer transferCommandBuffer;

  /** @brief List of extensions supported by the chosen physical device */
  std::vector<std::string> supportedExtensions;

  /** @brief Set of device extensions to be enabled for this example (must be
   * set in the derived constructor) */
  std::vector<const char *> enabledDeviceExtensions;
  std::vector<const char *> enabledInstanceExtensions;
  /** @brief Optional pNext structure for passing extension structures to device
   * creation */
  void *deviceCreatepNextChain = nullptr;
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
  VkQueryPool compactedSizeQueryPool;
  bool queryRequested = false;

  /** @brief Pipeline stages used to wait at for graphics queue submissions */
  VkPipelineStageFlags submitPipelineStages =
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;   // not sure I need this one
  // Contains command buffers and semaphores to be presented to the queue
  VkSubmitInfo submitInfo{};
  // Command buffers used for rendering
  // std::vector<VkCommandBuffer> drawCmdBuffers;
  // Global render pass for frame buffer writes
  VkRenderPass renderPass = VK_NULL_HANDLE;
  // List of available frame buffers (same as number of swap chain images)
  std::vector<VkFramebuffer> frameBuffers;
  // Active frame buffer index
  uint32_t currentBuffer = 0;

  // Pipeline cache object
  VkPipelineCache pipelineCache;

  // ray tracing pipeline and layout
  bool raytracingPipelineOutOfDate = true;
  bool computePipelinesOutOfDate = true;

  VkPipeline raytracingPipeline = VK_NULL_HANDLE;
  VkPipelineLayout raytracingPipelineLayout = VK_NULL_HANDLE;

  std::vector<Module *> modules;
  std::vector<Compute *> computes;
  std::vector<RayGen *> raygens;
  std::vector<Miss *> misses;
  std::vector<Callable *> callables;
  std::vector<GeomType *> geomTypes;
  std::vector<Buffer *> buffers;
  std::vector<ImageResource *> imageResources;
  std::vector<Geom *> geoms;
  std::vector<Accel *> accels;

  VkDescriptorPoolSize descriptorPoolSize = {};
  VkDescriptorPool descriptorPool = VK_NULL_HANDLE;

  VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
  VkDescriptorSet descriptorSet = VK_NULL_HANDLE;

  VkDescriptorPool imguiPool = VK_NULL_HANDLE;

  std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups{};
  Buffer *raygenTable = nullptr;
  Buffer *missTable = nullptr;
  Buffer *callableTable = nullptr;
  Buffer *hitgroupTable = nullptr;

  std::map<std::string, Compute *> internalComputePrograms;

  Module *radixSortModule = nullptr;
  // Module *scanModule = nullptr;
  Module *fallbacksModule = nullptr;

  VkPipelineShaderStageCreateInfo LSSIntersectionShaderStage;

  // TODO, we can probably refactor this...
  struct SortStages {
    Stage Count;
    Stage CountReduce;
    Stage Scan;
    Stage ScanAdd;
    Stage Scatter;
    Stage ScatterPayload;

    // For now, copied over from sample
    VkDescriptorSetLayout m_SortDescriptorSetLayoutInputOutputs;
    VkDescriptorSetLayout m_SortDescriptorSetLayoutScan;
    VkDescriptorSetLayout m_SortDescriptorSetLayoutScratch;

    VkDescriptorSet m_SortDescriptorSetInputOutput[2];
    VkDescriptorSet m_SortDescriptorSetScanSets[2];
    VkDescriptorSet m_SortDescriptorSetScratch;

    VkPipelineLayout layout;

    VkDescriptorPool pool = VK_NULL_HANDLE;
  };
  SortStages sortStages;

  void freeDebugCallback(VkInstance instance);

  size_t getNumHitRecords();

  void enumerateInstanceValidationLayers();
  void enumerateInstanceExtensions();

  Context(int32_t *requestedDeviceIDs, int numRequestedDevices);

  void destroy();

  ~Context() {};

  void buildSBT(GPRTBuildSBTFlags flags);

  // void buildPrograms()
  // {
  //   // At the moment, we don't actually build our programs here.
  // }

  // Todo, refactor this code...
  void setupInternalPrograms();

  void destroyInternalPrograms();

  VkCommandBuffer beginSingleTimeCommands(VkCommandPool pool);

  void endSingleTimeCommands(VkCommandBuffer commandBuffer, VkCommandPool pool, VkQueue queue);

  void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout);

  // For ImGui
  void setRasterAttachments(Texture *colorTexture, Texture *depthTexture);

  void rasterizeGui();
};

struct Module {
  Context* context;

  int32_t virtualAddress = -1;

  spv_reflect::ShaderModule shaderModule;
  std::vector<uint32_t> binary;

  struct DescriptorBinding {
    uint32_t bindingNumber;
    std::string variableName;
    SpvReflectDescriptorType type;
    uint32_t count;
  };

  struct DescriptorSet {
    uint32_t setNumber;
    uint32_t bindingCount;
    std::map<uint32_t, DescriptorBinding> bindings;
  };

  struct EntryPoint {
    std::string name;
    uint32_t descriptorSetCount;
    std::map<uint32_t, DescriptorSet> descriptorSets;
  };

  std::map<std::string, EntryPoint> EntryPoints;
  std::vector<std::string> EntryPointNames;

  Module(Context* context, GPRTProgram program) {
    this->context = context;

    // Hunt for an existing free virtual address
    for (uint32_t i = 0; i < context->modules.size(); ++i) {
      if (context->modules[i] == nullptr) {
        context->modules[i] = this;
        virtualAddress = i;
        break;
      }
    }
    // If we cant find a free spot, allocate a new one
    if (virtualAddress == -1) {
      context->modules.push_back(this);
      virtualAddress = (uint32_t) context->modules.size() - 1;
    }

    size_t sizeOfProgram = program.size();
    binary.resize(sizeOfProgram / 4);
    memcpy(binary.data(), program.data(), sizeOfProgram);

    shaderModule = spv_reflect::ShaderModule(program.size(), program.data(), SPV_REFLECT_MODULE_FLAG_NONE);

    // Enumerate entry points
    uint32_t numEntryPoints = shaderModule.GetEntryPointCount();
    for (uint32_t eid = 0; eid < numEntryPoints; ++eid) {
      std::string entrypointName = std::string(shaderModule.GetEntryPointName(eid));
      EntryPoint entry = {};
      entry.name = entrypointName;

      // Enumerate descriptor sets for the given entry point
      shaderModule.EnumerateEntryPointDescriptorSets(entrypointName.c_str(), &entry.descriptorSetCount, nullptr);
      std::vector<SpvReflectDescriptorSet *> descriptorSets(entry.descriptorSetCount);
      shaderModule.EnumerateEntryPointDescriptorSets(entrypointName.c_str(), &entry.descriptorSetCount,
                                                     descriptorSets.data());
      for (uint32_t did = 0; did < entry.descriptorSetCount; ++did) {
        DescriptorSet descriptorSet = {};
        descriptorSet.setNumber = descriptorSets[did]->set;

        // Enumerate bindings for the given descriptor set and entry point
        shaderModule.EnumerateEntryPointDescriptorBindings(entrypointName.c_str(), &descriptorSet.bindingCount,
                                                           nullptr);
        std::vector<SpvReflectDescriptorBinding *> bindings(descriptorSet.bindingCount);
        shaderModule.EnumerateEntryPointDescriptorBindings(entrypointName.c_str(), &descriptorSet.bindingCount,
                                                           bindings.data());

        for (uint32_t bid = 0; bid < descriptorSet.bindingCount; ++bid) {
          DescriptorBinding binding = {};
          binding.bindingNumber = bindings[bid]->binding;
          binding.type = bindings[bid]->descriptor_type;
          binding.count = bindings[bid]->count;
          binding.variableName = std::string(bindings[bid]->name);
          descriptorSet.bindings[binding.bindingNumber] = binding;
        }

        entry.descriptorSets[descriptorSet.setNumber] = descriptorSet;
      }

      EntryPoints[entrypointName] = entry;
      EntryPointNames.push_back(entrypointName);
    }
  }

  bool checkForEntrypoint(const char *entrypoint) { return EntryPoints.find(entrypoint) != EntryPoints.end(); }

  void destroy() {
    // Free slot for subsequent use
    context->modules[virtualAddress] = nullptr;
  }
  ~Module() {}
};

struct Buffer {
  Context* context;

  VkPhysicalDeviceMemoryProperties memoryProperties;
  
  /** @brief Usage flags to be filled by external source at buffer creation */
  VkBufferUsageFlags usageFlags;

  bool hostVisible;

  VkBuffer buffer = VK_NULL_HANDLE;
  VmaAllocation allocation;
  VkDeviceAddress deviceAddress = 0;

  // Some operations require buffers to be in a separate array.
  // Instead, we make our own virtual "sampler address space".
  uint32_t virtualAddress = -1;

  struct StagingBuffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VmaAllocation allocation;
  } stagingBuffer;

  VkDeviceSize size = 0;
  VkDeviceSize alignment = 16;
  void *mapped = nullptr;

  VkResult map(VkDeviceSize mapSize = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
    if (mapped)
      return VK_SUCCESS;

    if (hostVisible) {
      vmaInvalidateAllocation(context->allocator, allocation, 0, VK_WHOLE_SIZE);
      return vmaMapMemory(context->allocator, allocation, &mapped);
    } else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for buffer map! : \n" + errorString(err));

      // To do, consider allowing users to specify offsets here...
      VkBufferCopy region;
      region.srcOffset = offset;
      region.dstOffset = 0;
      region.size = (mapSize == VK_WHOLE_SIZE) ? size : mapSize;
      vkCmdCopyBuffer(context->graphicsCommandBuffer, buffer, stagingBuffer.buffer, 1, &region);

      err = vkEndCommandBuffer(context->graphicsCommandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for buffer map! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;
      submitInfo.pWaitDstStageMask = nullptr;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;

      err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(context->graphicsQueue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for buffer map! : \n" + errorString(err));

      vmaInvalidateAllocation(context->allocator, stagingBuffer.allocation, 0, VK_WHOLE_SIZE);
      return vmaMapMemory(context->allocator, stagingBuffer.allocation, &mapped);
    }
  }

  void unmap(VkDeviceSize mapSize = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
    if (!mapped)
      return;

    if (hostVisible) {
      if (mapped) {
        vmaFlushAllocation(context->allocator, allocation, 0, VK_WHOLE_SIZE);
        vmaUnmapMemory(context->allocator, allocation);
        mapped = nullptr;
      }
    } else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for buffer map! : \n" + errorString(err));

      // To do, consider allowing users to specify offsets here...
      VkBufferCopy region;
      region.srcOffset = 0;
      region.dstOffset = offset;
      region.size = (mapSize == VK_WHOLE_SIZE) ? size : mapSize;
      vkCmdCopyBuffer(context->graphicsCommandBuffer, stagingBuffer.buffer, buffer, 1, &region);

      err = vkEndCommandBuffer(context->graphicsCommandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for buffer map! : \n" + errorString(err));

      VkSubmitInfo submitInfo;
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;
      submitInfo.pWaitDstStageMask = nullptr;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;

      err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(context->graphicsQueue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for buffer map! : \n" + errorString(err));

      vmaFlushAllocation(context->allocator, stagingBuffer.allocation, 0, VK_WHOLE_SIZE);
      vmaUnmapMemory(context->allocator, stagingBuffer.allocation);
      mapped = nullptr;
    }
  }

  // flushes from host to device
  void flush() {
    if (hostVisible) {
      vmaFlushAllocation(context->allocator, allocation, 0, VK_WHOLE_SIZE);
    } else {
      vmaFlushAllocation(context->allocator, stagingBuffer.allocation, 0, VK_WHOLE_SIZE);
    }
  }

  // invalidates from device back to host
  void invalidate() {
    // device never directly writes to staging buffer
    vmaInvalidateAllocation(context->allocator, allocation, 0, VK_WHOLE_SIZE);
  }

  VkDeviceAddress getDeviceAddress() {
    VkBufferDeviceAddressInfoKHR info = {};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO_KHR;
    info.buffer = buffer;
    VkDeviceAddress addr = gprt::vkGetBufferDeviceAddress(context->logicalDevice, &info);
    return addr;
  }

  /*! Calls vkDestroy on the buffer, and frees underlying memory */
  void destroy() {
    // Free sampler slot for use by subsequently made buffers
    context->buffers[virtualAddress] = nullptr;

    unmap();

    if (buffer) {
      vmaDestroyBuffer(context->allocator, buffer, allocation);
      // vkDestroyBuffer(device, buffer, nullptr);
      buffer = VK_NULL_HANDLE;
    }
    if (stagingBuffer.buffer) {
      vmaDestroyBuffer(context->allocator, stagingBuffer.buffer, stagingBuffer.allocation);
      // vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
      // vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);
      stagingBuffer.buffer = VK_NULL_HANDLE;
    }
  }

  /* Sets all bytes to 0 */
  void clear() {
    if (hostVisible) {
      if (!mapped)
        map();
      memset(mapped, 0, size);
    } else {
      map();
      memset(mapped, 0, size);
      unmap();
    }
  }

  void resize(size_t bytes, bool preserveContents) {
    // If the size is already okay, do nothing
    if (size == bytes)
      return;

    if (hostVisible) {
      // if we are host visible, we need to create a new buffer before releasing the
      // previous one to preserve values...

      if (preserveContents) {
        // Create the new buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;

        VkBuffer newBuffer;
        VmaAllocation newAllocation;
        VK_CHECK_RESULT(vmaCreateBuffer(context->allocator, &bufferCreateInfo, &allocInfo, &newBuffer, &newAllocation, nullptr));

        // Copy contents from old to new
        VkResult err;
        VkCommandBufferBeginInfo cmdBufInfo{};
        cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
        if (err)
          LOG_ERROR("failed to begin command buffer for buffer resize! : \n" + errorString(err));

        VkBufferCopy region;
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = std::min(size, VkDeviceSize(bytes));
        vkCmdCopyBuffer(context->graphicsCommandBuffer, buffer, newBuffer, 1, &region);

        err = vkEndCommandBuffer(context->graphicsCommandBuffer);
        if (err)
          LOG_ERROR("failed to end command buffer for buffer resize! : \n" + errorString(err));

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.pNext = NULL;
        submitInfo.waitSemaphoreCount = 0;
        submitInfo.pWaitSemaphores = nullptr;
        submitInfo.pWaitDstStageMask = nullptr;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
        submitInfo.signalSemaphoreCount = 0;
        submitInfo.pSignalSemaphores = nullptr;

        err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        if (err)
          LOG_ERROR("failed to submit to queue for buffer resize! : \n" + errorString(err));

        err = vkQueueWaitIdle(context->graphicsQueue);
        if (err)
          LOG_ERROR("failed to wait for queue idle for buffer resize! : \n" + errorString(err));

        // Free old buffer
        vmaUnmapMemory(context->allocator, allocation);
        vmaDestroyBuffer(context->allocator, buffer, allocation);

        // Assign new buffer
        buffer = newBuffer;
        allocation = newAllocation;
        size = bytes;
        deviceAddress = getDeviceAddress();
        vmaInvalidateAllocation(context->allocator, allocation, 0, VK_WHOLE_SIZE);
        vmaMapMemory(context->allocator, allocation, &mapped);
      } else {
        // Not preserving contents, we can delete old host buffer before making new one

        // Free old buffer
        vmaUnmapMemory(context->allocator, allocation);
        vmaDestroyBuffer(context->allocator, buffer, allocation);

        // Create the new buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;

        VK_CHECK_RESULT(vmaCreateBuffer(context->allocator, &bufferCreateInfo, &allocInfo, &buffer, &allocation, nullptr));
        vmaMapMemory(context->allocator, allocation, &mapped);
        size = bytes;
        deviceAddress = getDeviceAddress();
      }

    } else {
      // if we are device visible, we can use the staging buffer to temporarily hold previous values, and free
      // the old buffer before creating a new buffer.
      vmaInvalidateAllocation(context->allocator, allocation, 0, VK_WHOLE_SIZE);

      // Copy existing contents into staging buffer
      if (preserveContents) {
        VkResult err;
        VkCommandBufferBeginInfo cmdBufInfo{};
        cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
        if (err)
          LOG_ERROR("failed to begin command buffer for buffer resize! : \n" + errorString(err));

        VkBufferCopy region;
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = std::min(size, VkDeviceSize(bytes));
        vkCmdCopyBuffer(context->graphicsCommandBuffer, buffer, stagingBuffer.buffer, 1, &region);

        err = vkEndCommandBuffer(context->graphicsCommandBuffer);
        if (err)
          LOG_ERROR("failed to end command buffer for buffer resize! : \n" + errorString(err));

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.pNext = NULL;
        submitInfo.waitSemaphoreCount = 0;
        submitInfo.pWaitSemaphores = nullptr;
        submitInfo.pWaitDstStageMask = nullptr;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
        submitInfo.signalSemaphoreCount = 0;
        submitInfo.pSignalSemaphores = nullptr;

        err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        if (err)
          LOG_ERROR("failed to submit to queue for buffer resize! : \n" + errorString(err));

        err = vkQueueWaitIdle(context->graphicsQueue);
        if (err)
          LOG_ERROR("failed to wait for queue idle for buffer resize! : \n" + errorString(err));
      }

      // Release old device buffer
      vmaDestroyBuffer(context->allocator, buffer, allocation);

      {
        // Create the new buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
        VK_CHECK_RESULT(vmaCreateBuffer(context->allocator, &bufferCreateInfo, &allocInfo, &buffer, &allocation, nullptr));
      }

      if (preserveContents) {
        // Copy contents from old staging buffer to new buffer
        VkResult err;
        VkCommandBufferBeginInfo cmdBufInfo{};
        cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
        if (err)
          LOG_ERROR("failed to begin command buffer for buffer resize! : \n" + errorString(err));

        VkBufferCopy region;
        region.srcOffset = 0;
        region.dstOffset = 0;
        region.size = std::min(size, VkDeviceSize(bytes));
        vkCmdCopyBuffer(context->graphicsCommandBuffer, stagingBuffer.buffer, buffer, 1, &region);

        err = vkEndCommandBuffer(context->graphicsCommandBuffer);
        if (err)
          LOG_ERROR("failed to end command buffer for buffer resize! : \n" + errorString(err));

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.pNext = NULL;
        submitInfo.waitSemaphoreCount = 0;
        submitInfo.pWaitSemaphores = nullptr;
        submitInfo.pWaitDstStageMask = nullptr;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
        submitInfo.signalSemaphoreCount = 0;
        submitInfo.pSignalSemaphores = nullptr;

        err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        if (err)
          LOG_ERROR("failed to submit to queue for buffer resize! : \n" + errorString(err));

        err = vkQueueWaitIdle(context->graphicsQueue);
        if (err)
          LOG_ERROR("failed to wait for queue idle for buffer resize! : \n" + errorString(err));
      }

      size = bytes;
      deviceAddress = getDeviceAddress();

      // Release old staging buffer
      if (mapped) {
        vmaUnmapMemory(context->allocator, stagingBuffer.allocation);
      }
      vmaDestroyBuffer(context->allocator, stagingBuffer.buffer, stagingBuffer.allocation);
      {
        // Create the new staging buffer handle
        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.usage = usageFlags;
        bufferCreateInfo.size = bytes;

        VmaAllocationCreateInfo allocInfo = {};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
        VK_CHECK_RESULT(vmaCreateBuffer(context->allocator, &bufferCreateInfo, &allocInfo, &stagingBuffer.buffer,
                                        &stagingBuffer.allocation, nullptr));
      }

      // if buffer was previously mapped, restore mapped state
      if (mapped) {
        mapped = nullptr;
        map();
      }
    }
  }

  size_t getSize() { return (size_t) size; }

  /* Default Constructor */
  Buffer() {};

  ~Buffer() {};

  Buffer(Context* context, VkBufferUsageFlags _usageFlags, VkMemoryPropertyFlags _memoryPropertyFlags, VkDeviceSize _size, VkDeviceSize _alignment,
         void *data = nullptr) {
    this->context = context;

    // Hunt for an existing free virtual address for this buffer
    for (uint32_t i = 0; i < context->buffers.size(); ++i) {
      if (context->buffers[i] == nullptr) {
        context->buffers[i] = this;
        virtualAddress = i;
        break;
      }
    }
    // If we cant find a free spot in the current buffer list, allocate a new
    // one
    if (virtualAddress == -1) {
      context->buffers.push_back(this);
      virtualAddress = (uint32_t) context->buffers.size() - 1;
    }

    // For performance reasons, round the size up for vectorized/multiword reads
    uint32_t multiwordSize = 16;
    _size = (_size + (multiwordSize - 1) / multiwordSize);

    usageFlags = _usageFlags;
    size = _size;
    alignment = _alignment;
    
    // Check if the buffer can be mapped to a host pointer.
    // If the buffer isn't host visible, this is buffer and requires
    // an additional staging buffer...
    if ((_memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0)
      hostVisible = true;
    else
      hostVisible = false;

    // Create the buffer handle
    VkBufferCreateInfo bufferCreateInfo{};
    bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferCreateInfo.usage = usageFlags;
    bufferCreateInfo.size = size;

    VmaAllocationCreateInfo allocInfo = {};
    if (hostVisible) {
      allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
      allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    } else {
      allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    }

    VK_CHECK_RESULT(vmaCreateBufferWithAlignment(context->allocator, &bufferCreateInfo, &allocInfo, alignment, &buffer,
                                                 &allocation, nullptr));

    // VK_CHECK_RESULT(vkCreateBuffer(logicalDevice, &bufferCreateInfo, nullptr, &buffer));

    if (!hostVisible) {
      const VkBufferUsageFlags bufferUsageFlags =
          // means we can use this buffer to transfer into another
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
          // means we can use this buffer to receive data transferred from
          // another
          VK_BUFFER_USAGE_TRANSFER_DST_BIT;

      VkBufferCreateInfo bufferCreateInfo{};
      bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
      bufferCreateInfo.usage = bufferUsageFlags;
      bufferCreateInfo.size = size;
      // VK_CHECK_RESULT(vkCreateBuffer(logicalDevice, &bufferCreateInfo, nullptr, &stagingBuffer.buffer));

      // VmaAllocationCreateInfo allocInfo = {};
      // allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
      // allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
      // VK_CHECK_RESULT(vmaCreateBufferWithAlignment(allocator, &bufferCreateInfo, &allocInfo, alignment,
      //                                              &stagingBuffer.buffer, &stagingBuffer.allocation, nullptr));
      // std::cout<<std::endl;
      VmaAllocationCreateInfo allocInfo = {};
      allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
      allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
      VK_CHECK_RESULT(vmaCreateBuffer(context->allocator, &bufferCreateInfo, &allocInfo, &stagingBuffer.buffer,
                                      &stagingBuffer.allocation, nullptr));
    }

    // If a pointer to the buffer data has been passed, map the buffer and
    // copy over the data
    if (data != nullptr) {
      map();
      memcpy(mapped, data, size);
      unmap();
    }

    // means we can get this buffer's address with vkGetBufferDeviceAddress
    if ((usageFlags & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) != 0)
      deviceAddress = getDeviceAddress();
  }
};

inline size_t
gprtFormatGetSize(GPRTFormat format) {
  switch (format) {
  case GPRT_FORMAT_R8_UINT:
    return 1;
  case GPRT_FORMAT_R16_UNORM:
    return 2;
  case GPRT_FORMAT_R8G8B8A8_UNORM:
    return 4;
  case GPRT_FORMAT_R8G8B8A8_SRGB:
    return 4;
  case GPRT_FORMAT_R32_SFLOAT:
    return 4;
  case GPRT_FORMAT_D32_SFLOAT:
    return 4;
  case GPRT_FORMAT_R32G32_SFLOAT:
    return 8;
  case GPRT_FORMAT_R32G32B32A32_SFLOAT:
    return 16;
  default:
    throw std::runtime_error("Error, unhandled image format");
    return -1;
  }
}

typedef enum {
  GPRT_IMAGE_RESOURCE_TYPE_UNKNOWN = 0x0,
  GPRT_IMAGE_RESOURCE_TYPE_SAMPLER = 0x1,
  GPRT_IMAGE_RESOURCE_TYPE_TEXTURE_1D = 0x2,
  GPRT_IMAGE_RESOURCE_TYPE_TEXTURE_2D = 0x3,
  GPRT_IMAGE_RESOURCE_TYPE_TEXTURE_3D = 0x4
} ImageResourceType;

struct ImageResource {
  Context *context;

  // Unlike buffers, image resources in Vulkan don't (currently) support device addresses.
  // Some vendors, like NVIDIA, use a separate hardware table for images and samplers.
  // This table stores headers for where memory of textures reside (eg, multiple mip levels),
  // alongside other information for the texture units on how to interpret that texture data.
  //
  // Still, we wish to access image resources in a "bindless" way. To do so, we will
  // create our own "virtual" address space.
  uint32_t address = -1;

  ~ImageResource() {};

  ImageResource(Context *context) {
    this->context = context;

    // Hunt for an existing free address for this image resource
    for (uint32_t i = 0; i < context->imageResources.size(); ++i) {
      if (context->imageResources[i] == nullptr) {
        context->imageResources[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->imageResources.push_back(this);
      address = (uint32_t) context->imageResources.size() - 1;
    }
  };

  virtual void destroy() {
    // Free slot for use by subsequently made image resource
    context->imageResources[address] = nullptr;
  }

  virtual ImageResourceType getType() { return GPRT_IMAGE_RESOURCE_TYPE_UNKNOWN; }
};

struct Texture : public ImageResource {
  VkPhysicalDeviceMemoryProperties memoryProperties;

  /** @brief Usage flags to be filled by external source at image creation */
  VkImageUsageFlags usageFlags;

  /** @brief Memory property flags to be filled by external source at image
   * creation */
  VkMemoryPropertyFlags memoryPropertyFlags;

  bool hostVisible;

  VkImage image = VK_NULL_HANDLE;
  VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
  VkDeviceMemory memory = VK_NULL_HANDLE;

  VkImageView imageView = VK_NULL_HANDLE;

  uint32_t mipLevels;

  VkImageType imageType;

  VkFormat format;
  VkImageAspectFlagBits aspectFlagBits;

  uint32_t width;
  uint32_t height;
  uint32_t depth;
  VkDeviceSize size = 0;
  VkDeviceSize alignment = 0;
  void *mapped = nullptr;

  VkSubresourceLayout subresourceLayout{};

  struct StagingBuffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
  } stagingBuffer;

  ~Texture() {};

  void setImageLayout(VkCommandBuffer cmdbuffer, VkImage image, VkImageLayout oldImageLayout,
                      VkImageLayout newImageLayout, VkImageSubresourceRange subresourceRange,
                      VkPipelineStageFlags srcStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                      VkPipelineStageFlags dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT) {
    // Create an image barrier object
    VkImageMemoryBarrier imageMemoryBarrier{};
    imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;

    // If we're transferring which queue owns this image, we need to set these.
    imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    imageMemoryBarrier.oldLayout = oldImageLayout;
    imageMemoryBarrier.newLayout = newImageLayout;
    imageMemoryBarrier.image = image;
    imageMemoryBarrier.subresourceRange = subresourceRange;

    // Source layouts (old)
    // Source access mask controls actions that have to be finished on the old
    // layout before it will be transitioned to the new layout
    switch (oldImageLayout) {
    case VK_IMAGE_LAYOUT_UNDEFINED:
      // Image layout is undefined (or does not matter)
      // Only valid as initial layout
      // No flags required, listed only for completeness
      imageMemoryBarrier.srcAccessMask = 0;
      break;

    case VK_IMAGE_LAYOUT_PREINITIALIZED:
      // Image is preinitialized
      // Only valid as initial layout for linear images, preserves memory
      // contents Make sure host writes have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
      // Image is a color attachment
      // Make sure any writes to the color buffer have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
      // Image is a depth/stencil attachment
      // Make sure any writes to the depth/stencil buffer have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
      // Image is a transfer source
      // Make sure any reads from the image have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      break;

    case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
      // Image is a transfer destination
      // Make sure any writes to the image have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
      // Image is read by a shader
      // Make sure any shader reads from the image have been finished
      imageMemoryBarrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
      break;
    default:
      // Other source layouts aren't handled (yet)
      break;
    }

    // Target layouts (new)
    // Destination access mask controls the dependency for the new image layout
    switch (newImageLayout) {
    case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
      // Image will be used as a transfer destination
      // Make sure any writes to the image have been finished
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
      // Image will be used as a transfer source
      // Make sure any reads from the image have been finished
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      break;

    case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
      // Image will be used as a color attachment
      // Make sure any writes to the color buffer have been finished
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
      // Image layout will be used as a depth/stencil attachment
      // Make sure any writes to depth/stencil buffer have been finished
      imageMemoryBarrier.dstAccessMask =
          imageMemoryBarrier.dstAccessMask | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      break;

    case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
      // Image will be read in a shader (sampler, input attachment)
      // Make sure any writes to the image have been finished
      if (imageMemoryBarrier.srcAccessMask == 0) {
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
      }
      imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
      break;
    default:
      // Other source layouts aren't handled (yet)
      break;
    }

    // Put barrier inside setup command buffer
    vkCmdPipelineBarrier(cmdbuffer, srcStageMask, dstStageMask, 0, 0, nullptr, 0, nullptr, 1, &imageMemoryBarrier);
  }

  VkResult map(VkDeviceSize mapSize = VK_WHOLE_SIZE, VkDeviceSize offset = 0) {
    if (hostVisible) {
      // Assuming layout is general
      if (mapped)
        return VK_SUCCESS;
      else
        return vkMapMemory(context->logicalDevice, memory, offset, size, 0, &mapped);
    } else {
      // Finding that this causes bugs on Intel ARC. It seems that
      // vkCmdCopyImageToBuffer doesn't respect vulkan fences. We don't need
      // this, at the moment textures can only be written to by the host... But
      // it might be worth filing a bug over...

      // Update: we actually do need this to work in order to download the contents
      // of rasterized images.

      // VkResult err;
      // VkCommandBufferBeginInfo cmdBufInfo{};
      // cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      // err = vkBeginCommandBuffer(commandBuffer, &cmdBufInfo);
      // if (err) LOG_ERROR("failed to begin command buffer for texture map! :
      // \n" + errorString(err));

      // // transition device to a transfer source format
      // setImageLayout(commandBuffer, image, layout,
      // VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, {aspectFlagBits, 0,
      // mipLevels, 0, 1}); layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

      // VkBufferImageCopy copyRegion;
      // copyRegion.imageOffset.x = 0;
      // copyRegion.imageOffset.y = 0;
      // copyRegion.imageOffset.z = 0;
      // copyRegion.imageExtent.width = width;
      // copyRegion.imageExtent.height = height;
      // copyRegion.imageExtent.depth = depth;
      // copyRegion.bufferOffset = 0;
      // copyRegion.bufferRowLength = 0;
      // copyRegion.bufferImageHeight = 0;
      // copyRegion.imageSubresource.aspectMask = aspectFlagBits;
      // copyRegion.imageSubresource.baseArrayLayer = 0;
      // copyRegion.imageSubresource.layerCount = 1;
      // copyRegion.imageSubresource.mipLevel = 0;
      // vkCmdCopyImageToBuffer(commandBuffer, image, layout,
      // stagingBuffer.buffer, 1, &copyRegion);

      // // transition device to previous format
      // setImageLayout(commandBuffer, image, layout,
      // VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, {aspectFlagBits,
      // 0, mipLevels, 0, 1}); layout =
      // VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

      // err = vkEndCommandBuffer(commandBuffer);
      // if (err) LOG_ERROR("failed to end command buffer for texture map! :
      // \n" + errorString(err));

      // VkSubmitInfo submitInfo{};
      // submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      // submitInfo.pNext = NULL;
      // submitInfo.waitSemaphoreCount = 0;
      // submitInfo.pWaitSemaphores =
      // nullptr;//&acquireImageSemaphoreHandleList[currentFrame];
      // submitInfo.pWaitDstStageMask = nullptr;//&pipelineStageFlags;
      // submitInfo.commandBufferCount = 1;
      // submitInfo.pCommandBuffers = &commandBuffer;
      // submitInfo.signalSemaphoreCount = 0;
      // submitInfo.pSignalSemaphores =
      // nullptr;//&writeImageSemaphoreHandleList[currentImageIndex]}; err =
      // vkQueueSubmit(queue, 1, &submitInfo, nullptr); if (err)
      // LOG_ERROR("failed to submit to queue for texture map! : \n" +
      // errorString(err));

      // err = vkQueueWaitIdle(queue);
      // if (err) LOG_ERROR("failed to wait for queue idle for texture map! :
      // \n" + errorString(err));

      // todo, transfer device data to host
      if (mapped)
        return VK_SUCCESS;
      // else return vkMapMemory(device, stagingImage.memory, offset, mapSize,
      // 0, &mapped);
      else
        return vkMapMemory(context->logicalDevice, stagingBuffer.memory, offset, mapSize, 0, &mapped);
    }
  }

  void unmap() {
    if (hostVisible) {
      // assuming layout is general
      if (mapped) {
        vkUnmapMemory(context->logicalDevice, memory);
        mapped = nullptr;
      }
    } else {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for texture map! : \n" + errorString(err));

      // transition device to a transfer destination format
      setImageLayout(context->graphicsCommandBuffer, image, layout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                     {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
      layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

      // copy data
      VkBufferImageCopy copyRegion;
      copyRegion.imageOffset.x = 0;
      copyRegion.imageOffset.y = 0;
      copyRegion.imageOffset.z = 0;
      copyRegion.imageExtent.width = width;
      copyRegion.imageExtent.height = height;
      copyRegion.imageExtent.depth = depth;
      copyRegion.bufferOffset = 0;
      copyRegion.bufferRowLength = 0;
      copyRegion.bufferImageHeight = 0;
      copyRegion.imageSubresource.aspectMask = aspectFlagBits;
      copyRegion.imageSubresource.baseArrayLayer = 0;
      copyRegion.imageSubresource.layerCount = 1;
      copyRegion.imageSubresource.mipLevel = 0;
      vkCmdCopyBufferToImage(context->graphicsCommandBuffer, stagingBuffer.buffer, image, layout, 1, &copyRegion);

      // transition device to an optimal device format
      setImageLayout(context->graphicsCommandBuffer, image, layout, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                     {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
      layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

      err = vkEndCommandBuffer(context->graphicsCommandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for texture map! : \n" + errorString(err));

      VkSubmitInfo submitInfo{};
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for texture map! : \n" + errorString(err));

      err = vkQueueWaitIdle(context->graphicsQueue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for texture map! : \n" + errorString(err));

      // todo, transfer device data to device
      if (mapped) {
        // vkUnmapMemory(device, stagingImage.memory);
        vkUnmapMemory(context->logicalDevice, stagingBuffer.memory);
        mapped = nullptr;
      }
    }
  }

  void clear() {
    VkResult err;
    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for texture mipmap generation! : \n" + errorString(err));

    // Move to a destination optimal format
    setImageLayout(context->graphicsCommandBuffer, image, layout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                   {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});

    // clear the image
    if (aspectFlagBits == VK_IMAGE_ASPECT_DEPTH_BIT) {
      VkClearDepthStencilValue val;
      val.depth = 1.f;
      val.stencil = 0;
      VkImageSubresourceRange range = {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1};
      vkCmdClearDepthStencilImage(context->graphicsCommandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &val, 1,
                                  &range);
    } else {
      VkClearColorValue val;
      val.float32[0] = val.float32[1] = val.float32[2] = val.float32[3] = 0.f;
      val.int32[0] = val.int32[1] = val.int32[2] = val.int32[3] = 0;
      val.uint32[0] = val.uint32[1] = val.uint32[2] = val.uint32[3] = 0;
      VkImageSubresourceRange range = {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1};
      vkCmdClearColorImage(context->graphicsCommandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &val, 1,
                           &range);
    }

    // Now go back to the previous layout
    setImageLayout(context->graphicsCommandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layout,
                   {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});

    err = vkEndCommandBuffer(context->graphicsCommandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for texture mipmap generation! : \n" + errorString(err));

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for texture mipmap generation! : \n" + errorString(err));

    err = vkQueueWaitIdle(context->graphicsQueue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for texture mipmap generation! : \n" + errorString(err));
  }

  void generateMipmap() {
    // do nothing if we don't have a mipmap to generate
    if (mipLevels == 1)
      return;

    // double check we have the right usage flags...
    // Shouldn't happen, but doesn't hurt to double check.
    if ((usageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) == 0)
      LOG_ERROR("image needs transfer src usage bit for texture mipmap "
                "generation! \n");

    if ((usageFlags & VK_IMAGE_USAGE_TRANSFER_DST_BIT) == 0)
      LOG_ERROR("image needs transfer dst usage bit for texture mipmap "
                "generation! \n");

    VkResult err;
    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for texture mipmap generation! : \n" + errorString(err));

    // transition device to a transfer destination format
    setImageLayout(context->graphicsCommandBuffer, image, layout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                   {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
    layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.image = image;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.subresourceRange.aspectMask = aspectFlagBits;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;
    barrier.subresourceRange.levelCount = 1;

    int32_t mipWidth = width;
    int32_t mipHeight = height;
    int32_t mipDepth = depth;

    // note, loop here starts at 1, not 0
    for (uint32_t i = 1; i < mipLevels; i++) {
      // just transitioning the layouts of individual mips
      barrier.subresourceRange.baseMipLevel = i - 1;
      barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
      barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
      barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

      // this will wait for level i-1 to be filled from either a
      // vkCmdCopyBufferToImae call, or the previous blit command
      vkCmdPipelineBarrier(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT,
                           VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

      // here, we specify the regions to use for the blit
      VkImageBlit blit{};
      blit.srcOffsets[0] = {0, 0, 0};
      blit.srcOffsets[1] = {mipWidth, mipHeight, mipDepth};
      blit.srcSubresource.aspectMask = aspectFlagBits;
      blit.srcSubresource.mipLevel = i - 1;
      blit.srcSubresource.baseArrayLayer = 0;
      blit.srcSubresource.layerCount = 1;
      blit.dstOffsets[0] = {0, 0, 0};
      blit.dstOffsets[1] = {mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1,
                            mipDepth > 1 ? mipDepth / 2 : 1};
      blit.dstSubresource.aspectMask = aspectFlagBits;
      blit.dstSubresource.mipLevel = i;
      blit.dstSubresource.baseArrayLayer = 0;
      blit.dstSubresource.layerCount = 1;

      // This blit will downsample the current mip layer into the one above.
      // It also transitions the image
      vkCmdBlitImage(context->graphicsCommandBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image,
                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

      // Now, transition the layer to VK_IMAGE_LAYOUT_READ_ONLY_OPTIMAL
      barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
      barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

      vkCmdPipelineBarrier(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT,
                           VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

      // now, divide the current mip dimensions by two.
      // mip levels can't be smaller than 1 though.
      if (mipWidth > 1)
        mipWidth /= 2;
      if (mipHeight > 1)
        mipHeight /= 2;
      if (mipDepth > 1)
        mipDepth /= 2;
    }

    // at the very end, we need one more barrier to transition the lastmip level
    barrier.subresourceRange.baseMipLevel = mipLevels - 1;
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

    // Now, all layers in the mip chan have this layout
    layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    err = vkEndCommandBuffer(context->graphicsCommandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for texture mipmap generation! : \n" + errorString(err));

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for texture mipmap generation! : \n" + errorString(err));

    err = vkQueueWaitIdle(context->graphicsQueue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for texture mipmap generation! : \n" + errorString(err));
  }

  Texture(Context *context, VkImageUsageFlags _usageFlags, VkMemoryPropertyFlags _memoryPropertyFlags, VkImageType type,
          VkFormat _format, uint32_t _width, uint32_t _height, uint32_t _depth, bool allocateMipmap,
          const void *data = nullptr)
      : ImageResource(context) {
    this->context = context;
    memoryPropertyFlags = _memoryPropertyFlags;
    usageFlags = _usageFlags;
    width = _width;
    height = _height;
    depth = _depth;
    format = _format;
    size = width * height * depth * gprtFormatGetSize((GPRTFormat) format);
    imageType = type;

    aspectFlagBits = (format == VK_FORMAT_D32_SFLOAT) ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT;

    uint32_t largestDimension = std::max(std::max(width, height), depth);
    if (allocateMipmap) {
      // Compute total mip levels (each being half the previous)
      // floor here accounts for non-power-of-two textures.
      mipLevels = static_cast<uint32_t>(std::floor(std::log2(largestDimension))) + 1;
    } else {
      mipLevels = 1;
    }

    // Check if the image can be mapped to a host pointer.
    // If the image isn't host visible, this is image and requires
    // an additional staging image...
    if ((memoryPropertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0)
      hostVisible = true;
    else
      hostVisible = false;

    vkGetPhysicalDeviceMemoryProperties(context->physicalDevice, &memoryProperties);

    auto getMemoryType = [this](uint32_t typeBits, VkMemoryPropertyFlags properties,
                                VkBool32 *memTypeFound = nullptr) -> uint32_t {
      // memory type bits is a bitmask and contains one bit set for every
      // supported memory type. Bit i is set if and only if the memory type i in
      // the memory properties is supported.
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
      } else {
        LOG_ERROR("Could not find a matching memory type");
      }
      return -1;
    };

    // Create the image handle
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = type;
    imageInfo.extent.width = width;
    imageInfo.extent.height = height;
    imageInfo.extent.depth = depth;
    imageInfo.mipLevels = mipLevels;
    imageInfo.arrayLayers = 1;
    imageInfo.format = format;

    // can be VK_IMAGE_TILING_LINEAR or VK_IMAGE_TILING_OPTIMAL.
    // VK_IMAGE_TILING_LINEAR means texels are laid out in a row-major order
    // VK_IMAGE_TILING_OPTIMAL means texels are laid out in an order that is
    // implementation defined for optimal access LINEAR is required for reading
    // texels directly. This tiling cannot be changed.
    if (hostVisible)
      imageInfo.tiling = VK_IMAGE_TILING_LINEAR;
    else
      imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;

    // Can be either VK_IMAGE_LAYOUT_UNDEFINED or
    // VK_IMAGE_LAYOUT_PREINITIALIZED. Preinitialized is required if we're
    // uploading straight to texture as if it were
    //   a staging image. If we instead use a staging buffer, then this should
    //   be undefined
    if (hostVisible)
      imageInfo.initialLayout = VK_IMAGE_LAYOUT_PREINITIALIZED;
    else
      imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    imageInfo.usage = usageFlags;
    if (aspectFlagBits == VK_IMAGE_ASPECT_COLOR_BIT)
      imageInfo.usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if (aspectFlagBits == VK_IMAGE_ASPECT_DEPTH_BIT)
      imageInfo.usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;

    // Since this image is oonly going to be used by graphics queues, we have
    // this set to exclusive.
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    // If this image were to be used with MSAA as an attachment, we'd set this
    // to something other than
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.flags = 0;   // Optional, but some options here for sparse images,
                           // like volumes with large sections of just air values.

    VK_CHECK_RESULT(vkCreateImage(context->logicalDevice, &imageInfo, nullptr, &image));

    if (!hostVisible) {
      const VkBufferUsageFlags bufferUsageFlags =
          // means we can use this buffer to transfer into another
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
          // means we can use this buffer to receive data transferred from
          // another
          VK_BUFFER_USAGE_TRANSFER_DST_BIT;

      VkBufferCreateInfo bufferCreateInfo{};
      bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
      bufferCreateInfo.usage = bufferUsageFlags;
      bufferCreateInfo.size = size;
      VK_CHECK_RESULT(vkCreateBuffer(context->logicalDevice, &bufferCreateInfo, nullptr, &stagingBuffer.buffer));
    }

    // Create the memory backing up the image handle
    VkMemoryRequirements memReqs;
    vkGetImageMemoryRequirements(context->logicalDevice, image, &memReqs);
    VkMemoryAllocateInfo memAllocInfo{};
    memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    memAllocInfo.allocationSize = memReqs.size;
    // Find a memory type index that firts the properties of the image
    memAllocInfo.memoryTypeIndex = getMemoryType(memReqs.memoryTypeBits, memoryPropertyFlags);

    VK_CHECK_RESULT(vkAllocateMemory(context->logicalDevice, &memAllocInfo, nullptr, &memory));
    alignment = memReqs.alignment;

    // Attach the memory to the image object
    VK_CHECK_RESULT(vkBindImageMemory(context->logicalDevice, image, memory, 0));

    if (!hostVisible) {
      const VkMemoryPropertyFlags memoryPropertyFlags =
          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |   // mappable to host with
                                                  // vkMapMemory
          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;   // means "flush" and
                                                  // "invalidate"  not needed

      // Create the memory backing up the staging buffer handle
      VkMemoryRequirements memReqs;
      vkGetBufferMemoryRequirements(context->logicalDevice, stagingBuffer.buffer, &memReqs);
      VkMemoryAllocateInfo memAllocInfo{};
      memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
      memAllocInfo.allocationSize = memReqs.size;
      // Find a memory type index that fits the properties of the buffer
      memAllocInfo.memoryTypeIndex = getMemoryType(memReqs.memoryTypeBits, memoryPropertyFlags);

      VK_CHECK_RESULT(vkAllocateMemory(context->logicalDevice, &memAllocInfo, nullptr, &stagingBuffer.memory));

      // Attach the memory to the buffer object
      VkResult err =
          vkBindBufferMemory(context->logicalDevice, stagingBuffer.buffer, stagingBuffer.memory, /* offset */ 0);
      if (err)
        LOG_ERROR("failed to bind staging buffer memory! : \n" + errorString(err));
    }

    // We need to transition the image into a known layout
    {
      VkResult err;
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for buffer map! : \n" + errorString(err));

      setImageLayout(context->graphicsCommandBuffer, image, layout, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                     {uint32_t(aspectFlagBits), 0, mipLevels, 0, 1});
      layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

      err = vkEndCommandBuffer(context->graphicsCommandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for buffer map! : \n" + errorString(err));

      VkSubmitInfo submitInfo{};
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for buffer map! : \n" + errorString(err));

      err = vkQueueWaitIdle(context->graphicsQueue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for buffer map! : \n" + errorString(err));
    }

    // Now we need an image view
    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = image;
    // note, image type and image view type share common enum values up to a
    // point... todo, how to handle cube maps? 1d/2d arrays? cube map arrays?
    viewInfo.viewType = (VkImageViewType) type;
    viewInfo.format = format;
    viewInfo.subresourceRange.aspectMask = aspectFlagBits;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = mipLevels;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    VK_CHECK_RESULT(vkCreateImageView(context->logicalDevice, &viewInfo, nullptr, &imageView));

    if (hostVisible) {
      VkImageSubresource subRes = {};
      subRes.aspectMask = aspectFlagBits;
      subRes.mipLevel = 0;
      subRes.arrayLayer = 0;
      vkGetImageSubresourceLayout(context->logicalDevice, image, &subRes, &subresourceLayout);
    }

    if (data != nullptr) {
      map();
      memcpy(mapped, data, size);
      unmap();

      if (mipLevels > 1)
        generateMipmap();
    }

    // Write the image into the desscriptor array
    VkDescriptorImageInfo imageDescriptor = {};
    imageDescriptor.imageView = imageView;
    imageDescriptor.imageLayout = layout;

    VkWriteDescriptorSet writeDescriptorSet = {};
    writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet.dstBinding = 0;
    writeDescriptorSet.dstArrayElement = address;
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.pBufferInfo = 0;
    writeDescriptorSet.dstSet = context->descriptorSet;
    writeDescriptorSet.pImageInfo = &imageDescriptor;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    vkUpdateDescriptorSets(context->logicalDevice, 1, &writeDescriptorSet, 0, nullptr);
  }

  /*! Calls vkDestroy on the image, and frees underlying memory */
  void destroy() override {
    ImageResource::destroy();

    if (imageView) {
      vkDestroyImageView(context->logicalDevice, imageView, nullptr);
      imageView = VK_NULL_HANDLE;
    }
    if (image) {
      vkDestroyImage(context->logicalDevice, image, nullptr);
      image = VK_NULL_HANDLE;
    }
    if (memory) {
      vkFreeMemory(context->logicalDevice, memory, nullptr);
      memory = VK_NULL_HANDLE;
    }
    if (stagingBuffer.buffer) {
      vkDestroyBuffer(context->logicalDevice, stagingBuffer.buffer, nullptr);
      stagingBuffer.buffer = VK_NULL_HANDLE;
    }
    if (stagingBuffer.memory) {
      vkFreeMemory(context->logicalDevice, stagingBuffer.memory, nullptr);
      stagingBuffer.memory = VK_NULL_HANDLE;
    }
  }

  virtual ImageResourceType getType() {
    if (imageType == VK_IMAGE_TYPE_1D)
      return GPRT_IMAGE_RESOURCE_TYPE_TEXTURE_1D;
    if (imageType == VK_IMAGE_TYPE_2D)
      return GPRT_IMAGE_RESOURCE_TYPE_TEXTURE_2D;
    if (imageType == VK_IMAGE_TYPE_3D)
      return GPRT_IMAGE_RESOURCE_TYPE_TEXTURE_3D;
    return GPRT_IMAGE_RESOURCE_TYPE_UNKNOWN;
  }
};

struct Sampler : public ImageResource {
  VkSampler sampler;

  ~Sampler() {};

  Sampler(Context *context, VkFilter magFilter, VkFilter minFilter, VkSamplerMipmapMode mipFilter, uint32_t anisotropy,
          VkSamplerAddressMode addressMode, VkBorderColor borderColor)
      : ImageResource(context) {

    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = magFilter;
    samplerInfo.minFilter = minFilter;
    samplerInfo.addressModeU = addressMode;
    samplerInfo.addressModeV = addressMode;
    samplerInfo.addressModeW = addressMode;
    samplerInfo.anisotropyEnable = (anisotropy == 1) ? VK_FALSE : VK_TRUE;

    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(context->physicalDevice, &properties);
    samplerInfo.maxAnisotropy = std::min(properties.limits.maxSamplerAnisotropy, float(anisotropy));
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
    samplerInfo.mipmapMode = mipFilter;
    samplerInfo.mipLodBias = 0.0f;
    samplerInfo.minLod = 0.0f;
    samplerInfo.maxLod = VK_LOD_CLAMP_NONE;
    samplerInfo.borderColor = borderColor;

    VK_CHECK_RESULT(vkCreateSampler(context->logicalDevice, &samplerInfo, nullptr, &sampler));

    // Write the image into the desscriptor array
    VkDescriptorImageInfo imageDescriptor = {};
    imageDescriptor.sampler = sampler;

    VkWriteDescriptorSet writeDescriptorSet = {};
    writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet.dstBinding = 0;
    writeDescriptorSet.dstArrayElement = address;
    writeDescriptorSet.descriptorCount = 1;
    writeDescriptorSet.pBufferInfo = 0;
    writeDescriptorSet.dstSet = context->descriptorSet;
    writeDescriptorSet.pImageInfo = &imageDescriptor;
    writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
    vkUpdateDescriptorSets(context->logicalDevice, 1, &writeDescriptorSet, 0, nullptr);
  }

  void destroy() override {
    ImageResource::destroy();

    if (sampler) {
      vkDestroySampler(context->logicalDevice, sampler, nullptr);
    }
  }

  virtual ImageResourceType getType() { return GPRT_IMAGE_RESOURCE_TYPE_SAMPLER; }
};

struct Compute {
  Context *context;

  // Our own virtual "compute address space".
  uint32_t address = -1;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};

  std::string entryPoint;

  VkShaderModule shaderModule = VK_NULL_HANDLE;
  VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
  VkPipeline pipeline = VK_NULL_HANDLE;

  Compute(Context *context, Module *module, const char *_entryPoint) {
    this->context = context;

    // Hunt for an existing free address for this compute kernel
    for (uint32_t i = 0; i < context->computes.size(); ++i) {
      if (context->computes[i] == nullptr) {
        context->computes[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->computes.push_back(this);
      address = (uint32_t) context->computes.size() - 1;
    }

    std::vector<unsigned int, std::allocator<unsigned int>> binary;
    entryPoint = std::string(_entryPoint);
    binary = module->binary;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);   // sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data();                         //(uint32_t*)binary->wordCount;//programBytes;

    VK_CHECK_RESULT(vkCreateShaderModule(context->logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);
  }
  ~Compute() {}

  void buildPipeline(VkDescriptorSetLayout descriptorSetLayout) {
    // If we already have a pipeline layout, free it so that we can make a new one
    if (pipelineLayout) {
      vkDestroyPipelineLayout(context->logicalDevice, pipelineLayout, nullptr);
      pipelineLayout = VK_NULL_HANDLE;
    }

    // If we already have a pipeline, free it so that we can make a new one
    if (pipeline) {
      vkDestroyPipeline(context->logicalDevice, pipeline, nullptr);
      pipeline = VK_NULL_HANDLE;
    }

    // currently not using cache.
    VkPipelineCache cache = VK_NULL_HANDLE;

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.size = PUSH_CONSTANTS_LIMIT;
    pushConstantRange.offset = 0;
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
    pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    std::vector<VkDescriptorSetLayout> layouts = {descriptorSetLayout};
    pipelineLayoutCreateInfo.setLayoutCount = (uint32_t) layouts.size();
    pipelineLayoutCreateInfo.pSetLayouts = layouts.data();
    pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
    pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

    if (vkCreatePipelineLayout(context->logicalDevice, &pipelineLayoutCreateInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
      throw std::runtime_error("failed to create pipeline layout!");
    }

    VkComputePipelineCreateInfo computePipelineCreateInfo = {};
    computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    computePipelineCreateInfo.layout = pipelineLayout;
    computePipelineCreateInfo.flags = 0;
    computePipelineCreateInfo.stage = shaderStage;

    // If the below function is crashing, double check that all parameters to the compute kernel are tagged as
    // "uniform".
    VkResult err = vkCreateComputePipelines(context->logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr, &pipeline);
    if (err) {
      LOG_ERROR("failed to create compute pipeline! Please verify that \"" + entryPoint +
                "\" is an existing entrypoint name\n" + errorString(err));
    }
  }

  void destroy() {
    // Free compute slot for use by subsequently made compute kernels
    context->computes[address] = nullptr;

    if (pipelineLayout) {
      vkDestroyPipelineLayout(context->logicalDevice, pipelineLayout, nullptr);
    }

    if (pipeline) {
      vkDestroyPipeline(context->logicalDevice, pipeline, nullptr);
    }

    if (shaderModule) {
      vkDestroyShaderModule(context->logicalDevice, shaderModule, nullptr);
    }
  }
};


struct SBTEntry {
  size_t recordSize = 0;
  uint8_t *SBTRecord = nullptr;
};

struct RayGen : public SBTEntry {
  Context* context;

  // Our own virtual "raygen address space".
  uint32_t address = -1;
  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  std::string entryPoint;

  RayGen(Context* context, Module *module, const char *_entryPoint, size_t recordSize) : SBTEntry() {
    this->context = context;

    // Hunt for an existing free address for this raygen kernel
    for (uint32_t i = 0; i < context->raygens.size(); ++i) {
      if (context->raygens[i] == nullptr) {
        context->raygens[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->raygens.push_back(this);
      address = (uint32_t) context->raygens.size() - 1;
    }

    // Fetch the SPIRV
    std::vector<unsigned int, std::allocator<unsigned int>> binary;
    entryPoint = std::string(_entryPoint);
    binary = module->binary;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);   // sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data();                         //(uint32_t*)binary->wordCount;//programBytes;

    VkResult err = vkCreateShaderModule(context->logicalDevice, &moduleCreateInfo, NULL, &shaderModule);
    if (err)
      LOG_ERROR("failed to create shader module! : \n" + errorString(err));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);

    this->recordSize = recordSize;
    this->SBTRecord = (uint8_t *) malloc(recordSize);
  }
  ~RayGen() {}
  void destroy() {
    // Free raygen slot for use by subsequently made raygen kernels
    context->raygens[address] = nullptr;

    vkDestroyShaderModule(context->logicalDevice, shaderModule, nullptr);
    free(this->SBTRecord);
  }
};

struct Miss : public SBTEntry {
  Context *context;

  // Our own virtual "miss address space".
  uint32_t address = -1;
  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  std::string entryPoint;

  Miss(Context* context, Module *module, const char *_entryPoint, size_t recordSize) : SBTEntry() {
    this->context = context;

    // Hunt for an existing free address for this miss kernel
    for (uint32_t i = 0; i < context->misses.size(); ++i) {
      if (context->misses[i] == nullptr) {
        context->misses[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->misses.push_back(this);
      address = (uint32_t) context->misses.size() - 1;
    }

    // Fetch the SPIRV
    std::vector<unsigned int, std::allocator<unsigned int>> binary;

    entryPoint = std::string(_entryPoint);
    binary = module->binary;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);   // sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data();                         //(uint32_t*)binary->wordCount;//programBytes;

    VK_CHECK_RESULT(vkCreateShaderModule(context->logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_MISS_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);

    this->recordSize = recordSize;
    this->SBTRecord = (uint8_t *) malloc(recordSize);
  }
  ~Miss() {}
  void destroy() {
    // Free miss slot for use by subsequently made miss kernels
    context->misses[address] = nullptr;

    vkDestroyShaderModule(context->logicalDevice, shaderModule, nullptr);
    free(this->SBTRecord);
  }
};

struct Callable : public SBTEntry {
  Context* context;

  // Our own virtual "callable address space".
  uint32_t address = -1;

  VkShaderModule shaderModule;
  VkPipelineShaderStageCreateInfo shaderStage{};
  VkShaderModuleCreateInfo moduleCreateInfo{};
  std::string entryPoint;

  Callable(Context* context, Module *module, const char *_entryPoint, size_t recordSize) : SBTEntry() {
    this->context = context;

    // Hunt for an existing free address for this callable kernel
    for (uint32_t i = 0; i < context->callables.size(); ++i) {
      if (context->callables[i] == nullptr) {
        context->callables[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->callables.push_back(this);
      address = (uint32_t) context->callables.size() - 1;
    }

    // Fetch the SPIRV
    std::vector<unsigned int, std::allocator<unsigned int>> binary;

    entryPoint = std::string(_entryPoint);
    binary = module->binary;

    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);   // sizeOfProgramBytes;
    moduleCreateInfo.pCode = binary.data();                         //(uint32_t*)binary->wordCount;//programBytes;

    VK_CHECK_RESULT(vkCreateShaderModule(context->logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_CALLABLE_BIT_KHR;
    shaderStage.module = shaderModule;
    shaderStage.pName = entryPoint.c_str();
    assert(shaderStage.module != VK_NULL_HANDLE);

    this->recordSize = recordSize;
    this->SBTRecord = (uint8_t *) malloc(recordSize);
  }
  ~Callable() {}
  void destroy() {
    // Free callable slot for use by subsequently made callable kernels
    context->callables[address] = nullptr;

    vkDestroyShaderModule(context->logicalDevice, shaderModule, nullptr);
    free(this->SBTRecord);
  }
};

/* An abstraction for any sort of geometry type - describes the
    programs to use, and structure of the SBT records, when building
    shader binding tables (SBTs) with geometries of this type. This
    will later get subclassed into triangle geometries, user/custom
    primitive geometry types, etc */
struct GeomType : public SBTEntry {
  Context* context;

  // Our own virtual "geom types address space".
  uint32_t address = -1;

  uint32_t numRayTypes;

  std::vector<VkPipelineShaderStageCreateInfo> closestHitShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> anyHitShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> intersectionShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> vertexShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> pixelShaderStages;
  std::vector<VkPipelineShaderStageCreateInfo> closestNeighborShaderStages;

  std::vector<std::string> closestHitShaderEntryPoints;
  std::vector<std::string> anyHitShaderEntryPoints;
  std::vector<std::string> intersectionShaderEntryPoints;
  std::vector<std::string> vertexShaderEntryPoints;
  std::vector<std::string> pixelShaderEntryPoints;
  std::vector<std::string> closestNeighborShaderEntryPoints;

  std::vector<bool> closestHitShaderUsed;
  std::vector<bool> intersectionShaderUsed;
  std::vector<bool> anyHitShaderUsed;
  std::vector<bool> vertexShaderUsed;
  std::vector<bool> pixelShaderUsed;
  std::vector<bool> closestNeighborShaderUsed;

  GeomType(Context* context, uint32_t numRayTypes, size_t recordSize) : SBTEntry() {
    this->context = context;

    // Hunt for an existing free address for this geom type
    for (uint32_t i = 0; i < context->geomTypes.size(); ++i) {
      if (context->geomTypes[i] == nullptr) {
        context->geomTypes[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->geomTypes.push_back(this);
      address = (uint32_t) context->geomTypes.size() - 1;
    }

    this->numRayTypes = numRayTypes;
    closestHitShaderStages.resize(numRayTypes, {});
    anyHitShaderStages.resize(numRayTypes, {});
    intersectionShaderStages.resize(numRayTypes, {});
    vertexShaderStages.resize(numRayTypes, {});
    pixelShaderStages.resize(numRayTypes, {});
    closestNeighborShaderStages.resize(numRayTypes, {});

    closestHitShaderEntryPoints.resize(numRayTypes, {});
    anyHitShaderEntryPoints.resize(numRayTypes, {});
    intersectionShaderEntryPoints.resize(numRayTypes, {});
    vertexShaderEntryPoints.resize(numRayTypes, {});
    pixelShaderEntryPoints.resize(numRayTypes, {});
    closestNeighborShaderEntryPoints.resize(numRayTypes, {});

    closestHitShaderUsed.resize(numRayTypes, false);
    intersectionShaderUsed.resize(numRayTypes, false);
    anyHitShaderUsed.resize(numRayTypes, false);
    vertexShaderUsed.resize(numRayTypes, false);
    pixelShaderUsed.resize(numRayTypes, false);
    closestNeighborShaderUsed.resize(numRayTypes, false);

    // store size, but don't allocate. Will be done by geom instances.
    this->recordSize = recordSize;
  }
  ~GeomType() {
    // vkDestroyShaderModule(logicalDevice, shaderModule, nullptr);
  }

  virtual GPRTGeomKind getKind() { return GPRT_UNKNOWN; }

  void setClosestHit(int rayType, Module *module, const char *entryPoint) {
    context->raytracingPipelineOutOfDate = true;

    closestHitShaderUsed[rayType] = true;
    std::vector<unsigned int, std::allocator<unsigned int>> binary;
    closestHitShaderEntryPoints[rayType] = std::string(entryPoint);
    binary = module->binary;

    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(context->logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    closestHitShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    closestHitShaderStages[rayType].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    closestHitShaderStages[rayType].module = shaderModule;
    closestHitShaderStages[rayType].pName = closestHitShaderEntryPoints[rayType].c_str();
    assert(closestHitShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  void setAnyHit(int rayType, Module *module, const char *entryPoint) {
    context->raytracingPipelineOutOfDate = true;

    anyHitShaderUsed[rayType] = true;
    std::vector<unsigned int, std::allocator<unsigned int>> binary;
    anyHitShaderEntryPoints[rayType] = std::string(entryPoint);
    binary = module->binary;

    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(context->logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    anyHitShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    anyHitShaderStages[rayType].stage = VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
    anyHitShaderStages[rayType].module = shaderModule;
    anyHitShaderStages[rayType].pName = anyHitShaderEntryPoints[rayType].c_str();
    assert(anyHitShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  void setIntersection(int rayType, Module *module, const char *entryPoint) {
    context->raytracingPipelineOutOfDate = true;

    intersectionShaderUsed[rayType] = true;
    std::vector<unsigned int, std::allocator<unsigned int>> binary;
    intersectionShaderEntryPoints[rayType] = std::string(entryPoint);
    binary = module->binary;

    VkShaderModuleCreateInfo moduleCreateInfo{};
    moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
    moduleCreateInfo.pCode = binary.data();

    VkShaderModule shaderModule;
    VK_CHECK_RESULT(vkCreateShaderModule(context->logicalDevice, &moduleCreateInfo, NULL, &shaderModule));

    intersectionShaderStages[rayType].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    intersectionShaderStages[rayType].stage = VK_SHADER_STAGE_INTERSECTION_BIT_KHR;
    intersectionShaderStages[rayType].module = shaderModule;
    intersectionShaderStages[rayType].pName = intersectionShaderEntryPoints[rayType].c_str();
    assert(intersectionShaderStages[rayType].module != VK_NULL_HANDLE);
  }

  // This is a bit dumb. We shouldn't be creating all these shader modules...
  void destroy() {
    // Free geomtype slot for use by subsequently made geomtypes
    context->geomTypes[address] = nullptr;

    for (uint32_t i = 0; i < closestHitShaderStages.size(); ++i) {
      if (closestHitShaderStages[i].module)
        vkDestroyShaderModule(context->logicalDevice, closestHitShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < anyHitShaderStages.size(); ++i) {
      if (anyHitShaderStages[i].module)
        vkDestroyShaderModule(context->logicalDevice, anyHitShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < intersectionShaderStages.size(); ++i) {
      if (intersectionShaderStages[i].module)
        vkDestroyShaderModule(context->logicalDevice, intersectionShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < vertexShaderStages.size(); ++i) {
      if (vertexShaderStages[i].module)
        vkDestroyShaderModule(context->logicalDevice, vertexShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < pixelShaderStages.size(); ++i) {
      if (pixelShaderStages[i].module)
        vkDestroyShaderModule(context->logicalDevice, pixelShaderStages[i].module, nullptr);
    }
    for (uint32_t i = 0; i < closestNeighborShaderStages.size(); ++i) {
      if (closestNeighborShaderStages[i].module)
        vkDestroyShaderModule(context->logicalDevice, closestNeighborShaderStages[i].module, nullptr);
    }
  }

  virtual Geom *createGeom() { return nullptr; };
};

/*! An actual geometry object with primitives - this class is still
  abstract, and will get fleshed out in its derived classes
  (AABBGeom, TriangleGeom, ...) */
struct Geom : public SBTEntry {
  Context* context;

  // Our own virtual "geometry address space".
  VkDeviceAddress address = -1;

  Geom(Context* context) : SBTEntry() {
    this->context = context;

    // Hunt for an existing free address for this geometry
    for (uint32_t i = 0; i < context->geoms.size(); ++i) {
      if (context->geoms[i] == nullptr) {
        context->geoms[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->geoms.push_back(this);
      address = context->geoms.size() - 1;
    }
  };
  ~Geom() {};

  void destroy() {
    // Free geometry slot for use by subsequently made geometries
    context->geoms[address] = nullptr;
  }

  /*! This acts as a template that describes this geometry's variables and
      programs. */
  GeomType *geomType;
};

struct TriangleGeomType : public GeomType {
  TriangleGeomType(Context* context, uint32_t numRayTypes, size_t recordSize)
      : GeomType(context, numRayTypes, recordSize) {}
  ~TriangleGeomType() {}

  Geom *createGeom();

  GPRTGeomKind getKind() { return GPRT_TRIANGLES; }
};

struct TriangleGeom : public Geom {
  struct {
    uint32_t count = 0;         // number of indices
    uint32_t stride = 0;        // stride between indices
    uint32_t offset = 0;        // offset in bytes to the first index
    uint32_t firstVertex = 0;   // added to the index values before fetching vertices
    Buffer *buffer = nullptr;
  } index;

  struct {
    uint32_t count = 0;    // number of vertices
    uint32_t stride = 0;   // stride between vertices
    uint32_t offset = 0;   // an offset in bytes to the first vertex
    std::vector<Buffer *> buffers;
  } vertex;

  TriangleGeom(TriangleGeomType *_geomType) : Geom(_geomType->context) {
    geomType = (GeomType *) _geomType;

    // Allocate the variables for this geometry
    this->SBTRecord = (uint8_t *) malloc(geomType->recordSize);
    this->recordSize = geomType->recordSize;
  };
  ~TriangleGeom() { free(this->SBTRecord); };

  void setVertices(Buffer *vertices, uint32_t count, uint32_t stride, uint32_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    vertex.buffers.resize(1);
    vertex.buffers[0] = vertices;
    vertex.count = count;
    vertex.stride = stride;
    vertex.offset = offset;
  }

  void setIndices(Buffer *indices, uint32_t count, uint32_t stride, uint32_t offset) {
    index.buffer = indices;
    index.count = count;
    index.stride = stride;
    index.offset = offset;
  }
};

Geom *TriangleGeomType::createGeom() {
  return new TriangleGeom(this);
}

struct SphereGeomType : public GeomType {
  SphereGeomType(Context* context, uint32_t numRayTypes, size_t recordSize)
      : GeomType(context, numRayTypes, recordSize) {}
  ~SphereGeomType() {}

  Geom *createGeom();

  GPRTGeomKind getKind() { return GPRT_SPHERES; }
};

struct SphereGeom : public Geom {
  struct {
    uint32_t count = 0;    // number of vertices
    uint32_t stride = 0;   // stride between vertices
    uint32_t offset = 0;   // an offset in bytes to the first vertex
    std::vector<Buffer *> buffers;
  } vertex;

  // false: return entry hits, true: return exit hits
  bool exitTest = false;

  SphereGeom(SphereGeomType *_geomType) : Geom(_geomType->context) {
    geomType = (GeomType *) _geomType;

    // Allocate the variables for this geometry
    this->SBTRecord = (uint8_t *) malloc(geomType->recordSize);
    this->recordSize = geomType->recordSize;
  };
  ~SphereGeom() { free(this->SBTRecord); };

  void setVertices(Buffer *vertices, uint32_t count, uint32_t stride, uint32_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    vertex.buffers.resize(1);
    vertex.buffers[0] = vertices;
    vertex.count = count;
    vertex.stride = stride;
    vertex.offset = offset;
  }
};

Geom *SphereGeomType::createGeom() {
  return new SphereGeom(this);
}

struct LSSGeomType : public GeomType {
  LSSGeomType(Context* context, uint32_t numRayTypes, size_t recordSize)
      : GeomType(context, numRayTypes, recordSize) {}
  ~LSSGeomType() {}

  Geom *createGeom();

  GPRTGeomKind getKind() { return GPRT_LSS; }
};

struct LSSGeom : public Geom {
  struct {
    uint32_t count = 0;         // number of indices
    uint32_t stride = 0;        // stride between indices
    uint32_t offset = 0;        // offset in bytes to the first index
    uint32_t firstVertex = 0;   // added to the index values before fetching vertices
    Buffer *buffer = nullptr;
  } index;

  struct {
    uint32_t count = 0;    // number of vertices
    uint32_t stride = 0;   // stride between vertices
    uint32_t offset = 0;   // an offset in bytes to the first vertex
    std::vector<Buffer *> buffers;
  } vertex;

  // true: endcap0 enabled, false: endcap0 disabled
  bool endcap0 = true;
  // true: endcap1 enabled, false: endcap1 disabled
  bool endcap1 = true;
  // false: return entry hits, true: return exit hits
  bool exitTest = false;

  LSSGeom(LSSGeomType *_geomType) : Geom(_geomType->context) {
    geomType = (GeomType *) _geomType;

    // Allocate the variables for this geometry
    this->SBTRecord = (uint8_t *) malloc(geomType->recordSize);
    this->recordSize = geomType->recordSize;
  };
  ~LSSGeom() { free(this->SBTRecord); };

  void setVertices(Buffer *vertices, uint32_t count, uint32_t stride, uint32_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    vertex.buffers.resize(1);
    vertex.buffers[0] = vertices;
    vertex.count = count;
    vertex.stride = stride;
    vertex.offset = offset;
  }

  void setIndices(Buffer *indices, uint32_t count, uint32_t stride, uint32_t offset) {
    index.buffer = indices;
    index.count = count;
    index.stride = stride;
    index.offset = offset;
  }
};

Geom *LSSGeomType::createGeom() {
  return new LSSGeom(this);
}

struct SolidGeomType : public GeomType {
  SolidGeomType(Context* context, uint32_t numRayTypes, size_t recordSize)
      : GeomType(context, numRayTypes, recordSize) {}
  ~SolidGeomType() {}
  Geom *createGeom();

  GPRTGeomKind getKind() { return GPRT_SOLIDS; }
};

struct SolidGeom : public Geom {
  struct {
    uint32_t count;
    uint32_t stride;
    uint32_t offset;
    std::vector<Buffer *> buffers;
  } aabb;

  struct {
    uint32_t count = 0;         // number of indices
    uint32_t stride = 0;        // stride between indices
    uint32_t offset = 0;        // offset in bytes to the first index
    uint32_t firstVertex = 0;   // added to the index values before fetching vertices
    Buffer *buffer = nullptr;
  } index;

  struct {
    uint32_t count = 0;    // number of vertices
    uint32_t stride = 0;   // stride between vertices
    uint32_t offset = 0;   // an offset in bytes to the first vertex
    std::vector<Buffer *> buffers;
  } vertex;

  struct {
    uint32_t count = 0;         // number of indices
    uint32_t stride = 0;        // stride between indices
    uint32_t offset = 0;        // offset in bytes to the first index
    uint32_t firstVertex = 0;   // added to the index values before fetching vertices
    Buffer *buffer = nullptr;
  } types;

  SolidGeom(SolidGeomType *_geomType) : Geom(_geomType->context) {
    geomType = (GeomType *) _geomType;

    // Allocate the variables for this geometry
    this->SBTRecord = (uint8_t *) malloc(geomType->recordSize);
    this->recordSize = geomType->recordSize;
  };
  ~SolidGeom() { free(this->SBTRecord); };

  void setAABBs(Buffer *aabbs, uint32_t count, uint32_t stride, uint32_t offset) {
    aabb.buffers.resize(1);
    aabb.buffers[0] = aabbs;
    aabb.count = count;
    aabb.stride = stride;
    aabb.offset = offset;
  }

  void setVertices(Buffer *vertices, uint32_t count, uint32_t stride, uint32_t offset) {
    vertex.buffers.resize(1);
    vertex.buffers[0] = vertices;
    vertex.count = count;
    vertex.stride = stride;
    vertex.offset = offset;
  }

  void setIndices(Buffer *indices, uint32_t count, uint32_t stride, uint32_t offset) {
    index.buffer = indices;
    index.count = count;
    index.stride = stride;
    index.offset = offset;
  }

  void setTypes(Buffer *_types, uint32_t count, uint32_t stride, uint32_t offset) {
    types.buffer = _types;
    types.count = count;
    types.stride = stride;
    types.offset = offset;
  }
};

Geom *SolidGeomType::createGeom() {
  return new SolidGeom(this);
}

struct AABBGeomType : public GeomType {
  AABBGeomType(Context* context, uint32_t numRayTypes, size_t recordSize)
      : GeomType(context, numRayTypes, recordSize) {}
  ~AABBGeomType() {}
  Geom *createGeom();

  GPRTGeomKind getKind() { return GPRT_AABBS; }
};

struct AABBGeom : public Geom {
  struct {
    uint32_t count;
    uint32_t stride;
    uint32_t offset;
    std::vector<Buffer *> buffers;
  } aabb;

  AABBGeom(AABBGeomType *_geomType) : Geom(_geomType->context) {
    geomType = (GeomType *) _geomType;

    // Allocate the variables for this geometry
    this->SBTRecord = (uint8_t *) malloc(geomType->recordSize);
    this->recordSize = geomType->recordSize;
  };
  ~AABBGeom() { free(this->SBTRecord); };

  void setAABBs(Buffer *aabbs, uint32_t count, uint32_t stride, uint32_t offset) {
    // assuming no motion blurred triangles for now, so we assume 1 buffer
    aabb.buffers.resize(1);
    aabb.buffers[0] = aabbs;
    aabb.count = count;
    aabb.stride = stride;
    aabb.offset = offset;
  }
};

Geom *AABBGeomType::createGeom() {
  return new AABBGeom(this);
}

typedef enum {
  GPRT_UNKNOWN_ACCEL = 0x0,
  GPRT_AABB_ACCEL = 0x1,
  GPRT_INSTANCE_ACCEL = 0x2,
  GPRT_TRIANGLE_ACCEL = 0x3,
  GPRT_SPHERE_ACCEL = 0x4,
  GPRT_LSS_ACCEL = 0x5,
  GPRT_SOLID_ACCEL = 0x6
} AccelType;

struct Accel {
  // Our own virtual address space
  uint32_t address = -1;
  Context *context;
  bool isBottomLevel;

  VkAccelerationStructureKHR accelerationStructure = VK_NULL_HANDLE;
  VkAccelerationStructureKHR compactAccelerationStructure = VK_NULL_HANDLE;
  VkPhysicalDeviceAccelerationStructureFeaturesKHR accelerationStructureFeatures;
  VkPhysicalDeviceAccelerationStructurePropertiesKHR accelerationStructureProperties;
  GPRTBuildMode buildMode = GPRT_BUILD_MODE_UNINITIALIZED;
  bool allowCompaction = false;
  bool minimizeMemory = false;

  bool isBuilt = false;
  bool isCompact = false;

  // Caching these for fast tree updates
  std::vector<Geom *> geometries;
  std::vector<VkAccelerationStructureBuildRangeInfoKHR> accelerationBuildStructureRangeInfos;
  std::vector<VkAccelerationStructureBuildRangeInfoKHR *> accelerationBuildStructureRangeInfoPtrs;
  std::vector<VkAccelerationStructureGeometryKHR> accelerationStructureGeometries;
  std::vector<uint32_t> maxPrimitiveCounts;

private:
  Buffer *scratchBuffer = nullptr;   // Can we make this static? That way, all trees could share the scratch...
  Buffer *accelBuffer = nullptr;
  Buffer *compactBuffer = nullptr;

public:
  Accel(Context *context, bool isBottomLevel) {
    // Hunt for an existing free address for this accel
    for (uint32_t i = 0; i < context->accels.size(); ++i) {
      if (context->accels[i] == nullptr) {
        context->accels[i] = this;
        address = i;
        break;
      }
    }
    // If we cant find a free spot in the current list, allocate a new one
    if (address == -1) {
      context->accels.push_back(this);
      address = (uint32_t) context->accels.size() - 1;
    }

    this->isBottomLevel = isBottomLevel;

    this->context = context;
    accelerationStructureFeatures = {};
    accelerationStructureFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;

    VkPhysicalDeviceFeatures2 deviceFeatures2{};
    deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    deviceFeatures2.pNext = &accelerationStructureFeatures;

    vkGetPhysicalDeviceFeatures2(context->physicalDevice, &deviceFeatures2);

    accelerationStructureProperties = {};
    accelerationStructureProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR;

    VkPhysicalDeviceProperties2 deviceProperties2{};
    deviceProperties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    deviceProperties2.pNext = &accelerationStructureProperties;

    vkGetPhysicalDeviceProperties2(context->physicalDevice, &deviceProperties2);
  };

  ~Accel() {};

  // Acceleration structure handles will change only when the backing memory changes.
  // Backing memory will never change for BVH updates, and will generally (only?) change for rebuilds
  // where primitive counts increase.
  uint64_t getDeviceAddress() {
    Buffer *backingBuffer;
    if (this->isCompact) {
      if (!compactBuffer)
        LOG_ERROR("compactBuffer is nullptr! Was the tree built before this address was requested?");
      backingBuffer = compactBuffer;
    } else {
      if (!accelBuffer)
        LOG_ERROR("accelBuffer is nullptr! Was the tree built before this address was requested?");
      backingBuffer = accelBuffer;
    }

    return backingBuffer->getDeviceAddress();
  }

  VkDeviceAddress getScratchDeviceAddress() {
    if (!scratchBuffer)
      LOG_ERROR("Attempted to get device address for unallocated scratch buffer");

    return scratchBuffer->getDeviceAddress();
  }

  void freeCompactTree() {
    isCompact = false;

    if (compactBuffer) {
      gprt::vkDestroyAccelerationStructure(context->logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }
  }

  void freeFullTree() {
    if (accelBuffer) {
      gprt::vkDestroyAccelerationStructure(context->logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }
  }

  void freeScratchBuffer() {
    if (scratchBuffer) {
      scratchBuffer->destroy();
      scratchBuffer = nullptr;
    }
  }

  void resizeScratchBuffer(VkDeviceSize buildScratchSize) {
    if (scratchBuffer && scratchBuffer->size < buildScratchSize) {
      scratchBuffer->destroy();
      delete (scratchBuffer);
      scratchBuffer = nullptr;
    }

    if (!scratchBuffer) {
      scratchBuffer = new Buffer(context,
                                 // means that the buffer can be used in a VkDescriptorBufferInfo. //
                                 // Is this required? If not, remove this...
                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                     // means we can get this buffer's address with
                                     // vkGetBufferDeviceAddress
                                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                     // means we can use this buffer as a storage buffer
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                 // means that this memory is stored directly on the device
                                 //  (rather than the host, or in a special host/device section)
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buildScratchSize,
                                 accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);
    }
  }

  void resizeFullTree(VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo) {
    // Destroy old accel handle
    if (accelBuffer && accelBuffer->size < accelerationStructureBuildSizesInfo.accelerationStructureSize) {
      gprt::vkDestroyAccelerationStructure(context->logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
      accelBuffer->destroy();
      delete (accelBuffer);
      accelBuffer = nullptr;
    }

    if (!accelBuffer) {
      accelBuffer = new Buffer(context,
          // means we can use this buffer as a means of storing an acceleration
          // structure
          VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
              // means we can get this buffer's address with
              // vkGetBufferDeviceAddress
              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
              // means we can use this buffer as a storage buffer
              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
          // means that this memory is stored directly on the device
          //  (rather than the host, or in a special host/device section)
          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, accelerationStructureBuildSizesInfo.accelerationStructureSize,
          accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = accelBuffer->buffer;
      accelerationStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;

      if (isBottomLevel)
        accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      else
        accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

      // To enable reusing the device address...
      accelerationStructureCreateInfo.createFlags =
          VK_ACCELERATION_STRUCTURE_CREATE_DEVICE_ADDRESS_CAPTURE_REPLAY_BIT_KHR;
      accelerationStructureCreateInfo.deviceAddress = accelBuffer->deviceAddress;
      VkResult err = gprt::vkCreateAccelerationStructure(context->logicalDevice, &accelerationStructureCreateInfo,
                                                         nullptr, &accelerationStructure);
      if (err)
        LOG_ERROR("failed to create acceleration structure: \n" + errorString(err));
    }

    resizeScratchBuffer(accelerationStructureBuildSizesInfo.buildScratchSize);
  }

  VkDeviceAddress queryCompactSize() {
    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    VkResult err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for query compaction size! : \n" + errorString(err));

    // reset the query so we can use it again
    vkCmdResetQueryPool(context->graphicsCommandBuffer, context->compactedSizeQueryPool, 0, 1);

    gprt::vkCmdWriteAccelerationStructuresProperties(context->graphicsCommandBuffer, 1, &accelerationStructure,
                                                     VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR,
                                                     context->compactedSizeQueryPool, 0);

    err = vkEndCommandBuffer(context->graphicsCommandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for query compaction size! : \n" + errorString(err));

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for query compaction size! : \n" + errorString(err));

    err = vkQueueWaitIdle(context->graphicsQueue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for query compaction size! : \n" + errorString(err));

    uint64_t buffer[1] = {0};
    err = vkGetQueryPoolResults(context->logicalDevice, context->compactedSizeQueryPool, 0, 1, sizeof(VkDeviceSize),
                                buffer, sizeof(VkDeviceSize), VK_QUERY_RESULT_WAIT_BIT);
    VkDeviceSize compactedSize = buffer[0];

    if (err)
      LOG_ERROR("failed to get query pool results for query compaction size! : \n" + errorString(err));
    return compactedSize;
  }

  // Usually this is only called once for the lifetime of a tree, but for now just calling it "resize" to match above
  // functions
  void resizeCompactTree(VkDeviceSize compactedSize) {
    // allocate compact buffer and compact acceleration structure
    if (compactBuffer && compactBuffer->size != compactedSize) {
      // Destroy old accel handle too
      gprt::vkDestroyAccelerationStructure(context->logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
      compactBuffer->destroy();
      delete (compactBuffer);
      compactBuffer = nullptr;
    }

    if (!compactBuffer) {
      compactBuffer = new Buffer(context,
                                 // means we can use this buffer as a means of storing an acceleration
                                 // structure
                                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                     // means we can get this buffer's address with
                                     // vkGetBufferDeviceAddress
                                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                                     // means we can use this buffer as a storage buffer
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                 // means that this memory is stored directly on the device
                                 //  (rather than the host, or in a special host/device section)
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, compactedSize,
                                 accelerationStructureProperties.minAccelerationStructureScratchOffsetAlignment);

      VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
      accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
      accelerationStructureCreateInfo.buffer = compactBuffer->buffer;
      accelerationStructureCreateInfo.size = compactedSize;
      if (isBottomLevel)
        accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      else
        accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

      // To enable reusing the device address...
      accelerationStructureCreateInfo.createFlags =
          VK_ACCELERATION_STRUCTURE_CREATE_DEVICE_ADDRESS_CAPTURE_REPLAY_BIT_KHR;
      accelerationStructureCreateInfo.deviceAddress = compactBuffer->deviceAddress;
      VkResult err = gprt::vkCreateAccelerationStructure(context->logicalDevice, &accelerationStructureCreateInfo,
                                                         nullptr, &compactAccelerationStructure);
      if (err)
        LOG_ERROR("failed to create compact acceleration structure: \n" + errorString(err));
    }
  }

  // Copies over the uncompact tree into compacted tree memory
  void compactTree() {

    // Copy over the compacted acceleration structure
    VkCopyAccelerationStructureInfoKHR copyAccelerationStructureInfo{};
    copyAccelerationStructureInfo.sType = VK_STRUCTURE_TYPE_COPY_ACCELERATION_STRUCTURE_INFO_KHR;
    copyAccelerationStructureInfo.src = accelerationStructure;
    copyAccelerationStructureInfo.dst = compactAccelerationStructure;
    copyAccelerationStructureInfo.mode = VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR;
    copyAccelerationStructureInfo.pNext = nullptr;

    VkResult err;
    {
      VkCommandBufferBeginInfo cmdBufInfo{};
      cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
      if (err)
        LOG_ERROR("failed to begin command buffer for compaction! : \n" + errorString(err));

      gprt::vkCmdCopyAccelerationStructure(context->graphicsCommandBuffer, &copyAccelerationStructureInfo);

      err = vkEndCommandBuffer(context->graphicsCommandBuffer);
      if (err)
        LOG_ERROR("failed to end command buffer for compaction! : \n" + errorString(err));

      VkSubmitInfo submitInfo{};
      submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.pNext = NULL;
      submitInfo.waitSemaphoreCount = 0;
      submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
      submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
      submitInfo.commandBufferCount = 1;
      submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
      submitInfo.signalSemaphoreCount = 0;
      submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

      err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
      if (err)
        LOG_ERROR("failed to submit to queue for compaction! : \n" + errorString(err));

      err = vkQueueWaitIdle(context->graphicsQueue);
      if (err)
        LOG_ERROR("failed to wait for queue idle for compaction! : \n" + errorString(err));
    }
  }

  // maxPrimitiveCounts.size() should be equal to the number of geometry going into a tree.
  // For triangle geometry, each entry contains the number of triangles given

  // Call this after setting up
  // - std::vector<VkAccelerationStructureBuildRangeInfoKHR> accelerationBuildStructureRangeInfos;
  // - std::vector<VkAccelerationStructureBuildRangeInfoKHR *> accelerationBuildStructureRangeInfoPtrs;
  // - std::vector<VkAccelerationStructureGeometryKHR> accelerationStructureGeometries;
  // - std::vector<uint32_t> maxPrimitiveCounts;
  void innerBuildProc(GPRTBuildMode buildMode, bool allowCompaction, bool minimizeMemory) {
    VkResult err;

    // Get size info
    VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
    accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;

    if (isBottomLevel)
      accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    else
      accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                     VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_DATA_ACCESS_KHR;
    accelerationStructureBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

    VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
    accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
    gprt::vkGetAccelerationStructureBuildSizes(context->logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                               &accelerationStructureBuildGeometryInfo, maxPrimitiveCounts.data(),
                                               &accelerationStructureBuildSizesInfo);

    // for TLAS, called like this:
    // gprt::vkGetAccelerationStructureBuildSizes(context->logicalDevice,
    // VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
    //                                            &accelerationStructureBuildGeometryInfo, &primitive_count,
    //                                            &accelerationStructureBuildSizesInfo);

    // If previously compacted, free those resources up.
    freeCompactTree();

    // Have base class resize any backing memory allocations
    resizeFullTree(accelerationStructureBuildSizesInfo);

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;

    if (isBottomLevel)
      accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    else
      accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    else {
      LOG_ERROR("build mode not recognized!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    accelerationBuildGeometryInfo.dstAccelerationStructure = accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = getScratchDeviceAddress();

    // Build the acceleration structure on the device via a one-time command
    // buffer submission Some implementations may support acceleration structure
    // building on the host
    // (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands),
    // but we prefer device builds VkCommandBuffer commandBuffer =
    // vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(context->graphicsCommandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(context->graphicsCommandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    // VkFenceCreateInfo fenceInfo {};
    // fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    // fenceInfo.flags = 0;
    // VkFence fence;
    // err = vkCreateFence(logicalDevice, &fenceInfo, nullptr, &fence);
    // if (err) LOG_ERROR("failed to create fence for triangle accel build! :
    // \n" + errorString(err));

    err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for triangle accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(context->graphicsQueue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for triangle accel build! : \n" + errorString(err));

    // Wait for the fence to signal that command buffer has finished executing
    // err = vkWaitForFences(logicalDevice, 1, &fence, VK_TRUE, 100000000000
    // /*timeout*/); if (err) LOG_ERROR("failed to wait for fence for triangle
    // accel build! : \n" + errorString(err)); vkDestroyFence(logicalDevice,
    // fence, nullptr);

    // update last used build modes
    this->buildMode = buildMode;
    this->minimizeMemory = minimizeMemory;
    this->allowCompaction = allowCompaction;
    this->isCompact = false;

    // If we're minimizing memory usage, free scratch now
    // Otherwise, we keep it around to enable faster builts
    if (minimizeMemory)
      freeScratchBuffer();
  }

  virtual void build(GPRTBuildMode mode, bool allowCompaction, bool minimizeMemory) {};
  virtual void update() {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }
    if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE) {
      LOG_ERROR("Previous build mode must support updates!");
    }

    VkResult err;

    // if we previously minimized memory, we need to reallocate our scratch buffer...
    if (minimizeMemory) {
      // Get size info
      VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
      accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;

      if (isBottomLevel)
        accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
      else
        accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

      if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
        LOG_ERROR("build mode is uninitialized!");
      } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
        accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                                       VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
      else {
        LOG_ERROR("build mode not recognized!");
      }

      if (minimizeMemory) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
      }
      if (allowCompaction) {
        accelerationStructureBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
      }
      accelerationStructureBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
      accelerationStructureBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();

      VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
      accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
      gprt::vkGetAccelerationStructureBuildSizes(
          context->logicalDevice, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
          &accelerationStructureBuildGeometryInfo, maxPrimitiveCounts.data(), &accelerationStructureBuildSizesInfo);

      resizeScratchBuffer(accelerationStructureBuildSizesInfo.buildScratchSize);
    }

    VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
    accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;

    if (isBottomLevel)
      accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    else
      accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("build mode is uninitialized!");
    } else if (buildMode == GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else if (buildMode == GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE)
      accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR |
                                            VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
    else {
      LOG_ERROR("build mode unsupported!");
    }

    if (minimizeMemory) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_LOW_MEMORY_BIT_KHR;
    }
    if (allowCompaction) {
      accelerationBuildGeometryInfo.flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR;
    }
    accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
    accelerationBuildGeometryInfo.srcAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.dstAccelerationStructure =
        (isCompact) ? compactAccelerationStructure : accelerationStructure;
    accelerationBuildGeometryInfo.geometryCount = (uint32_t) accelerationStructureGeometries.size();
    accelerationBuildGeometryInfo.pGeometries = accelerationStructureGeometries.data();
    accelerationBuildGeometryInfo.scratchData.deviceAddress = getScratchDeviceAddress();

    VkCommandBufferBeginInfo cmdBufInfo{};
    cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);
    if (err)
      LOG_ERROR("failed to begin command buffer for triangle accel build! : \n" + errorString(err));

    gprt::vkCmdBuildAccelerationStructures(context->graphicsCommandBuffer, 1, &accelerationBuildGeometryInfo,
                                           accelerationBuildStructureRangeInfoPtrs.data());

    err = vkEndCommandBuffer(context->graphicsCommandBuffer);
    if (err)
      LOG_ERROR("failed to end command buffer for triangle accel build! : \n" + errorString(err));

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.pNext = NULL;
    submitInfo.waitSemaphoreCount = 0;
    submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
    submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
    submitInfo.signalSemaphoreCount = 0;
    submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

    err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    if (err)
      LOG_ERROR("failed to submit to queue for AABB accel build! : \n" + errorString(err));

    err = vkQueueWaitIdle(context->graphicsQueue);
    if (err)
      LOG_ERROR("failed to wait for queue idle for AABB accel build! : \n" + errorString(err));

    // Don't need to update device address, as neither our VkAccelerationStructure has not changed.
    // updateDeviceAddress(isCompact);

    // If we're minimizing memory usage, free scratch now
    if (minimizeMemory)
      freeScratchBuffer();
  };
  virtual void compact() {
    if (buildMode == GPRT_BUILD_MODE_UNINITIALIZED) {
      LOG_ERROR("Tree not previously built!");
    }
    if (!allowCompaction) {
      LOG_ERROR("Tree must have previously been built with compaction allowed.");
    }
    if (isCompact)
      return;   // tree is already compact.

    // Get size for compacted structure.
    VkDeviceSize compactedSize = queryCompactSize();

    // Allocate compact buffer and compact acceleration structure
    resizeCompactTree(compactedSize);

    // Finally, do the compaction of the full tree into the allocated compact tree
    compactTree();

    // free the original tree and buffer
    freeFullTree();

    // Track that current accel is now compact.
    this->isCompact = true;
  };
  virtual void destroy() {
    if (accelerationStructure) {
      gprt::vkDestroyAccelerationStructure(context->logicalDevice, accelerationStructure, nullptr);
      accelerationStructure = VK_NULL_HANDLE;
    }

    if (compactAccelerationStructure) {
      gprt::vkDestroyAccelerationStructure(context->logicalDevice, compactAccelerationStructure, nullptr);
      compactAccelerationStructure = VK_NULL_HANDLE;
    }

    if (accelBuffer) {
      accelBuffer->destroy();
      delete accelBuffer;
      accelBuffer = nullptr;
    }

    if (compactBuffer) {
      compactBuffer->destroy();
      delete compactBuffer;
      compactBuffer = nullptr;
    }

    if (scratchBuffer) {
      scratchBuffer->destroy();
      delete scratchBuffer;
      scratchBuffer = nullptr;
    }

    // Free slot for use by subsequently made accels
    context->accels[address] = nullptr;
  };
  virtual size_t getSize() {
    size_t size = 0;
    if (accelBuffer)
      size += accelBuffer->getSize();
    if (compactBuffer)
      size += compactBuffer->getSize();
    if (scratchBuffer)
      size += scratchBuffer->getSize();
    return size;
  };
  virtual AccelType getType() { return GPRT_UNKNOWN_ACCEL; }
};

struct TriangleAccel : public Accel {
  TriangleAccel(Context *context, std::vector<TriangleGeom*> geometries) : Accel(context, true) {
    this->geometries.resize(geometries.size());
    memcpy(this->geometries.data(), geometries.data(), sizeof(GPRTGeom *) * geometries.size());
  };

  ~TriangleAccel() {};

  AccelType getType() { return GPRT_TRIANGLE_ACCEL; }

  void build(GPRTBuildMode buildMode, bool allowCompaction, bool minimizeMemory) {
    this->buildMode = buildMode;

    accelerationBuildStructureRangeInfos.resize(geometries.size());
    accelerationBuildStructureRangeInfoPtrs.resize(geometries.size());
    accelerationStructureGeometries.resize(geometries.size());
    maxPrimitiveCounts.resize(geometries.size());
    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      TriangleGeom *triGeom = (TriangleGeom *) geometries[gid];
      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
      // geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
      //   means, anyhit shader is disabled

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on
      // build...

      geom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
      geom.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;

      // vertex data
      geom.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
      geom.geometry.triangles.vertexData.deviceAddress =
          triGeom->vertex.buffers[0]->deviceAddress + triGeom->vertex.offset;
      geom.geometry.triangles.vertexStride = triGeom->vertex.stride;
      geom.geometry.triangles.maxVertex = triGeom->vertex.count;

      // index data
      geom.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
      // note, offset accounted for in range
      geom.geometry.triangles.indexData.deviceAddress = triGeom->index.buffer->deviceAddress;
      maxPrimitiveCounts[gid] = triGeom->index.count;

      // transform data
      // note, offset accounted for in range
      geom.geometry.triangles.transformData.hostAddress = nullptr;
      // if the above is null, then that indicates identity

      auto &geomRange = accelerationBuildStructureRangeInfos[gid];
      accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
      geomRange.primitiveCount = triGeom->index.count;
      geomRange.primitiveOffset = triGeom->index.offset;
      geomRange.firstVertex = triGeom->index.firstVertex;
      geomRange.transformOffset = 0;
    }

    innerBuildProc(buildMode, allowCompaction, minimizeMemory);
  }
};

struct SphereAccel : public Accel {
#ifdef VK_NV_ray_tracing_linear_swept_spheres
  std::vector<VkAccelerationStructureGeometrySpheresDataNV> accelerationStructureGeometrySpheres;
#endif

  // AABBs for when we need to fall back to a software implementation
  GPRTBufferOf<float3> fallbackAABBs = nullptr;
  std::vector<uint32_t> fallbackAABBOffsets;

  SphereAccel(Context *context, std::vector<SphereGeom*> geometries) : Accel(context, true) {
    this->geometries.resize(geometries.size());
    memcpy(this->geometries.data(), geometries.data(), sizeof(GPRTGeom *) * geometries.size());

    // If we don't have hardware acceleration for spheres, fall back to AABBs
    if (!requestedFeatures.linearSweptSpheres) {
      fallbackAABBOffsets.resize(geometries.size() + 1);
      // Placeholder. The actual allocation here will vary from build to build.
      fallbackAABBs = gprtDeviceBufferCreate<float3>((GPRTContext) context, 1, nullptr);
    }
  };

  ~SphereAccel() {};

  AccelType getType() { return GPRT_SPHERE_ACCEL; }

  void build(GPRTBuildMode buildMode, bool allowCompaction, bool minimizeMemory) {
    this->buildMode = buildMode;

    maxPrimitiveCounts.resize(geometries.size());
    accelerationBuildStructureRangeInfos.resize(geometries.size());
    accelerationBuildStructureRangeInfoPtrs.resize(geometries.size());
    accelerationStructureGeometries.resize(geometries.size());
#ifdef VK_NV_ray_tracing_linear_swept_spheres
    accelerationStructureGeometrySpheres.resize(geometries.size());
#endif

    if (!requestedFeatures.linearSweptSpheres) {
      // Do a prefix sum over the sphere counts
      fallbackAABBOffsets[0] = 0;
      for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
        auto &geom = accelerationStructureGeometries[gid];
        SphereGeom *sphereGeom = (SphereGeom *) geometries[gid];
        fallbackAABBOffsets[gid + 1] = sphereGeom->vertex.count + fallbackAABBOffsets[gid];
      }

      // Resize the AABB buffer if needed...
      size_t requiredBytesForAABBs = 2 * sizeof(float3) * fallbackAABBOffsets[geometries.size()];
      if (gprtBufferGetSize(fallbackAABBs) != requiredBytesForAABBs) {
        gprtBufferResize((GPRTContext) context, fallbackAABBs, fallbackAABBOffsets[geometries.size()] * 2, false);
      }

      // Now populate the AABB buffer using the fallback bounds kernel
      auto SphereBounds = (GPRTComputeOf<SphereBoundsParameters>) context->internalComputePrograms["SphereBounds"];
      for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
        auto &geom = accelerationStructureGeometries[gid];
        SphereGeom *sphereGeom = (SphereGeom *) geometries[gid];

        SphereBoundsParameters params;
        params.aabbs = gprtBufferGetDevicePointer(fallbackAABBs);
        params.vertices = (float4 *) sphereGeom->vertex.buffers[0]->getDeviceAddress();
        params.offset = fallbackAABBOffsets[gid];
        params.count = fallbackAABBOffsets[gid + 1] - params.offset;
        gprtComputeLaunch(SphereBounds, uint3(((params.count + 255) / 256), 1, 1), uint3(256, 1, 1), params);
      }
    }

    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      SphereGeom *sphereGeom = (SphereGeom *) geometries[gid];

      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on
      // build...

      #ifdef VK_NV_ray_tracing_linear_swept_spheres
      // If we have hardware accelerated support for LSS, use the built-in type
      if (requestedFeatures.linearSweptSpheres) {
        // Specify that the geometry type is LSS
        geom.geometryType = VkGeometryTypeKHR::VK_GEOMETRY_TYPE_SPHERES_NV;

        // Pass the sphere structure into the pNext of the geometry
        VkAccelerationStructureGeometrySpheresDataNV &sphereData = accelerationStructureGeometrySpheres[gid];
        geom.pNext = &sphereData;

        // Fill out the LSS data
        sphereData.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_SPHERES_DATA_NV;
        sphereData.pNext = nullptr;

        // Vertex data (assuming an XYZR float4 format)
        sphereData.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        sphereData.vertexData.deviceAddress = sphereGeom->vertex.buffers[0]->deviceAddress + sphereGeom->vertex.offset;
        sphereData.vertexStride = sizeof(float4);
        
        sphereData.radiusFormat = VK_FORMAT_R32_SFLOAT;
        sphereData.radiusData.deviceAddress =
            sphereGeom->vertex.buffers[0]->deviceAddress + sphereGeom->vertex.offset + sizeof(float3);
        sphereData.radiusStride = sizeof(float4);

        // Index data
        sphereData.indexData = {};
        sphereData.indexStride = sizeof(uint32_t);
        sphereData.indexType = VK_INDEX_TYPE_NONE_KHR;

        maxPrimitiveCounts[gid] = sphereGeom->vertex.count;

        auto &geomRange = accelerationBuildStructureRangeInfos[gid];
        accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
        geomRange.primitiveCount = sphereGeom->vertex.count;
        geomRange.primitiveOffset = sphereGeom->vertex.offset;
        geomRange.firstVertex = 0;
        geomRange.transformOffset = 0;
      } else
#endif
      {
        geom.geometryType = VkGeometryTypeKHR::VK_GEOMETRY_TYPE_AABBS_KHR;

        // aabb data
        size_t offsetInBytes = fallbackAABBOffsets[gid] * 2 * sizeof(float3);
        geom.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
        geom.geometry.aabbs.pNext = VK_NULL_HANDLE;
        geom.geometry.aabbs.stride = 2 * sizeof(float3);
        geom.geometry.aabbs.data.deviceAddress = (VkDeviceAddress) gprtBufferGetDevicePointer(fallbackAABBs);
        // ((VkDeviceAddress) ( + offsetInBytes));

        auto &geomRange = accelerationBuildStructureRangeInfos[gid];
        accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
        geomRange.primitiveCount = sphereGeom->vertex.count;
        // geomRange.primitiveOffset = lssGeom->aabb.offset;
        geomRange.primitiveOffset = fallbackAABBOffsets[gid];
        geomRange.firstVertex = 0;   // unused
        geomRange.transformOffset = 0;
      }

      maxPrimitiveCounts[gid] = sphereGeom->vertex.count;
    }

    innerBuildProc(buildMode, allowCompaction, minimizeMemory);
  }
};

// todo, implement refit...
struct LSSAccel : public Accel {
  #ifdef VK_NV_ray_tracing_linear_swept_spheres
  std::vector<VkAccelerationStructureGeometryLinearSweptSpheresDataNV> accelerationStructureGeometryLinearSweptSpheres;
  #endif

  // AABBs for when we need to fall back to a software implementation
  GPRTBufferOf<float3> fallbackAABBs = nullptr;
  std::vector<uint32_t> fallbackAABBOffsets;

  bool useEndCaps;

  bool useHWIntersector;

  LSSAccel(Context *context, std::vector<LSSGeom*> geometries, GPRTLSSFlags flags) : Accel(context, true) {
    this->geometries.resize(geometries.size());
    memcpy(this->geometries.data(), geometries.data(), sizeof(GPRTGeom *) * geometries.size());

    useEndCaps = ((flags & GPRT_LSS_CHAINED_END_CAPS) != 0);

    // If we don't have hardware acceleration for LSS, fall back to AABBs
    if (!requestedFeatures.linearSweptSpheres) {
      fallbackAABBOffsets.resize(geometries.size() + 1);
      // Placeholder. The actual allocation here will vary from build to build.
      fallbackAABBs = gprtDeviceBufferCreate<float3>((GPRTContext) context, 1, nullptr);
    }
    else {
      useHWIntersector = true;

    }
  };

  ~LSSAccel() {};

  AccelType getType() { return GPRT_LSS_ACCEL; }

  void build(GPRTBuildMode buildMode, bool allowCompaction, bool minimizeMemory) {
    this->buildMode = buildMode;

    maxPrimitiveCounts.resize(geometries.size());
    accelerationBuildStructureRangeInfos.resize(geometries.size());
    accelerationBuildStructureRangeInfoPtrs.resize(geometries.size());
    accelerationStructureGeometries.resize(geometries.size());

    #ifdef VK_NV_ray_tracing_linear_swept_spheres
    accelerationStructureGeometryLinearSweptSpheres.resize(geometries.size());
    #endif

    if (!requestedFeatures.linearSweptSpheres) {
      // Do a prefix sum over the LSS counts
      fallbackAABBOffsets[0] = 0;
      for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
        auto &geom = accelerationStructureGeometries[gid];
        LSSGeom *lssGeom = (LSSGeom *) geometries[gid];
        fallbackAABBOffsets[gid + 1] = lssGeom->index.count + fallbackAABBOffsets[gid];
      }

      // Resize the AABB buffer if needed...
      size_t requiredBytesForAABBs = 2 * sizeof(float3) * fallbackAABBOffsets[geometries.size()];
      if (gprtBufferGetSize(fallbackAABBs) != requiredBytesForAABBs) {
        gprtBufferResize((GPRTContext) context, fallbackAABBs, fallbackAABBOffsets[geometries.size()] * 2, false);
      }

      // Now populate the AABB buffer using the fallback bounds kernel
      auto LSSBounds = (GPRTComputeOf<LSSBoundsParameters>) context->internalComputePrograms["LSSBounds"];
      for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
        auto &geom = accelerationStructureGeometries[gid];
        LSSGeom *lssGeom = (LSSGeom *) geometries[gid];

        LSSBoundsParameters params;
        params.aabbs = gprtBufferGetDevicePointer(fallbackAABBs);
        params.vertices = (float4 *) lssGeom->vertex.buffers[0]->getDeviceAddress();
        params.indices = (uint2 *) lssGeom->index.buffer->getDeviceAddress();
        params.offset = fallbackAABBOffsets[gid];
        params.count = fallbackAABBOffsets[gid + 1] - params.offset;
        params.endcap0 = lssGeom->endcap0;
        params.endcap1 = lssGeom->endcap1;
        gprtComputeLaunch(LSSBounds, uint3(((params.count + 255) / 256), 1, 1), uint3(256, 1, 1), params);
      }
    }

    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      LSSGeom *lssGeom = (LSSGeom *) geometries[gid];

      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on
      // build...

#ifdef VK_NV_ray_tracing_linear_swept_spheres
      // If we have hardware accelerated support for LSS, use the built-in type
      if (requestedFeatures.linearSweptSpheres) {
        // Specify that the geometry type is LSS
        geom.geometryType = VkGeometryTypeKHR::VK_GEOMETRY_TYPE_LINEAR_SWEPT_SPHERES_NV;

        // Pass the LSS structure into the pNext of the geometry
        VkAccelerationStructureGeometryLinearSweptSpheresDataNV &lssData =
            accelerationStructureGeometryLinearSweptSpheres[gid];
        geom.pNext = &lssData;

        // Fill out the LSS data
        lssData.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_LINEAR_SWEPT_SPHERES_DATA_NV;
        lssData.pNext = nullptr;

        // Vertex data (assuming an XYZR float4 format)
        lssData.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        lssData.vertexData.deviceAddress = lssGeom->vertex.buffers[0]->deviceAddress + lssGeom->vertex.offset;
        lssData.vertexStride = sizeof(float4);
        lssData.radiusFormat = VK_FORMAT_R32_SFLOAT;
        lssData.radiusData.deviceAddress =
            lssGeom->vertex.buffers[0]->deviceAddress + lssGeom->vertex.offset + sizeof(float3);
        lssData.radiusStride = sizeof(float4);

        // Index data
        lssData.indexType = VK_INDEX_TYPE_UINT32;
        lssData.indexData.deviceAddress = lssGeom->index.buffer->deviceAddress;
        lssData.indexStride = sizeof(uint32_t);//sizeof(uint2);
        lssData.indexingMode = VK_RAY_TRACING_LSS_INDEXING_MODE_LIST_NV;   // could also be successive

        // Can also be
        lssData.endCapsMode = (useEndCaps) ? VK_RAY_TRACING_LSS_PRIMITIVE_END_CAPS_MODE_CHAINED_NV : 
                                             VK_RAY_TRACING_LSS_PRIMITIVE_END_CAPS_MODE_NONE_NV;

        auto &geomRange = accelerationBuildStructureRangeInfos[gid];
        accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
        geomRange.primitiveCount = lssGeom->index.count;
        geomRange.primitiveOffset = lssGeom->index.offset;
        geomRange.firstVertex = lssGeom->index.firstVertex;
        geomRange.transformOffset = 0;
      }
      // Otherwise, fall back to AABBs and software isect.
      else
#endif
      {
        geom.geometryType = VkGeometryTypeKHR::VK_GEOMETRY_TYPE_AABBS_KHR;

        // aabb data
        size_t offsetInBytes = fallbackAABBOffsets[gid] * 2 * sizeof(float3);
        geom.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
        geom.geometry.aabbs.pNext = VK_NULL_HANDLE;
        geom.geometry.aabbs.stride = 2 * sizeof(float3);
        geom.geometry.aabbs.data.deviceAddress = (VkDeviceAddress) gprtBufferGetDevicePointer(fallbackAABBs);
        // ((VkDeviceAddress) ( + offsetInBytes));

        auto &geomRange = accelerationBuildStructureRangeInfos[gid];
        accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
        geomRange.primitiveCount = lssGeom->index.count;
        // geomRange.primitiveOffset = lssGeom->aabb.offset;
        geomRange.primitiveOffset = fallbackAABBOffsets[gid];
        geomRange.firstVertex = 0;   // unused
        geomRange.transformOffset = 0;
      }

      maxPrimitiveCounts[gid] = lssGeom->index.count;
    }

    innerBuildProc(buildMode, allowCompaction, minimizeMemory);
  }
};

struct SolidAccel : public Accel {
  // AABBs for when we need to fall back to a software implementation
  GPRTBufferOf<float4> AABBs = nullptr;
  std::vector<uint32_t> AABBOffsets;

  SolidAccel(Context *context, std::vector<SolidGeom*> geometries) : Accel(context, true) {
    this->geometries.resize(geometries.size());
    memcpy(this->geometries.data(), geometries.data(), sizeof(GPRTGeom *) * geometries.size());

    {
      AABBOffsets.resize(geometries.size() + 1);
      // Placeholder. The actual allocation here will vary from build to build.
      AABBs = gprtDeviceBufferCreate<float4>((GPRTContext) context, 1, nullptr);
    }
  };

  ~SolidAccel() {};

  AccelType getType() { return GPRT_SOLID_ACCEL; }

  void build(GPRTBuildMode buildMode, bool allowCompaction, bool minimizeMemory) {
    this->buildMode = buildMode;

    accelerationBuildStructureRangeInfos.resize(geometries.size());
    accelerationBuildStructureRangeInfoPtrs.resize(geometries.size());
    accelerationStructureGeometries.resize(geometries.size());
    maxPrimitiveCounts.resize(geometries.size());

    {
      // Do a prefix sum over the prim counts
      AABBOffsets[0] = 0;
      for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
        auto &geom = accelerationStructureGeometries[gid];
        SolidGeom *solidGeom = (SolidGeom *) geometries[gid];
        AABBOffsets[gid + 1] = solidGeom->index.count + AABBOffsets[gid];
      }

      // Resize the AABB buffer if needed...
      size_t requiredBytesForAABBs = 2 * sizeof(float4) * AABBOffsets[geometries.size()];
      if (gprtBufferGetSize(AABBs) != requiredBytesForAABBs) {
        gprtBufferResize((GPRTContext) context, AABBs, AABBOffsets[geometries.size()] * 2, false);
      }

      // Now populate the AABB buffer using the fallback bounds kernel
      auto SolidBounds = (GPRTComputeOf<SolidParameters>) context->internalComputePrograms["SolidBounds"];
      for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
        auto &geom = accelerationStructureGeometries[gid];
        SolidGeom *solidGeom = (SolidGeom *) geometries[gid];

        SolidParameters params;
        params.aabbs = gprtBufferGetDevicePointer(AABBs);
        params.vertices = (float4 *) solidGeom->vertex.buffers[0]->getDeviceAddress();
        params.indices = (uint4 *) solidGeom->index.buffer->getDeviceAddress();
        params.types = (uint8_t *) solidGeom->types.buffer->getDeviceAddress();
        params.verticesOffset = solidGeom->vertex.offset;
        params.verticesStride = solidGeom->vertex.stride;
        params.indicesOffset = solidGeom->index.offset;
        params.indicesStride = solidGeom->index.stride;
        params.typesOffset = solidGeom->types.offset;
        params.typesStride = solidGeom->types.stride;
        params.offset = AABBOffsets[gid];
        params.count = AABBOffsets[gid + 1] - params.offset;
        gprtComputeLaunch(SolidBounds, uint3(((params.count + 255) / 256), 1, 1), uint3(256, 1, 1), params);
      }
    }

    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      SolidGeom *solidGeom = (SolidGeom *) geometries[gid];

      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on
      // build...
      {
        geom.geometryType = VkGeometryTypeKHR::VK_GEOMETRY_TYPE_AABBS_KHR;

        // aabb data
        size_t offsetInBytes = AABBOffsets[gid] * 2 * sizeof(float4);
        geom.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
        geom.geometry.aabbs.pNext = VK_NULL_HANDLE;
        geom.geometry.aabbs.stride = 2 * sizeof(float4);
        geom.geometry.aabbs.data.deviceAddress = (VkDeviceAddress) gprtBufferGetDevicePointer(AABBs);
        // ((VkDeviceAddress) ( + offsetInBytes));

        auto &geomRange = accelerationBuildStructureRangeInfos[gid];
        accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
        geomRange.primitiveCount = solidGeom->index.count;
        // geomRange.primitiveOffset = solidGeom->aabb.offset;
        geomRange.primitiveOffset = AABBOffsets[gid];
        geomRange.firstVertex = 0;   // unused
        geomRange.transformOffset = 0;
      }

      maxPrimitiveCounts[gid] = solidGeom->index.count;
    }

    innerBuildProc(buildMode, allowCompaction, minimizeMemory);
  }

  //   for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
  //     auto &geom = accelerationStructureGeometries[gid];
  //     AABBGeom *aabbGeom = (AABBGeom *) geometries[gid];

  //     geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  //     // geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
  //     //   means, anyhit shader is disabled

  //     // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
  //     //   means, anyhit should only be called once.
  //     //   If absent, then an anyhit shader might be called more than once...
  //     geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
  //     // apparently, geom.flags can't be 0, otherwise we get a device loss on
  //     // build...

  //     geom.geometryType = VK_GEOMETRY_TYPE_AABBS_KHR;

  //     // aabb data
  //     geom.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
  //     geom.geometry.aabbs.pNext = VK_NULL_HANDLE;
  //     geom.geometry.aabbs.data.deviceAddress = aabbGeom->aabb.buffers[0]->deviceAddress;
  //     geom.geometry.aabbs.stride = aabbGeom->aabb.stride;

  //     auto &geomRange = accelerationBuildStructureRangeInfos[gid];
  //     accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
  //     geomRange.primitiveCount = aabbGeom->aabb.count;
  //     geomRange.primitiveOffset = aabbGeom->aabb.offset;
  //     geomRange.firstVertex = 0;   // unused
  //     geomRange.transformOffset = 0;

  //     maxPrimitiveCounts[gid] = aabbGeom->aabb.count;
  //   }

  //   innerBuildProc(buildMode, allowCompaction, minimizeMemory);
  // }
};

struct AABBAccel : public Accel {
  AABBAccel(Context *context, std::vector<AABBGeom*> geometries) : Accel(context, true) {
    this->geometries.resize(geometries.size());
    memcpy(this->geometries.data(), geometries.data(), sizeof(GPRTGeom *) * geometries.size());
  };

  ~AABBAccel() {};

  AccelType getType() { return GPRT_AABB_ACCEL; }

  void build(GPRTBuildMode buildMode, bool allowCompaction, bool minimizeMemory) {
    this->buildMode = buildMode;

    accelerationBuildStructureRangeInfos.resize(geometries.size());
    accelerationBuildStructureRangeInfoPtrs.resize(geometries.size());
    accelerationStructureGeometries.resize(geometries.size());
    maxPrimitiveCounts.resize(geometries.size());

    for (uint32_t gid = 0; gid < geometries.size(); ++gid) {
      auto &geom = accelerationStructureGeometries[gid];
      AABBGeom *aabbGeom = (AABBGeom *) geometries[gid];

      geom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
      // geom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
      //   means, anyhit shader is disabled

      // geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      //   means, anyhit should only be called once.
      //   If absent, then an anyhit shader might be called more than once...
      geom.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;
      // apparently, geom.flags can't be 0, otherwise we get a device loss on
      // build...

      geom.geometryType = VK_GEOMETRY_TYPE_AABBS_KHR;

      // aabb data
      geom.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
      geom.geometry.aabbs.pNext = VK_NULL_HANDLE;
      geom.geometry.aabbs.data.deviceAddress = aabbGeom->aabb.buffers[0]->deviceAddress;
      geom.geometry.aabbs.stride = aabbGeom->aabb.stride;

      auto &geomRange = accelerationBuildStructureRangeInfos[gid];
      accelerationBuildStructureRangeInfoPtrs[gid] = &accelerationBuildStructureRangeInfos[gid];
      geomRange.primitiveCount = aabbGeom->aabb.count;
      geomRange.primitiveOffset = aabbGeom->aabb.offset;
      geomRange.firstVertex = 0;   // unused
      geomRange.transformOffset = 0;

      maxPrimitiveCounts[gid] = aabbGeom->aabb.count;
    }

    innerBuildProc(buildMode, allowCompaction, minimizeMemory);
  }
};

struct InstanceAccel : public Accel {
  uint32_t numInstances = -1;

  // externally assigned
  Buffer *instancesBuffer = nullptr;

  // std::vector<Accel *> instances;

  // the total number of geometries referenced by this instance accel's BLASes
  uint32_t numGeometries = 0;

  // caching for faster updates
  size_t instanceOffset = -1;

  // todo, accept this in constructor
  VkBuildAccelerationStructureFlagsKHR flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

  InstanceAccel(Context *context, uint32_t numInstances, Buffer *instancesBuffer) : Accel(context, false) {
    this->numInstances = numInstances;
    this->instancesBuffer = instancesBuffer;
  }

  ~InstanceAccel() {};

  void setNumGeometries(uint32_t numGeometries) { this->numGeometries = numGeometries; }

  uint32_t getNumGeometries() {
    if (this->numGeometries == -1) {
      LOG_ERROR("Error, numGeometries for this instance must be set by the user!");
    }
    return this->numGeometries;
  }

  AccelType getType() { return GPRT_INSTANCE_ACCEL; }

  void updateSBTOffsets() {
    instanceOffset = 0;
    // for (uint32_t i = 0; i < context->accels.size(); ++i) {
    //   if (context->accels[i] == this)
    //     break;
    //   if (context->accels[i]->getType() == GPRT_INSTANCE_ACCEL) {
    //     InstanceAccel *instanceAccel = (InstanceAccel *) context->accels[i];
    //     size_t numGeometry = instanceAccel->getNumGeometries();
    //     instanceOffset += numGeometry * requestedFeatures.numRayTypes;
    //   }
    // }

    // instancesBuffer->map();

    // Populate SBT offsets one by one
    // int numGeometriesInTLAS = numInstances;
    // int blasOffsetWithinCurrentTLAS = 0;
    // for (uint32_t i = 0; i < numInstances; ++i) {
    //   gprt::Instance *instance = &((gprt::Instance *) instancesBuffer->mapped)[i];
    //   instance->__gprtSBTOffset = i * requestedFeatures.numRayTypes;
      // instanceOffset + blasOffsetWithinCurrentTLAS;
      // if (instance->__gprtAccelAddress != 0) {
      //   Accel *accel = context->accels[instance->__gprtInstanceIndex];

        // for now, this works. Eventually I'd like this to act more like a lookup / prefix sum...
        // int numGeometriesInAccel = accel->geometries.size();
        // numGeometriesInTLAS += accel->geometries.size();
        // blasOffsetWithinCurrentTLAS += numGeometriesInAccel * requestedFeatures.numRayTypes;
      // }
    // }
    // instancesBuffer->unmap();
    setNumGeometries(numInstances);
  }

  void build(GPRTBuildMode buildMode, bool allowCompaction, bool minimizeMemory) {
    this->buildMode = buildMode;
    updateSBTOffsets();

    VkAccelerationStructureGeometryKHR accelerationStructureGeometry{};
    accelerationStructureGeometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
    accelerationStructureGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    accelerationStructureGeometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
    accelerationStructureGeometry.geometry.instances.sType =
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
    accelerationStructureGeometry.geometry.instances.arrayOfPointers = VK_FALSE;
    accelerationStructureGeometry.geometry.instances.data.deviceAddress = instancesBuffer->deviceAddress;

    VkAccelerationStructureBuildRangeInfoKHR accelerationStructureBuildRangeInfo{};
    accelerationStructureBuildRangeInfo.primitiveCount = numInstances;
    accelerationStructureBuildRangeInfo.primitiveOffset = 0;
    accelerationStructureBuildRangeInfo.firstVertex = 0;
    accelerationStructureBuildRangeInfo.transformOffset = 0;

    this->accelerationStructureGeometries.resize(1);
    this->accelerationStructureGeometries[0] = accelerationStructureGeometry;

    this->maxPrimitiveCounts.resize(1);
    this->maxPrimitiveCounts[0] = numInstances;

    this->accelerationBuildStructureRangeInfos.resize(1);
    this->accelerationBuildStructureRangeInfos[0] = accelerationStructureBuildRangeInfo;

    this->accelerationBuildStructureRangeInfoPtrs.resize(1);
    this->accelerationBuildStructureRangeInfoPtrs[0] = &this->accelerationBuildStructureRangeInfos[0];

    innerBuildProc(buildMode, allowCompaction, minimizeMemory);
  }

  void update() {
    updateSBTOffsets();
    Accel::update();
  }
};

size_t
Context::getNumHitRecords() {
  // The total number of geometries is the number of geometries referenced
  // by each top level acceleration structure.
  int totalGeometries = 0;
  for (int accelID = 0; accelID < accels.size(); ++accelID) {
    Accel *accel = accels[accelID];
    if (!accel)
      continue;
    if (accel->getType() == GPRT_INSTANCE_ACCEL) {
      InstanceAccel *tlas = (InstanceAccel *) accel;
      totalGeometries += tlas->getNumGeometries();
    }
  }

  int numHitRecords = totalGeometries * requestedFeatures.numRayTypes;
  return numHitRecords;
}

void
Context::buildSBT(GPRTBuildSBTFlags flags) {
  auto alignedSize = [](uint32_t value, uint32_t alignment) -> uint32_t {
    return (value + alignment - 1) & ~(alignment - 1);
  };

  auto align_to = [](uint64_t val, uint64_t align) -> uint64_t { return ((val + align - 1) / align) * align; };

  // Build / update the ray tracing pipeline if required
  if (raytracingPipelineOutOfDate) {
    LOG_INFO("Building ray tracing pipeline");

    VkPushConstantRange pushConstantRange = {};
    pushConstantRange.size = PUSH_CONSTANTS_LIMIT;
    pushConstantRange.offset = 0;
    pushConstantRange.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR |
                                   VK_SHADER_STAGE_INTERSECTION_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
                                   VK_SHADER_STAGE_CALLABLE_BIT_KHR | VK_SHADER_STAGE_RAYGEN_BIT_KHR;

    VkPipelineLayoutCreateInfo pipelineLayoutCI{};
    pipelineLayoutCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;

    // testing new mutable descriptor setup
    std::vector<VkDescriptorSetLayout> layouts = {descriptorSetLayout};
    pipelineLayoutCI.setLayoutCount = (uint32_t) layouts.size();
    pipelineLayoutCI.pSetLayouts = layouts.data();

    pipelineLayoutCI.pushConstantRangeCount = 1;
    pipelineLayoutCI.pPushConstantRanges = &pushConstantRange;

    if (raytracingPipelineLayout != VK_NULL_HANDLE) {
      vkDestroyPipelineLayout(logicalDevice, raytracingPipelineLayout, nullptr);
      raytracingPipelineLayout = VK_NULL_HANDLE;
    }
    VK_CHECK_RESULT(vkCreatePipelineLayout(logicalDevice, &pipelineLayoutCI, nullptr, &raytracingPipelineLayout));

    /*
      Setup ray tracing shader groups
    */
    std::vector<VkPipelineShaderStageCreateInfo> shaderStages;
    shaderGroups.clear();

    // Ray generation stages / groups
    {
      for (auto raygen : raygens) {
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

    // Miss stages / groups
    {
      for (auto miss : misses) {
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

    // Callable stages / groups
    {
      for (auto callable : callables) {
        shaderStages.push_back(callable->shaderStage);
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
    {
      for (uint32_t i = 0; i < geomTypes.size(); ++i) {
        auto &geomType = geomTypes[i];
        VkRayTracingShaderGroupTypeKHR shaderGroupType;

        if (geomType->getKind() == GPRT_TRIANGLES)
          shaderGroupType = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
        else if (geomType->getKind() == GPRT_AABBS)
          shaderGroupType = VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR;
        else if (geomType->getKind() == GPRT_SOLIDS)
          shaderGroupType = VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR;
        else if (geomType->getKind() == GPRT_LSS) {
          if (requestedFeatures.linearSweptSpheres) {
            shaderGroupType = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_NV;   // ?
          } else {
            // AABB fallback
            shaderGroupType = VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR;
          }
        } else if (geomType->getKind() == GPRT_SPHERES) {
          if (requestedFeatures.linearSweptSpheres) {
            shaderGroupType = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_NV;   // ?
          } else {
            // AABB fallback
            shaderGroupType = VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR;
          }
        } else {
          LOG_ERROR("Unsupported geometry type found.");
        }

        for (uint32_t rayType = 0; rayType < requestedFeatures.numRayTypes; ++rayType) {
          VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
          shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
          shaderGroup.type = shaderGroupType;

          // init all to unused
          shaderGroup.generalShader = VK_SHADER_UNUSED_KHR;
          shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
          shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
          shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;

          // populate hit group programs using geometry type, recycling any previously referenced shader stages
          if (geomType->closestHitShaderUsed[rayType]) {
            shaderStages.push_back(geomType->closestHitShaderStages[rayType]);
            shaderGroup.closestHitShader = int(shaderStages.size() - 1);
          }

          if (geomType->anyHitShaderUsed[rayType]) {
            shaderStages.push_back(geomType->anyHitShaderStages[rayType]);
            shaderGroup.anyHitShader = int(shaderStages.size() - 1);
          }

          if (geomType->intersectionShaderUsed[rayType]) {
            shaderStages.push_back(geomType->intersectionShaderStages[rayType]);
            shaderGroup.intersectionShader = int(shaderStages.size() - 1);
          }
          shaderGroups.push_back(shaderGroup);
        }
      }
    }

    if (raygens.size() > 0) {
      /*
        Create the ray tracing pipeline
      */
      VkRayTracingPipelineInterfaceCreateInfoKHR pipelineInterfaceCreateInfo{};
      pipelineInterfaceCreateInfo.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_INTERFACE_CREATE_INFO_KHR;
      pipelineInterfaceCreateInfo.maxPipelineRayPayloadSize = requestedFeatures.maxRayPayloadSize;
      pipelineInterfaceCreateInfo.maxPipelineRayHitAttributeSize = requestedFeatures.maxRayHitAttributeSize;

      void* pNext = nullptr;
      #ifdef VK_NV_ray_tracing_linear_swept_spheres
      if (requestedFeatures.linearSweptSpheres) {
        VkPipelineCreateFlags2CreateInfo pipelineCreateFlags;
        pipelineCreateFlags.sType = VK_STRUCTURE_TYPE_PIPELINE_CREATE_FLAGS_2_CREATE_INFO;
        pipelineCreateFlags.flags = VK_PIPELINE_CREATE_2_RAY_TRACING_ALLOW_SPHERES_AND_LINEAR_SWEPT_SPHERES_BIT_NV ;
        pipelineCreateFlags.pNext = nullptr;
        pNext = &pipelineCreateFlags;
      }
      #endif

      VkRayTracingPipelineCreateInfoKHR rayTracingPipelineCI{};
      rayTracingPipelineCI.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
      rayTracingPipelineCI.stageCount = static_cast<uint32_t>(shaderStages.size());
      rayTracingPipelineCI.pStages = shaderStages.data();
      rayTracingPipelineCI.groupCount = static_cast<uint32_t>(shaderGroups.size());
      rayTracingPipelineCI.pGroups = shaderGroups.data();
      rayTracingPipelineCI.maxPipelineRayRecursionDepth = requestedFeatures.rayRecursionDepth;
      rayTracingPipelineCI.layout = raytracingPipelineLayout;
      rayTracingPipelineCI.pLibraryInterface = &pipelineInterfaceCreateInfo;
      rayTracingPipelineCI.pNext = pNext;

      LOG_INFO("Creating VkRayTracingPipelineCreateInfoKHR with max recursion depth of " +
               std::to_string(requestedFeatures.rayRecursionDepth) + ".");

      if (raytracingPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(logicalDevice, raytracingPipeline, nullptr);
        raytracingPipeline = VK_NULL_HANDLE;
      }
      VkResult err = gprt::vkCreateRayTracingPipelines(logicalDevice, VK_NULL_HANDLE, VK_NULL_HANDLE, 1,
                                                       &rayTracingPipelineCI, nullptr, &raytracingPipeline);
      if (err) {
        LOG_ERROR("failed to create ray tracing pipeline! Are all entrypoint names correct? \n" + errorString(err));
      }
    }

    // Mark our ray tracing pipeline as "updated".
    raytracingPipelineOutOfDate = false;
  }

  const uint32_t handleSize = rayTracingPipelineProperties.shaderGroupHandleSize;
  const uint32_t maxGroupSize = rayTracingPipelineProperties.maxShaderGroupStride;
  const uint32_t groupAlignment = rayTracingPipelineProperties.shaderGroupHandleAlignment;
  const uint32_t maxShaderRecordStride = rayTracingPipelineProperties.maxShaderGroupStride;

  if (requestedFeatures.hitRecordSize > maxGroupSize) {
    LOG_ERROR("Requested hit record size is too large! Max record size for this platform is " +
              std::to_string(maxGroupSize) + " bytes.");
  }
  if (requestedFeatures.raygenRecordSize > maxGroupSize) {
    LOG_ERROR("Requested raygen record size is too large! Max record size for this platform is " +
              std::to_string(maxGroupSize) + " bytes.");
  }
  if (requestedFeatures.missRecordSize > maxGroupSize) {
    LOG_ERROR("Requested miss record size is too large! Max record size for this platform is " +
              std::to_string(maxGroupSize) + " bytes.");
  }
  if (requestedFeatures.callableRecordSize> maxGroupSize) {
    LOG_ERROR("Requested callable record size is too large! Max record size for this platform is " +
              std::to_string(maxGroupSize) + " bytes.");
  }

  // Doubling the record size to allow GPRT to store internal facing data
  const uint32_t raygenRecordSize = alignedSize(std::min(maxGroupSize, requestedFeatures.raygenRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);
  const uint32_t hitRecordSize = alignedSize(std::min(maxGroupSize, requestedFeatures.hitRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);
  const uint32_t missRecordSize = alignedSize(std::min(maxGroupSize, requestedFeatures.missRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);
  const uint32_t callableRecordSize = alignedSize(std::min(maxGroupSize, requestedFeatures.callableRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);

  // Check here to confirm we really do have ray tracing programs. With raster support, sometimes
  // we might only have raster programs, and no RT programs.
  if (raygens.size() > 0) {
    const uint32_t groupCount = static_cast<uint32_t>(shaderGroups.size());
    const uint32_t sbtSize = groupCount * handleSize;

    std::vector<uint8_t> shaderHandleStorage(sbtSize);
    VkResult err = gprt::vkGetRayTracingShaderGroupHandles(logicalDevice, raytracingPipeline, 0, groupCount, sbtSize,
                                                           shaderHandleStorage.data());
    if (err)
      LOG_ERROR("failed to get ray tracing shader group handles! : \n" + errorString(err));

    const VkBufferUsageFlags bufferUsageFlags =
        // means we can use this buffer as a SBT
        VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR |
        // means we can get this buffer's address with vkGetBufferDeviceAddress
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
        VK_BUFFER_USAGE_TRANSFER_DST_BIT |
        // means we can use this buffer as a storage buffer resource
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    const VkMemoryPropertyFlags memoryUsageFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;   // means most efficient for
                                                                                        // device access
    // const VkMemoryPropertyFlags memoryUsageFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |   // mappable to host with
    //                                                                                        // vkMapMemory
    //                                                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;   // means "flush" and
    //                                                                                        // "invalidate" not needed

    // std::cout<<"Todo, get some smarter memory allocation working..."
    // <<std::endl;

    // allocate / resize ray generation table
    size_t numRayGens = raygens.size();
    if (raygenTable && raygenTable->size != raygenRecordSize * numRayGens) {
      raygenTable->destroy();
      delete raygenTable;
      raygenTable = nullptr;
    }
    if (!raygenTable && raygens.size() > 0) {
      raygenTable =
          new Buffer(this, bufferUsageFlags,
                     memoryUsageFlags, raygenRecordSize * numRayGens, rayTracingPipelineProperties.shaderGroupBaseAlignment);
    }

    // allocate / resize miss table
    size_t numMissProgs = misses.size();
    if (missTable && missTable->size != missRecordSize * numMissProgs) {
      missTable->destroy();
      delete missTable;
      missTable = nullptr;
    }
    if (!missTable && misses.size() > 0) {
      missTable = new Buffer(this, bufferUsageFlags,
                             memoryUsageFlags, missRecordSize * numMissProgs,
                             rayTracingPipelineProperties.shaderGroupBaseAlignment);
    }

    // allocate / resize callable table
    size_t numCallableProgs = callables.size();
    if (callableTable && callableTable->size != callableRecordSize * numCallableProgs) {
      callableTable->destroy();
      delete callableTable;
      callableTable = nullptr;
    }
    if (!callableTable && callables.size() > 0) {
      callableTable = new Buffer(this,
                                 bufferUsageFlags, memoryUsageFlags, callableRecordSize * numCallableProgs,
                                 rayTracingPipelineProperties.shaderGroupBaseAlignment);
    }

    // allocate / resize hit group table
    size_t numHitRecords = getNumHitRecords();
    if (hitgroupTable && hitgroupTable->size != hitRecordSize * numHitRecords) {
      hitgroupTable->destroy();
      delete hitgroupTable;
      hitgroupTable = nullptr;
    }
    if (!hitgroupTable && numHitRecords > 0) {
      hitgroupTable = new Buffer(this,
                                 bufferUsageFlags, memoryUsageFlags, hitRecordSize * numHitRecords,
                                 rayTracingPipelineProperties.shaderGroupBaseAlignment);
    }

    // Raygen records
    if ((flags & GPRTBuildSBTFlags::GPRT_SBT_RAYGEN) != 0 && raygens.size() > 0) {
      raygenTable->map();
      uint8_t *mapped = ((uint8_t *) (raygenTable->mapped));

      for (uint32_t idx = 0; idx < raygens.size(); ++idx) {
        size_t recordStride = raygenRecordSize;
        size_t handleStride = handleSize;

        // First, copy handle
        size_t recordOffset = recordStride * idx;
        size_t handleOffset = handleStride * idx;
        memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

        // Then, copy params following handle
        recordOffset = recordOffset + handleSize;
        uint8_t *params = mapped + recordOffset;
        RayGen *raygen = raygens[idx];
        memcpy(params, raygen->SBTRecord, raygen->recordSize);
      }
      raygenTable->unmap();
    }

    // Miss records
    if ((flags & GPRTBuildSBTFlags::GPRT_SBT_MISS) != 0 && misses.size() > 0) {
      missTable->map();
      uint8_t *mapped = ((uint8_t *) (missTable->mapped));

      for (uint32_t idx = 0; idx < misses.size(); ++idx) {
        size_t recordStride = missRecordSize;
        size_t handleStride = handleSize;

        // First, copy handle
        size_t recordOffset = recordStride * idx;   // + recordStride * numRayGens;
        size_t handleOffset = handleStride * idx + handleStride * numRayGens;
        memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

        // Then, copy params following handle
        recordOffset = recordOffset + handleSize;
        uint8_t *params = mapped + recordOffset;
        Miss *miss = misses[idx];
        memcpy(params, miss->SBTRecord, miss->recordSize);
      }

      missTable->unmap();
    }

    // Callable records
    if ((flags & GPRTBuildSBTFlags::GPRT_SBT_CALLABLE) != 0 && callables.size() > 0) {
      callableTable->map();
      uint8_t *mapped = ((uint8_t *) (callableTable->mapped));

      for (uint32_t idx = 0; idx < callables.size(); ++idx) {
        size_t recordStride = callableRecordSize;
        size_t handleStride = handleSize;

        // First, copy handle
        size_t recordOffset = recordStride * idx;   // + recordStride * numRayGens;
        size_t handleOffset = handleStride * idx + handleStride * (numRayGens + numMissProgs);
        memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

        // Then, copy params following handle
        recordOffset = recordOffset + handleSize;
        uint8_t *params = mapped + recordOffset;
        Callable *callable = callables[idx];
        memcpy(params, callable->SBTRecord, callable->recordSize);
      }

      callableTable->unmap();
    }

    // Hit records
    if ((flags & GPRTBuildSBTFlags::GPRT_SBT_HITGROUP) != 0 && numHitRecords > 0) {
      hitgroupTable->map();
      uint8_t *mapped = ((uint8_t *) (hitgroupTable->mapped));

      for (int tlasID = 0; tlasID < accels.size(); ++tlasID) {
        Accel *tlas = accels[tlasID];
        if (!tlas)
          continue;
        if (tlas->getType() == GPRT_INSTANCE_ACCEL) {
          InstanceAccel *instanceAccel = (InstanceAccel *) tlas;
          // this is an issue, because if instances can be set on device, we
          // don't have a list of instances we can iterate through and copy the
          // SBT data... So, if we have a bunch of instances set by reference on
          // device, we need to eventually do something smarter here...
          // Todo, move this setup to the device...

          bool previouslyMapped = (instanceAccel->instancesBuffer->mapped != nullptr);
          if (!previouslyMapped) instanceAccel->instancesBuffer->map();
          gprt::Instance *instances = (gprt::Instance *) instanceAccel->instancesBuffer->mapped;

          for (uint32_t blasID = 0; blasID < instanceAccel->numInstances; ++blasID) {
            gprt::Instance instance = instances[blasID];
            uint64_t accelAddr = instance.__gprtAccelAddress;
            Accel *blas = accels[instance.__gprtSBTOffset];

            // Accel *blas = instanceAccel->instances[blasID];
            if (blas->getType() != GPRT_INSTANCE_ACCEL) {
              for (uint32_t geomID = 0; geomID < blas->geometries.size(); ++geomID) {
                auto &geom = blas->geometries[geomID];

                for (uint32_t rayType = 0; rayType < requestedFeatures.numRayTypes; ++rayType) {
                  size_t recordStride = hitRecordSize;
                  size_t handleStride = handleSize;

                  // First, copy handle
                  size_t recordOffset =
                      recordStride * (rayType + requestedFeatures.numRayTypes * geomID + instance.__gprtSBTOffset);
                  size_t handleOffset =
                      handleStride * (geom->geomType->address * requestedFeatures.numRayTypes + rayType) +
                      handleStride * (numRayGens + numMissProgs + numCallableProgs);
                  memcpy(mapped + recordOffset, shaderHandleStorage.data() + handleOffset, handleSize);

                  // Then, copy params following handle
                  recordOffset = recordOffset + handleSize;
                  uint8_t *params = mapped + recordOffset;
                  memcpy(params, geom->SBTRecord, geom->recordSize);

                  // If implementing a software fallback, additionally memcpy data required for intersection testing
                  uint8_t *internalParams = params + (requestedFeatures.hitRecordSize);
                  if (geom->geomType->getKind() == GPRT_LSS && !requestedFeatures.linearSweptSpheres) {
                    LSSGeom *lss = (LSSGeom *) geom;
                    LSSParameters isectParams;
                    isectParams.vertices = (float4 *) lss->vertex.buffers[0]->getDeviceAddress();
                    isectParams.indices = (uint2 *) lss->index.buffer->getDeviceAddress();
                    isectParams.endcap0 = lss->endcap0;
                    isectParams.endcap1 = lss->endcap1;
                    isectParams.exitTest = lss->exitTest;
                    isectParams.numLSS = lss->index.count;
                    memcpy(internalParams, &isectParams, sizeof(LSSParameters));
                  }

                  if (geom->geomType->getKind() == GPRT_SPHERES && !requestedFeatures.linearSweptSpheres) {
                    SphereGeom *s = (SphereGeom *) geom;
                    SphereParameters isectParams;
                    isectParams.vertices = (float4 *) s->vertex.buffers[0]->getDeviceAddress();
                    isectParams.exitTest = s->exitTest;
                    memcpy(internalParams, &isectParams, sizeof(SphereParameters));
                  }

                  if (geom->geomType->getKind() == GPRT_SOLIDS) {
                    SolidGeom *solidGeom = (SolidGeom *) geom;
                    SolidParameters params;
                    params.vertices = (float4 *) solidGeom->vertex.buffers[0]->getDeviceAddress();
                    params.indices = (uint4 *) solidGeom->index.buffer->getDeviceAddress();
                    params.types = (uint8_t *) solidGeom->types.buffer->getDeviceAddress();
                    params.verticesOffset = solidGeom->vertex.offset;
                    params.verticesStride = solidGeom->vertex.stride;
                    params.indicesOffset = solidGeom->index.offset;
                    params.indicesStride = solidGeom->index.stride;
                    params.typesOffset = solidGeom->types.offset;
                    params.typesStride = solidGeom->types.stride;
                    memcpy(internalParams, &params, sizeof(SolidParameters));
                  }
                }
              }
            } else {
              LOG_ERROR("Instance accels referenced by other instance accels is currently unsupported.");
            }
          }

          if (!previouslyMapped) instanceAccel->instancesBuffer->unmap();
        }
      }
      hitgroupTable->unmap();
    }
  }
}

void
Context::destroy() {
  if (descriptorSet) {
    vkFreeDescriptorSets(logicalDevice, descriptorPool, 1, &descriptorSet);
    descriptorSet = nullptr;
  }

  if (descriptorSetLayout) {
    vkDestroyDescriptorSetLayout(logicalDevice, descriptorSetLayout, nullptr);
    descriptorSetLayout = nullptr;
  }

  if (descriptorPool) {
    vkDestroyDescriptorPool(logicalDevice, descriptorPool, nullptr);
    descriptorPool = nullptr;
  }
  
  if (imguiPool) {
    vkDestroyDescriptorPool(logicalDevice, imguiPool, nullptr);
    imguiPool = nullptr;
  }
  if (imgui.renderPass) {
    ImGui_ImplVulkan_Shutdown();
    vkDestroyRenderPass(logicalDevice, imgui.renderPass, nullptr);
    imgui.renderPass = nullptr;
  }
  if (imgui.frameBuffer) {
    vkDestroyFramebuffer(logicalDevice, imgui.frameBuffer, nullptr);
    imgui.frameBuffer = nullptr;
  }

  if (imageAvailableSemaphore) {
    vkDestroySemaphore(logicalDevice, imageAvailableSemaphore, nullptr);
    imageAvailableSemaphore = nullptr;
  }
  if (renderFinishedSemaphore) {
    vkDestroySemaphore(logicalDevice, renderFinishedSemaphore, nullptr);
    renderFinishedSemaphore = nullptr;
  }

  if (inFlightFence) {
    vkDestroyFence(logicalDevice, inFlightFence, nullptr);
    inFlightFence = nullptr;
  }

  if (swapchain) {
    vkDestroySwapchainKHR(logicalDevice, swapchain, nullptr);
    swapchain = nullptr;
  }
  if (window) {
    glfwDestroyWindow(window);
    glfwTerminate();
    window = nullptr;
  }
  if (surface) {
    vkDestroySurfaceKHR(instance, surface, nullptr);
    surface = nullptr;
  }

  if (raytracingPipelineLayout) {
    vkDestroyPipelineLayout(logicalDevice, raytracingPipelineLayout, nullptr);
    raytracingPipelineLayout = nullptr;
  }
  if (raytracingPipeline) {
    vkDestroyPipeline(logicalDevice, raytracingPipeline, nullptr);
    raytracingPipeline = nullptr;
  }

  // if (fillInstanceDataStage.layout) {
  //   vkDestroyPipelineLayout(logicalDevice, fillInstanceDataStage.layout, nullptr);
  //   fillInstanceDataStage.layout = nullptr;
  // }
  // if (fillInstanceDataStage.pipeline) {
  //   vkDestroyPipeline(logicalDevice, fillInstanceDataStage.pipeline, nullptr);
  //   fillInstanceDataStage.pipeline = nullptr;
  // }
  // if (fillInstanceDataStage.module) {
  //   vkDestroyShaderModule(logicalDevice, fillInstanceDataStage.module, nullptr);
  //   fillInstanceDataStage.module = nullptr;
  // }

  destroyInternalPrograms();

  if (raygenTable) {
    raygenTable->destroy();
    delete raygenTable;
    raygenTable = nullptr;
  }
  if (missTable) {
    missTable->destroy();
    delete missTable;
    missTable = nullptr;
  }
  if (hitgroupTable) {
    hitgroupTable->destroy();
    delete hitgroupTable;
    hitgroupTable = nullptr;
  }

  descriptorPoolSize = {};

  vkFreeCommandBuffers(logicalDevice, graphicsCommandPool, 1, &graphicsCommandBuffer);
  vkFreeCommandBuffers(logicalDevice, computeCommandPool, 1, &computeCommandBuffer);
  vkFreeCommandBuffers(logicalDevice, transferCommandPool, 1, &transferCommandBuffer);
  vkDestroyCommandPool(logicalDevice, graphicsCommandPool, nullptr);
  vkDestroyCommandPool(logicalDevice, computeCommandPool, nullptr);
  vkDestroyCommandPool(logicalDevice, transferCommandPool, nullptr);

  // verify these are all cleared.
  for (uint32_t i = 0; i < geomTypes.size(); ++i) {
    if (geomTypes[i] != nullptr) {
      geomTypes[i]->destroy();
    }
  }
  geomTypes.resize(0);

  for (uint32_t i = 0; i < accels.size(); ++i) {
    if (accels[i] != nullptr) {
      accels[i]->destroy();
    }
  }
  accels.resize(0);

  for (uint32_t i = 0; i < buffers.size(); ++i) {
    if (buffers[i] != nullptr) {
      buffers[i]->destroy();
    }
  }
  buffers.resize(0);

  for (uint32_t i = 0; i < imageResources.size(); ++i) {
    if (imageResources[i] != nullptr) {
      imageResources[i]->destroy();
    }
  }
  imageResources.resize(0);

  for (uint32_t i = 0; i < computes.size(); ++i) {
    if (computes[i] != nullptr) {
      computes[i]->destroy();
    }
  }
  computes.resize(0);

  for (uint32_t i = 0; i < raygens.size(); ++i) {
    if (raygens[i] != nullptr) {
      raygens[i]->destroy();
    }
  }
  raygens.resize(0);

  for (uint32_t i = 0; i < misses.size(); ++i) {
    if (misses[i] != nullptr) {
      misses[i]->destroy();
    }
  }
  misses.resize(0);

  for (uint32_t i = 0; i < modules.size(); ++i) {
    if (modules[i] != nullptr) {
      modules[i]->destroy();
    }
  }
  modules.resize(0);

  vmaDestroyAllocator(allocator);

  vkDestroyQueryPool(logicalDevice, queryPool, nullptr);
  vkDestroyQueryPool(logicalDevice, compactedSizeQueryPool, nullptr);
  vkDestroyDevice(logicalDevice, nullptr);

  if (requestedFeatures.debugPrintf)
    freeDebugCallback(instance);
  vkDestroyInstance(instance, nullptr);
}

void
Context::freeDebugCallback(VkInstance instance) {
  if (gprt::debugUtilsMessenger != VK_NULL_HANDLE) {
    gprt::vkDestroyDebugUtilsMessengerEXT(instance, gprt::debugUtilsMessenger, nullptr);
  }
}

void Context::enumerateInstanceValidationLayers()
{
  uint32_t layerCount = 0;
  VkResult result = vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
  if (result != VK_SUCCESS) LOG_ERROR("Failed to enumerate instance layers");

  if (layerCount > 0) {
    supportedInstanceLayers.resize(layerCount);
    result = vkEnumerateInstanceLayerProperties(&layerCount, supportedInstanceLayers.data());
    if (result != VK_SUCCESS) LOG_ERROR("Failed to enumerate instance layers");

    for (VkLayerProperties layer : supportedInstanceLayers) {
      supportedInstanceLayerNames.push_back(layer.layerName);
    }
  }
}

void Context::enumerateInstanceExtensions()
{
  uint32_t instExtCount = 0;
  VkResult result = vkEnumerateInstanceExtensionProperties(nullptr, &instExtCount, nullptr);
  if (result != VK_SUCCESS) LOG_ERROR("Failed to enumerate extensions");

  if (instExtCount > 0) {
    supportedInstanceExtensions.resize(instExtCount);
    if (vkEnumerateInstanceExtensionProperties(nullptr, &instExtCount, &supportedInstanceExtensions.front()) == VK_SUCCESS) {
      for (VkExtensionProperties extension : supportedInstanceExtensions) {
        supportedInstanceExtensionNames.push_back(extension.extensionName);
      }
    }
  }
}

Context::Context(int32_t *requestedDeviceIDs, int numRequestedDevices) {
  enumerateInstanceValidationLayers();
  enumerateInstanceExtensions();

  appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  appInfo.pApplicationName = "GPRT";
  appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
  appInfo.pEngineName = "GPRT";
  appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
  appInfo.apiVersion = VK_API_VERSION_1_4;
  appInfo.pNext = VK_NULL_HANDLE;

  /// 1. Create Instance
  std::vector<const char *> instanceExtensions;   // = { VK_KHR_SURFACE_EXTENSION_NAME };
#if defined(VK_USE_PLATFORM_MACOS_MVK) && (VK_HEADER_VERSION >= 216)
  instanceExtensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
  instanceExtensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
#endif

  VkValidationFeatureEnableEXT enabled[] = {VK_VALIDATION_FEATURE_ENABLE_DEBUG_PRINTF_EXT};
  VkValidationFeaturesEXT validationFeatures{VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT};
  validationFeatures.disabledValidationFeatureCount = 0;
  validationFeatures.enabledValidationFeatureCount = 1;
  validationFeatures.pDisabledValidationFeatures = nullptr;
  validationFeatures.pEnabledValidationFeatures = enabled;

  VkInstanceCreateInfo instanceCreateInfo{};
  instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instanceCreateInfo.pApplicationInfo = &appInfo;
  // instanceCreateInfo.pNext = VK_NULL_HANDLE;

  if (requestedFeatures.debugPrintf) {
    instanceCreateInfo.pNext = &validationFeatures;
  } else {
    LOG_WARNING("Debug printf disabled");
    instanceCreateInfo.pNext = VK_NULL_HANDLE;
  }

  uint32_t glfwExtensionCount = 0;
  const char **glfwExtensions;
  if (requestedFeatures.window) {
    if (!glfwInit()) {
      LOG_WARNING("Unable to create window. Falling back to headless mode.");
      requestedFeatures.window = false;
    } else {
      if (!glfwVulkanSupported()) {
        LOG_ERROR("Window requested but unsupported!");
      }
      glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
      for (uint32_t i = 0; i < glfwExtensionCount; ++i) {
        instanceExtensions.push_back(glfwExtensions[i]);
      }
    }
  }

#if defined(VK_USE_PLATFORM_MACOS_MVK) && (VK_HEADER_VERSION >= 216)
  instanceCreateInfo.flags = VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif

  // Useful to disable, since some profiling tools don't support this.
  if (requestedFeatures.debugPrintf) {
    instanceExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
  }

  if (instanceExtensions.size() > 0) {
    instanceCreateInfo.enabledExtensionCount = (uint32_t) instanceExtensions.size();
    instanceCreateInfo.ppEnabledExtensionNames = instanceExtensions.data();
  }

  // need this for printf to work
  const char *layerNames[1] = {"VK_LAYER_KHRONOS_validation"};
  instanceCreateInfo.ppEnabledLayerNames = &layerNames[0];

  if (requestedFeatures.debugPrintf) {
    // Todo, look and see if any profiling layers might be active. If so, we should disable printf.
    if (std::find(supportedInstanceLayerNames.begin(), supportedInstanceLayerNames.end(), layerNames[0]) ==
        supportedInstanceLayerNames.end()) {
      LOG_WARNING("VK_LAYER_KHRONOS_validation is not present. Device-side printf will be disabled.");
    }
    else {
      instanceCreateInfo.enabledLayerCount = 1;
    }
  } else {
    instanceCreateInfo.enabledLayerCount = 0;
  }

  VkResult err;
  err = vkCreateInstance(&instanceCreateInfo, nullptr, &instance);
  if (err) {
    LOG_ERROR("failed to create instance! : \n" + errorString(err));
  }

  /// 1.5 - create a window and surface if requested
  if (requestedFeatures.window) {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    // todo, allow the window to resize and recreate swapchain
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    window = glfwCreateWindow(requestedFeatures.windowProperties.initialWidth,
                              requestedFeatures.windowProperties.initialHeight,
                              requestedFeatures.windowProperties.title.c_str(), NULL, NULL);

    VkResult err = glfwCreateWindowSurface(instance, window, nullptr, &surface);
    if (err != VK_SUCCESS) {
      LOG_ERROR("failed to create window surface! : \n" + errorString(err));
    }
    // Poll some initial event values
    glfwPollEvents();
  }

  // Setup debug printf callback
  if (requestedFeatures.debugPrintf) {
    gprt::vkCreateDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT"));
    gprt::vkDestroyDebugUtilsMessengerEXT = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));

    VkDebugUtilsMessengerCreateInfoEXT debugUtilsMessengerCI{};
    debugUtilsMessengerCI.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    debugUtilsMessengerCI.messageSeverity =
        // VK_DEBUG_REPORT_INFORMATION_BIT_EXT
        // | VK_DEBUG_REPORT_WARNING_BIT_EXT
        // | VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT
        // | VK_DEBUG_REPORT_ERROR_BIT_EXT
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT;
    debugUtilsMessengerCI.messageType =
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT;
    debugUtilsMessengerCI.pfnUserCallback = debugUtilsMessengerCallback;
    VkResult result =
        gprt::vkCreateDebugUtilsMessengerEXT(instance, &debugUtilsMessengerCI, nullptr, &gprt::debugUtilsMessenger);
    assert(result == VK_SUCCESS);
  }

  /// 2. Select a Physical Device

  // Physical device
  uint32_t gpuCount = 0;
  // Get number of available physical devices
  VK_CHECK_RESULT(vkEnumeratePhysicalDevices(instance, &gpuCount, nullptr));
  if (gpuCount == 0) {
    LOG_ERROR("No device with Vulkan support found : \n" + errorString(err));
  }
  // Enumerate devices
  std::vector<VkPhysicalDevice> physicalDevices(gpuCount);
  err = vkEnumeratePhysicalDevices(instance, &gpuCount, physicalDevices.data());
  if (err) {
    LOG_ERROR("Could not enumerate physical devices : \n" + errorString(err));
  }

  // GPU selection

  auto physicalDeviceTypeString = [](VkPhysicalDeviceType type) -> std::string {
    switch (type) {
#define STR(r)                                                                                                         \
  case VK_PHYSICAL_DEVICE_TYPE_##r:                                                                                    \
    return #r
      STR(OTHER);
      STR(INTEGRATED_GPU);
      STR(DISCRETE_GPU);
      STR(VIRTUAL_GPU);
      STR(CPU);
#undef STR
    default:
      return "UNKNOWN_DEVICE_TYPE";
    }
  };

  auto extensionSupported = [](std::string extension, std::vector<std::string> supportedExtensions) -> bool {
    return (std::find(supportedExtensions.begin(), supportedExtensions.end(), extension) != supportedExtensions.end());
  };

  /* function that checks if the selected physical device meets requirements
   */
  struct FallbackRequests {
    bool lss = false;
  };

  auto checkDeviceExtensionSupport = [](VkPhysicalDevice device, std::vector<const char *> deviceExtensions, FallbackRequests &fallbackRequests) -> bool {
    uint32_t extensionCount;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

    std::set<std::string> requiredExtensions;
    for (auto &cstr : deviceExtensions) {
      requiredExtensions.insert(std::string(cstr));
    }

    for (const auto &extension : availableExtensions) {
      requiredExtensions.erase(extension.extensionName);
    }

    if (requestedFeatures.linearSweptSpheres) {
      if (requiredExtensions.find(VK_NV_RAY_TRACING_LINEAR_SWEPT_SPHERES_EXTENSION_NAME) != requiredExtensions.end()) {
        if (requiredExtensions.find(VK_NV_RAY_TRACING_EXTENSION_NAME) == requiredExtensions.end()) {
          requiredExtensions.erase(VK_NV_RAY_TRACING_LINEAR_SWEPT_SPHERES_EXTENSION_NAME);
          fallbackRequests.lss = true;
        }
      }
    }

    return requiredExtensions.empty();
  };

  // For -spirv-reflect support.
  enabledDeviceExtensions.push_back(VK_GOOGLE_USER_TYPE_EXTENSION_NAME);

  // enabledDeviceExtensions.push_back(VK_KHR_MAINTENANCE_5_EXTENSION_NAME);
  enabledDeviceExtensions.push_back(VK_KHR_SHADER_CLOCK_EXTENSION_NAME);

  // This makes structs follow a C-like structure. Modifies alignment rules for uniform buffers,
  // sortage buffers and push constants, allowing non-scalar types to be aligned solely based on the size of their
  // components, without additional requirements.
  enabledDeviceExtensions.push_back(VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME);

  // Allows storing different descriptor types in the same binding.
  enabledDeviceExtensions.push_back(VK_EXT_MUTABLE_DESCRIPTOR_TYPE_EXTENSION_NAME);

  // Ray tracing related extensions required
  enabledDeviceExtensions.push_back(VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME);

  // Ray tracing related extensions required
  enabledDeviceExtensions.push_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
  enabledDeviceExtensions.push_back(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME);
  enabledDeviceExtensions.push_back(VK_KHR_RAY_TRACING_POSITION_FETCH_EXTENSION_NAME);

  // Required by VK_KHR_acceleration_structure
  enabledDeviceExtensions.push_back(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  enabledDeviceExtensions.push_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
  enabledDeviceExtensions.push_back(VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME);

  enabledDeviceExtensions.push_back(VK_KHR_SHADER_NON_SEMANTIC_INFO_EXTENSION_NAME);

  // Required for VK_KHR_ray_tracing_pipeline
  enabledDeviceExtensions.push_back(VK_KHR_SPIRV_1_4_EXTENSION_NAME);

  // required for vulkan memory model stuff
  enabledDeviceExtensions.push_back(VK_KHR_VULKAN_MEMORY_MODEL_EXTENSION_NAME);

  // Required by VK_KHR_spirv_1_4
  enabledDeviceExtensions.push_back(VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME);

  // Required for floating point atomics
  enabledDeviceExtensions.push_back(VK_EXT_SHADER_ATOMIC_FLOAT_EXTENSION_NAME);

  // Required for vkCmdPushDescriptor (Now included with Vulkan 1.4)
  // enabledDeviceExtensions.push_back(VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME);

  if (requestedFeatures.window) {
    // If the device will be used for presenting to a display via a swapchain
    // we need to request the swapchain extension
    enabledDeviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
  }

  if (requestedFeatures.rayQueries) {
    // If the device will be using ray queries for inline ray tracing,
    // we need to explicitly request this.
    enabledDeviceExtensions.push_back(VK_KHR_RAY_QUERY_EXTENSION_NAME);
  }

  if (requestedFeatures.invocationReordering) {
    enabledDeviceExtensions.push_back(VK_NV_RAY_TRACING_INVOCATION_REORDER_EXTENSION_NAME);
  }

  if (requestedFeatures.linearSweptSpheres) {
    #ifdef VK_NV_ray_tracing_linear_swept_spheres
    enabledDeviceExtensions.push_back(VK_NV_RAY_TRACING_LINEAR_SWEPT_SPHERES_EXTENSION_NAME);
    #else
    LOG_WARNING("Hardware acceleration for LSS unavailable. Using software fallback.");
    requestedFeatures.linearSweptSpheres = false;
    #endif
  }

#if defined(VK_USE_PLATFORM_MACOS_MVK) && (VK_HEADER_VERSION >= 216)
  enabledDeviceExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
#endif

  // Select physical device to be used
  // Defaults to the first device unless specified by command line
  uint32_t selectedDevice = -1;   // TODO

  std::vector<uint32_t> usableDevices;
  std::vector<FallbackRequests> usableDeviceFallbackRequests;

  LOG_INFO("Searching for usable Vulkan physical device...");
  for (uint32_t i = 0; i < gpuCount; i++) {
    VkPhysicalDeviceProperties deviceProperties;
    vkGetPhysicalDeviceProperties(physicalDevices[i], &deviceProperties);

    std::string message =
        std::string("Device [") + std::to_string(i) + std::string("] : ") + std::string(deviceProperties.deviceName);
    message += std::string(", Type : ") + physicalDeviceTypeString(deviceProperties.deviceType);
    message += std::string(", API : ") + std::to_string(deviceProperties.apiVersion >> 22) + std::string(".") +
               std::to_string(((deviceProperties.apiVersion >> 12) & 0x3ff)) + std::string(".") +
               std::to_string(deviceProperties.apiVersion & 0xfff);
    LOG_INFO(message);

    std::vector<const char *> currentExtensionsList = enabledDeviceExtensions;
    if (requestedFeatures.aiDenoiser.requested && deviceProperties.vendorID == VENDOR_ID_NVIDIA) {
      // If NVIDIA, these are needed to use DLSS
      currentExtensionsList.push_back(VK_NVX_BINARY_IMPORT_EXTENSION_NAME);
      currentExtensionsList.push_back(VK_NVX_IMAGE_VIEW_HANDLE_EXTENSION_NAME);
    }
    else if (requestedFeatures.aiDenoiser.requested) {
      LOG_WARNING("\tDevice unusable... (GPRT missing AI denoising support for vendor \"" + getVendorString(deviceProperties.vendorID) + "\")");
      continue;
    }
    
    FallbackRequests fallbackRequests;
    if (checkDeviceExtensionSupport(physicalDevices[i], currentExtensionsList, fallbackRequests)) {
      usableDevices.push_back(i);
      usableDeviceFallbackRequests.push_back(fallbackRequests);
      LOG_INFO("\tFound usable device");
    } else {
      // Get list of supported extensions
      uint32_t devExtCount = 0;
      vkEnumerateDeviceExtensionProperties(physicalDevices[i], nullptr, &devExtCount, nullptr);
      std::vector<VkExtensionProperties> extensions(devExtCount);
      std::vector<std::string> supportedExtensions;
      if (vkEnumerateDeviceExtensionProperties(physicalDevices[i], nullptr, &devExtCount, &extensions.front()) ==
          VK_SUCCESS) {
        for (auto ext : extensions) {
          supportedExtensions.push_back(ext.extensionName);
        }
      }

      for (const char *enabledExtension : currentExtensionsList) {
        if (!extensionSupported(enabledExtension, supportedExtensions)) {
          LOG_WARNING("\tDevice unusable... Requested device extension \"" << enabledExtension << "\" is not present.");
        }
      }
    }
  }

  if (usableDevices.size() == 0) {
    LOG_ERROR("Unable to find physical device meeting requirements");
  } else {
    uint32_t requestedDevice = 0;

    if (numRequestedDevices == 0 || requestedDeviceIDs == nullptr) {
      LOG_INFO("Selecting first usable device");
      if (usableDeviceFallbackRequests[0].lss ) {
        LOG_WARNING("Hardware acceleration for LSS unavailable. Using software fallback.");
        requestedFeatures.linearSweptSpheres = false;
        enabledDeviceExtensions.erase(
            std::remove(enabledDeviceExtensions.begin(), enabledDeviceExtensions.end(), std::string(VK_NV_RAY_TRACING_LINEAR_SWEPT_SPHERES_EXTENSION_NAME)), 
            enabledDeviceExtensions.end()
        );
      
      }
    } else if (numRequestedDevices > 1) {
      LOG_ERROR("Multi-GPU support not yet implemented (on the backlog)");
    } else {
      requestedDevice = requestedDeviceIDs[0];
      if (requestedDevice > usableDevices.size()) {
        LOG_ERROR("Requested device is out of range!");
      }
    }
    selectedDevice = usableDevices[requestedDevice];
  }

  physicalDevice = physicalDevices[selectedDevice];

  // Store properties (including limits), features and memory properties of
  // the physical device Device properties also contain limits and sparse
  // properties
  vkGetPhysicalDeviceProperties(physicalDevice, &deviceProperties);

  if (requestedFeatures.aiDenoiser.requested && deviceProperties.vendorID == VENDOR_ID_NVIDIA) {
    // If NVIDIA, these are needed to use DLSS
    enabledDeviceExtensions.push_back(VK_NVX_BINARY_IMPORT_EXTENSION_NAME);
    enabledDeviceExtensions.push_back(VK_NVX_IMAGE_VIEW_HANDLE_EXTENSION_NAME);
  }

  subgroupProperties = {};
  subgroupProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SUBGROUP_PROPERTIES;
  subgroupProperties.pNext = NULL;

  rayTracingPipelineProperties = {};
  rayTracingPipelineProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
  rayTracingPipelineProperties.pNext = &subgroupProperties;

  deviceProperties2 = {};
  deviceProperties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
  deviceProperties2.pNext = &rayTracingPipelineProperties;
  vkGetPhysicalDeviceProperties2(physicalDevice, &deviceProperties2);

  // Features should be checked by the end application before using them
  void *pNext = nullptr;
  // VkPhysicalDeviceFloatControlsPropertiesKHR floatControlsProperties = {
  //     VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FLOAT_CONTROLS_PROPERTIES_KHR};
  // floatControlsProperties.denormBehaviorIndependence = VK_SHADER_FLOAT_CONTROLS_INDEPENDENCE_ALL;
  // floatControlsProperties.pNext = pNext;
  // pNext = &floatControlsProperties;

  shaderClockFeatures = {};
  shaderClockFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_CLOCK_FEATURES_KHR;
  shaderClockFeatures.shaderSubgroupClock = true;
  shaderClockFeatures.shaderDeviceClock = true;
  shaderClockFeatures.pNext = pNext;
  pNext = &shaderClockFeatures;

  mutableDescriptorFeatures = {};
  mutableDescriptorFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MUTABLE_DESCRIPTOR_TYPE_FEATURES_EXT;
  mutableDescriptorFeatures.mutableDescriptorType = true;
  mutableDescriptorFeatures.pNext = pNext;
  pNext = &mutableDescriptorFeatures;

  atomicFloatFeatures = {};
  atomicFloatFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_ATOMIC_FLOAT_FEATURES_EXT;
  atomicFloatFeatures.shaderBufferFloat32Atomics = true;
  atomicFloatFeatures.shaderSharedFloat32Atomics = true;
  atomicFloatFeatures.pNext = pNext;
  pNext = &atomicFloatFeatures;

  // VkPhysicalDeviceShaderAtomicFloat2FeaturesEXT atomicFloat2Features = {};
  // atomicFloat2Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_ATOMIC_FLOAT_2_FEATURES_EXT;
  // atomicFloat2Features.shaderBufferFloat32AtomicMinMax = true;
  // atomicFloat2Features.shaderSharedFloat32AtomicMinMax = true;
  // atomicFloat2Features.pNext = pNext;
  // pNext = &atomicFloat2Features;

  invocationReorderFeatures = {};
  invocationReorderFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_INVOCATION_REORDER_FEATURES_NV;
  if (requestedFeatures.rayQueries) {
    // invocationReorderFeatures.rayTracingInvocationReorder = requestedFeatures.invocationReordering;
    invocationReorderFeatures.pNext = pNext;
    pNext = &invocationReorderFeatures;
  }

  #ifdef VK_NV_ray_tracing_linear_swept_spheres
  linearSweptSpheresFeatures = {};
  linearSweptSpheresFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_LINEAR_SWEPT_SPHERES_FEATURES_NV;
  if (requestedFeatures.linearSweptSpheres) {
    linearSweptSpheresFeatures.pNext = pNext;
    pNext = &linearSweptSpheresFeatures;
  }
  #endif

  accelerationStructureFeatures = {};
  accelerationStructureFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
  accelerationStructureFeatures.pNext = pNext;
  pNext = &accelerationStructureFeatures;

  rtPipelineFeatures = {};
  rtPipelineFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
  rtPipelineFeatures.pNext = pNext;
  pNext = &rtPipelineFeatures;

  rtQueryFeatures = {};
  rtQueryFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
  if (requestedFeatures.rayQueries) {
    // rtQueryFeatures.rayQuery = requestedFeatures.rayQueries;
    rtQueryFeatures.pNext = pNext;
    pNext = &rtQueryFeatures;
  }

  deviceVulkan12Features = {};
  deviceVulkan12Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  deviceVulkan12Features.pNext = pNext;
  pNext = &deviceVulkan12Features;

  deviceVulkan11Features = {};
  deviceVulkan11Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES;
  deviceVulkan11Features.pNext = pNext;
  pNext = &deviceVulkan11Features;

  deviceFeatures2 = {};
  deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  deviceFeatures2.pNext = pNext;

  // fill in above structs
  vkGetPhysicalDeviceFeatures(physicalDevice, &deviceFeatures);
  vkGetPhysicalDeviceFeatures2(physicalDevice, &deviceFeatures2);

  // Memory properties are used regularly for creating all kinds of buffers
  vkGetPhysicalDeviceMemoryProperties(physicalDevice, &deviceMemoryProperties);

  // Queue family properties, used for setting up requested queues upon device
  // creation
  uint32_t queueFamilyCount;
  vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, nullptr);
  assert(queueFamilyCount > 0);
  queueFamilyProperties.resize(queueFamilyCount);
  vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilyProperties.data());

  // Get list of supported extensions
  uint32_t devExtCount = 0;
  vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &devExtCount, nullptr);
  if (devExtCount > 0) {
    std::vector<VkExtensionProperties> extensions(devExtCount);
    if (vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &devExtCount, &extensions.front()) ==
        VK_SUCCESS) {
      for (auto ext : extensions) {
        supportedExtensions.push_back(ext.extensionName);
      }
    }
  }

  // Now, create the logical device and all requested queues
  std::vector<VkDeviceQueueCreateInfo> queueCreateInfos{};
  // Get queue family indices for the requested queue family types
  // Note that the indices may overlap depending on the implementation

  auto getQueueFamilyIndex = [](VkQueueFlagBits queueFlags,
                                std::vector<VkQueueFamilyProperties> queueFamilyProperties) -> uint32_t {
    // Dedicated queue for compute
    // Try to find a queue family index that supports compute but not graphics
    if (queueFlags & VK_QUEUE_COMPUTE_BIT)
      for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProperties.size()); i++)
        if ((queueFamilyProperties[i].queueFlags & queueFlags) &&
            ((queueFamilyProperties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0))
          return i;

    // Dedicated queue for transfer
    // Try to find a queue family index that supports transfer but not
    // graphics and compute
    if (queueFlags & VK_QUEUE_TRANSFER_BIT)
      for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProperties.size()); i++)
        if ((queueFamilyProperties[i].queueFlags & queueFlags) &&
            ((queueFamilyProperties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0) &&
            ((queueFamilyProperties[i].queueFlags & VK_QUEUE_COMPUTE_BIT) == 0))
          return i;

    // For other queue types or if no separate compute queue is present,
    // return the first one to support the requested flags
    for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProperties.size()); i++)
      if (queueFamilyProperties[i].queueFlags & queueFlags)
        return i;
    LOG_ERROR("Could not find a matching queue family index");
    return -1;
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
    if (queueFamilyIndices.compute != queueFamilyIndices.graphics) {
      // If compute family index differs, we need an additional queue create
      // info for the compute queue
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
    if ((queueFamilyIndices.transfer != queueFamilyIndices.graphics) &&
        (queueFamilyIndices.transfer != queueFamilyIndices.compute)) {
      // If compute family index differs, we need an additional queue create
      // info for the compute queue
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

  VkDeviceCreateInfo deviceCreateInfo = {};
  deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  deviceCreateInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
  deviceCreateInfo.pQueueCreateInfos = queueCreateInfos.data();
  deviceCreateInfo.pEnabledFeatures = nullptr;   // Must be nullptr if deviceFeatures2 is used to request features
  deviceCreateInfo.pNext = &deviceFeatures2;

  // If a pNext(Chain) has been passed, we need to add it to the device
  // creation info VkPhysicalDeviceFeatures2 physicalDeviceFeatures2{}; if
  // (pNextChain) {
  //   physicalDeviceFeatures2.sType =
  //   VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  //   physicalDeviceFeatures2.features = enabledFeatures;
  //   physicalDeviceFeatures2.pNext = pNextChain;
  //   deviceCreateInfo.pEnabledFeatures = nullptr;
  //   deviceCreateInfo.pNext = &physicalDeviceFeatures2;
  // }

  // // Enable the debug marker extension if it is present (likely meaning a
  // debugging tool is present) if
  // (extensionSupported(VK_EXT_DEBUG_MARKER_EXTENSION_NAME))
  // {
  //   enabledDeviceExtensions.push_back(VK_EXT_DEBUG_MARKER_EXTENSION_NAME);
  //   enableDebugMarkers = true;
  // }

  if (enabledDeviceExtensions.size() > 0) {
    deviceCreateInfo.enabledExtensionCount = (uint32_t) enabledDeviceExtensions.size();
    deviceCreateInfo.ppEnabledExtensionNames = enabledDeviceExtensions.data();
  }

  // this->enabledFeatures = enabledFeatures;
  err = vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &logicalDevice);
  if (err) {
    LOG_ERROR("Could not create logical devices : \n" + errorString(err));
  }

  // Create vulkan memory allocator
  VmaVulkanFunctions vulkanFunctions = {};
  vulkanFunctions.vkGetInstanceProcAddr = &vkGetInstanceProcAddr;
  vulkanFunctions.vkGetDeviceProcAddr = &vkGetDeviceProcAddr;

  VmaAllocatorCreateInfo allocatorCreateInfo = {};
  allocatorCreateInfo.vulkanApiVersion = VK_API_VERSION_1_2;
  allocatorCreateInfo.physicalDevice = physicalDevice;
  allocatorCreateInfo.device = logicalDevice;
  allocatorCreateInfo.instance = instance;
  allocatorCreateInfo.pVulkanFunctions = &vulkanFunctions;
  allocatorCreateInfo.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
  err = vmaCreateAllocator(&allocatorCreateInfo, &allocator);
  if (err) {
    LOG_ERROR("Could not create vulkan memory allocator : \n" + errorString(err));
  }

  // Get the ray tracing and acceleration structure related function pointers
  // required by this sample
  gprt::vkGetBufferDeviceAddress = reinterpret_cast<PFN_vkGetBufferDeviceAddressKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkGetBufferDeviceAddressKHR"));
  gprt::vkCmdBuildAccelerationStructures = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkCmdBuildAccelerationStructuresKHR"));
  gprt::vkCmdCopyAccelerationStructure = reinterpret_cast<PFN_vkCmdCopyAccelerationStructureKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkCmdCopyAccelerationStructureKHR"));
  gprt::vkBuildAccelerationStructures = reinterpret_cast<PFN_vkBuildAccelerationStructuresKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkBuildAccelerationStructuresKHR"));
  gprt::vkCopyAccelerationStructure = reinterpret_cast<PFN_vkCopyAccelerationStructureKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkCopyAccelerationStructureKHR"));
  gprt::vkCreateAccelerationStructure = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkCreateAccelerationStructureKHR"));
  gprt::vkDestroyAccelerationStructure = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkDestroyAccelerationStructureKHR"));
  gprt::vkGetAccelerationStructureBuildSizes = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkGetAccelerationStructureBuildSizesKHR"));
  gprt::vkGetAccelerationStructureDeviceAddress = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkGetAccelerationStructureDeviceAddressKHR"));
  gprt::vkCmdTraceRays =
      reinterpret_cast<PFN_vkCmdTraceRaysKHR>(vkGetDeviceProcAddr(logicalDevice, "vkCmdTraceRaysKHR"));
  gprt::vkGetRayTracingShaderGroupHandles = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkGetRayTracingShaderGroupHandlesKHR"));
  gprt::vkCreateRayTracingPipelines = reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(
      vkGetDeviceProcAddr(logicalDevice, "vkCreateRayTracingPipelinesKHR"));
  gprt::vkCmdWriteAccelerationStructuresProperties =
      reinterpret_cast<PFN_vkCmdWriteAccelerationStructuresPropertiesKHR>(
          vkGetDeviceProcAddr(logicalDevice, "vkCmdWriteAccelerationStructuresPropertiesKHR"));

  auto createCommandPool = [&](uint32_t queueFamilyIndex,
                               VkCommandPoolCreateFlags createFlags =
                                   VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT) -> VkCommandPool {
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
  queryPoolCreateInfo.pNext = nullptr;   // Optional
  queryPoolCreateInfo.flags = 0;         // Reserved for future use, must be 0!

  queryPoolCreateInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
  queryPoolCreateInfo.queryCount = 2;   // for now, just two queries, a before and an after.

  err = vkCreateQueryPool(logicalDevice, &queryPoolCreateInfo, nullptr, &queryPool);
  if (err)
    LOG_ERROR("Failed to create time query pool!\n" + errorString(err));

  // We also need a query pool to determine compacted tree sizes
  VkQueryPoolCreateInfo compactedSizeQueryPoolCreateInfo{};
  compactedSizeQueryPoolCreateInfo.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
  compactedSizeQueryPoolCreateInfo.pNext = nullptr;   // Optional
  compactedSizeQueryPoolCreateInfo.flags = 0;         // Reserved for future use, must be 0!

  compactedSizeQueryPoolCreateInfo.queryType = VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR;
  compactedSizeQueryPoolCreateInfo.queryCount = 1;   // for now, just one query

  err = vkCreateQueryPool(logicalDevice, &compactedSizeQueryPoolCreateInfo, nullptr, &compactedSizeQueryPool);
  if (err)
    LOG_ERROR("Failed to create time query pool!\n" + errorString(err));

  /// 5. Allocate command buffers
  VkCommandBufferAllocateInfo cmdBufAllocateInfo{};
  cmdBufAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  cmdBufAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  cmdBufAllocateInfo.commandBufferCount = 1;

  cmdBufAllocateInfo.commandPool = graphicsCommandPool;
  err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &graphicsCommandBuffer);
  if (err)
    LOG_ERROR("Could not create graphics command buffer : \n" + errorString(err));

  cmdBufAllocateInfo.commandPool = computeCommandPool;
  err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &computeCommandBuffer);
  if (err)
    LOG_ERROR("Could not create compute command buffer : \n" + errorString(err));

  cmdBufAllocateInfo.commandPool = transferCommandPool;
  err = vkAllocateCommandBuffers(logicalDevice, &cmdBufAllocateInfo, &transferCommandBuffer);
  if (err)
    LOG_ERROR("Could not create transfer command buffer : \n" + errorString(err));

  /// 6. Get queue handles
  vkGetDeviceQueue(logicalDevice, queueFamilyIndices.graphics, 0, &graphicsQueue);
  vkGetDeviceQueue(logicalDevice, queueFamilyIndices.compute, 0, &computeQueue);
  vkGetDeviceQueue(logicalDevice, queueFamilyIndices.transfer, 0, &transferQueue);

  // 7. Create a module for internal device entry points
  radixSortModule = new Module(this, sortDeviceCode);
  // scanModule = new Module(scanDeviceCode);
  fallbacksModule = new Module(this, fallbacksDeviceCode);

  // Swapchain semaphores and fences
  if (requestedFeatures.window) {
    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

    if (vkCreateSemaphore(logicalDevice, &semaphoreInfo, nullptr, &imageAvailableSemaphore) != VK_SUCCESS ||
        vkCreateSemaphore(logicalDevice, &semaphoreInfo, nullptr, &renderFinishedSemaphore) != VK_SUCCESS ||
        vkCreateFence(logicalDevice, &fenceInfo, nullptr, &inFlightFence) != VK_SUCCESS) {
      LOG_ERROR("Failed to create swapchain semaphores");
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
      if (formats[i].format == surfaceFormat.format && formats[i].colorSpace == surfaceFormat.colorSpace)
        surfaceFormatFound = true;
    if (!surfaceFormatFound)
      LOG_ERROR("Error, unable to find RGBA8 SRGB surface format...");

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
      if (presentModes[i] == presentMode)
        presentModeFound = true;
    if (!presentModeFound)
      LOG_ERROR("Error, unable to find vsync present mode...");

    // Window extent is the number of pixels truly used by the surface. For
    // high dps displays (like on mac) the window extent might be larger
    // than the screen coordinate size.
    if (surfaceCapabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
      windowExtent = surfaceCapabilities.currentExtent;
    } else {
      int width, height;
      glfwGetFramebufferSize(window, &width, &height);
      VkExtent2D actualExtent = {static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
      actualExtent.width = std::clamp(actualExtent.width, surfaceCapabilities.minImageExtent.width,
                                      surfaceCapabilities.maxImageExtent.width);
      actualExtent.height = std::clamp(actualExtent.height, surfaceCapabilities.minImageExtent.height,
                                       surfaceCapabilities.maxImageExtent.height);
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
    createInfo.imageArrayLayers = 1;   // might be 2 for stereoscopic 3D images like VR
    createInfo.imageUsage =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;   // might need to change this...
    // currently assuming graphics and present queue are the same...
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.queueFamilyIndexCount = 0;       // optional if exclusive
    createInfo.pQueueFamilyIndices = nullptr;   // optional if exclusive
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
      LOG_ERROR("Error, failed to create swap chain : \n" + errorString(err));
    }

    // Now, receive the swapchain images
    vkGetSwapchainImagesKHR(logicalDevice, swapchain, &surfaceImageCount, nullptr);
    swapchainImages.resize(surfaceImageCount);
    vkGetSwapchainImagesKHR(logicalDevice, swapchain, &surfaceImageCount, swapchainImages.data());

    /* Transition all images to presentable */
    for (uint32_t i = 0; i < swapchainImages.size(); ++i) {
      transitionImageLayout(swapchainImages[i], surfaceFormat.format, VK_IMAGE_LAYOUT_UNDEFINED,
                            VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    }

    // And acquire the first image to use
    vkAcquireNextImageKHR(logicalDevice, swapchain, UINT64_MAX, imageAvailableSemaphore, VK_NULL_HANDLE,
                          &currentImageIndex);
  }

  // Allocate resource heap
  {
    descriptorPoolSize.type = VK_DESCRIPTOR_TYPE_MUTABLE_EXT;
    descriptorPoolSize.descriptorCount = requestedFeatures.maxDescriptorCount;

    VkDescriptorPoolCreateInfo descriptorPoolInfo{};
    descriptorPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    descriptorPoolInfo.poolSizeCount = 1;
    descriptorPoolInfo.pPoolSizes = &descriptorPoolSize;
    descriptorPoolInfo.maxSets = 1;   // just one descriptor set for now.
    descriptorPoolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    // "If [below] does not exist, the descriptor pool allocates enough memory to be able to allocate a [mutable
    // descriptor]
    //  with any supported VkDescriptorType as a mutable descriptor."
    // descriptorPoolInfo.pNext = &mutableTypeInfo;
    VK_CHECK_RESULT(vkCreateDescriptorPool(logicalDevice, &descriptorPoolInfo, nullptr, &descriptorPool));

    VkDescriptorType cbvSrvUavTypes[] = {VK_DESCRIPTOR_TYPE_SAMPLER,
                                         VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
                                         VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
                                         VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER,
                                         VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER,
                                         VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                                         VK_DESCRIPTOR_TYPE_STORAGE_BUFFER};

    VkMutableDescriptorTypeListEXT cbvSrvUavTypeList = {};
    cbvSrvUavTypeList.descriptorTypeCount = sizeof(cbvSrvUavTypes) / sizeof(VkDescriptorType);
    cbvSrvUavTypeList.pDescriptorTypes = cbvSrvUavTypes;

    VkMutableDescriptorTypeCreateInfoEXT mutableTypeInfo = {};
    mutableTypeInfo.sType = VK_STRUCTURE_TYPE_MUTABLE_DESCRIPTOR_TYPE_CREATE_INFO_EXT;
    mutableTypeInfo.pNext = NULL;
    mutableTypeInfo.mutableDescriptorTypeListCount = 1;
    mutableTypeInfo.pMutableDescriptorTypeLists = &cbvSrvUavTypeList;

    VkDescriptorSetLayoutBindingFlagsCreateInfoEXT setLayoutBindingFlags{};
    setLayoutBindingFlags.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO_EXT;
    std::vector<VkDescriptorBindingFlagsEXT> descriptorBindingFlags = {
        VK_DESCRIPTOR_BINDING_VARIABLE_DESCRIPTOR_COUNT_BIT_EXT, VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT_EXT};
    setLayoutBindingFlags.bindingCount = 1;
    setLayoutBindingFlags.pBindingFlags = descriptorBindingFlags.data();
    setLayoutBindingFlags.pNext = &mutableTypeInfo;

    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_MUTABLE_EXT;
    binding.descriptorCount = descriptorPoolSize.descriptorCount;
    binding.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR |
                         VK_SHADER_STAGE_INTERSECTION_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
                         VK_SHADER_STAGE_CALLABLE_BIT_KHR | VK_SHADER_STAGE_RAYGEN_BIT_KHR |
                         VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;
    binding.pImmutableSamplers = NULL;

    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {binding};
    VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo{};
    descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    descriptorSetLayoutCreateInfo.pBindings = setLayoutBindings.data();
    descriptorSetLayoutCreateInfo.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
    descriptorSetLayoutCreateInfo.pNext = &setLayoutBindingFlags;
    VK_CHECK_RESULT(
        vkCreateDescriptorSetLayout(logicalDevice, &descriptorSetLayoutCreateInfo, nullptr, &descriptorSetLayout));

    // Now, making the descriptor sets (for now, just putting samplers here...)
    VkDescriptorSetVariableDescriptorCountAllocateInfoEXT variableDescriptorCountAllocInfo{};

    uint32_t variableDescCounts[] = {static_cast<uint32_t>(descriptorPoolSize.descriptorCount)};

    variableDescriptorCountAllocInfo.sType =
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_VARIABLE_DESCRIPTOR_COUNT_ALLOCATE_INFO_EXT;
    variableDescriptorCountAllocInfo.descriptorSetCount = 1;
    variableDescriptorCountAllocInfo.pDescriptorCounts = variableDescCounts;

    VkDescriptorSetAllocateInfo descriptorSetAllocateInfo{};
    descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    descriptorSetAllocateInfo.descriptorPool = descriptorPool;
    descriptorSetAllocateInfo.pSetLayouts = &descriptorSetLayout;
    descriptorSetAllocateInfo.descriptorSetCount = 1;
    descriptorSetAllocateInfo.pNext = &variableDescriptorCountAllocInfo;

    VK_CHECK_RESULT(vkAllocateDescriptorSets(logicalDevice, &descriptorSetAllocateInfo, &descriptorSet));
  }

  // Init imgui
  if (requestedFeatures.window) {
    // 1: create descriptor pool for IMGUI
    // the size of the pool is very oversize, but it's copied from imgui demo itself.
    VkDescriptorPoolSize pool_sizes[] = {{VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
                                         {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
                                         {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
                                         {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
                                         {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
                                         {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}};

    VkDescriptorPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 1000;
    pool_info.poolSizeCount = (uint32_t) std::size(pool_sizes);
    pool_info.pPoolSizes = pool_sizes;

    VK_CHECK_RESULT(vkCreateDescriptorPool(logicalDevice, &pool_info, nullptr, &imguiPool));

    // 2: initialize imgui library

    // this initializes the core structures of imgui
    ImGui::CreateContext();

    // this initializes imgui for SDL
    ImGui_ImplGlfw_InitForVulkan(window, true);

    // Call new frame here to initialize some internal imgui data
    ImGui_ImplGlfw_NewFrame();
  }

  // Init denoisers
  if (requestedFeatures.aiDenoiser.requested) {
    if (deviceProperties.vendorID == VENDOR_ID_NVIDIA) {
      NVSDK_NGX_Result initResult = NVSDK_NGX_VULKAN_Init(aiDenoising.kAppID, L".", instance, physicalDevice, logicalDevice);
      if (NVSDK_NGX_FAILED(initResult)) {
        if (initResult == NVSDK_NGX_Result_FAIL_FeatureNotSupported || initResult == NVSDK_NGX_Result_FAIL_PlatformError) {
          LOG_ERROR("NVIDIA NGX is not available on this hardware/platform " + aiDenoising.resultToString(initResult));
        }
        else {
          LOG_ERROR("Failed to initialize NGX " + aiDenoising.resultToString(initResult));
        }
      }
      NVSDK_NGX_Result paramsResult = NVSDK_NGX_VULKAN_GetCapabilityParameters(&aiDenoising.ngxParameters);
      
      NVSDK_NGX_Result optimalSettingsResult = NGX_DLSS_GET_OPTIMAL_SETTINGS
      (
        aiDenoising.ngxParameters,
        requestedFeatures.aiDenoiser.outputWidth,
        requestedFeatures.aiDenoiser.outputHeight,
        NVSDK_NGX_PerfQuality_Value_MaxQuality,
        &aiDenoising.optimalSettings.optimalRenderSize.x,
        &aiDenoising.optimalSettings.optimalRenderSize.y,
        &aiDenoising.optimalSettings.maxRenderSize.x,
        &aiDenoising.optimalSettings.maxRenderSize.y,
        &aiDenoising.optimalSettings.minRenderSize.x,
        &aiDenoising.optimalSettings.minRenderSize.y,
        &aiDenoising.optimalSettings.sharpness
      );

      VkCommandBuffer commandList = beginSingleTimeCommands(graphicsCommandPool);

      NVSDK_NGX_DLSS_Create_Params dlssParams = {};
      unsigned int creationNodeMask = 1;
      unsigned int visibilityNodeMask = 1;      
      
      dlssParams.InFeatureCreateFlags = requestedFeatures.aiDenoiser.flags;
      dlssParams.Feature.InWidth = aiDenoising.optimalSettings.optimalRenderSize.x;
      dlssParams.Feature.InHeight = aiDenoising.optimalSettings.optimalRenderSize.y;
      dlssParams.Feature.InTargetWidth = requestedFeatures.aiDenoiser.outputWidth;
      dlssParams.Feature.InTargetHeight = requestedFeatures.aiDenoiser.outputHeight;
      dlssParams.Feature.InPerfQualityValue = NVSDK_NGX_PerfQuality_Value_MaxQuality; // for now, can change later.
      
      NVSDK_NGX_Result createResult = NGX_VULKAN_CREATE_DLSS_EXT(commandList, creationNodeMask, visibilityNodeMask, &aiDenoising.ngxHandle, aiDenoising.ngxParameters, &dlssParams);
      endSingleTimeCommands(commandList, graphicsCommandPool, graphicsQueue);
    }
    else {
      LOG_ERROR("Unimplemented");
    }
  }

  // Finally, setup the internal shader stages and build an initial shader binding table
  setupInternalPrograms();
  buildSBT(GPRT_SBT_ALL);
};

// void buildPrograms()
// {
//   // At the moment, we don't actually build our programs here.
// }

// Todo, refactor this code...
void
Context::setupInternalPrograms() {

  // // For filling out instance data
  // {
  //   fillInstanceDataStage.entryPoint = "gprtFillInstanceData";

  //   // todo, consider refactoring this into a more official "Compute" shader
  //   // object

  //   VkResult err;
  //   // currently not using cache.
  //   VkPipelineCache cache = VK_NULL_HANDLE;

  //   VkPushConstantRange pushConstantRange = {};
  //   pushConstantRange.size = 128;
  //   pushConstantRange.offset = 0;
  //   pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  //   VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
  //   pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  //   pipelineLayoutCreateInfo.setLayoutCount = 0;      // if ever we use descriptors
  //   pipelineLayoutCreateInfo.pSetLayouts = nullptr;   // if ever we use descriptors
  //   pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
  //   pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

  //   err = vkCreatePipelineLayout(logicalDevice, &pipelineLayoutCreateInfo, nullptr, &fillInstanceDataStage.layout);

  //   std::string entryPoint = std::string("__compute__") + std::string(fillInstanceDataStage.entryPoint);
  //   auto binary = internalModule->getBinary("COMPUTE");

  //   VkShaderModuleCreateInfo moduleCreateInfo = {};
  //   moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  //   moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
  //   moduleCreateInfo.pCode = binary.data();

  //   err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &fillInstanceDataStage.module);

  //   VkPipelineShaderStageCreateInfo shaderStage = {};
  //   shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  //   shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  //   shaderStage.module = fillInstanceDataStage.module;
  //   shaderStage.pName = entryPoint.c_str();

  //   VkComputePipelineCreateInfo computePipelineCreateInfo = {};
  //   computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  //   computePipelineCreateInfo.layout = fillInstanceDataStage.layout;
  //   computePipelineCreateInfo.flags = 0;
  //   computePipelineCreateInfo.stage = shaderStage;

  //   // At this point, create all internal compute pipelines as well.
  //   err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr,
  //                                 &fillInstanceDataStage.pipeline);
  // }

  // Radix sorter stages
  {
    // currently not using cache.
    VkPipelineCache cache = VK_NULL_HANDLE;

    VkDescriptorPoolSize poolSize;
    poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSize.descriptorCount = 16;

    VkDescriptorPoolCreateInfo descriptorPoolInfo{};
    descriptorPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    descriptorPoolInfo.poolSizeCount = 1;
    descriptorPoolInfo.pPoolSizes = &poolSize;
    descriptorPoolInfo.maxSets = 5;
    descriptorPoolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    VK_CHECK_RESULT(vkCreateDescriptorPool(logicalDevice, &descriptorPoolInfo, nullptr, &sortStages.pool));

    sortStages.Count.entryPoint = "Count";
    sortStages.CountReduce.entryPoint = "CountReduce";
    sortStages.Scan.entryPoint = "Scan";
    sortStages.ScanAdd.entryPoint = "ScanAdd";
    sortStages.Scatter.entryPoint = "Scatter";
    sortStages.ScatterPayload.entryPoint = "ScatterPayload";

    // Create binding for Radix sort passes
    VkDescriptorSetLayoutBinding layout_bindings_set_InputOutputs[] = {
        {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // SrcBuffer (sort)
        {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // DstBuffer (sort)
        {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // ScrPayload (sort only)
        {3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // DstPayload (sort only)
    };

    VkDescriptorSetLayoutBinding layout_bindings_set_Scan[] = {
        {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // ScanSrc
        {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // ScanDst
        {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // ScanScratch
    };

    VkDescriptorSetLayoutBinding layout_bindings_set_Scratch[] = {
        {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // Scratch (sort only)
        {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_ALL, nullptr},   // Scratch (reduced)
    };

    auto AllocDescriptor = [&](VkDescriptorPool pool, VkDescriptorSetLayout layout, VkDescriptorSet *descriptorSet) {
      VkDescriptorSetAllocateInfo descriptorSetAllocateInfo{};
      descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
      descriptorSetAllocateInfo.descriptorPool = pool;
      descriptorSetAllocateInfo.pSetLayouts = &layout;
      descriptorSetAllocateInfo.descriptorSetCount = 1;
      descriptorSetAllocateInfo.pNext = nullptr;
      VkResult err = vkAllocateDescriptorSets(logicalDevice, &descriptorSetAllocateInfo, descriptorSet);
      if (err != VK_SUCCESS) {
        LOG_ERROR("failed to allocate descriptor! \n" + errorString(err));
      }
    };

    VkResult vkResult;

    VkDescriptorSetLayoutCreateInfo descriptor_set_layout_create_info = {
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    descriptor_set_layout_create_info.pNext = nullptr;
    descriptor_set_layout_create_info.flags = 0;
    descriptor_set_layout_create_info.pBindings = layout_bindings_set_InputOutputs;
    descriptor_set_layout_create_info.bindingCount = 4;
    vkResult = vkCreateDescriptorSetLayout(logicalDevice, &descriptor_set_layout_create_info, nullptr,
                                           &sortStages.m_SortDescriptorSetLayoutInputOutputs);
    VK_CHECK_RESULT(vkResult);
    AllocDescriptor(sortStages.pool, sortStages.m_SortDescriptorSetLayoutInputOutputs,
                    &sortStages.m_SortDescriptorSetInputOutput[0]);
    AllocDescriptor(sortStages.pool, sortStages.m_SortDescriptorSetLayoutInputOutputs,
                    &sortStages.m_SortDescriptorSetInputOutput[1]);

    descriptor_set_layout_create_info.pBindings = layout_bindings_set_Scan;
    descriptor_set_layout_create_info.bindingCount = 3;
    vkResult = vkCreateDescriptorSetLayout(logicalDevice, &descriptor_set_layout_create_info, nullptr,
                                           &sortStages.m_SortDescriptorSetLayoutScan);
    VK_CHECK_RESULT(vkResult);
    AllocDescriptor(sortStages.pool, sortStages.m_SortDescriptorSetLayoutScan,
                    &sortStages.m_SortDescriptorSetScanSets[0]);
    AllocDescriptor(sortStages.pool, sortStages.m_SortDescriptorSetLayoutScan,
                    &sortStages.m_SortDescriptorSetScanSets[1]);

    descriptor_set_layout_create_info.pBindings = layout_bindings_set_Scratch;
    descriptor_set_layout_create_info.bindingCount = 2;
    vkResult = vkCreateDescriptorSetLayout(logicalDevice, &descriptor_set_layout_create_info, nullptr,
                                           &sortStages.m_SortDescriptorSetLayoutScratch);
    VK_CHECK_RESULT(vkResult);
    AllocDescriptor(sortStages.pool, sortStages.m_SortDescriptorSetLayoutScratch,
                    &sortStages.m_SortDescriptorSetScratch);

    // Create constant range representing our static constant
    VkPushConstantRange constant_range;
    constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    constant_range.offset = 0;
    constant_range.size = sizeof(ParallelSortCB);

    // Create the pipeline layout (Root signature)
    VkPipelineLayoutCreateInfo layout_create_info = {VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layout_create_info.pNext = nullptr;
    layout_create_info.flags = 0;
    layout_create_info.setLayoutCount = 3;
    VkDescriptorSetLayout layouts[] = {sortStages.m_SortDescriptorSetLayoutInputOutputs,
                                       sortStages.m_SortDescriptorSetLayoutScan,
                                       sortStages.m_SortDescriptorSetLayoutScratch};
    layout_create_info.pSetLayouts = layouts;
    layout_create_info.pushConstantRangeCount = 1;
    layout_create_info.pPushConstantRanges = &constant_range;

    VkResult bCreatePipelineLayout =
        vkCreatePipelineLayout(logicalDevice, &layout_create_info, nullptr, &sortStages.layout);
    assert(bCreatePipelineLayout == VK_SUCCESS);

    {
      VkResult err = VK_SUCCESS;
      std::string entryPoint = sortStages.Count.entryPoint;
      auto binary = radixSortModule->binary;

      VkShaderModuleCreateInfo moduleCreateInfo = {};
      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
      moduleCreateInfo.pCode = binary.data();

      err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &sortStages.Count.module);

      VkPipelineShaderStageCreateInfo shaderStage = {};
      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shaderStage.module = sortStages.Count.module;
      shaderStage.pName = entryPoint.c_str();

      VkComputePipelineCreateInfo computePipelineCreateInfo = {};
      computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
      computePipelineCreateInfo.layout = sortStages.layout;
      computePipelineCreateInfo.flags = 0;
      computePipelineCreateInfo.stage = shaderStage;

      // At this point, create all internal compute pipelines as well.
      err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr,
                                     &sortStages.Count.pipeline);
      if (err != VK_SUCCESS) {
        LOG_ERROR("failed to create sort pipeline! Are all entrypoint names correct? \n" + errorString(err));
      }
    }

    {
      VkResult err = VK_SUCCESS;
      std::string entryPoint = sortStages.CountReduce.entryPoint;
      auto binary = radixSortModule->binary;

      VkShaderModuleCreateInfo moduleCreateInfo = {};
      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
      moduleCreateInfo.pCode = binary.data();

      err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &sortStages.CountReduce.module);

      VkPipelineShaderStageCreateInfo shaderStage = {};
      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shaderStage.module = sortStages.CountReduce.module;
      shaderStage.pName = entryPoint.c_str();

      VkComputePipelineCreateInfo computePipelineCreateInfo = {};
      computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
      computePipelineCreateInfo.layout = sortStages.layout;
      computePipelineCreateInfo.flags = 0;
      computePipelineCreateInfo.stage = shaderStage;

      // At this point, create all internal compute pipelines as well.
      err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr,
                                     &sortStages.CountReduce.pipeline);
      if (err != VK_SUCCESS) {
        LOG_ERROR("failed to create sort pipeline! Are all entrypoint names correct? \n" + errorString(err));
      }
    }

    {
      VkResult err = VK_SUCCESS;
      std::string entryPoint = sortStages.Scan.entryPoint;
      auto binary = radixSortModule->binary;

      VkShaderModuleCreateInfo moduleCreateInfo = {};
      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
      moduleCreateInfo.pCode = binary.data();

      err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &sortStages.Scan.module);

      VkPipelineShaderStageCreateInfo shaderStage = {};
      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shaderStage.module = sortStages.Scan.module;
      shaderStage.pName = entryPoint.c_str();

      VkComputePipelineCreateInfo computePipelineCreateInfo = {};
      computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
      computePipelineCreateInfo.layout = sortStages.layout;
      computePipelineCreateInfo.flags = 0;
      computePipelineCreateInfo.stage = shaderStage;

      // At this point, create all internal compute pipelines as well.
      err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr,
                                     &sortStages.Scan.pipeline);
      if (err != VK_SUCCESS) {
        LOG_ERROR("failed to create sort pipeline! Are all entrypoint names correct? \n" + errorString(err));
      }
    }

    {
      VkResult err = VK_SUCCESS;
      std::string entryPoint = sortStages.ScanAdd.entryPoint;
      auto binary = radixSortModule->binary;

      VkShaderModuleCreateInfo moduleCreateInfo = {};
      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
      moduleCreateInfo.pCode = binary.data();

      err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &sortStages.ScanAdd.module);

      VkPipelineShaderStageCreateInfo shaderStage = {};
      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shaderStage.module = sortStages.ScanAdd.module;
      shaderStage.pName = entryPoint.c_str();

      VkComputePipelineCreateInfo computePipelineCreateInfo = {};
      computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
      computePipelineCreateInfo.layout = sortStages.layout;
      computePipelineCreateInfo.flags = 0;
      computePipelineCreateInfo.stage = shaderStage;

      // At this point, create all internal compute pipelines as well.
      err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr,
                                     &sortStages.ScanAdd.pipeline);
      if (err != VK_SUCCESS) {
        LOG_ERROR("failed to create sort pipeline! Are all entrypoint names correct? \n" + errorString(err));
      }
    }

    {
      VkResult err = VK_SUCCESS;
      std::string entryPoint = sortStages.Scatter.entryPoint;
      auto binary = radixSortModule->binary;

      VkShaderModuleCreateInfo moduleCreateInfo = {};
      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
      moduleCreateInfo.pCode = binary.data();

      err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &sortStages.Scatter.module);

      VkPipelineShaderStageCreateInfo shaderStage = {};
      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shaderStage.module = sortStages.Scatter.module;
      shaderStage.pName = entryPoint.c_str();

      VkComputePipelineCreateInfo computePipelineCreateInfo = {};
      computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
      computePipelineCreateInfo.layout = sortStages.layout;
      computePipelineCreateInfo.flags = 0;
      computePipelineCreateInfo.stage = shaderStage;

      // At this point, create all internal compute pipelines as well.
      err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr,
                                     &sortStages.Scatter.pipeline);
      if (err != VK_SUCCESS) {
        LOG_ERROR("failed to create sort pipeline! Are all entrypoint names correct? \n" + errorString(err));
      }
    }

    {
      VkResult err = VK_SUCCESS;
      std::string entryPoint = sortStages.ScatterPayload.entryPoint;
      auto binary = radixSortModule->binary;

      VkShaderModuleCreateInfo moduleCreateInfo = {};
      moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
      moduleCreateInfo.codeSize = binary.size() * sizeof(uint32_t);
      moduleCreateInfo.pCode = binary.data();

      err = vkCreateShaderModule(logicalDevice, &moduleCreateInfo, NULL, &sortStages.ScatterPayload.module);

      VkPipelineShaderStageCreateInfo shaderStage = {};
      shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      shaderStage.module = sortStages.ScatterPayload.module;
      shaderStage.pName = entryPoint.c_str();

      VkComputePipelineCreateInfo computePipelineCreateInfo = {};
      computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
      computePipelineCreateInfo.layout = sortStages.layout;
      computePipelineCreateInfo.flags = 0;
      computePipelineCreateInfo.stage = shaderStage;

      // At this point, create all internal compute pipelines as well.
      err = vkCreateComputePipelines(logicalDevice, cache, 1, &computePipelineCreateInfo, nullptr,
                                     &sortStages.ScatterPayload.pipeline);
      if (err != VK_SUCCESS) {
        LOG_ERROR("failed to create sort pipeline! Are all entrypoint names correct? \n" + errorString(err));
      }
    }

    // todo, destroy the above stuff
  }

  // Scanning stages
  // {
  //   Context *context = this;
  //   internalComputePrograms.insert(
  //       {"InitScan", new Compute(context, context->logicalDevice, scanModule, "InitScan", 0)});
  //   internalComputePrograms.insert(
  //       {"Scan", new Compute(context, context->logicalDevice, scanModule,
  //                            ("Scan_" + std::to_string(subgroupProperties.subgroupSize)).c_str(), 0)});
  // }

  // Fallback primitive programs
  {
    Context *context = this;
    internalComputePrograms.insert({"LSSBounds", new Compute(context, fallbacksModule, "LSSBounds")});
    internalComputePrograms.insert({"SphereBounds", new Compute(context, fallbacksModule, "SphereBounds")});
    internalComputePrograms.insert({"SolidBounds", new Compute(context, fallbacksModule, "SolidBounds")});
  }
  computePipelinesOutOfDate = true;
}

void
Context::destroyInternalPrograms() {
  // destroy sort stages
  {
    vkDestroyPipeline(logicalDevice, sortStages.ScatterPayload.pipeline, nullptr);
    vkDestroyPipeline(logicalDevice, sortStages.Scatter.pipeline, nullptr);
    vkDestroyPipeline(logicalDevice, sortStages.ScanAdd.pipeline, nullptr);
    vkDestroyPipeline(logicalDevice, sortStages.Scan.pipeline, nullptr);
    vkDestroyPipeline(logicalDevice, sortStages.CountReduce.pipeline, nullptr);
    vkDestroyPipeline(logicalDevice, sortStages.Count.pipeline, nullptr);
    sortStages.ScatterPayload.pipeline = VK_NULL_HANDLE;
    sortStages.Scatter.pipeline = VK_NULL_HANDLE;
    sortStages.ScanAdd.pipeline = VK_NULL_HANDLE;
    sortStages.Scan.pipeline = VK_NULL_HANDLE;
    sortStages.CountReduce.pipeline = VK_NULL_HANDLE;
    sortStages.Count.pipeline = VK_NULL_HANDLE;

    vkDestroyShaderModule(logicalDevice, sortStages.ScatterPayload.module, nullptr);
    vkDestroyShaderModule(logicalDevice, sortStages.Scatter.module, nullptr);
    vkDestroyShaderModule(logicalDevice, sortStages.ScanAdd.module, nullptr);
    vkDestroyShaderModule(logicalDevice, sortStages.Scan.module, nullptr);
    vkDestroyShaderModule(logicalDevice, sortStages.CountReduce.module, nullptr);
    vkDestroyShaderModule(logicalDevice, sortStages.Count.module, nullptr);
    sortStages.ScatterPayload.module = VK_NULL_HANDLE;
    sortStages.Scatter.module = VK_NULL_HANDLE;
    sortStages.ScanAdd.module = VK_NULL_HANDLE;
    sortStages.Scan.module = VK_NULL_HANDLE;
    sortStages.CountReduce.module = VK_NULL_HANDLE;
    sortStages.Count.module = VK_NULL_HANDLE;

    vkDestroyPipelineLayout(logicalDevice, sortStages.layout, nullptr);
    sortStages.layout = VK_NULL_HANDLE;

    vkFreeDescriptorSets(logicalDevice, sortStages.pool, 1, &sortStages.m_SortDescriptorSetScratch);
    vkFreeDescriptorSets(logicalDevice, sortStages.pool, 2, sortStages.m_SortDescriptorSetScanSets);
    vkFreeDescriptorSets(logicalDevice, sortStages.pool, 2, sortStages.m_SortDescriptorSetInputOutput);

    vkDestroyDescriptorSetLayout(logicalDevice, sortStages.m_SortDescriptorSetLayoutScratch, nullptr);
    vkDestroyDescriptorSetLayout(logicalDevice, sortStages.m_SortDescriptorSetLayoutScan, nullptr);
    vkDestroyDescriptorSetLayout(logicalDevice, sortStages.m_SortDescriptorSetLayoutInputOutputs, nullptr);

    vkDestroyDescriptorPool(logicalDevice, sortStages.pool, nullptr);
  }

  std::vector<std::string> progNames;
  for (auto &program : internalComputePrograms) {
    progNames.push_back(program.first);
  }

  for (auto &progName : progNames) {
    internalComputePrograms[progName]->destroy();
    delete internalComputePrograms[progName];
    internalComputePrograms.erase(progName);
  }
}

VkCommandBuffer
Context::beginSingleTimeCommands(VkCommandPool pool) {
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

void
Context::endSingleTimeCommands(VkCommandBuffer commandBuffer, VkCommandPool pool, VkQueue queue) {
  vkEndCommandBuffer(commandBuffer);

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &commandBuffer;

  vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
  vkQueueWaitIdle(queue);

  vkFreeCommandBuffers(logicalDevice, pool, 1, &commandBuffer);
}

void
Context::transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout) {
  VkCommandBuffer commandBuffer = beginSingleTimeCommands(graphicsCommandPool);

  VkImageMemoryBarrier barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;

  // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents
  // of the original image. I'm assuming this is ok.
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
  } else if (oldLayout == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
    sourceStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;

    destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dstAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
  } else {
    throw std::invalid_argument("unsupported layout transition!");
  }

  vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);

  endSingleTimeCommands(commandBuffer, graphicsCommandPool, graphicsQueue);
}

// For ImGui
void
Context::setRasterAttachments(Texture *colorTexture, Texture *depthTexture) {
  if (colorTexture->width != depthTexture->width || colorTexture->height != depthTexture->height) {
    throw std::runtime_error("Error, color and depth attachment textures must have equal dimensions!");
  } else {
    imgui.width = colorTexture->width;
    imgui.height = colorTexture->height;
  }

  imgui.colorAttachment = colorTexture;
  imgui.depthAttachment = depthTexture;

  VkAttachmentDescription colorAttachment{};
  colorAttachment.format = colorTexture->format;
  colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
  // clear here says to clear the values to a constant at start.
  // colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;   // DONT_CARE;
  // save rasterized fragments to memory
  colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  // not currently using a stencil
  colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  // Initial and final layouts of the texture
  colorAttachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  colorAttachment.finalLayout = VK_IMAGE_LAYOUT_GENERAL;

  VkAttachmentDescription depthAttachment{};
  depthAttachment.format = depthTexture->format;
  depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
  depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;   // VK_ATTACHMENT_LOAD_OP_CLEAR;
  depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depthAttachment.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
  depthAttachment.finalLayout = VK_IMAGE_LAYOUT_GENERAL;

  std::vector<VkAttachmentDescription> attachments = {colorAttachment, depthAttachment};

  VkAttachmentReference colorAttachmentRef{};
  colorAttachmentRef.attachment = 0;
  colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  VkAttachmentReference depthAttachmentRef{};
  depthAttachmentRef.attachment = 1;
  depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &colorAttachmentRef;
  subpass.pDepthStencilAttachment = &depthAttachmentRef;

  VkSubpassDependency dependency{};
  dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  dependency.dstSubpass = 0;
  dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  dependency.srcAccessMask = 0;
  dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
  dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

  VkRenderPassCreateInfo createInfo{};
  createInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  createInfo.pNext = nullptr;
  createInfo.attachmentCount = (uint32_t) attachments.size();
  createInfo.pAttachments = attachments.data();
  createInfo.subpassCount = 1;
  createInfo.pSubpasses = &subpass;
  createInfo.dependencyCount = 1;
  createInfo.pDependencies = &dependency;

  vkCreateRenderPass(logicalDevice, &createInfo, nullptr, &imgui.renderPass);

  VkImageView attachmentViews[] = {imgui.colorAttachment->imageView, imgui.depthAttachment->imageView};

  VkFramebufferCreateInfo framebufferInfo{};
  framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
  framebufferInfo.renderPass = imgui.renderPass;
  framebufferInfo.attachmentCount = 2;
  framebufferInfo.pAttachments = attachmentViews;
  framebufferInfo.width = imgui.width;
  framebufferInfo.height = imgui.height;
  framebufferInfo.layers = 1;

  if (vkCreateFramebuffer(logicalDevice, &framebufferInfo, nullptr, &imgui.frameBuffer) != VK_SUCCESS) {
    throw std::runtime_error("failed to create framebuffer!");
  }

  // this initializes imgui for Vulkan
  ImGui_ImplVulkan_InitInfo init_info = {};
  init_info.Instance = instance;
  init_info.PhysicalDevice = physicalDevice;
  init_info.Device = logicalDevice;
  init_info.Queue = graphicsQueue;
  init_info.DescriptorPool = imguiPool;
  init_info.MinImageCount = 2;
  init_info.ImageCount = 2;
  init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

  ImGui_ImplVulkan_Init(&init_info, imgui.renderPass);

  // execute a gpu command to upload imgui font textures
  VkCommandBufferBeginInfo cmdBufInfo{};
  cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  VK_CHECK_RESULT(vkBeginCommandBuffer(graphicsCommandBuffer, &cmdBufInfo));
  ImGui_ImplVulkan_CreateFontsTexture(graphicsCommandBuffer);
  VK_CHECK_RESULT(vkEndCommandBuffer(graphicsCommandBuffer));

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.pNext = NULL;
  submitInfo.waitSemaphoreCount = 0;
  submitInfo.pWaitSemaphores = nullptr;
  submitInfo.pWaitDstStageMask = nullptr;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &graphicsCommandBuffer;
  submitInfo.signalSemaphoreCount = 0;
  submitInfo.pSignalSemaphores = nullptr;

  VK_CHECK_RESULT(vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE));
  VK_CHECK_RESULT(vkQueueWaitIdle(graphicsQueue));

  // clear font textures from cpu data
  ImGui_ImplVulkan_DestroyFontUploadObjects();
}

void
Context::rasterizeGui() {
  ImGui::Render();
  ImDrawData *draw_data = ImGui::GetDrawData();

  VkResult err;
  VkCommandBufferBeginInfo cmdBufInfo{};
  cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  VkRenderPassBeginInfo renderPassBeginInfo = {};
  renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  renderPassBeginInfo.pNext = nullptr;
  renderPassBeginInfo.renderPass = imgui.renderPass;
  renderPassBeginInfo.renderArea.offset.x = 0;
  renderPassBeginInfo.renderArea.offset.y = 0;
  renderPassBeginInfo.renderArea.extent.width = imgui.width;
  renderPassBeginInfo.renderArea.extent.height = imgui.height;
  renderPassBeginInfo.clearValueCount = 0;
  renderPassBeginInfo.pClearValues = nullptr;
  renderPassBeginInfo.framebuffer = imgui.frameBuffer;

  err = vkBeginCommandBuffer(graphicsCommandBuffer, &cmdBufInfo);

  // Transition our attachments into optimal attachment formats
  imgui.colorAttachment->setImageLayout(graphicsCommandBuffer, imgui.colorAttachment->image,
                                        imgui.colorAttachment->layout, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                        {VK_IMAGE_ASPECT_COLOR_BIT, 0, imgui.colorAttachment->mipLevels, 0, 1});

  imgui.depthAttachment->setImageLayout(graphicsCommandBuffer, imgui.depthAttachment->image,
                                        imgui.depthAttachment->layout, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                                        {VK_IMAGE_ASPECT_DEPTH_BIT, 0, imgui.depthAttachment->mipLevels, 0, 1});

  // This will clear the color and depth attachment
  vkCmdBeginRenderPass(graphicsCommandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

  // Record dear imgui primitives into command buffer
  ImGui_ImplVulkan_RenderDrawData(draw_data, graphicsCommandBuffer);

  vkCmdEndRenderPass(graphicsCommandBuffer);

  // At the end of the renderpass, we'll transition the layout back to it's previous layout
  imgui.colorAttachment->setImageLayout(graphicsCommandBuffer, imgui.colorAttachment->image, VK_IMAGE_LAYOUT_GENERAL,
                                        imgui.colorAttachment->layout,
                                        {VK_IMAGE_ASPECT_COLOR_BIT, 0, imgui.colorAttachment->mipLevels, 0, 1});

  imgui.depthAttachment->setImageLayout(graphicsCommandBuffer, imgui.depthAttachment->image, VK_IMAGE_LAYOUT_GENERAL,
                                        imgui.depthAttachment->layout,
                                        {VK_IMAGE_ASPECT_DEPTH_BIT, 0, imgui.depthAttachment->mipLevels, 0, 1});

  err = vkEndCommandBuffer(graphicsCommandBuffer);
  if (err)
    LOG_ERROR("failed to end command buffer! : \n" + errorString(err));

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.pNext = NULL;
  submitInfo.waitSemaphoreCount = 0;
  submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
  submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &graphicsCommandBuffer;
  submitInfo.signalSemaphoreCount = 0;
  submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

  err = vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
  if (err)
    LOG_ERROR("failed to submit to queue! : \n" + errorString(err));

  err = vkQueueWaitIdle(graphicsQueue);
  if (err)
    LOG_ERROR("failed to wait for queue idle! : \n" + errorString(err));
}

GPRT_API void
gprtRequestWindow(uint32_t initialWidth, uint32_t initialHeight, const char *title) {
  LOG_API_CALL();
  requestedFeatures.window = true;
  requestedFeatures.windowProperties.initialWidth = initialWidth;
  requestedFeatures.windowProperties.initialHeight = initialHeight;
  requestedFeatures.windowProperties.title = std::string(title);
}

GPRT_API void
gprtRequestRayTypeCount(uint32_t rayTypeCount) {
  LOG_API_CALL();
  requestedFeatures.numRayTypes = rayTypeCount;
}

GPRT_API void
gprtRequestRayQueries() {
  LOG_API_CALL();
  requestedFeatures.rayQueries = true;
}

GPRT_API void
gprtRequestMaxPayloadSize(uint32_t payloadSize) {
  LOG_API_CALL();
  requestedFeatures.maxRayPayloadSize = payloadSize;
}


GPRT_API void
gprtRequestDenoiser(uint32_t outputWidth, uint32_t outputHeight, GPRTDenoiseFlags flags) {
  LOG_API_CALL();
  requestedFeatures.aiDenoiser.requested = true;
  requestedFeatures.aiDenoiser.flags = flags;
  requestedFeatures.aiDenoiser.outputWidth = outputWidth;
  requestedFeatures.aiDenoiser.outputHeight = outputHeight;
}

GPRT_API void
gprtRequestMaxAttributeSize(uint32_t attributeSize) {
  LOG_API_CALL();
  if (attributeSize > 32) LOG_ERROR("Max attribute size is too large. Must be 32 bytes or smaller.");
  requestedFeatures.maxRayHitAttributeSize = attributeSize;
}

GPRT_API void
gprtRequestRayRecursionDepth(uint32_t rayRecursionDepth) {
  LOG_API_CALL();
  if (rayRecursionDepth > 32) LOG_ERROR("Max recursion depth is too large. Must be 32 or smaller.");
  requestedFeatures.rayRecursionDepth = rayRecursionDepth;
}

GPRT_API void
gprtRequestRecordSizes(uint32_t raygenRecordSize, uint32_t hitRecordSize, uint32_t missRecordSize, uint32_t callableRecordSize) {
  LOG_API_CALL();
  requestedFeatures.raygenRecordSize = raygenRecordSize;
  requestedFeatures.hitRecordSize = hitRecordSize;
  requestedFeatures.missRecordSize = missRecordSize;
  requestedFeatures.callableRecordSize = callableRecordSize;
}

GPRT_API bool
gprtWindowShouldClose(GPRTContext _context) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  if (!requestedFeatures.window)
    return true;

  glfwPollEvents();

  // Start the Dear ImGui frame
  // ImGui_ImplVulkan_NewFrame(); // I'm not entirely convinced this is needed?
  ImGui_ImplGlfw_NewFrame();   // if GLFW isn't available this might be odd...

  return glfwWindowShouldClose(context->window);
}

GPRT_API void
gprtSetWindowTitle(GPRTContext _context, const char *title) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  if (!requestedFeatures.window)
    return;

  glfwSetWindowTitle(context->window, title);
}

GPRT_API void
gprtGetCursorPos(GPRTContext _context, double *xpos, double *ypos) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  if (!requestedFeatures.window)
    return;

  glfwGetCursorPos(context->window, xpos, ypos);
}

GPRT_API void
gprtGrabAndHideCursor(GPRTContext _context, bool enabled) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  if (!requestedFeatures.window)
    return;

  glfwSetInputMode(context->window, GLFW_CURSOR, enabled ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
}

GPRT_API int
gprtGetMouseButton(GPRTContext _context, int button) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  if (!requestedFeatures.window)
    return GPRT_RELEASE;

  return glfwGetMouseButton(context->window, button);
}

GPRT_API int
gprtGetKey(GPRTContext _context, int key) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  if (!requestedFeatures.window)
    return GPRT_RELEASE;

  return glfwGetKey(context->window, key);
}

GPRT_API double
gprtGetTime(GPRTContext _context) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  if (!requestedFeatures.window)
    return 0.0;
  return glfwGetTime();
}

GPRT_API void gprtGetDenoiserRenderSize(GPRTContext _context, uint32_t *width, uint32_t *height)
{
  LOG_API_CALL();
  if (!requestedFeatures.aiDenoiser.requested)
    return;
  Context *context = (Context *) _context;
  *width = context->aiDenoising.optimalSettings.optimalRenderSize.x;
  *height = context->aiDenoising.optimalSettings.optimalRenderSize.y;
}

GPRT_API void
gprtTexturePresent(GPRTContext _context, GPRTTexture _texture) {
  LOG_API_CALL();
  if (!requestedFeatures.window)
    return;
  Context *context = (Context *) _context;
  Texture *texture = (Texture *) _texture;

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

    // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents
    // of the original image. I'm assuming this is ok.
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

    vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
  }

  // transfer the texture to a transfer source
  texture->setImageLayout(commandBuffer, texture->image, texture->layout, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                          {VK_IMAGE_ASPECT_COLOR_BIT, 0, texture->mipLevels, 0, 1});

  // now do the transfer
  {
    VkImageBlit region{};

    region.srcOffsets[0].x = 0;
    region.srcOffsets[0].y = 0;
    region.srcOffsets[0].z = 0;
    region.srcOffsets[1].x = context->windowExtent.width;
    region.srcOffsets[1].y = context->windowExtent.height;
    region.srcOffsets[1].z = 1;

    region.dstOffsets[0].x = 0;
    region.dstOffsets[0].y = 0;
    region.dstOffsets[0].z = 0;
    region.dstOffsets[1].x = texture->width;
    region.dstOffsets[1].y = texture->height;
    region.dstOffsets[1].z = 1;

    region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.srcSubresource.baseArrayLayer = 0;
    region.srcSubresource.layerCount = 1;
    region.srcSubresource.mipLevel = 0;

    region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.dstSubresource.baseArrayLayer = 0;
    region.dstSubresource.layerCount = 1;
    region.dstSubresource.mipLevel = 0;

    vkCmdBlitImage(commandBuffer, texture->image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                   context->swapchainImages[context->currentImageIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                   &region, VK_FILTER_LINEAR);
  }

  // now go from TRANSFER_DST back to PRESENT_SRC
  {
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;

    // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents
    // of the original image. I'm assuming this is ok.
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

    vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
  }

  // and revert the texture format back to its previous layout
  texture->setImageLayout(commandBuffer, texture->image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, texture->layout,
                          {VK_IMAGE_ASPECT_COLOR_BIT, 0, texture->mipLevels, 0, 1});

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

  VkResult err2 = vkAcquireNextImageKHR(context->logicalDevice, context->swapchain, UINT64_MAX, VK_NULL_HANDLE,
                                        context->inFlightFence, &context->currentImageIndex);
  vkWaitForFences(context->logicalDevice, 1, &context->inFlightFence, true, UINT_MAX);
  vkResetFences(context->logicalDevice, 1, &context->inFlightFence);
}

GPRT_API void
gprtBufferPresent(GPRTContext _context, GPRTBuffer _buffer) {
  LOG_API_CALL();
  if (!requestedFeatures.window)
    return;
  Context *context = (Context *) _context;
  Buffer *buffer = (Buffer *) _buffer;

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

    // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents
    // of the original image. I'm assuming this is ok.
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

    vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
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

    // If this layout is "VK_IMAGE_LAYOUT_UNDEFINED", we might lose the contents
    // of the original image. I'm assuming this is ok.
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

    vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
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

  VkResult err2 = vkAcquireNextImageKHR(context->logicalDevice, context->swapchain, UINT64_MAX, VK_NULL_HANDLE,
                                        context->inFlightFence, &context->currentImageIndex);
  vkWaitForFences(context->logicalDevice, 1, &context->inFlightFence, true, UINT_MAX);
  vkResetFences(context->logicalDevice, 1, &context->inFlightFence);
}

GPRT_API void
gprtGuiSetRasterAttachments(GPRTContext _context, GPRTTexture _colorAttachment, GPRTTexture _depthAttachment) {
  Context *context = (Context *) _context;
  Texture *colorAttachment = (Texture *) _colorAttachment;
  Texture *depthAttachment = (Texture *) _depthAttachment;
  context->setRasterAttachments(colorAttachment, depthAttachment);
}

GPRT_API void
gprtGuiRasterize(GPRTContext _context) {
  Context *context = (Context *) _context;
  context->rasterizeGui();
}

GPRT_API GPRTContext
gprtContextCreate(int32_t *requestedDeviceIDs, int numRequestedDevices) {
  LOG_API_CALL();
  Context *context = new Context(requestedDeviceIDs, numRequestedDevices);
  return (GPRTContext) context;
}

GPRT_API void
gprtContextDestroy(GPRTContext _context) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  context->destroy();
  delete context;
  context = nullptr;
}

GPRT_API void
gprtContextSetRayTypeCount(GPRTContext _context, uint32_t numRayTypes) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  requestedFeatures.numRayTypes = numRayTypes;
}

GPRT_API size_t
gprtContextGetRayTypeCount(GPRTContext _context) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  return requestedFeatures.numRayTypes;
}

GPRT_API GPRTModule
gprtModuleCreate(GPRTContext _context, GPRTProgram spvCode) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Module *module = new Module(context, spvCode);
  return (GPRTModule) module;
}

GPRT_API void
gprtModuleDestroy(GPRTModule _module) {
  LOG_API_CALL();
  Module *module = (Module *) _module;
  module->destroy();
  delete module;
  module = nullptr;
}

GPRT_API GPRTGeom
gprtGeomCreate(GPRTContext _context, GPRTGeomType _geomType) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  GeomType *geomType = (GeomType *) _geomType;

  // depending on what the geomType is, we'll use this inherited "createGeom"
  // function to construct the appropriate geometry
  Geom *geometry = geomType->createGeom();
  return (GPRTGeom) geometry;
}

GPRT_API void
gprtGeomDestroy(GPRTGeom _geometry) {
  LOG_API_CALL();
  Geom *geometry = (Geom *) _geometry;
  geometry->destroy();
  delete geometry;
  geometry = nullptr;
}

GPRT_API void *
gprtGeomGetParameters(GPRTGeom _geometry, int deviceID) {
  LOG_API_CALL();
  Geom *geometry = (Geom *) _geometry;
  if (geometry->SBTRecord == nullptr)
    LOG_ERROR("Associated hit record for geometry type does not have any uniform parameters.");
  return geometry->SBTRecord;
}

GPRT_API void
gprtGeomSetParameters(GPRTGeom _geometry, void *parameters, int deviceID) {
  LOG_API_CALL();
  Geom *geometry = (Geom *) _geometry;
  if (geometry->SBTRecord == nullptr)
    LOG_ERROR("Associated hit record for geometry type does not have any uniform parameters.");
  memcpy(geometry->SBTRecord, parameters, geometry->recordSize);
}

// ==================================================================
// "Triangles" functions
// ==================================================================
GPRT_API void
gprtTrianglesSetVertices(GPRTGeom _triangles, GPRTBuffer _vertices, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  TriangleGeom *triangles = (TriangleGeom *) _triangles;
  if (triangles->geomType->getKind() != GPRT_TRIANGLES) LOG_ERROR("Calling gprtTrianglesSetVertices on non-triangular geometry type!");
  Buffer *vertices = (Buffer *) _vertices;
  triangles->setVertices(vertices, count, stride, offset);
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

GPRT_API void
gprtTrianglesSetIndices(GPRTGeom _triangles, GPRTBuffer _indices, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  TriangleGeom *triangles = (TriangleGeom *) _triangles;
  if (triangles->geomType->getKind() != GPRT_TRIANGLES) LOG_ERROR("Calling gprtTrianglesSetIndices on non-triangular geometry type!");
  Buffer *indices = (Buffer *) _indices;
  triangles->setIndices(indices, count, stride, offset);
}

GPRT_API void
gprtSpheresSetVertices(GPRTGeom _sphereGeom, GPRTBuffer _vertices, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  SphereGeom *sphereGeom = (SphereGeom *) _sphereGeom;
  if (sphereGeom->geomType->getKind() != GPRT_SPHERES) LOG_ERROR("Calling gprtSpheresSetVertices on non-sphere geometry type!");
  Buffer *vertices = (Buffer *) _vertices;
  sphereGeom->setVertices(vertices, count, stride, offset);
}

GPRT_API void
gprtLSSSetVertices(GPRTGeom _lssGeom, GPRTBuffer _vertices, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  LSSGeom *lssGeom = (LSSGeom *) _lssGeom;
  if (lssGeom->geomType->getKind() != GPRT_LSS) LOG_ERROR("Calling gprtLSSSetVertices on non-LSS geometry type!");
  Buffer *vertices = (Buffer *) _vertices;
  lssGeom->setVertices(vertices, count, stride, offset);
}

GPRT_API void
gprtLSSSetIndices(GPRTGeom _lssGeom, GPRTBuffer _indices, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  LSSGeom *lssGeom = (LSSGeom *) _lssGeom;
  if (lssGeom->geomType->getKind() != GPRT_LSS) LOG_ERROR("Calling gprtLSSSetIndices on non-LSS geometry type!");
  Buffer *indices = (Buffer *) _indices;
  lssGeom->setIndices(indices, count, stride, offset);
}

GPRT_API void
gprtSolidsSetVertices(GPRTGeom _solidGeom, GPRTBuffer _vertices, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  SolidGeom *solidGeom = (SolidGeom *) _solidGeom;
  if (solidGeom->geomType->getKind() != GPRT_SOLIDS) LOG_ERROR("Calling gprtSolidsSetVertices on non-solid geometry type!");
  Buffer *vertices = (Buffer *) _vertices;
  solidGeom->setVertices(vertices, count, stride, offset);
}

GPRT_API void
gprtSolidsSetIndices(GPRTGeom _solidGeom, GPRTBuffer _indices, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  SolidGeom *solidGeom = (SolidGeom *) _solidGeom;
  if (solidGeom->geomType->getKind() != GPRT_SOLIDS) LOG_ERROR("Calling gprtSolidsSetIndices on non-solid geometry type!");
  Buffer *indices = (Buffer *) _indices;
  solidGeom->setIndices(indices, count, stride, offset);
}

GPRT_API void
gprtSolidsSetTypes(GPRTGeom _solidGeom, GPRTBuffer _types, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  SolidGeom *solidGeom = (SolidGeom *) _solidGeom;
  if (solidGeom->geomType->getKind() != GPRT_SOLIDS) LOG_ERROR("Calling gprtSolidsSetTypes on non-solid geometry type!");
  Buffer *types = (Buffer *) _types;
  solidGeom->setTypes(types, count, stride, offset);
}

void
gprtSolidsSetPositions(GPRTGeom _solidGeom, GPRTBuffer _positions, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  SolidGeom *solidGeom = (SolidGeom *) _solidGeom;
  if (solidGeom->geomType->getKind() != GPRT_SOLIDS) LOG_ERROR("Calling gprtSolidsSetPositions on non-solid geometry type!");
  Buffer *positions = (Buffer *) _positions;
  solidGeom->setAABBs(positions, count, stride, offset);
}

void
gprtAABBsSetPositions(GPRTGeom _aabbs, GPRTBuffer _positions, uint32_t count, uint32_t stride, uint32_t offset) {
  LOG_API_CALL();
  AABBGeom *aabbs = (AABBGeom *) _aabbs;
  if (aabbs->geomType->getKind() != GPRT_AABBS) LOG_ERROR("Calling gprtAABBsSetPositions on non-AABB geometry type!");
  Buffer *positions = (Buffer *) _positions;
  aabbs->setAABBs(positions, count, stride, offset);
}

GPRT_API GPRTRayGen
gprtRayGenCreate(GPRTContext _context, GPRTModule _module, const char *programName, size_t recordSize) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Module *module = (Module *) _module;

  if (!module->checkForEntrypoint(programName)) {
    LOG_ERROR("RayGen program " + std::string(programName) + " not found in module!");
  }

  RayGen *raygen = new RayGen(context, module, programName, recordSize);

  // Creating a raygen program requires rebuilding a RT pipeline.
  context->raytracingPipelineOutOfDate = true;

  return (GPRTRayGen) raygen;
}

template <>
GPRTRayGenOf<void>
gprtRayGenCreate<void>(GPRTContext _context, GPRTModule _module, const char *programName) {
  return (GPRTRayGenOf<void>) gprtRayGenCreate(_context, _module, programName, 0);
}

GPRT_API void
gprtRayGenDestroy(GPRTRayGen _rayGen) {
  LOG_API_CALL();
  RayGen *rayGen = (RayGen *) _rayGen;
  rayGen->destroy();
  delete rayGen;
  rayGen = nullptr;

  // todo, remove from context->raygenPrograms, rebuild pipelines
}

GPRT_API void *
gprtRayGenGetParameters(GPRTRayGen _rayGen, int deviceID) {
  LOG_API_CALL();
  RayGen *rayGen = (RayGen *) _rayGen;
  return rayGen->SBTRecord;
}

GPRT_API void
gprtRayGenSetParameters(GPRTRayGen _rayGen, void *parameters, int deviceID) {
  LOG_API_CALL();
  RayGen *rayGen = (RayGen *) _rayGen;
  memcpy(rayGen->SBTRecord, parameters, rayGen->recordSize);
}

GPRT_API
GPRTCompute
gprtComputeCreate(GPRTContext _context, GPRTModule _module, const char *programName) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Module *module = (Module *) _module;

  if (!module->checkForEntrypoint(programName)) {
    LOG_ERROR("Compute program " + std::string(programName) + " not found in module!");
  }

  Compute *compute = new Compute(context, module, programName);

  // Notify context that we need to build this compute pipeline
  context->computePipelinesOutOfDate = true;

  return (GPRTCompute) compute;
}

GPRT_API
void
gprtComputeDestroy(GPRTCompute _compute) {
  LOG_API_CALL();
  Compute *compute = (Compute *) _compute;
  compute->destroy();
  delete compute;
  compute = nullptr;
}

GPRT_API GPRTMiss
gprtMissCreate(GPRTContext _context, GPRTModule _module, const char *programName, size_t recordSize) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Module *module = (Module *) _module;

  if (!module->checkForEntrypoint(programName)) {
    LOG_ERROR("Miss program " + std::string(programName) + " not found in module!");
  }

  Miss *missProg = new Miss(context, module, programName, recordSize);

  // Creating a miss program requires rebuilding a RT pipeline.
  context->raytracingPipelineOutOfDate = true;

  return (GPRTMiss) missProg;
}

template <>
GPRTMissOf<void>
gprtMissCreate<void>(GPRTContext _context, GPRTModule _module, const char *programName) {
  return (GPRTMissOf<void>) gprtMissCreate(_context, _module, programName, 0);
}

/*! sets the given miss program for the given ray type */
GPRT_API void
gprtMissSet(GPRTContext _context, int rayType, GPRTMiss _missToUse) {
  GPRT_NOTIMPLEMENTED;
}

GPRT_API void
gprtMissDestroy(GPRTMiss _miss) {
  LOG_API_CALL();
  Miss *missProg = (Miss *) _miss;
  missProg->destroy();
  delete missProg;
  missProg = nullptr;

  // todo, update context->missPrograms list... rebuild pipelines
}

GPRT_API void *
gprtMissGetParameters(GPRTMiss _miss, int deviceID) {
  LOG_API_CALL();
  Miss *miss = (Miss *) _miss;
  if (miss->SBTRecord == nullptr)
    LOG_ERROR("Miss entry point \"" + miss->entryPoint + "\" does not have any uniform parameters.");
  return miss->SBTRecord;
}

GPRT_API void
gprtMissSetParameters(GPRTMiss _miss, void *parameters, int deviceID) {
  LOG_API_CALL();
  Miss *miss = (Miss *) _miss;
  if (miss->SBTRecord == nullptr)
    LOG_ERROR("Miss entry point \"" + miss->entryPoint + "\" does not have any uniform parameters.");
  memcpy(miss->SBTRecord, parameters, miss->recordSize);
}

GPRT_API uint32_t
gprtMissGetIndex(GPRTMiss _miss, int deviceID) {
  LOG_API_CALL();
  Miss *miss = (Miss *) _miss;
  if (miss->address == -1)
    LOG_ERROR("Miss entry point \"" + miss->entryPoint + "\" is null.");
  return miss->address;
}

GPRT_API GPRTCallable
gprtCallableCreate(GPRTContext _context, GPRTModule _module, const char *programName, size_t recordSize) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Module *module = (Module *) _module;

  if (!module->checkForEntrypoint(programName)) {
    LOG_ERROR("Callable program " + std::string(programName) + " not found in module!");
  }

  Callable *callableProg = new Callable(context, module, programName, recordSize);

  // Creating a callable program requires rebuilding a RT pipeline.
  context->raytracingPipelineOutOfDate = true;

  return (GPRTCallable) callableProg;
}

template <>
GPRTCallableOf<void>
gprtCallableCreate<void>(GPRTContext _context, GPRTModule _module, const char *programName) {
  return (GPRTCallableOf<void>) gprtCallableCreate(_context, _module, programName, 0);
}

/*! sets the given callable program for the given ray type */
GPRT_API void
gprtCallableSet(GPRTContext _context, int rayType, GPRTCallable _callableToUse) {
  GPRT_NOTIMPLEMENTED;
}

GPRT_API void
gprtCallableDestroy(GPRTCallable _callable) {
  LOG_API_CALL();
  Callable *callableProg = (Callable *) _callable;
  callableProg->destroy();
  delete callableProg;
  callableProg = nullptr;

  // todo, update context->callablePrograms list... rebuild pipelines
}

GPRT_API void *
gprtCallableGetParameters(GPRTCallable _callable, int deviceID) {
  LOG_API_CALL();
  Callable *callable = (Callable *) _callable;
  return callable->SBTRecord;
}

GPRT_API void
gprtCallableSetParameters(GPRTCallable _callable, void *parameters, int deviceID) {
  LOG_API_CALL();
  Callable *callable = (Callable *) _callable;
  memcpy(callable->SBTRecord, parameters, callable->recordSize);
}

GPRT_API GPRTGeomType
gprtGeomTypeCreate(GPRTContext _context, GPRTGeomKind kind, size_t recordSize) {
  LOG_API_CALL();
  Context *context = (Context *) _context;

  GeomType *geomType = nullptr;

  switch (kind) {
  case GPRT_TRIANGLES:
    geomType = new TriangleGeomType(context, requestedFeatures.numRayTypes, recordSize);
    break;
  case GPRT_AABBS:
    geomType = new AABBGeomType(context, requestedFeatures.numRayTypes, recordSize);
    break;
  case GPRT_SOLIDS:
    geomType = new SolidGeomType(context, requestedFeatures.numRayTypes, recordSize);
    // Supply a built-in software intersector
    for (int i = 0; i < requestedFeatures.numRayTypes; i++) {
      gprtGeomTypeSetIntersectionProg((GPRTGeomType) geomType, i, (GPRTModule) context->fallbacksModule,
                                      "SolidIntersection");
    }
    break;
  case GPRT_LSS:
    geomType = new LSSGeomType(context, requestedFeatures.numRayTypes, recordSize);
    // Supply a software fallback intersectors when hardware support is missing
    for (int i = 0; i < requestedFeatures.numRayTypes; i++) {
      if (!requestedFeatures.linearSweptSpheres) {
        gprtGeomTypeSetIntersectionProg((GPRTGeomType) geomType, i, (GPRTModule) context->fallbacksModule,
                                        "LSSIntersection");
      }
    }
    break;
  case GPRT_SPHERES:
    geomType = new SphereGeomType(context, requestedFeatures.numRayTypes, recordSize);
    // Supply a software fallback intersectors when hardware support is missing
    for (int i = 0; i < requestedFeatures.numRayTypes; i++) {
      if (!requestedFeatures.linearSweptSpheres) {
        gprtGeomTypeSetIntersectionProg((GPRTGeomType) geomType, i, (GPRTModule) context->fallbacksModule,
                                        "SphereIntersection");
      }
    }
    break;
  default:
    GPRT_NOTIMPLEMENTED;
    break;
  }

  return (GPRTGeomType) geomType;
}

template <>
GPRTGeomTypeOf<void>
gprtGeomTypeCreate<void>(GPRTContext context, GPRTGeomKind kind) {
  return (GPRTGeomTypeOf<void>) gprtGeomTypeCreate(context, kind, 0);
}

GPRT_API void
gprtGeomTypeDestroy(GPRTGeomType _geomType) {
  LOG_API_CALL();
  GeomType *geomType = (GeomType *) _geomType;
  geomType->destroy();
  delete geomType;
  geomType = nullptr;
}

GPRT_API void
gprtGeomTypeSetClosestHitProg(GPRTGeomType _geomType, int rayType, GPRTModule _module, const char *progName) {
  LOG_API_CALL();
  GeomType *geomType = (GeomType *) _geomType;
  Module *module = (Module *) _module;

  if (!module->checkForEntrypoint(progName)) {
    LOG_ERROR("ClosestHit program " + std::string(progName) + " not found in module!");
  }

  geomType->setClosestHit(rayType, module, progName);
}

GPRT_API void
gprtGeomTypeSetAnyHitProg(GPRTGeomType _geomType, int rayType, GPRTModule _module, const char *progName) {
  LOG_API_CALL();
  GeomType *geomType = (GeomType *) _geomType;
  Module *module = (Module *) _module;

  if (!module->checkForEntrypoint(progName)) {
    LOG_ERROR("AnyHit program " + std::string(progName) + " not found in module!");
  }

  geomType->setAnyHit(rayType, module, progName);
}

GPRT_API void
gprtGeomTypeSetIntersectionProg(GPRTGeomType _geomType, int rayType, GPRTModule _module, const char *progName) {
  LOG_API_CALL();
  GeomType *geomType = (GeomType *) _geomType;
  if (geomType->getKind() == GPRT_TRIANGLES) {
    LOG_ERROR("Custom intersection programs are not allowed for the built-in triangle type");
  }

  Module *module = (Module *) _module;

  if (!module->checkForEntrypoint(progName)) {
    LOG_ERROR("Intersection program " + std::string(progName) + " not found in module!");
  }

  geomType->setIntersection(rayType, module, progName);
}

GPRT_API GPRTSampler
gprtSamplerCreate(GPRTContext _context, GPRTFilter magFilter, GPRTFilter minFilter, GPRTFilter mipFilter,
                  uint32_t anisotropy, GPRTSamplerAddressMode addressMode, GPRTBorderColor borderColor) {
  LOG_API_CALL();

  Context *context = (Context *) _context;
  Sampler *sampler = new Sampler(context, (VkFilter) magFilter, (VkFilter) minFilter, (VkSamplerMipmapMode) mipFilter,
                                 anisotropy, (VkSamplerAddressMode) addressMode, (VkBorderColor) borderColor);

  return (GPRTSampler) sampler;
}

GPRT_API void
gprtSamplerDestroy(GPRTSampler _sampler) {
  LOG_API_CALL();

  Sampler *sampler = (Sampler *) _sampler;
  sampler->destroy();

  delete sampler;
  sampler = nullptr;
}

GPRT_API uint32_t
gprtSamplerGetIndex(GPRTSampler _sampler, int deviceID) {
  LOG_API_CALL();
  Sampler *sampler = (Sampler *) _sampler;
  return sampler->address;
}

GPRT_API GPRTTexture
gprtHostTextureCreate(GPRTContext _context, GPRTImageType type, GPRTFormat format, uint32_t width, uint32_t height,
                      uint32_t depth, bool allocateMipmaps, const void *init) {
  LOG_API_CALL();

  const VkImageUsageFlags imageUsageFlags =
      // means we can make an image view required to assign this image to a
      // descriptor
      VK_IMAGE_USAGE_SAMPLED_BIT |
      VK_IMAGE_USAGE_STORAGE_BIT |
      // means we can use this image to transfer into another
      VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
      // means we can use this image to receive data transferred from another
      VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags =
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |   // mappable to host with vkMapMemory
      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;   // means "flush" and "invalidate"
                                              // not needed

  Context *context = (Context *) _context;
  Texture *texture = new Texture(context, imageUsageFlags, memoryUsageFlags, (VkImageType) type, (VkFormat) format,
                                 width, height, depth, allocateMipmaps, init);

  // Pin the texture to the host
  texture->map();

  return (GPRTTexture) texture;
}

GPRT_API GPRTTexture
gprtDeviceTextureCreate(GPRTContext _context, GPRTImageType type, GPRTFormat format, uint32_t width, uint32_t height,
                        uint32_t depth, bool allocateMipmaps, const void *init) {
  LOG_API_CALL();

  const VkImageUsageFlags imageUsageFlags =
      // means we can make an image view required to assign this image to a
      // descriptor
      VK_IMAGE_USAGE_SAMPLED_BIT |
      VK_IMAGE_USAGE_STORAGE_BIT |
      // means we can use this image to transfer into another
      VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
      // means we can use this image to receive data transferred from another
      VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;   // means most efficient for
                                                                                        // device access

  Context *context = (Context *) _context;
  Texture *texture = new Texture(context, imageUsageFlags, memoryUsageFlags, (VkImageType) type, (VkFormat) format,
                                 width, height, depth, allocateMipmaps, init);

  return (GPRTTexture) texture;
}

GPRT_API GPRTTexture
gprtSharedTextureCreate(GPRTContext _context, GPRTImageType type, GPRTFormat format, uint32_t width, uint32_t height,
                        uint32_t depth, bool allocateMipmaps, const void *init) {
  LOG_API_CALL();

  const VkImageUsageFlags imageUsageFlags =
      // means we can make an image view required to assign this image to a
      // descriptor
      VK_IMAGE_USAGE_SAMPLED_BIT |
      VK_IMAGE_USAGE_STORAGE_BIT |
      // means we can use this image to transfer into another
      VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
      // means we can use this image to receive data transferred from another
      VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags =
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |    // mappable to host with vkMapMemory
      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |   // means "flush" and "invalidate"
                                               // not needed
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;     // means most efficient for device
                                               // access

  Context *context = (Context *) _context;
  Texture *texture = new Texture(context, imageUsageFlags, memoryUsageFlags, (VkImageType) type, (VkFormat) format,
                                 width, height, depth, allocateMipmaps, init);

  // Pin the texture to the host
  texture->map();

  return (GPRTTexture) texture;
}

GPRT_API size_t
gprtTextureGetRowPitch(GPRTTexture _texture) {
  Texture *texture = (Texture *) _texture;
  return texture->subresourceLayout.rowPitch;
}

GPRT_API size_t
gprtTextureGetDepthPitch(GPRTTexture _texture) {
  Texture *texture = (Texture *) _texture;
  return texture->subresourceLayout.depthPitch;
}

GPRT_API void
gprtTextureGenerateMipmap(GPRTTexture _texture) {
  Texture *texture = (Texture *) _texture;
  texture->generateMipmap();
}

GPRT_API void *
gprtTextureGetPointer(GPRTTexture _texture, int deviceID) {
  LOG_API_CALL();
  Texture *texture = (Texture *) _texture;
  return texture->mapped;
}

GPRT_API uint32_t
gprtTextureGetIndex(GPRTTexture _texture, int deviceID) {
  LOG_API_CALL();
  Texture *texture = (Texture *) _texture;
  return texture->address;
}

GPRT_API void
gprtTextureClear(GPRTTexture _texture) {
  LOG_API_CALL();
  Texture *texture = (Texture *) _texture;
  texture->clear();
}

GPRT_API void
gprtTextureDenoise(GPRTContext _context, const GPRTDenoiseParams &params)
{
  LOG_API_CALL();
  Context *context = (Context *) _context;

  if (context->deviceProperties.vendorID == VENDOR_ID_NVIDIA)
  {
    /* Transition resolve image to a general format */
    // We need to carve out an API to allow textures to transition from readonly to readwrite by the end user
    auto oldResolveLayout = ((Texture*)params.resolvedColor)->layout;
    if (oldResolveLayout != VK_IMAGE_LAYOUT_GENERAL)
    {
      VkImageSubresourceRange imageSubresource;
      imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;   // temporary, just assuming color for now.
      imageSubresource.baseArrayLayer = 0;
      imageSubresource.baseMipLevel = 0;
      imageSubresource.layerCount = 1;
      imageSubresource.levelCount = 1;
      
      VkCommandBuffer commandList = context->beginSingleTimeCommands(context->graphicsCommandPool);
      ((Texture*)params.resolvedColor)->setImageLayout(commandList, ((Texture*)params.resolvedColor)->image, 
        oldResolveLayout, VK_IMAGE_LAYOUT_GENERAL, imageSubresource);
      context->endSingleTimeCommands(commandList, context->graphicsCommandPool, context->graphicsQueue);
      ((Texture*)params.resolvedColor)->layout = VK_IMAGE_LAYOUT_GENERAL;
    }

    // Buffers don't actually seem to be supported.
    // auto getBufferResource = [](Buffer* pBuffer) -> NVSDK_NGX_Resource_VK
    // {
    //   if (!pBuffer) return {};
    //   return NVSDK_NGX_Create_Buffer_Resource_VK(pBuffer->buffer, pBuffer->getSize(), true);
    // };

    // auto getAspectMaskFromFormat = [](VkFormat format) -> VkImageAspectFlags
    // {
    //     switch (format)
    //     {
    //     case VK_FORMAT_D16_UNORM_S8_UINT:
    //     case VK_FORMAT_D24_UNORM_S8_UINT:
    //     case VK_FORMAT_D32_SFLOAT_S8_UINT:
    //         return VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
    //     case VK_FORMAT_D16_UNORM:
    //     case VK_FORMAT_D32_SFLOAT:
    //     case VK_FORMAT_X8_D24_UNORM_PACK32:
    //         return VK_IMAGE_ASPECT_DEPTH_BIT;
    //     case VK_FORMAT_S8_UINT:
    //         return VK_IMAGE_ASPECT_STENCIL_BIT;
    //     default:
    //         return VK_IMAGE_ASPECT_COLOR_BIT;
    //     }
    // };

    auto getImageView = [](Texture* pTexture, bool isReadWrite = false) -> NVSDK_NGX_Resource_VK
    {
        if (!pTexture)
            return {};

        VkImageView imageView = pTexture->imageView;
        VkImage image = pTexture->image; 
        VkFormat format = pTexture->format;
        VkImageSubresourceRange range;
        range.aspectMask = pTexture->aspectFlagBits;//getAspectMaskFromFormat(format);
        range.baseMipLevel = 0;
        range.levelCount = 1;
        range.baseArrayLayer = 0;
        range.layerCount = 1;
        return NVSDK_NGX_Create_ImageView_Resource_VK(
            imageView, image, range, format, pTexture->width, pTexture->height, isReadWrite
        );
    };

    NVSDK_NGX_Resource_VK unresolvedColorBuffer = getImageView((Texture*)params.unresolvedColor);
    NVSDK_NGX_Resource_VK motionVectorsBuffer = getImageView((Texture*)params.diffuseMotionVectors);
    NVSDK_NGX_Resource_VK resolvedColorBuffer = getImageView((Texture*)params.resolvedColor, true);
    NVSDK_NGX_Resource_VK depthBuffer = getImageView((Texture*)params.depthBuffer);
    // NVSDK_NGX_Resource_VK exposureBuffer = getBufferResource(pExposure);
    
    VkCommandBuffer commandList = context->beginSingleTimeCommands(context->graphicsCommandPool);

    NVSDK_NGX_VK_DLSS_Eval_Params evalParams = {};
    evalParams.Feature.pInColor  = &unresolvedColorBuffer;
    evalParams.Feature.pInOutput = &resolvedColorBuffer; 
    evalParams.pInDepth          = &depthBuffer;
    evalParams.pInMotionVectors  = &motionVectorsBuffer;
    evalParams.InJitterOffsetX = params.jitter.x;
    evalParams.InJitterOffsetY = params.jitter.y; // todo...
    evalParams.InMVScaleX = 1.f;
    evalParams.InMVScaleY = 1.f;
    evalParams.Feature.InSharpness = context->aiDenoising.optimalSettings.sharpness;
    evalParams.InRenderSubrectDimensions.Width = context->aiDenoising.optimalSettings.optimalRenderSize.x;//requestedFeatures.aiDenoiser.outputWidth;
    evalParams.InRenderSubrectDimensions.Height = context->aiDenoising.optimalSettings.optimalRenderSize.y;//requestedFeatures.aiDenoiser.outputHeight;

    NVSDK_NGX_Result result = NGX_VULKAN_EVALUATE_DLSS_EXT(commandList, context->aiDenoising.ngxHandle, context->aiDenoising.ngxParameters, &evalParams);
    context->endSingleTimeCommands(commandList, context->graphicsCommandPool, context->graphicsQueue);


    if (oldResolveLayout != VK_IMAGE_LAYOUT_GENERAL)
    {
      VkImageSubresourceRange imageSubresource;
      imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;   // temporary, just assuming color for now.
      imageSubresource.baseArrayLayer = 0;
      imageSubresource.baseMipLevel = 0;
      imageSubresource.layerCount = 1;
      imageSubresource.levelCount = 1;
      
      VkCommandBuffer commandList = context->beginSingleTimeCommands(context->graphicsCommandPool);
      ((Texture*)params.resolvedColor)->setImageLayout(commandList, ((Texture*)params.resolvedColor)->image, 
      VK_IMAGE_LAYOUT_GENERAL, oldResolveLayout, imageSubresource);
      context->endSingleTimeCommands(commandList, context->graphicsCommandPool, context->graphicsQueue);
      ((Texture*)params.resolvedColor)->layout = oldResolveLayout;
    }



    // if (NVSDK_NGX_FAILED(result)) {LOG_WARNING("Failed to NGX_VULKAN_EVALUATE_DLSS_EXT for DLSS: {}", context->aiDenoising.resultToString(result));}
    


    // Determine pass I/O sizes based on bound textures.
    // const auto& pOutput = renderData.getTexture(kOutput);
    // const auto& pColor = renderData.getTexture(kColorInput);
    // FALCOR_ASSERT(pColor && pOutput);

    // mPassOutputSize = {pOutput->getWidth(), pOutput->getHeight()};
    // const uint2 inputSize = {pColor->getWidth(), pColor->getHeight()};

    // if (!mEnabled || !mpScene)
    // {
    //     pRenderContext->blit(pColor->getSRV(), pOutput->getRTV());
    //     return;
    // }

    // if (mExposureUpdated)
    // {
    //     float exposure = pow(2.f, mExposure);
    //     pRenderContext->updateTextureData(mpExposure.get(), &exposure);
    //     mExposureUpdated = false;
    // }

    // if (mRecreate || any(inputSize != mInputSize))
    // {
    //     mRecreate = false;
    //     mInputSize = inputSize;

    //     initializeDLSS(pRenderContext);

    //     // If pass output is configured to be fixed to DLSS output, but the sizes don't match,
    //     // we'll trigger a graph recompile to update the pass I/O size requirements.
    //     // This causes a one frame delay, but unfortunately we don't know the size until after initializeDLSS().
    //     if (mOutputSizeSelection == RenderPassHelpers::IOSize::Fixed && any(mPassOutputSize != mDLSSOutputSize))
    //         requestRecompile();
    // }

    // {
    //     // Fetch inputs and verify their dimensions.
    //     auto getInput = [=](const std::string& name)
    //     {
    //         auto tex = renderData.getTexture(name);
    //         if (!tex)
    //             FALCOR_THROW("DLSSPass: Missing input '{}'", name);
    //         if (tex->getWidth() != mInputSize.x || tex->getHeight() != mInputSize.y)
    //             FALCOR_THROW("DLSSPass: Input '{}' has mismatching size. All inputs must be of the same size.", name);
    //         return tex;
    //     };

    //     auto color = getInput(kColorInput);
    //     auto depth = getInput(kDepthInput);
    //     auto motionVectors = getInput(kMotionVectorsInput);

    //     // Determine if we can write directly to the render pass output.
    //     // Otherwise we'll output to an internal buffer and blit to the pass output.
    //     FALCOR_ASSERT(mpOutput->getWidth() == mDLSSOutputSize.x && mpOutput->getHeight() == mDLSSOutputSize.y);
    //     bool useInternalBuffer = (any(mDLSSOutputSize != mPassOutputSize) || pOutput->getFormat() != mpOutput->getFormat());

    //     auto output = useInternalBuffer ? mpOutput : pOutput;

    //     // In DLSS X-jitter should go left-to-right, Y-jitter should go top-to-bottom.
    //     // Falcor is using math::perspective() that gives coordinate system with
    //     // X from -1 to 1, left-to-right, and Y from -1 to 1, bottom-to-top.
    //     // Therefore, we need to flip the Y-jitter only.
    //     const auto& camera = mpScene->getCamera();
    //     float2 jitterOffset = float2(camera->getJitterX(), -camera->getJitterY()) * float2(inputSize);
    //     float2 motionVectorScale = float2(1.f, 1.f);
    //     if (mMotionVectorScale == MotionVectorScale::Relative)
    //         motionVectorScale = inputSize;

    //     mpNGXWrapper->evaluateDLSS(
    //         pRenderContext,
    //         color.get(),
    //         output.get(),
    //         motionVectors.get(),
    //         depth.get(),
    //         mpExposure.get(),
    //         false,
    //         mSharpness,
    //         jitterOffset,
    //         motionVectorScale
    //     );

    //     // Resample the upscaled result from DLSS to the pass output if needed.
    //     if (useInternalBuffer)
    //         pRenderContext->blit(mpOutput->getSRV(), pOutput->getRTV());
    // }
  }
  else
  {
    LOG_ERROR("Unimplemented");
  }
}

GPRT_API void
gprtTextureMap(GPRTTexture _texture, int deviceID) {
  LOG_API_CALL();
  Texture *texture = (Texture *) _texture;
  texture->map();
}

GPRT_API void
gprtTextureUnmap(GPRTTexture _texture, int deviceID) {
  LOG_API_CALL();
  Texture *texture = (Texture *) _texture;
  texture->unmap();
}

GPRT_API void
gprtTextureDestroy(GPRTTexture _texture) {
  LOG_API_CALL();
  Texture *texture = (Texture *) _texture;
  texture->destroy();
  delete texture;
  texture = nullptr;
}

GPRT_API GPRTBuffer
gprtHostBufferCreate(GPRTContext _context, size_t size, size_t count, const void *init, size_t alignment) {
  LOG_API_CALL();
  const VkBufferUsageFlags bufferUsageFlags =
      // means we can get this buffer's address with vkGetBufferDeviceAddress
      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
      // I guess I need this to use these buffers as input to tree builds?
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
      // means we can use this buffer to transfer into another
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
      // means we can use this buffer to receive data transferred from another
      VK_BUFFER_USAGE_TRANSFER_DST_BIT |
      // means we can use this buffer as an index buffer for rasterization
      VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
      // means we can use this buffer as a storage buffer resource
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags =
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |   // mappable to host with vkMapMemory
      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;   // means "flush" and "invalidate"
                                              // not needed

  Context *context = (Context *) _context;
  Buffer *buffer =
      new Buffer(context, bufferUsageFlags, memoryUsageFlags, size * count, alignment);

  // Pin the buffer to the host
  buffer->map();

  if (init) {
    void *mapped = buffer->mapped;
    memcpy(mapped, init, size * count);
  }
  return (GPRTBuffer) buffer;
}

GPRT_API GPRTBuffer
gprtDeviceBufferCreate(GPRTContext _context, size_t size, size_t count, const void *init, size_t alignment) {
  LOG_API_CALL();
  const VkBufferUsageFlags bufferUsageFlags =
      // means we can get this buffer's address with vkGetBufferDeviceAddress
      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
      // I guess I need this to use these buffers as input to tree builds?
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
      // means we can use this buffer to transfer into another
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
      // means we can use this buffer to receive data transferred from another
      VK_BUFFER_USAGE_TRANSFER_DST_BIT |
      // means we can use this buffer as an index buffer for rasterization
      VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
      // means we can use this buffer as a storage buffer resource
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;   // means most efficient for
                                                                                        // device access

  Context *context = (Context *) _context;
  Buffer *buffer =
      new Buffer(context, bufferUsageFlags, memoryUsageFlags, size * count, alignment);

  if (init) {
    buffer->map();
    void *mapped = buffer->mapped;
    memcpy(mapped, init, size * count);
    buffer->unmap();
  }
  return (GPRTBuffer) buffer;
}

GPRT_API GPRTBuffer
gprtSharedBufferCreate(GPRTContext _context, size_t size, size_t count, const void *init, size_t alignment) {
  LOG_API_CALL();
  const VkBufferUsageFlags bufferUsageFlags =
      // means we can get this buffer's address with vkGetBufferDeviceAddress
      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
      // I guess I need this to use these buffers as input to tree builds?
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
      // means we can use this buffer to transfer into another
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
      // means we can use this buffer to receive data transferred from another
      VK_BUFFER_USAGE_TRANSFER_DST_BIT |
      // means we can use this buffer as an index buffer for rasterization
      VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
      // means we can use this buffer as a storage buffer resource
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags =
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |    // mappable to host with vkMapMemory
      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |   // means "flush" and "invalidate"
                                               // not needed
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;     // means most efficient for device
                                               // access

  Context *context = (Context *) _context;
  Buffer *buffer =
      new Buffer(context, bufferUsageFlags, memoryUsageFlags, size * count, alignment);

  // Pin the buffer to the host
  buffer->map();

  if (init) {
    void *mapped = buffer->mapped;
    memcpy(mapped, init, size * count);
    buffer->flush();
  }
  return (GPRTBuffer) buffer;
}

GPRT_API void
gprtBufferClear(GPRTBuffer _buffer) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  buffer->clear();
}

GPRT_API void
gprtBufferDestroy(GPRTBuffer _buffer) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  buffer->destroy();
  delete buffer;
  buffer = nullptr;
}

GPRT_API size_t
gprtBufferGetSize(GPRTBuffer _buffer, int deviceID) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  return buffer->getSize();
}

GPRT_API void *
gprtBufferGetHostPointer(GPRTBuffer _buffer, int deviceID) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  return buffer->mapped;
}

GPRT_API void *
gprtBufferGetDevicePointer(GPRTBuffer _buffer, int deviceID) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  uint64_t addr = buffer->getDeviceAddress();
  return (void *) addr;
}

GPRT_API void
gprtBufferMap(GPRTBuffer _buffer, int deviceID) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  buffer->map();
}

GPRT_API void
gprtBufferUnmap(GPRTBuffer _buffer, int deviceID) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  buffer->unmap();
}

GPRT_API void
gprtBufferCopy(GPRTContext _context, GPRTBuffer _source, GPRTBuffer _destination, size_t srcOffset, size_t dstOffset,
               size_t size, size_t count, int srcDeviceID, int dstDeviceID) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Buffer *destination = (Buffer *) _destination;
  Buffer *source = (Buffer *) _source;

  VkCommandBuffer commandBuffer = context->beginSingleTimeCommands(context->graphicsCommandPool);

  VkBufferCopy region;
  region.srcOffset = srcOffset * size;
  region.dstOffset = dstOffset * size;
  region.size = count * size;
  vkCmdCopyBuffer(commandBuffer, source->buffer, destination->buffer, 1, &region);

  context->endSingleTimeCommands(commandBuffer, context->graphicsCommandPool, context->graphicsQueue);
}

void
gprtBufferTextureCopy(GPRTContext _context, GPRTBuffer _buffer, GPRTTexture _texture, uint32_t bufferOffset,
                      uint32_t bufferRowLength, uint32_t bufferImageHeight, uint32_t imageOffsetX,
                      uint32_t imageOffsetY, uint32_t imageOffsetZ, uint32_t imageExtentX, uint32_t imageExtentY,
                      uint32_t imageExtentZ, int srcDeviceID, int dstDeviceID) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Texture *texture = (Texture *) _texture;
  Buffer *buffer = (Buffer *) _buffer;

  VkCommandBuffer commandBuffer = context->beginSingleTimeCommands(context->graphicsCommandPool);

  VkImageLayout originalLayout = texture->layout;

  // transition to a transfer destination format
  texture->setImageLayout(commandBuffer, texture->image, texture->layout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                          {uint32_t(texture->aspectFlagBits), 0, texture->mipLevels, 0, 1});
  texture->layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

  VkBufferImageCopy region;
  region.bufferOffset = bufferOffset;
  region.bufferRowLength = bufferRowLength;
  region.bufferImageHeight = bufferImageHeight;
  region.imageOffset.x = imageOffsetX;
  region.imageOffset.y = imageOffsetY;
  region.imageOffset.z = imageOffsetZ;
  region.imageExtent.width = imageExtentX;
  region.imageExtent.height = imageExtentY;
  region.imageExtent.depth = imageExtentZ;
  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;   // temporary, just assuming color for now.
  region.imageSubresource.baseArrayLayer = 0;
  region.imageSubresource.mipLevel = 0;
  region.imageSubresource.layerCount = 1;
  vkCmdCopyBufferToImage(commandBuffer, buffer->buffer, texture->image, texture->layout, 1, &region);

  // transition back to original format
  texture->setImageLayout(commandBuffer, texture->image, texture->layout, originalLayout,
                          {uint32_t(texture->aspectFlagBits), 0, texture->mipLevels, 0, 1});
  texture->layout = originalLayout;

  context->endSingleTimeCommands(commandBuffer, context->graphicsCommandPool, context->graphicsQueue);
}

void
gprtBufferResize(GPRTContext _context, GPRTBuffer _buffer, size_t size, size_t count, bool preserveContents,
                 int deviceID) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Buffer *buffer = (Buffer *) _buffer;
  buffer->resize(size * count, preserveContents);
}

uint32_t
bufferScan(GPRTContext _context, GPRTBuffer _input, GPRTBuffer _output, GPRTBuffer _scratch, bool partition,
           bool select, bool selectPositive) {
  LOG_ERROR("Not implemented yet\n");
  return -1;

  // LOG_API_CALL();
  // Context *context = (Context *) _context;
  // Buffer *input = (Buffer *) _input;
  // Buffer *output = (Buffer *) _output;
  // Buffer *scratch = (Buffer *) _scratch;

  // // note, input->getSize() here should always be a multiple of 16 bytes
  // uint32_t numItems = uint32_t(input->getSize() / sizeof(uint32_t));

  // auto InitChainedDecoupledExclusive = (GPRTComputeOf<gprt::ScanParams>)
  // context->internalComputePrograms["InitScan"]; auto ChainedDecoupledExclusive = (GPRTComputeOf<gprt::ScanParams>)
  // context->internalComputePrograms["Scan"]; uint32_t numThreadBlocks = (numItems + (SCAN_PARTITON_SIZE - 1)) /
  // SCAN_PARTITON_SIZE;

  // // Each group gets an aggregate/inclusive prefix and a status flag.
  // // We also reserve one int for the total aggregate count, and one
  // // for group-to-partition scheduling
  // if (scratch->getSize() < (numThreadBlocks + 2) * sizeof(uint32_t)) {
  //   // updating SBT, since we're using atomics here...
  //   // (requires updating descriptor sets when buffer allocations change)
  //   scratch->resize((numThreadBlocks + 2) * sizeof(uint32_t), false);
  //   gprtBuildShaderBindingTable(_context);
  // }

  // // Todo, remove these buffer handles in favor of device pointers
  // gprt::ScanParams scanParams;
  // scanParams.size = numItems;
  // scanParams.output = gprt::Buffer{output->virtualAddress, 0};   // gprtBufferGetHandle(_output);
  // scanParams.input = gprt::Buffer{input->virtualAddress, 0};     // gprtBufferGetHandle(_input);
  // scanParams.state = gprt::Buffer{scratch->virtualAddress, 0};   // gprtBufferGetHandle(_scratch);
  // scanParams.inputPtr = (uint64_t)gprtBufferGetDevicePointer(_input);
  // scanParams.outputPtr = (uint64_t)gprtBufferGetDevicePointer(_output);
  // scanParams.flags = 0;
  // if (partition)
  //   scanParams.flags |= SCAN_PARTITION;
  // else if (select)
  //   scanParams.flags |= SCAN_SELECT;

  // if (selectPositive)
  //   scanParams.flags |= SCAN_SELECT_POSITIVE;

  // uint3 numGroups = {numThreadBlocks, 1, 1};
  // uint3 groupSize = {1, 1, 1};
  // gprtComputeLaunch(InitChainedDecoupledExclusive, numGroups, groupSize, scanParams);

  // if (context->subgroupProperties.subgroupSize == 32)
  //   groupSize = {32, SCAN_PARTITON_SIZE / (16 * 32), 1};
  // else
  //   groupSize = {64, SCAN_PARTITON_SIZE / (16 * 64), 1};   // Todo, account for lane sizes other than 32/64
  // gprtComputeLaunch(ChainedDecoupledExclusive, numGroups, groupSize, scanParams);

  // scratch->map(sizeof(uint32_t), 0);
  // uint32_t total = *((uint32_t *) scratch->mapped);
  // scratch->unmap(sizeof(uint32_t), 0);
  // return total;
}

uint32_t
gprtBufferExclusiveSum(GPRTContext _context, GPRTBuffer _input, GPRTBuffer _output, GPRTBuffer _scratch) {
  // Redirection here, since we might eventually change the below function to support inclusive and exclusive,
  // also to include operators other than addition.
  return bufferScan(_context, _input, _output, _scratch, false, false, false);
}

uint32_t
gprtBufferPartition(GPRTContext _context, GPRTBuffer _input, bool selectPositive, GPRTBuffer _output,
                    GPRTBuffer _scratch) {
  return bufferScan(_context, _input, _output, _scratch, true, false, selectPositive);
}

uint32_t
gprtBufferSelect(GPRTContext _context, GPRTBuffer _input, bool selectPositive, GPRTBuffer _output,
                 GPRTBuffer _scratch) {
  return bufferScan(_context, _input, _output, _scratch, false, true, selectPositive);
}

void
bufferSort(GPRTContext _context, GPRTBuffer _keys, GPRTBuffer _values, GPRTBuffer _scratch) {
  LOG_API_CALL();

  Context *context = (Context *) _context;
  Buffer *keys = (Buffer *) _keys;
  Buffer *values = (Buffer *) _values;
  Buffer *scratch = (Buffer *) _scratch;

  bool bHasPayload = false;
  if (values) {
    if (keys->getSize() != values->getSize())
      LOG_ERROR("Keys and Values buffers must be equal in size\n");

    bHasPayload = true;
  }

  uint32_t numKeys = uint32_t(keys->getSize() / sizeof(uint64_t));
  uint32_t maxNumThreadgroups = 800;
  ParallelSortCB constantBufferData = {0};

  // Allocate the scratch buffers needed for radix sort
  auto alignedSize = [](size_t value, size_t alignment) -> size_t {
    return (value + alignment - 1) & ~(alignment - 1);
  };
  uint32_t offsetAlignment = (uint32_t) context->deviceProperties.limits.minStorageBufferOffsetAlignment;

  uint64_t scratchBufferSize;
  uint64_t reducedScratchBufferSize;
  ParallelSort_CalculateScratchResourceSize(numKeys, scratchBufferSize, reducedScratchBufferSize);
  scratchBufferSize = alignedSize(scratchBufferSize, offsetAlignment);
  reducedScratchBufferSize = alignedSize(reducedScratchBufferSize, offsetAlignment);

  uint64_t keysSize = alignedSize(keys->size, offsetAlignment);
  uint64_t valuesSize = ((bHasPayload) ? alignedSize(values->size, offsetAlignment) : 0);

  scratch->resize(keysSize + valuesSize + scratchBufferSize + reducedScratchBufferSize,
                  /*don't transfer old contents*/ false);
  // All offsets must be a multiple of device limit VkPhysicalDeviceLimits::minStorageBufferOffseteAlignment
  size_t valuesOffset = keysSize;
  size_t scratchOffset = keysSize + valuesSize;
  size_t reducedScratchOffset = keysSize + valuesSize + scratchBufferSize;

  uint32_t NumThreadgroupsToRun;
  uint32_t NumReducedThreadgroupsToRun;
  ParallelSort_SetConstantAndDispatchData(numKeys, maxNumThreadgroups, constantBufferData, NumThreadgroupsToRun,
                                          NumReducedThreadgroupsToRun);

  auto BindUAVBuffer = [&](VkBuffer *pBuffer, VkDeviceSize *Offsets, VkDescriptorSet &DescriptorSet,
                           uint32_t Binding /*=0*/, uint32_t Count /*=1*/) {
    std::vector<VkDescriptorBufferInfo> bufferInfos;
    for (uint32_t i = 0; i < Count; i++) {
      VkDescriptorBufferInfo bufferInfo;
      bufferInfo.buffer = pBuffer[i];
      bufferInfo.offset = Offsets[i];
      bufferInfo.range = VK_WHOLE_SIZE;
      bufferInfos.push_back(bufferInfo);
    }

    VkWriteDescriptorSet write_set = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    write_set.pNext = nullptr;
    write_set.dstSet = DescriptorSet;
    write_set.dstBinding = Binding;
    write_set.dstArrayElement = 0;
    write_set.descriptorCount = Count;
    write_set.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    write_set.pImageInfo = nullptr;
    write_set.pBufferInfo = bufferInfos.data();
    write_set.pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(context->logicalDevice, 1, &write_set, 0, nullptr);
  };

  // Do binding setups
  {
    VkBuffer BufferMaps[4];
    VkDeviceSize Offsets1[4] = {0, 0, 0, 0};

    // Map inputs/outputs
    BufferMaps[0] = keys->buffer;
    BufferMaps[1] = scratch->buffer;
    if (bHasPayload) {
      BufferMaps[2] = values->buffer;
      BufferMaps[3] = scratch->buffer;
      Offsets1[2] = 0;
      Offsets1[3] = valuesOffset;
    }
    BindUAVBuffer(BufferMaps, Offsets1, context->sortStages.m_SortDescriptorSetInputOutput[0], 0,
                  (bHasPayload) ? 4 : 2);

    BufferMaps[0] = scratch->buffer;
    BufferMaps[1] = keys->buffer;
    if (bHasPayload) {
      BufferMaps[2] = scratch->buffer;
      BufferMaps[3] = values->buffer;
      Offsets1[2] = valuesOffset;
      Offsets1[3] = 0;
    }
    BindUAVBuffer(BufferMaps, Offsets1, context->sortStages.m_SortDescriptorSetInputOutput[1], 0,
                  (bHasPayload) ? 4 : 2);

    // Map scan sets (reduced, scratch)
    VkDeviceSize Offsets2[4] = {reducedScratchOffset, reducedScratchOffset, 0, 0};
    BufferMaps[0] = BufferMaps[1] = scratch->buffer;
    BufferMaps[2] = scratch->buffer;
    BindUAVBuffer(BufferMaps, Offsets2, context->sortStages.m_SortDescriptorSetScanSets[0], 0, 3);

    BufferMaps[0] = BufferMaps[1] = scratch->buffer;
    BufferMaps[2] = scratch->buffer;
    VkDeviceSize Offsets3[4] = {scratchOffset, scratchOffset, reducedScratchOffset, 0};
    BindUAVBuffer(BufferMaps, Offsets3, context->sortStages.m_SortDescriptorSetScanSets[1], 0, 3);

    // Map Scratch areas (fixed)
    BufferMaps[0] = scratch->buffer;
    BufferMaps[1] = scratch->buffer;
    VkDeviceSize Offsets4[4] = {scratchOffset, reducedScratchOffset, 0, 0};
    BindUAVBuffer(BufferMaps, Offsets4, context->sortStages.m_SortDescriptorSetScratch, 0, 2);
  }

  // Transition barrier
  auto BufferTransition = [](VkBuffer buffer, VkAccessFlags before, VkAccessFlags after, VkDeviceSize offset,
                             VkDeviceSize size) {
    VkBufferMemoryBarrier bufferBarrier = {};
    bufferBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    bufferBarrier.srcAccessMask = before;
    bufferBarrier.dstAccessMask = after;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.buffer = buffer;
    bufferBarrier.size = VK_WHOLE_SIZE;
    // bufferBarrier.offset = offset;
    // bufferBarrier.size = size;

    return bufferBarrier;
  };

  VkCommandBuffer commandList = context->beginSingleTimeCommands(context->graphicsCommandPool);

  // Bind the scratch descriptor sets
  vkCmdBindDescriptorSets(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.layout, 2, 1,
                          &context->sortStages.m_SortDescriptorSetScratch, 0, nullptr);

  // Push the data into the constant buffer and bind
  vkCmdPushConstants(commandList, context->sortStages.layout, VK_SHADER_STAGE_ALL, 0, sizeof(ParallelSortCB),
                     &constantBufferData);

  // Perform Radix Sort (currently only support 64-bit key/payload sorting
  uint32_t inputSet = 0;
  VkBufferMemoryBarrier Barriers[3];
  for (uint64_t Shift = 0; Shift < 64u; Shift += PARALLELSORT_SORT_BITS_PER_PASS) {
    // Update the bit shift
    vkCmdPushConstants(commandList, context->sortStages.layout, VK_SHADER_STAGE_ALL, sizeof(ParallelSortCB) - 4, 4,
                       &Shift);
    //     vkCmdPushConstants(commandList, m_SortPipelineLayout, VK_SHADER_STAGE_ALL, 0, 4, &Shift);

    // Bind input/output for this pass
    vkCmdBindDescriptorSets(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.layout, 0, 1,
                            &context->sortStages.m_SortDescriptorSetInputOutput[inputSet], 0, nullptr);

    // Sort Count
    {
      vkCmdBindPipeline(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.Count.pipeline);
      vkCmdDispatch(commandList, NumThreadgroupsToRun, 1, 1);
    }

    // UAV barrier on the sum table
    Barriers[0] =
        BufferTransition(scratch->buffer, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                         VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, scratchOffset, scratchBufferSize);
    vkCmdPipelineBarrier(commandList, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                         nullptr, 1, Barriers, 0, nullptr);

    // Sort Reduce
    {
      vkCmdBindPipeline(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.CountReduce.pipeline);
      vkCmdDispatch(commandList, NumReducedThreadgroupsToRun, 1, 1);
    }
    // UAV barrier on the reduced sum table
    Barriers[0] = BufferTransition(scratch->buffer, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                                   VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, reducedScratchOffset,
                                   reducedScratchBufferSize);
    vkCmdPipelineBarrier(commandList, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                         nullptr, 1, Barriers, 0, nullptr);

    // Sort Scan
    {
      // First do scan prefix of reduced values
      vkCmdBindDescriptorSets(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.layout, 1, 1,
                              &context->sortStages.m_SortDescriptorSetScanSets[0], 0, nullptr);
      vkCmdBindPipeline(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.Scan.pipeline);
      assert(NumReducedThreadgroupsToRun < PARALLELSORT_ELEMENTS_PER_THREAD * PARALLELSORT_THREADGROUP_SIZE &&
             "Need to account for bigger reduced histogram scan");
      vkCmdDispatch(commandList, 1, 1, 1);

      // UAV barrier on the reduced sum table
      Barriers[0] = BufferTransition(scratch->buffer, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                                     VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, reducedScratchOffset,
                                     reducedScratchBufferSize);
      vkCmdPipelineBarrier(commandList, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                           nullptr, 1, Barriers, 0, nullptr);

      // Next do scan prefix on the histogram with partial sums that we just did
      vkCmdBindDescriptorSets(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.layout, 1, 1,
                              &context->sortStages.m_SortDescriptorSetScanSets[1], 0, nullptr);
      vkCmdBindPipeline(commandList, VK_PIPELINE_BIND_POINT_COMPUTE, context->sortStages.ScanAdd.pipeline);
      vkCmdDispatch(commandList, NumReducedThreadgroupsToRun, 1, 1);
    }

    // UAV barrier on the sum table
    Barriers[0] =
        BufferTransition(scratch->buffer, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                         VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, scratchOffset, scratchBufferSize);
    vkCmdPipelineBarrier(commandList, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                         nullptr, 1, Barriers, 0, nullptr);

    // Sort Scatter
    {
      vkCmdBindPipeline(commandList, VK_PIPELINE_BIND_POINT_COMPUTE,
                        bHasPayload ? context->sortStages.ScatterPayload.pipeline
                                    : context->sortStages.Scatter.pipeline);
      vkCmdDispatch(commandList, NumThreadgroupsToRun, 1, 1);
    }

    // Finish doing everything and barrier for the next pass
    VkBuffer keysBuffer = (inputSet) ? scratch->buffer : keys->buffer;
    Barriers[0] = BufferTransition(keysBuffer, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                                   VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, 0, keysSize);
    vkCmdPipelineBarrier(commandList, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                         nullptr, 1, Barriers, 0, nullptr);

    if (bHasPayload) {
      VkBuffer valsBuffer = (inputSet) ? scratch->buffer : values->buffer;
      VkDeviceSize offset = (inputSet) ? valuesOffset : 0;
      Barriers[0] = BufferTransition(valsBuffer, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                                     VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, offset, valuesSize);
      vkCmdPipelineBarrier(commandList, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
                           nullptr, 1, Barriers, 0, nullptr);
    }

    // Swap read/write sources
    inputSet = !inputSet;
  }
  context->endSingleTimeCommands(commandList, context->graphicsCommandPool, context->graphicsQueue);
}

GPRT_API void
gprtBufferSort(GPRTContext _context, GPRTBuffer _buffer, GPRTBuffer _scratch) {
  bufferSort(_context, _buffer, nullptr, _scratch);
}

GPRT_API void
gprtBufferSortPayload(GPRTContext _context, GPRTBuffer _keys, GPRTBuffer _values, GPRTBuffer _scratch) {
  bufferSort(_context, _keys, _values, _scratch);
}

// GPRT_API gprt::Buffer
// gprtBufferGetHandle(GPRTBuffer _buffer, int deviceID) {
//   LOG_API_CALL();
//   Buffer *buffer = (Buffer *) _buffer;
//   return gprt::Buffer{buffer->virtualAddress, 0};
// }

GPRT_API uint64_t
gprtBufferGetDeviceAddress(GPRTBuffer _buffer, int deviceID) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;
  return buffer->getDeviceAddress();
}

GPRT_API void
gprtBufferSaveImage(GPRTBuffer _buffer, uint32_t width, uint32_t height, const char *imageName) {
  LOG_API_CALL();
  Buffer *buffer = (Buffer *) _buffer;

  // Keep track of whether the buffer was mapped before this call
  bool mapped = true;
  if (buffer->mapped == nullptr)
    mapped = false;

  // If not mapped currently, map it
  if (!mapped)
    buffer->map();

  const uint8_t *fb = (const uint8_t *) buffer->mapped;
  std::vector<uint8_t> swizzled(width * height * 4);
  for (uint32_t pid = 0; pid < width * height; ++pid) {
    swizzled[pid * 4 + 0] = fb[pid * 4 + 2];
    swizzled[pid * 4 + 1] = fb[pid * 4 + 1];
    swizzled[pid * 4 + 2] = fb[pid * 4 + 0];
    swizzled[pid * 4 + 3] = fb[pid * 4 + 3];
  }

  stbi_write_png(imageName, width, height, 4, swizzled.data(), (uint32_t) (width) * sizeof(uint32_t));

  // Return mapped to previous state
  if (!mapped)
    buffer->unmap();
}

GPRT_API void
gprtTextureSaveImage(GPRTTexture _texture, const char *imageName) {
  LOG_API_CALL();
  Texture *texture = (Texture *) _texture;

  // Keep track of whether the texture was mapped before this call
  bool mapped = true;
  if (texture->mapped == nullptr)
    mapped = false;

  // If not mapped currently, map it
  if (!mapped)
    texture->map();

  if (texture->format != VK_FORMAT_R8G8B8A8_SRGB && texture->format != VK_FORMAT_R32G32B32A32_SFLOAT) {
    LOG_ERROR("Error, unsupported image format!");
  }

  if (texture->format == VK_FORMAT_R8G8B8A8_SRGB) {
    const uint8_t *fb = (const uint8_t *) texture->mapped;
    stbi_write_png(imageName, texture->width, texture->height, 4, fb, (uint32_t) (texture->width) * sizeof(uint32_t));
  }
  else if (texture->format == VK_FORMAT_R32G32B32A32_SFLOAT) {
    const float *fb = (const float *) texture->mapped;
    for (int i = 0; i < texture->width* texture->height * 4; ++i)
    {
      std::cout<<fb[i]<<std::endl;
    }
    stbi_write_hdr(imageName, texture->width, texture->height, 4, fb);
  }

  // Return mapped to previous state
  if (!mapped)
    texture->unmap();
}

GPRT_API GPRTAccel
gprtAABBAccelCreate(GPRTContext _context, GPRTGeom _geom, unsigned int flags) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Geom* geom = ((Geom*)_geom);
  if (geom->geomType->getKind() != GPRT_AABBS) {
    LOG_ERROR("Given geometry was made from an incompatible geometry type.");
  }
  std::vector<AABBGeom*> geo = {(AABBGeom *) _geom};
  AABBAccel *accel = new AABBAccel(context, geo);
  return (GPRTAccel) accel;
}

GPRT_API GPRTAccel
gprtTriangleAccelCreate(GPRTContext _context, GPRTGeom _geom, unsigned int flags) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Geom* geom = ((Geom*)_geom);
  if (geom->geomType->getKind() != GPRT_TRIANGLES) {
    LOG_ERROR("Given geometry was made from an incompatible geometry type.");
  }

  std::vector<TriangleGeom*> geo = {(TriangleGeom *) _geom};
  TriangleAccel *accel = new TriangleAccel(context, geo);
  return (GPRTAccel) accel;
}

GPRT_API GPRTAccel
gprtSphereAccelCreate(GPRTContext _context, GPRTGeom _geom, unsigned int flags) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Geom* geom = ((Geom*)_geom);
  if (geom->geomType->getKind() != GPRT_SPHERES) {
    LOG_ERROR("Given geometry was made from an incompatible geometry type.");
  }
  std::vector<SphereGeom*> geo = {(SphereGeom *) _geom};
  SphereAccel *accel = new SphereAccel(context, geo);
  return (GPRTAccel) accel;
}

GPRT_API GPRTAccel
gprtLSSAccelCreate(GPRTContext _context, GPRTGeom _geom, GPRTLSSFlags flags) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Geom* geom = ((Geom*)_geom);
  if (geom->geomType->getKind() != GPRT_LSS) {
    LOG_ERROR("Given geometry was made from an incompatible geometry type.");
  }
  std::vector<LSSGeom*> geo = {(LSSGeom *) _geom};
  LSSAccel *accel = new LSSAccel(context, geo, flags);
  return (GPRTAccel) accel;
}

GPRT_API GPRTAccel
gprtSolidAccelCreate(GPRTContext _context, GPRTGeom _geom, unsigned int flags) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  Geom* geom = ((Geom*)_geom);
  if (geom->geomType->getKind() != GPRT_SOLIDS) {
    LOG_ERROR("Given geometry was made from an incompatible geometry type.");
  }
  std::vector<SolidGeom*> geo = {(SolidGeom *) _geom};
  SolidAccel *accel = new SolidAccel(context, geo);
  return (GPRTAccel) accel;
}

GPRT_API GPRTAccel
gprtInstanceAccelCreate(GPRTContext _context, uint32_t numInstances, GPRTBufferOf<gprt::Instance> instancesBuffer) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  InstanceAccel *accel = new InstanceAccel(context, numInstances, (Buffer *) instancesBuffer);
  return (GPRTAccel) accel;
}

GPRT_API void
gprtAccelDestroy(GPRTAccel _accel) {
  LOG_API_CALL();
  Accel *accel = (Accel *) _accel;
  accel->destroy();
  delete accel;
  accel = nullptr;
}

GPRT_API void
gprtAccelBuild(GPRTContext _context, GPRTAccel _accel, GPRTBuildMode mode, bool allowCompaction, bool minimizeMemory) {
  Accel *accel = (Accel *) _accel;
  Context *context = (Context *) _context;
  accel->build(mode, allowCompaction, minimizeMemory);
}

GPRT_API void
gprtAccelUpdate(GPRTContext _context, GPRTAccel _accel) {
  Accel *accel = (Accel *) _accel;
  Context *context = (Context *) _context;
  accel->update();
}

GPRT_API void
gprtAccelCompact(GPRTContext _context, GPRTAccel _accel) {
  Accel *accel = (Accel *) _accel;
  Context *context = (Context *) _context;
  accel->compact();
}

GPRT_API size_t
gprtAccelGetSize(GPRTAccel _accel, int deviceID) {
  Accel *accel = (Accel *) _accel;
  return accel->getSize();
}

GPRT_API const void*
gprtAccelGetDeviceAddress(GPRTAccel _accel, int deviceID) {
  Accel *accel = (Accel *) _accel;
  if (accel->isBottomLevel)
    LOG_ERROR("Handles are only available for instance acceleration structures");
  return (const void*) accel->getDeviceAddress();
}

GPRT_API gprt::Instance
gprtAccelGetInstance(GPRTAccel _accel) {
  Accel *accel = (Accel *) _accel;
  if (!accel->isBottomLevel)
    LOG_ERROR("Instances are only available for bottom level acceleration structures");
  gprt::Instance newInstance = gprt::Instance();
  newInstance.__gprtAccelAddress = accel->getDeviceAddress();
  newInstance.instanceCustomIndex = accel->address; // our own virtual address handle.index;
  newInstance.__gprtSBTOffset = accel->address;
  newInstance.flags = 0;
  newInstance.mask = 0b11111111;
  newInstance.transform =
      float3x4(float4(1.0f, 0.0f, 0.0f, 0.0f), float4(0.0f, 1.0f, 0.0f, 0.0f), float4(0.0f, 0.0f, 1.0f, 0.0f));
  return newInstance;
}

GPRT_API void
gprtBuildShaderBindingTable(GPRTContext _context, GPRTBuildSBTFlags flags) {
  LOG_API_CALL();
  Context *context = (Context *) _context;
  context->buildSBT(flags);
}

GPRT_API void
gprtRayGenLaunch1D(GPRTContext _context, GPRTRayGen _rayGen, uint32_t dims_x, size_t pushConstantsSize,
                   void *pushConstants) {
  LOG_API_CALL();
  gprtRayGenLaunch2D(_context, _rayGen, dims_x, 1, pushConstantsSize, pushConstants);
}

GPRT_API void
gprtRayGenLaunch2D(GPRTContext _context, GPRTRayGen _rayGen, uint32_t dims_x, uint32_t dims_y, size_t pushConstantsSize,
                   void *pushConstants) {
  LOG_API_CALL();
  gprtRayGenLaunch3D(_context, _rayGen, dims_x, dims_y, 1, pushConstantsSize, pushConstants);
}

GPRT_API void
gprtRayGenLaunch3D(GPRTContext _context, GPRTRayGen _rayGen, uint32_t dims_x, uint32_t dims_y, uint32_t dims_z,
                   size_t pushConstantsSize, void *pushConstants) {
  LOG_API_CALL();
  assert(_rayGen);

  Context *context = (Context *) _context;
  RayGen *raygen = (RayGen *) _rayGen;
  VkResult err;

  VkCommandBufferBeginInfo cmdBufInfo{};
  cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);

  std::vector<VkDescriptorSet> descriptorSets = {context->descriptorSet};
  vkCmdBindDescriptorSets(context->graphicsCommandBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
                          context->raytracingPipelineLayout, 0, (uint32_t) descriptorSets.size(), descriptorSets.data(),
                          0, NULL);

  if (context->queryRequested) {
    vkCmdResetQueryPool(context->graphicsCommandBuffer, context->queryPool, 0, 2);
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, context->queryPool, 0);
  }

  vkCmdBindPipeline(context->graphicsCommandBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
                    context->raytracingPipeline);

  if (pushConstantsSize > 0) {
    if (pushConstantsSize > PUSH_CONSTANTS_LIMIT)
      LOG_ERROR("Push constants size exceeds maximum PUSH_CONSTANTS_LIMIT byte limit!");
    vkCmdPushConstants(context->graphicsCommandBuffer, context->raytracingPipelineLayout,
                       VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR |
                           VK_SHADER_STAGE_INTERSECTION_BIT_KHR | VK_SHADER_STAGE_MISS_BIT_KHR |
                           VK_SHADER_STAGE_CALLABLE_BIT_KHR | VK_SHADER_STAGE_RAYGEN_BIT_KHR,
                       0, pushConstantsSize, pushConstants);
  }

  auto getBufferDeviceAddress = [](VkDevice device, VkBuffer buffer) -> uint64_t {
    VkBufferDeviceAddressInfoKHR bufferDeviceAI{};
    bufferDeviceAI.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
    bufferDeviceAI.buffer = buffer;
    return gprt::vkGetBufferDeviceAddress(device, &bufferDeviceAI);
  };

  auto alignedSize = [](uint32_t value, uint32_t alignment) -> uint32_t {
    return (value + alignment - 1) & ~(alignment - 1);
  };

  const uint32_t handleSize = context->rayTracingPipelineProperties.shaderGroupHandleSize;
  const uint32_t maxGroupSize = context->rayTracingPipelineProperties.maxShaderGroupStride;
  const uint32_t groupAlignment = context->rayTracingPipelineProperties.shaderGroupHandleAlignment;
  const uint32_t maxShaderRecordStride = context->rayTracingPipelineProperties.maxShaderGroupStride;

  const uint32_t raygenRecordSize   = alignedSize(std::min(maxGroupSize, requestedFeatures.raygenRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);
  const uint32_t hitRecordSize      = alignedSize(std::min(maxGroupSize, requestedFeatures.hitRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);
  const uint32_t missRecordSize     = alignedSize(std::min(maxGroupSize, requestedFeatures.missRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);
  const uint32_t callableRecordSize = alignedSize(std::min(maxGroupSize, requestedFeatures.callableRecordSize + requestedFeatures.internalAdditionalSize), groupAlignment);


  uint64_t raygenBaseAddr = getBufferDeviceAddress(context->logicalDevice, context->raygenTable->buffer);
  uint64_t missBaseAddr =
      (context->missTable) ? getBufferDeviceAddress(context->logicalDevice, context->missTable->buffer) : 0;
  uint64_t callableBaseAddr =
      (context->callableTable) ? getBufferDeviceAddress(context->logicalDevice, context->callableTable->buffer) : 0;
  uint64_t hitgroupBaseAddr =
      (context->hitgroupTable) ? getBufferDeviceAddress(context->logicalDevice, context->hitgroupTable->buffer) : 0;

  // find raygen in current list of raygens
  int raygenOffset = 0;
  for (int i = 0; i < context->raygens.size(); ++i) {
    if (context->raygens[i] == raygen) {
      raygenOffset = i * raygenRecordSize;
      break;
    }
  }

  VkStridedDeviceAddressRegionKHR raygenShaderSbtEntry{};
  raygenShaderSbtEntry.deviceAddress = raygenBaseAddr + raygenOffset;
  raygenShaderSbtEntry.stride = raygenRecordSize;
  raygenShaderSbtEntry.size = raygenShaderSbtEntry.stride;   // for raygen, can only be one. this needs to
                                                             // be the same as stride.
  // * context->raygenPrograms.size();

  VkStridedDeviceAddressRegionKHR missShaderSbtEntry{};
  if (context->misses.size() > 0) {
    missShaderSbtEntry.deviceAddress = missBaseAddr;
    missShaderSbtEntry.stride = missRecordSize;
    missShaderSbtEntry.size = missShaderSbtEntry.stride * context->misses.size();
  }

  VkStridedDeviceAddressRegionKHR callableShaderSbtEntry{};
  if (context->callables.size() > 0) {
    callableShaderSbtEntry.deviceAddress = callableBaseAddr;
    callableShaderSbtEntry.stride = callableRecordSize;
    callableShaderSbtEntry.size = callableShaderSbtEntry.stride * context->callables.size();
  }

  VkStridedDeviceAddressRegionKHR hitShaderSbtEntry{};
  size_t numHitRecords = context->getNumHitRecords();
  if (numHitRecords > 0) {
    hitShaderSbtEntry.deviceAddress = hitgroupBaseAddr;
    hitShaderSbtEntry.stride = hitRecordSize;
    hitShaderSbtEntry.size = hitShaderSbtEntry.stride * numHitRecords;
  }

  gprt::vkCmdTraceRays(context->graphicsCommandBuffer, &raygenShaderSbtEntry, &missShaderSbtEntry, &hitShaderSbtEntry,
                       &callableShaderSbtEntry, dims_x, dims_y, dims_z);

  if (context->queryRequested)
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, context->queryPool, 1);

  err = vkEndCommandBuffer(context->graphicsCommandBuffer);
  if (err)
    LOG_ERROR("failed to end command buffer! : \n" + errorString(err));

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.pNext = NULL;
  submitInfo.waitSemaphoreCount = 0;
  submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
  submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
  submitInfo.signalSemaphoreCount = 0;
  submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

  err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
  if (err)
    LOG_ERROR("failed to submit to queue! : \n" + errorString(err));

  err = vkQueueWaitIdle(context->graphicsQueue);
  if (err)
    LOG_ERROR("failed to wait for queue idle! : \n" + errorString(err));
}

void
_gprtComputeLaunch(GPRTCompute _compute, uint3 numGroups, uint3 groupSize,
                   std::array<char, PUSH_CONSTANTS_LIMIT> pushConstants) {
  Compute *compute = (Compute *) _compute;
  Context *context = compute->context;
  
  // Build / update the compute pipeline if required
  if (compute->pipeline == VK_NULL_HANDLE) {
    compute->buildPipeline(context->descriptorSetLayout);
  }

  VkResult err;

  VkCommandBufferBeginInfo cmdBufInfo{};
  cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  err = vkBeginCommandBuffer(context->graphicsCommandBuffer, &cmdBufInfo);

  if (context->queryRequested) {
    vkCmdResetQueryPool(context->graphicsCommandBuffer, context->queryPool, 0, 2);
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, context->queryPool, 0);
  }

  vkCmdBindPipeline(context->graphicsCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, compute->pipeline);

  std::vector<VkDescriptorSet> descriptorSets = {context->descriptorSet};

  vkCmdBindDescriptorSets(context->graphicsCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, compute->pipelineLayout, 0,
                          (uint32_t) descriptorSets.size(), descriptorSets.data(), 0, nullptr);

  if (pushConstants.size() > 0) {
    vkCmdPushConstants(context->graphicsCommandBuffer, compute->pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                       PUSH_CONSTANTS_LIMIT, pushConstants.data());
  }

  vkCmdDispatch(context->graphicsCommandBuffer, uint32_t(numGroups[0]), uint32_t(numGroups[1]), uint32_t(numGroups[2]));

  if (context->queryRequested)
    vkCmdWriteTimestamp(context->graphicsCommandBuffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, context->queryPool, 1);

  err = vkEndCommandBuffer(context->graphicsCommandBuffer);
  if (err)
    LOG_ERROR("failed to end command buffer! : \n" + errorString(err));

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.pNext = NULL;
  submitInfo.waitSemaphoreCount = 0;
  submitInfo.pWaitSemaphores = nullptr;     //&acquireImageSemaphoreHandleList[currentFrame];
  submitInfo.pWaitDstStageMask = nullptr;   //&pipelineStageFlags;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &context->graphicsCommandBuffer;
  submitInfo.signalSemaphoreCount = 0;
  submitInfo.pSignalSemaphores = nullptr;   //&writeImageSemaphoreHandleList[currentImageIndex]};

  err = vkQueueSubmit(context->graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
  if (err)
    LOG_ERROR("failed to submit to queue! : \n" + errorString(err));

  err = vkQueueWaitIdle(context->graphicsQueue);
  if (err)
    LOG_ERROR("failed to wait for queue idle! : \n" + errorString(err));
}

GPRT_API void
gprtBeginProfile(GPRTContext _context) {
  LOG_API_CALL();
  assert(_context);
  Context *context = (Context *) _context;
  context->queryRequested = true;
}

GPRT_API float
gprtEndProfile(GPRTContext _context) {
  LOG_API_CALL();
  assert(_context);
  Context *context = (Context *) _context;

  if (context->queryRequested != true)
    LOG_ERROR("Requested profile data without calling gprtBeginProfile");
  context->queryRequested = false;

  uint64_t timestampsResults[2];
  VkResult result =
      vkGetQueryPoolResults(context->logicalDevice, context->queryPool, 0, 2, sizeof(uint64_t) * 2, timestampsResults,
                            sizeof(uint64_t), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
  if (result != VK_SUCCESS)
    LOG_ERROR("Failed to receive query results!");

  VkPhysicalDeviceProperties deviceProperties;
  vkGetPhysicalDeviceProperties(context->physicalDevice, &deviceProperties);
  float nanosecondsInTimestamp = deviceProperties.limits.timestampPeriod;
  float timestampValueInMilliseconds =
      ((timestampsResults[1] - timestampsResults[0]) * nanosecondsInTimestamp) / 1000000.f;

  // I'm not sure why, but the results above seem to change a lot between windows and linux...
  // They seem accurate above for windows systems.
  return timestampValueInMilliseconds;
}
