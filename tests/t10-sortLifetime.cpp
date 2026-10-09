#include <gprt.h>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() try {
  auto context = gprtContextCreate();
  auto scratch = gprtDeviceBufferCreate<uint64_t>(context, 131072);
  auto scratchAddress = gprtBufferGetDevicePointer(scratch);
  auto scratchSize = gprtBufferGetSize(scratch);
  for (size_t count : {1024, 2048, 512}) {
    std::vector<uint64_t> keys(count), values(count);
    for (size_t i = 0; i < count; ++i) {
      keys[i] = count - i - 1;
      values[i] = keys[i] ^ 0xabcdef;
    }
    auto keyBuffer = gprtDeviceBufferCreate<uint64_t>(context, count, keys.data());
    auto valueBuffer = gprtDeviceBufferCreate<uint64_t>(context, count, values.data());
    auto otherKeys = gprtDeviceBufferCreate<uint64_t>(context, count, keys.data());
    for (int i = 0; i < 80; ++i) {
      gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
      gprtBufferSort(context, otherKeys, scratch);
    }
    if (gprtBufferGetSize(scratch) != scratchSize || gprtBufferGetDevicePointer(scratch) != scratchAddress)
      throw std::runtime_error("Sort reallocated sufficient caller-owned scratch");
    gprtComputeSynchronize(context);
    gprtBufferMap(keyBuffer);
    gprtBufferMap(valueBuffer);
    gprtBufferMap(otherKeys);
    for (size_t i = 0; i < count; ++i)
      if (gprtBufferGetHostPointer(keyBuffer)[i] != i ||
          gprtBufferGetHostPointer(valueBuffer)[i] != (i ^ 0xabcdef) ||
          gprtBufferGetHostPointer(otherKeys)[i] != i)
        throw std::runtime_error("Sort key or payload mismatch");
    gprtBufferUnmap(keyBuffer);
    gprtBufferUnmap(valueBuffer);
    gprtBufferUnmap(otherKeys);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
    gprtBufferResize(context, scratch, 1, false);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer);
    gprtBufferResize(context, scratch, 131072, false);
    scratchAddress = gprtBufferGetDevicePointer(scratch);
    scratchSize = gprtBufferGetSize(scratch);
    gprtBufferDestroy(valueBuffer);
    gprtBufferDestroy(keyBuffer);
    gprtBufferDestroy(otherKeys);
  }
  gprtBufferDestroy(scratch);
  gprtContextDestroy(context);
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}
