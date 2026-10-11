#include <gprt.h>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() try {
  auto context = gprtContextCreate();
  auto scratch = gprtDeviceBufferCreate<uint64_t>(context, 2097152);
  auto scratchAddress = gprtBufferGetDevicePointer(scratch);
  auto scratchSize = gprtBufferGetSize(scratch);
  for (size_t count : {1, 511, 1024, 2049, 524289}) {
    std::vector<uint64_t> keys(count), values(count);
    for (size_t i = 0; i < count; ++i) {
      keys[i] = (uint64_t((i / 3) * 73 % count) << 33) ^ 0xfedcba9876543210ull;
      values[i] = keys[i] ^ 0xabcdef;
    }
    keys[0] = ~uint64_t(0);
    values[0] = keys[0] ^ 0xabcdef;
    auto keyBuffer = gprtDeviceBufferCreate<uint64_t>(context, count, keys.data());
    auto valueBuffer = gprtDeviceBufferCreate<uint64_t>(context, count, values.data());
    auto otherKeys = gprtDeviceBufferCreate<uint64_t>(context, count, keys.data());
    std::sort(keys.begin(), keys.end());
    for (int i = 0; i < (count == 511 ? 1 : count > 2049 ? 2 : 80); ++i) {
      if (count == 511) {
        gprtBufferSortPayload(context, keyBuffer, valueBuffer);
        gprtBufferSort(context, otherKeys);
      } else {
        gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
        gprtBufferSort(context, otherKeys, scratch);
      }
    }
    if (gprtBufferGetSize(scratch) != scratchSize || gprtBufferGetDevicePointer(scratch) != scratchAddress)
      throw std::runtime_error("Sort reallocated sufficient caller-owned scratch");
    gprtComputeSynchronize(context);
    gprtBufferMap(keyBuffer);
    gprtBufferMap(valueBuffer);
    gprtBufferMap(otherKeys);
    for (size_t i = 0; i < count; ++i)
      if (gprtBufferGetHostPointer(keyBuffer)[i] != keys[i] ||
          gprtBufferGetHostPointer(valueBuffer)[i] != (keys[i] ^ 0xabcdef) ||
          gprtBufferGetHostPointer(otherKeys)[i] != keys[i])
        throw std::runtime_error("Sort key or payload mismatch");
    gprtBufferUnmap(keyBuffer);
    gprtBufferUnmap(valueBuffer);
    gprtBufferUnmap(otherKeys);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
    gprtComputeSynchronize(context);
    gprtBufferResize(context, scratch, 1, false);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer);
    gprtBufferSort(context, otherKeys);
    gprtComputeSynchronize(context);
    gprtBufferResize(context, scratch, 2097152, false);
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
