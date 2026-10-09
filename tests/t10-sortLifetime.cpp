#include <gprt.h>
#include <stdexcept>
#include <vector>

int main() {
  auto context = gprtContextCreate();
  auto scratch = gprtDeviceBufferCreate<uint64_t>(context);
  for (size_t count : {1024, 2048, 512}) {
    std::vector<uint64_t> keys(count), values(count);
    for (size_t i = 0; i < count; ++i) {
      keys[i] = count - i - 1;
      values[i] = keys[i] ^ 0xabcdef;
    }
    auto keyBuffer = gprtDeviceBufferCreate<uint64_t>(context, count, keys.data());
    auto valueBuffer = gprtDeviceBufferCreate<uint64_t>(context, count, values.data());
    for (int i = 0; i < 8; ++i) gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
    gprtComputeSynchronize(context);
    gprtBufferMap(keyBuffer);
    gprtBufferMap(valueBuffer);
    for (size_t i = 0; i < count; ++i)
      if (gprtBufferGetHostPointer(keyBuffer)[i] != i ||
          gprtBufferGetHostPointer(valueBuffer)[i] != (i ^ 0xabcdef))
        throw std::runtime_error("Sort key or payload mismatch");
    gprtBufferUnmap(keyBuffer);
    gprtBufferUnmap(valueBuffer);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
    gprtBufferResize(context, scratch, 1, false);
    gprtBufferSortPayload(context, keyBuffer, valueBuffer, scratch);
    gprtBufferDestroy(valueBuffer);
    gprtBufferDestroy(keyBuffer);
  }
  gprtBufferDestroy(scratch);
  gprtContextDestroy(context);
}
