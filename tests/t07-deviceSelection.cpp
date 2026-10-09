#include <gprt.h>
#include <iostream>
#include <stdexcept>

// Supply GPRT_VISIBLE_DEVICES externally; optional arguments select an API index
// and indicate that rejection is expected.
int main(int argc, char **argv) {
  bool expectFailure = argc > 2;
  try {
    int32_t index = argc > 1 ? std::stoi(argv[1]) : 0;
    auto context = gprtContextCreate(&index, 1);
    gprtContextDestroy(context);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return expectFailure ? 0 : 1;
  }
  return expectFailure ? 1 : 0;
}
