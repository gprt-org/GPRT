#include <gprt.h>
#include <cstdlib>

// Supply GPRT_VISIBLE_DEVICES externally; optional arguments select an API index
// and indicate that rejection is expected.
int main(int argc, char **argv) {
  bool expectFailure = argc > 2;
  for (int attempt = 0; attempt < 3; ++attempt) {
    int32_t index = argc > 1 ? std::atoi(argv[1]) : 0;
    auto context = gprtContextCreate(&index, 1);
    if (context) gprtContextDestroy(context);
    if ((context == nullptr) != expectFailure) return 1;
  }
  return 0;
}
