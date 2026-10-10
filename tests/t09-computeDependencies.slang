#include "gprt.slang"

[shader("compute")]
[numthreads(1, 1, 1)]
void produce(uniform uint32_t *value) {
  *value = 41;
}

[shader("compute")]
[numthreads(1, 1, 1)]
void consume(uniform uint32_t *value) {
  *value += 1;
}

struct Record { uint32_t *value; };
[shader("raygeneration")]
void raygen(uniform Record record) {
  *record.value += 1;
}
