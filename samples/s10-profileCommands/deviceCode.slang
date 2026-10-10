#include "gprt.slang"
// The raygen consumes the compute result through an explicit timeline dependency.

[shader("compute")]
[numthreads(1, 1, 1)]
void compute(uniform uint32_t *value) { *value = 41; }

struct Record { uint32_t *input; uint32_t *value; };
[shader("raygeneration")]
void raygen(uniform Record record) { *record.value = *record.input + 1; }
