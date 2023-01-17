#include "sharedCode.h"

GPRT_VERTEX_PROGRAM(simpleVertex, (VertexData, record)) { printf("Hello from vertex program\n"); }

GPRT_PIXEL_PROGRAM(simplePixel, (PixelData, record)) { printf("Hello from pixel program\n"); }
