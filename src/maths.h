#pragma once

#include <stdlib.h>


static inline float frand(float range) {
    return ((float)rand() / (float)RAND_MAX) * range;
}


static inline float DotProduct(float ax, float ay, float bx, float by) {
    return ax * bx + ay * by;
}
