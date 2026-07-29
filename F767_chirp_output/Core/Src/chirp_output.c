#include "chirp_output.h"
#include <math.h>
#include <stdint.h>

#define PI 3.14159265358979323846f

#define CHIRP_SAMPLE    ((uint32_t)(DAC_FS * CHIRP_DURATION_SEC + 0.5f))

