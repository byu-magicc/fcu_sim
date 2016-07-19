#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mavlink.h"

#include "param.h"
#include "mixer.h"

//TODO temporary
#include <stdio.h>
#include <string.h>

// global variable definitions
params_t _params;

float get_param_float(param_id_t id)
{
  return *(float *) &_params.values[id];
}



#ifdef __cplusplus
}
#endif
