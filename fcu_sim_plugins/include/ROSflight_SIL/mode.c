#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

#include "mode.h"

armed_state_t _armed_state;

void init_mode(void)
{
  _armed_state = ARMED;
}

bool check_mode(uint32_t now) {return true;}

#ifdef __cplusplus
}
#endif
