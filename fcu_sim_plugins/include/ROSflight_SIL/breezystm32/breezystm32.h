#ifndef BREEZYSTM32_H
#define BREEZYSTM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void pwmInit(bool a, bool b, bool c, int d, int e);
void pwmWriteMotor(int32_t index, int32_t value);

#ifdef __cplusplus
}
#endif

#endif
