#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


extern uint16_t _rc_signals[8];
extern uint64_t SIL_now_us;

extern float accel_read_raw[3];
extern float gyro_read_raw[3];
extern float temp_read_raw;

void SIL_call_IMU_ISR(void);

