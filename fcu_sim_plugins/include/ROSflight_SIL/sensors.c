#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "param.h"
#include "sensors.h"

// global variable definitions
bool _imu_ready;
vector_t _accel;
vector_t _gyro;
uint32_t _imu_time;

static bool update_imu(void)
{
    _imu_time = 0;
    // correct according to known biases and temperature compensation
    _accel.x = 0;
    _accel.y = 0;
    _accel.z = 0;
    _gyro.x = 0;
    _gyro.y = 0;
    _gyro.z = 0;

    return true;
}

#ifdef __cplusplus
}
#endif
