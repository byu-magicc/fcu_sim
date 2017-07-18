/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University, Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifdef __cplusplus
extern "C" {
#endif

#include <ROSflight_SIL.h>
#include "board.h"

#ifndef M_PI
#define M_PI 3.14159
#endif

// setup
void init_board(void)
{
  return;
}

void board_reset(bool bootloader)
{
  return;
}

// clockmikey_ROSflight_SIL
uint64_t SIL_now_us;
uint64_t clock_micros(void) {return SIL_now_us;}
uint32_t clock_millis(void) {return SIL_now_us/1000;}
void clock_delay(uint32_t ms) {}

// serial
void serial_init(uint32_t baud_rate){}
void serial_write(uint8_t byte){}
uint16_t serial_bytes_available(void){return 0;}
uint8_t serial_read(void){return 0;}

// sensors
void sensors_init(int board_revision){}

// IMU
float accel_read_raw[3];
float gyro_read_raw[3];
float temp_read_raw;
void (*IMU_ISR_CB_ptr)(void) = NULL;
void imu_register_callback(void (*functionPtr)(void))
{
  IMU_ISR_CB_ptr = functionPtr;
}

void SIL_call_IMU_ISR(void)
{
  if (IMU_ISR_CB_ptr != NULL)
  {
      IMU_ISR_CB_ptr();
  }
}
void imu_read_accel(float accel[3])
{
  for(int i = 0; i < 3; i++)
    accel[i] = accel_read_raw[i];
}

void imu_read_gyro(float gyro[3])
{
  for (int i = 0; i < 3; i++)
    gyro[i] = gyro_read_raw[i];
}

float imu_read_temperature(void)
{
  return temp_read_raw;
}

bool mag_check(void)
{
  return false;
}
bool mag_present(void)
{
  return false;
}

void mag_read(float mag[3])
{
  return;
}

bool baro_present(void)
{
  return false;
}

void baro_read(float *altitude, float *pressure, float *temperature)
{
  return;
}

void baro_calibrate()
{
  return;
}

bool diff_pressure_present(void)
{
  return false;
}

bool diff_pressure_check(void)
{
  return false;
}

void diff_pressure_set_atm(float barometric_pressure)
{
  return;
}

void diff_pressure_calibrate()
{
  return;
}

void diff_pressure_read(float *diff_pressure, float *temperature, float *velocity)
{
  return;
}

bool sonar_present(void)
{
  return false;
}

bool sonar_check(void)
{
  return false;
}

float sonar_read(void)
{
  return 0.0;
}

// PWM
uint16_t _rc_signals[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  return;
}

uint16_t pwm_read(uint8_t channel)
{
  return _rc_signals[channel];
}

void pwm_write(uint8_t channel, uint16_t value)
{
  return;
}


// non-volatile memory
void memory_init() {}
bool memory_read(void *dest, size_t len) {return false;}
bool memory_write(const void *src, size_t len) {return false;}


// LEDs
void led0_on(void) {}
void led0_off(void) {}
void led0_toggle(void) {}

void led1_on(void) {}
void led1_off(void) {}
void led1_toggle(void) {}



#ifdef __cplusplus
}
#endif
