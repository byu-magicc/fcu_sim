/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "flash.h"
#include "SIL_board.h"

namespace rosflight {

void SIL_Board::init_board(void)
{

}

void SIL_Board::board_reset(bool bootloader)
{

}

// clock

uint32_t SIL_Board::clock_millis()
{
}

uint64_t SIL_Board::clock_micros()
{
}

void SIL_Board::clock_delay(uint32_t milliseconds)
{
}

// serial

void SIL_Board::serial_init(uint32_t baud_rate)
{

}

void SIL_Board::serial_write(uint8_t byte)
{
}

uint16_t SIL_Board::serial_bytes_available(void)
{
}

uint8_t SIL_Board::serial_read(void)
{
}

// sensors

void SIL_Board::sensors_init()
{
}

uint16_t SIL_Board::num_sensor_errors(void)
{
}

void SIL_Board::imu_register_callback(void (*callback)(void))
{
}

void SIL_Board::imu_read_accel(float accel[3])
{
}

void SIL_Board::imu_read_gyro(float gyro[3])
{
}

bool SIL_Board::imu_read_all(float accel[3], float* temperature, float gyro[3])
{
}

float SIL_Board::imu_read_temperature(void)
{
}

void SIL_Board::imu_not_responding_error(void)
{
}

bool SIL_Board::mag_present(void)
{
}

void SIL_Board::mag_read(float mag[3])
{
}

bool SIL_Board::mag_check(void)
{
}

bool SIL_Board::baro_present(void)
{
}

void SIL_Board::baro_read(float *altitude, float *pressure, float *temperature)
{
}

void SIL_Board::baro_calibrate()
{
}

bool SIL_Board::diff_pressure_present(void)
{
}

bool SIL_Board::diff_pressure_check(void)
{
}

void SIL_Board::diff_pressure_calibrate()
{
}

void SIL_Board::diff_pressure_set_atm(float barometric_pressure)
{
}

void SIL_Board::diff_pressure_read(float *diff_pressure, float *temperature, float *velocity)
{
}

bool SIL_Board::sonar_present(void)
{
}

bool SIL_Board::sonar_check(void)
{
}

float SIL_Board::sonar_read(void)
{
}

uint16_t num_sensor_errors(void)
{
}

// PWM

void SIL_Board::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
}

uint16_t SIL_Board::pwm_read(uint8_t channel)
{
}

void SIL_Board::pwm_write(uint8_t channel, uint16_t value)
{
}

bool SIL_Board::pwm_lost()
{
}

// non-volatile memory

void SIL_Board::memory_init(void)
{
}

bool SIL_Board::memory_read(void * dest, size_t len)
{
}

bool SIL_Board::memory_write(const void * src, size_t len)
{
}

// LED

void SIL_Board::led0_on(void) { }
void SIL_Board::led0_off(void) { }
void SIL_Board::led0_toggle(void) { }

void SIL_Board::led1_on(void) { }
void SIL_Board::led1_off(void) { }
void SIL_Board::led1_toggle(void) { }

}
