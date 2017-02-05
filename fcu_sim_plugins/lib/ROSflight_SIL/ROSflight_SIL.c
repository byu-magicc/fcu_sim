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

#include <breezystm32/breezystm32.h>

//======================================================================
// drv_pwm.h
void     pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec) {}
void     pwmWriteMotor(uint8_t index, uint16_t value) {}
uint16_t pwmRead(uint8_t channel) {}

//======================================================================
// drv_system.h
void systemInit(void) {}
void delayMicroseconds(uint32_t us) {}
void delay(uint32_t ms) {}

uint64_t micros(void) {return _imu_time;}
uint32_t millis(void) {return _imu_time/1000;}

// failure
void failureMode() {}

// bootloader/IAP
void systemReset(bool toBootloader) {}

//===================================================================
// flash.h
void initEEPROM() {}
bool readEEPROM() {return true;}
bool writeEEPROM() {return true;}

//===================================================================
// drv_uart.h
uint8_t Serial1 = 0;
void serialWrite(uint8_t serial, uint8_t ch) {}
bool serialTotalBytesWaiting(uint8_t serial) {return false;}
uint8_t serialRead(uint8_t serial) { return 0;}

//===================================================================
// printf.h
void tpf_sprintf(char* s, const char *fmt, ...) {}

#ifdef __cplusplus
}
#endif
