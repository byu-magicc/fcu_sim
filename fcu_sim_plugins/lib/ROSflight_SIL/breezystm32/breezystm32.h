#ifndef BREEZYSTM32_H
#define BREEZYSTM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include <stdint.h>
#include <stdbool.h>
#include <sensors.h>

//======================================================================
// drv_pwm.h
extern uint16_t _rc_signals[8];
void     pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec);
void     pwmWriteMotor(uint8_t index, uint16_t value);
uint16_t pwmRead(uint8_t channel);

//======================================================================
// drv_system.h
#define LED0_ON
#define LED0_OFF
#define LED0_TOGGLE
#define LED1_ON
#define LED1_OFF
#define LED1_TOGGLE

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint64_t micros(void);
uint32_t millis(void);

// Backup SRAM R/W
uint32_t rccReadBkpDr(void);
void rccWriteBkpDr(uint32_t value);

// failure
void failureMode();

// bootloader/IAP
void systemReset(bool toBootloader);

//===================================================================
// flash.h
void initEEPROM();
bool readEEPROM();
bool writeEEPROM();

//===================================================================
// drv_uart.h
#define USART1 0
#define MODE_RXTX 0

uint8_t uartOpen(int a, int b, int c, int d);

extern uint8_t Serial1;
void serialWrite(uint8_t serial, uint8_t ch);
bool serialTotalBytesWaiting(uint8_t serial);
uint8_t serialRead(uint8_t serial);

//===================================================================
// printf.h
//#define sprintf tpf_sprintf
//void tpf_sprintf(char* s, const char *fmt, ...);

//===================================================================
// drv_ms5611.h
bool ms5611_init(void);
void ms5611_update(void);
void ms5611_read(float* altitude, float* pressure, float* temperature);

//===================================================================
// drv_hmc5883l.h
bool hmc5883lInit(int boardVersion);
void hmc5883l_update();
void hmc5883l_read(int16_t *magData);

//===================================================================
// drv_mb1242_h
bool mb1242_init();
void mb1242_update();
float mb1242_read();

//===================================================================
// drv_ms4525.h
bool ms4525_init();
void ms4525_update();
void ms4525_read(float *differential_pressure, float *temp, float* velocity);

//===================================================================
// drv_mpu6050.h
void mpu6050_init(bool enableInterrupt, uint16_t * acc1G, float * gyroScale, int boardVersion);
void mpu6050_register_interrupt_cb(void (*functionPtr)(void));

// Blocking Read Functions
void mpu6050_read_accel(int16_t *accData);
void mpu6050_read_gyro(int16_t *gyroData);
void mpu6050_read_temperature(int16_t * tempData);


#ifdef __cplusplus
}
#endif


#endif
