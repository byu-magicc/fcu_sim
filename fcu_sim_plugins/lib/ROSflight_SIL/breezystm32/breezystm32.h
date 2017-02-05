#ifndef BREEZYSTM32_H
#define BREEZYSTM32_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>
#include <sensors.h>

//======================================================================
// drv_pwm.h
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
extern uint8_t Serial1;
void serialWrite(uint8_t serial, uint8_t ch);
bool serialTotalBytesWaiting(uint8_t serial);
uint8_t serialRead(uint8_t serial);

//===================================================================
// printf.h
#define sprintf tpf_sprintf
void tpf_sprintf(char* s, const char *fmt, ...);

//===================================================================
// drv_ms5611.h
bool ms5611_init();

//===================================================================
// drv_hmc5883l.h
bool hmc5883lInit(uint8_t board_rev);

//===================================================================
// drv_mb1242_h
bool mb1242_init();

//===================================================================
// drv_ms4525.h
bool ms4525_init();

//===================================================================
// drv_mpu6050.h
void mpu6050_init(bool enableInterrupt, uint16_t * acc1G, float * gyroScale, int boardVersion);
void mpu6050_register_interrupt_cb(void (*functionPtr)(void));


#ifdef __cplusplus
}
#endif


#endif
