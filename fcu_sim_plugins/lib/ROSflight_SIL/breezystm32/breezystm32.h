#define LED0_ON
#define LED1_ON
#define LED0_OFF
#define LED1_OFF
#define LED1_TOGGLE
#define LED0_TOGGLE

#include <stdint.h>
#include <stdbool.h>

//======================================================================
// PWM
void pwmInit(bool cppm, bool p2, bool p3, int refresh, int idle);

extern pid_con
