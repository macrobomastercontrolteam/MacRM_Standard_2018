#ifndef FRIC_H
#define FRIC_H
#include "main.h"

#define Fric_UP 1200
#define Fric_DOWN 1200
#define Fric_OFF 1000

extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
