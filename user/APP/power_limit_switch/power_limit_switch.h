#include "main.h"

#define CC1_VALUE  50 // ms, switch to lower power limit
#define CC2_VALUE 150 // ms, switch to higher power limit

void TIM4_Init(uint16_t arr, uint16_t psc);
void start_power_switching_timer(void);
