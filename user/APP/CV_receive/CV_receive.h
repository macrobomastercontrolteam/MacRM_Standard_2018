#include "main.h"

#include "cv.h"
#include "led.h"
#include "stm32f4xx.h"
#include "rng.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"


extern void cv_init(void);
