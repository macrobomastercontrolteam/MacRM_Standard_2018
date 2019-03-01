#include "main.h"


#include "cv.h"

#include "stm32f4xx.h"
#include "rng.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
static u16 cv_x=0;
static u16 cv_y=0;	
static bool_t sign_x=0;
static bool_t sign_y=0;
static int count=0;
