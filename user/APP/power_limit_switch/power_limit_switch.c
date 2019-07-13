#include "power_limit_switch.h"
#include "stm32f4xx.h"

void TIM4_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    // GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM4);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // TODO possibly needs higher
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    /* TIM4 */
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // TODO figure out a pin for this, 15 not used anywhere
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle; // 1 when high power, 0 when low power
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OCInitStructure.TIM_Pulse = 50;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_Pulse = 950;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1 | TIM_IT_CC2);
    TIM_SetCounter(TIM4, 0);
    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_Cmd(TIM4, ENABLE);
}

void start_power_switching_timer(void)
{
    // in main
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq (&RCC_Clocks);
    uint16_t multiplier;
    if(RCC_Clocks.PCLK1_Frequency == RCC_Clocks.SYSCLK_Frequency)
    {
      multiplier = 1;
    }
    else
    {
      multiplier = 2;
    }
    uint32_t TIM4CLK_Frequency = multiplier * RCC_Clocks.PCLK1_Frequency;
    // uint16_t TIM4CLK_Frequency = RCC_Clocks.PCLK1_Frequency;
    uint16_t TIM4COUNTER_Frequency = 1e3; // 1 kHz
    uint16_t TIM4COUNTER_Period    = 1e3; // 1 second
    TIM4_Init(TIM4COUNTER_Period, TIM4CLK_Frequency / TIM4COUNTER_Frequency); //
    // in main
}
