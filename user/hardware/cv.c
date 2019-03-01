
#include "main.h"
#include "stm32f4xx.h"
#include "cv.h"
void CV_Init(void)
{
        /* -------------- Enable Module Clock Source ----------------------------*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

        GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); //PG9 usart6 rx
                                                                  /* -------------- Configure GPIO ---------------------------------------*/
        {
                GPIO_InitTypeDef GPIO_InitStructure;
                USART_InitTypeDef USART_InitStructure;
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
                GPIO_Init(GPIOG, &GPIO_InitStructure);


                USART_InitStructure.USART_BaudRate = 9600;
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1;
                USART_InitStructure.USART_Parity = USART_Parity_No;
                USART_InitStructure.USART_Mode = USART_Mode_Tx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(USART6, &USART_InitStructure);


                USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

                USART_Cmd(USART6, ENABLE);
        }
				{
                NVIC_InitTypeDef NVIC_InitStructure;
                NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
        }
}
