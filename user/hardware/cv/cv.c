
#include "main.h"
#include "stm32f4xx.h"
#include "cv.h"

void CV_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure1;

		USART_InitTypeDef USART_InitStructure1;

		NVIC_InitTypeDef NVIC_InitStructure1;


 

//GPIOA?USART1????

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //?? GPIOA ??  

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//?? USART1 ??  

  

//USART_DeInit(USART1);  //???? 1

		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //PA9???USART1

		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //PA10???USART1

  

//USART1_TX   PA.9 PA.10 

GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOA9?GPIOA10

GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;//????

GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;  //?? 50MHz

GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP; //??????

GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_NOPULL; //??

GPIO_Init(GPIOG,&GPIO_InitStructure1); //???PA9,PA10

 

//USART ?????

USART_InitStructure1.USART_BaudRate = 9600;//?????9600;

USART_InitStructure1.USART_WordLength = USART_WordLength_8b;//???8?????

USART_InitStructure1.USART_StopBits = USART_StopBits_1;//?????

USART_InitStructure1.USART_Parity = USART_Parity_No;//??????

USART_InitStructure1.USART_HardwareFlowControl =USART_HardwareFlowControl_None;  

USART_InitStructure1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //????

USART_Init(USART6, &USART_InitStructure1); //?????

 



USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//????

//Usart1 NVIC ??

  NVIC_InitStructure1.NVIC_IRQChannel = USART6_IRQn;

NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority=1;//?????2

NVIC_InitStructure1.NVIC_IRQChannelSubPriority =1;    //????? 2

NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;      //IRQ ????

NVIC_Init(&NVIC_InitStructure1);  //??????????VIC ????

   USART_Cmd(USART6, ENABLE); 
}

void CV_restart()
{
        USART_Cmd(USART6, DISABLE);
        

        USART_ClearFlag(USART6, USART_FLAG_RXNE);


        USART_Cmd(USART6, ENABLE);
}
