/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ??can????????,??????can??????
  * @note       ?????freeRTOS??
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ??
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#define CVTASK_H
#include "CV_receive.h"
extern uint16_t cv_x;
extern uint16_t cv_y;	
extern uint8_t sign_x;
extern uint8_t sign_y;
extern int count;
//can1??
void cv_init(void)
{
    CV_Init();
	  cv_x = 0;
	  cv_y = 0;
	  sign_x = 0;
	  sign_y = 0;
	  count = 0;
}
void CV_unable(void)
{
        USART_Cmd(USART6, DISABLE);
}
void slove_CV_lost(void)
{
    CV_restart();
}

void USART6_IRQHandler(void)
{
	  u8 msg;
	  if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        msg=USART_ReceiveData(USART6);
				if((msg&192) == 0 && count == 0){
					
				    if((msg&32) == 32){
							  led_red_on();
                sign_x = 1;	
						}
            else{
						
			          sign_x = 0;		}
						msg = msg<<3;
						cv_x = msg;
						cv_x = cv_x<<3;
				
								
						count++;
				}
		    else if ((msg&192)==64 && count==1){
            cv_x += (msg&63);
					  count++;
					

				}
				else if ((msg&192)==128 && count==2){
	        	if ((msg&32) == 32){
								sign_y = 1;		}
						else{
								sign_y = 0;}
						msg = msg<<3;
						cv_y = msg;
						cv_y = cv_y<<3;
						count++;
      	}
				else if ((msg&192)==192 && count==3){
	        	cv_y +=(msg&63);
				  	count=0;
        }
				
				
				
		}
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
}
