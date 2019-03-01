/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#define CVTASK_H
#include "CV_receive.h"

//can1中断
void cv_init(void)
{
    CV_Init();
}
void CV_unable(void)
{
        USART_Cmd(USART6, DISABLE);
}
void USART6_IRQHandler(void)
{
	  u8 msg;
	  if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
      {
          USART_ReceiveData(USART6);
      }
    else if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        msg=USART_ReceiveData(USART6);
				if((msg & 192) == 0 && count == 0){
				    if((msg & 32) == 1){
                sign_x = 1;	
						}
            else{
			          sign_x = 0;		}
						cv_x = msg<<3;
						cv_x = cv_x<<3;
						count++;
				}
		    else if ((msg & 192)==64 && count==1){
            cv_x += msg & 63;
					  count++;
				}
				else if ((msg & 192)==128 && count==2){
	        	if ((msg & 32) == 1){
								sign_y = 1;		}
						else{
								sign_y = 0;}
						cv_y = msg<<3;
						cv_y = cv_y<<3;
						count++;
      	}
				else if ((msg & 192)==192 && count==3){
	        	cv_y += msg & 63;
				  	count=0;
        }
			}
}
