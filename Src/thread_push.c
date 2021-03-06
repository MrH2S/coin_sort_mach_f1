#include "cmsis_os.h"
#include "config.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

#include "drv_motor.h"
/**
* @brief def thread_push_id
*/
osThreadId thread_push_id;

/**
* @brief def mesq_id
*/
osMessageQId mesq_id;

/**
* @brief def thread_push func
*/
void thread_push_entry(void const * arg){
	while(1){
		osSignalWait(signal_thread_push,osWaitForever); 
		osEvent e = osMessageGet(mesq_id,osWaitForever);
		osDelay(300);
		switch(e.value.v){
			case GPIO_PIN_0:
				//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
				//SW0 closed
			  SW0_Close();
			  PUSH0_Forward();
			  //HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);

			  //wait to push back
			  osSignalWait(signal_push_back,osWaitForever);
			  osDelay(500);
				PUSH0_Back();
				//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
			
			  //wait to sw open
				osSignalWait(signal_sw_open,osWaitForever);
				SW0_Open();
        //HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
				break;

			case GPIO_PIN_1:
				//SW1 closed
			  SW1_Close();
			  PUSH1_Forward();
			  
			  //wait to push back
			  osSignalWait(signal_push_back,osWaitForever);
			  osDelay(500);
				PUSH1_Back();
			
			  //wait to sw open
				osSignalWait(signal_sw_open,osWaitForever);
				SW1_Open();
				break;
			
			case GPIO_PIN_2:
				//SW2 closed
			  SW2_Close();
			  PUSH2_Forward();
			  
			  //wait to push back
			  osSignalWait(signal_push_back,osWaitForever);
			  osDelay(500);
				PUSH2_Back();
			
			  //wait to sw open
				osSignalWait(signal_sw_open,osWaitForever);
				SW2_Open();
				break;
			
		}/*end of switch*/
		
	}
}
