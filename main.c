//*****************************************************************************
//SIMPLE MODBUS SLAVE EXAMPLE YURI GUSEV V1 example for discovery

//SIMPLE MODBUS SLAVE WITH FREERTOS EXAMPLE ALEXANDER KOMAROV example for discovery

//RS485 - USART3 tx -PB10, rx- PB-11, txen -PB12
//*****************************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"

#include "controller.h"
#include "FreeRTOS.h"
#include "task.h"

#include <misc.h>


/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void SetupUSART1(void);
void SetupTIM7(void);
void SetupGPIO(void);
void task1(void *pvParameters);
void task2(void *pvParameters);
//***************************************************************************
//send data from uart3 if data is ready
//***************************************************************************
void net_tx1(UART_DATA *uart) {
	if ((uart->txlen > 0) & (uart->txcnt == 0)) {
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
		USART_SendData(USART1, uart->buffer[uart->txcnt++]);
	}

}

//***************************************************************************
//  USART3 interrupt

void USART1_IRQHandler(void) {
	//CoEnterISR ( );
	//Receive Data register not empty interrupt
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		uart1.rxtimer = 0;

		if (uart1.rxcnt > (BUF_SZ - 2))
			uart1.rxcnt = 0;

		uart1.buffer[uart1.rxcnt++] = USART_ReceiveData(USART1);

	}

	//Transmission complete interrupt
	if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) {

		USART_ClearITPendingBit(USART1, USART_IT_TC);
		if (uart1.txcnt < uart1.txlen) {
			USART_SendData(USART1, uart1.buffer[uart1.txcnt++]);
		} else {
			uart1.txlen = 0;

			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			USART_ITConfig(USART1, USART_IT_TC, DISABLE);
		}
	}
}


//***************************************************************************
//Timer interrupt
//***************************************************************************
void TIM7_IRQHandler(void) {
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update); //очищаем прерывания
	if ((uart1.rxtimer++ > uart1.delay) & (uart1.rxcnt > 1))
		uart1.rxgap = 1;
		else
			uart1.rxgap = 0;



}

//***************************************************************************
//main()
//***************************************************************************
void task1(void *pvParameters){



	for (;;) {

		if (uart1.rxgap == 1) {
			MODBUS_SLAVE(&uart1);
			net_tx1(&uart1);
		}


	}
}

void task2(void *pvParameters){
	char buf_c[10];
	int c=0;
	for (;;) {
		vTaskDelay(1000);
		c++;
		res_table.regs[0]=c;
	}
}


int main(void) {

		//uarts inint+interrupts
		SetupUSART1(); //RS485


		//tim6 +interrupts
		SetupTIM7();

		//RS485 TXE PIN AND LED PINS
		SetupGPIO();

		SET_PAR[0] = 1; //address modmus

		//timer 0.0001sec one symbol on 9600 ~1ms
		uart1.delay = 30;

		res_table.regs[0] = 31000;
		res_table.regs[1] = 32000;
		res_table.regs[2] = 1;
		//res_table.bytes[0] = 10;
		 xTaskCreate( task1, ( signed char * ) "task1", configMINIMAL_STACK_SIZE, NULL, 2,
			                                ( xTaskHandle * ) NULL);

		 xTaskCreate( task2, ( signed char * ) "task2", configMINIMAL_STACK_SIZE, NULL, 2,
			                                ( xTaskHandle * ) NULL);

		 vTaskStartScheduler();

for(;;){}
}

//*************************************************************************************************
void SetupUSART1(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/*USART1: TX -> PB6, RX -> PB7 */
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	/* Enable GPIOB clock                                                   */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1 Rx as input floating                         */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  //Remap
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //Remap

	/* Configure USART1 Tx as alternate function push-pull            */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //Remap
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //Remap

	//setting parametrs common for all uarts
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);

	//Setting interrupts

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); /* Enable Receive interrupt */

}




//*******************************************************
void SetupTIM7() {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_DeInit(TIM7);

	//0.0001 sec setup APB=36Mhz/(36*100)
	TIM_TimeBaseStructure.TIM_Prescaler = 36;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 100;

	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);

	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}

//***********************************************************************

void SetupGPIO(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PC8,9 leds          */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
//***************************************************************************

