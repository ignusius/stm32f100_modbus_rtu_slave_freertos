//*****************************************************************************
//SIMPLE MODBUS SLAVE EXAMPLE YURI GUSEV V1 example for discovery
//RS485 - USART3 tx -PB10, rx- PB-11, txen -PB12
//*****************************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#include "controller.h"

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void SetupClock(void);
void SetupUSART1(void);
void SetupTIM7(void);
void SetupGPIO(void);

//***************************************************************************
//send data from uart3 if data is ready
//***************************************************************************
void net_tx1(UART_DATA *uart) {
	if ((uart->txlen > 0) & (uart->txcnt == 0)) {
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //выкл прерывание на прием
		USART_ITConfig(USART1, USART_IT_TC, ENABLE); //включаем на окочание передачи

		//rs485 txenable
		GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);
		//моргаем светодиодом и включаем rs485 на передачу
//		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);

		USART_SendData(USART1, uart->buffer[uart->txcnt++]);
	}

}

//***************************************************************************
// *  USART3 interrupt
// ќбработчик прерываний usart нужен дл€ двух вещей Ч прин€ти€ и отправки символов
// с включением и выключением соотвествующих прерываний
// **************************************************************************
void USART1_IRQHandler(void) {
	//Receive Data register not empty interrupt
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE); //очистка признака прерывани€
		uart1.rxtimer = 0;

		if (uart1.rxcnt > (BUF_SZ - 2))
			uart1.rxcnt = 0;

		uart1.buffer[uart1.rxcnt++] = USART_ReceiveData(USART1);

	}

	//Transmission complete interrupt
	if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) {

		USART_ClearITPendingBit(USART1, USART_IT_TC); //очистка признака прерывани€
		if (uart1.txcnt < uart1.txlen) {
			USART_SendData(USART1, uart1.buffer[uart1.txcnt++]);
		} else {
			//посылка закончилась и мы снимаем высокий уровень сRS485 TXE
			uart1.txlen = 0;
			//rs485 tx disable
			GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
			//led PC8
//			GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);

			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			USART_ITConfig(USART1, USART_IT_TC, DISABLE);
		}
	}
}


//***************************************************************************
//Timer interrupt
//***************************************************************************
void TIM7_IRQHandler(void) {
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update); //очищаем прерывани€

	//моргаем светодиодом дабы показать активность таймера
//	if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_8))
//		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
//	else
//		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);

	//если наш таймер больше уставки задержки и есть символы то есть gap -перерыв в посылке
	//и можно ее обрабатывать
	if ((uart1.rxtimer++ > uart1.delay) & (uart1.rxcnt > 1))
		uart1.rxgap = 1;
		else
			uart1.rxgap = 0;

}

void Delay(volatile uint32_t nCount) {
	for (; nCount != 0; nCount--);
}

//***************************************************************************
//main()
//***************************************************************************
int main(void) {
	SetupClock();

	//uarts inint+interrupts
	SetupUSART1(); //RS485

	//tim6 +interrupts
	SetupTIM7();

	//RS485 TXE PIN AND LED PINS
	SetupGPIO();

	SET_PAR[0] = 1; //адрес устройства

	//timer 0.0001sec one symbol on 9600 ~1ms
	uart1.delay = 30; //таймаут приема

	res_table.regs[0] = 0;
	res_table.bytes[0] = 50;

	//Main loop
	while (1) {

		if (uart1.rxgap == 1) { //ждем gap - т.е. промежуток
			MODBUS_SLAVE(&uart1); //провер€ем и если пакет нам по адресу и не битый то формируем ответ
			net_tx1(&uart1); //если есть признак готовности посылки инициализируем передачу
		}

	}
}

/***************************************************************************//**
 * @brief Setup clocks
 ******************************************************************************/
void SetupClock() {
	RCC_DeInit(); /* RCC system reset(for debug purpose)*/
	RCC_HSEConfig(RCC_HSE_ON); /* Enable HSE                         */

	/* Wait till HSE is ready                                               */
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
		;

	RCC_HCLKConfig(RCC_SYSCLK_Div1); /* HCLK   = SYSCLK                */
	RCC_PCLK2Config(RCC_HCLK_Div1); /* PCLK2  = HCLK                  */
	RCC_PCLK1Config(RCC_HCLK_Div2); /* PCLK1  = HCLK/2                */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4); /* ADCCLK = PCLK2/4               */

	/* PLLCLK = 8MHz * 3 = 24 MHz                                           */
	RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_4);

	RCC_PLLCmd(ENABLE); /* Enable PLL                     */

	/* Wait till PLL is ready                                               */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		;

	/* Select PLL as system clock source                                    */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source                         */
	while (RCC_GetSYSCLKSource() != 0x08)
		;

	/* Enable USART1 and GPIOA clock                                        */
	//   RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
}
//*************************************************************************************************
void SetupUSART1(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* —начала нужно затактировать модуль AFIO,
	 * управл€ющий альтернативными функци€ми.
	 */
	//todo
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* ѕереназначаем USART1: TX -> PB6, RX -> PB7 */
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
	TIM_TimeBaseStructure.TIM_Prescaler = 36; //на эту величину поделим частоту шины
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //частота без делени€ 36ћгц
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //считаем вверх
	TIM_TimeBaseStructure.TIM_Period = 100; //до этого значени€ будет считать таймер

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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure PB2 as rs485 tx select           */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PC8,9 leds          */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
//***************************************************************************

