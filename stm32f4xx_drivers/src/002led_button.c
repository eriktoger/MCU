/*
 * 002led_button.c
 *
 *  Created on: Jul 19, 2019
 *      Author: erik
 */

#include "stm32f407xx.h"
#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void){
	for( uint32_t i =0; i<500000; i++);
}

int main(void){
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


		GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIOBtn);


	while(1){

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED ){
			delay();
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}

	}
}
