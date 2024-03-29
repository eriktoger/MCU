/*
 * stm32f407xx.h
 *
 *  Created on: Jul 18, 2019
 *      Author: erik
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


/**************** START: Processor Specific details ****************************************/

/*
 * ARM Cortex M4 Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0 		((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1 		((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2 		((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3 		((volatile uint32_t*) 0xE000E10C)

/*
 * ARM Cortex M4 Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0 		((volatile uint32_t*) 0XE000E180)
#define NVIC_ICER1 		((volatile uint32_t*) 0XE000E184)
#define NVIC_ICER2 		((volatile uint32_t*) 0XE000E188)
#define NVIC_ICER3 		((volatile uint32_t*) 0XE000E18C)

/*
 * ARM Cortex M4 Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR ((volatile uint32_t *)0xE000E400)

/*
 * ARM Cortex M4 Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED 	4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM						0x1FFF0000U
#define SRAM SRAM1_BASEADDR		SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)


#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

/**************************** peripheral register definition structures *******************/

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;


typedef struct
{
  volatile uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  volatile uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  volatile uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  volatile uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  volatile uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  volatile uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  volatile uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  volatile uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  volatile uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  volatile uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  volatile uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  volatile uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  volatile uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  volatile uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  volatile uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  volatile uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  volatile uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  volatile uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  volatile uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  volatile uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  volatile uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  volatile uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

/*
 * peripheral register definition structure for
 */
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t      RESERVED1[2]; //these are hidden, offset jumps from 0x14 to 0x20 ( 14,18,1C,20)
	volatile uint32_t CMPCR;
	// they have two extras reserved2 and CFGR, not sure why

}SYSCFG_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t *) GPIOA_BASEADDR )
#define GPIOB			((GPIO_RegDef_t *) GPIOB_BASEADDR )
#define GPIOC			((GPIO_RegDef_t *) GPIOC_BASEADDR )
#define GPIOD			((GPIO_RegDef_t *) GPIOD_BASEADDR )
#define GPIOE			((GPIO_RegDef_t *) GPIOE_BASEADDR )
#define GPIOF			((GPIO_RegDef_t *) GPIOF_BASEADDR )
#define GPIOG			((GPIO_RegDef_t *) GPIOG_BASEADDR )
#define GPIOH			((GPIO_RegDef_t *) GPIOH_BASEADDR )
#define GPIOI			((GPIO_RegDef_t *) GPIOI_BASEADDR )

#define RCC  			((RCC_RegDef_t *) RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t *) EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)
/*
 *  Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() RCC->AHB1ENR |= ( 1<<0)
#define GPIOB_PCLK_EN() RCC->AHB1ENR |= ( 1<<1)
#define GPIOC_PCLK_EN() RCC->AHB1ENR |= ( 1<<2)
#define GPIOD_PCLK_EN() RCC->AHB1ENR |= ( 1<<3)
#define GPIOE_PCLK_EN() RCC->AHB1ENR |= ( 1<<4)
#define GPIOF_PCLK_EN() RCC->AHB1ENR |= ( 1<<5)
#define GPIOG_PCLK_EN() RCC->AHB1ENR |= ( 1<<6)
#define GPIOH_PCLK_EN() RCC->AHB1ENR |= ( 1<<7)
#define GPIOI_PCLK_EN() RCC->AHB1ENR |= ( 1<<8) // dont exist?

/*
 *  Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21 ) )
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22 ) )
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23 ) )

/*
 *  Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12 ) )
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14 ) )
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15 ) )
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13 ) )


/*
 *  Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCCK_EN() 	(RCC->APB2ENR |= (1<<4))
#define USART2_PCCK_EN() 	(RCC->APB1ENR |= (1<<17))
#define USART3_PCCK_EN() 	(RCC->APB1ENR |= (1<<18))
#define UART4_PCCK_EN() 	(RCC->APB1ENR |= (1<<19))
#define UART5_PCCK_EN() 	(RCC->APB1ENR |= (1<<20))
#define USART6_PCCK_EN() 	(RCC->APB1ENR |= (1<<5))
/*
 *  Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 <<14) )

/*
 *  Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() RCC->AHB1ENR &= ( 1<<0)
#define GPIOB_PCLK_DI() RCC->AHB1ENR &= ( 1<<1)
#define GPIOC_PCLK_DI() RCC->AHB1ENR &= ( 1<<2)
#define GPIOD_PCLK_DI() RCC->AHB1ENR &= ( 1<<3)
#define GPIOE_PCLK_DI() RCC->AHB1ENR &= ( 1<<4)
#define GPIOF_PCLK_DI() RCC->AHB1ENR &= ( 1<<5)
#define GPIOG_PCLK_DI() RCC->AHB1ENR &= ( 1<<6)
#define GPIOH_PCLK_DI() RCC->AHB1ENR &= ( 1<<7)
#define GPIOI_PCLK_DI() RCC->AHB1ENR &= ( 1<<8) // dont exist?

/*
 *  Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= (1 << 12 ) )
#define SPI2_PCLK_DI() (RCC->APB1ENR &= (1 << 14 ) )
#define SPI3_PCLK_DI() (RCC->APB1ENR &= (1 << 15 ) )
#define SPI4_PCLK_DI() (RCC->APB2ENR &= (1 << 13 ) )

/*
 *  Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCCK_DI() 	(RCC->APB2ENR &= (1<<4))
#define USART2_PCCK_DI() 	(RCC->APB1ENR &= (1<<17))
#define USART3_PCCK_DI() 	(RCC->APB1ENR &= (1<<18))
#define UART4_PCCK_DI() 	(RCC->APB1ENR &= (1<<19))
#define UART5_PCCK_DI() 	(RCC->APB1ENR &= (1<<20))
#define USART6_PCCK_DI() 	(RCC->APB1ENR &= (1<<5))

/*
 *  Clock Disable Macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= (1 <<14))
/*
 *  Macros to reset GPIOx peripherals
 */
// the |= sets it to resert and then we set it to non reset.
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0 )

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 * found in Table 61. Vector table for STM32F405xx/07xx and STM32F415xx/17xx
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

/*
 * macros for all the possibles prority levels
 */

#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI1 	1
#define NVIC_IRQ_PRI2	2
#define NVIC_IRQ_PRI3 	3
#define NVIC_IRQ_PRI4	4
#define NVIC_IRQ_PRI5 	5
#define NVIC_IRQ_PRI6	6
#define NVIC_IRQ_PRI7	7
#define NVIC_IRQ_PRI8	8
#define NVIC_IRQ_PRI9	9
#define NVIC_IRQ_PRI10	10
#define NVIC_IRQ_PRI11	11
#define NVIC_IRQ_PRI12	12
#define NVIC_IRQ_PRI13	13
#define NVIC_IRQ_PRI14	14
#define NVIC_IRQ_PRI15	15


//some generic macros

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
