/*
 * stm32f407xx.h
 *
 *  Created on: Jul 18, 2019
 *      Author: erik
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
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


#define EXT1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
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


typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESVERD1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED3;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;

}RCC_RegDeg_t;

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

#define RCC  			((RCC_RegDeg_t *) RCC_BASEADDR)

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

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 <<14))

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
//some generic macros

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
