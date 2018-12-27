#ifndef _SPI_
#define _SPI_

//#define HARD_SPI//硬件SPI

#ifdef HARD_SPI

#define SPI_MOSI_RCC                    RCC_APB2Periph_GPIOB
#define SPI_MOSI_GPIO                   GPIOB
#define SPI_MOSI_PIN                    GPIO_Pin_15
#define SPI_MOSI_L                    			GPIO_ResetBits(SPI_MOSI_GPIO, SPI_MOSI_PIN); //Data低
#define SPI_MOSI_H                    			GPIO_SetBits(SPI_MOSI_GPIO, SPI_MOSI_PIN); //Data高

#define SPI_SCK_GPIO                   		GPIOB
#define SPI_SCK_PIN                    		GPIO_Pin_13
#define SPI_SCK_L                    			GPIO_ResetBits(SPI_SCK_GPIO, SPI_SCK_PIN); //CLK低
#define SPI_SCK_H                    			GPIO_SetBits(SPI_SCK_GPIO, SPI_SCK_PIN); //CLK高

#define SPI_CS_GPIO                   		GPIOB
#define SPI_CS_PIN                    		GPIO_Pin_12
#define SPI_CS_L                    			GPIO_ResetBits(SPI_CS_GPIO, SPI_CS_PIN); //RCK低
#define SPI_CS_H                    			GPIO_SetBits(SPI_CS_GPIO, SPI_CS_PIN); //RCK高

#else

#define SPI_MOSI_RCC                    RCC_APB2Periph_GPIOE
#define SPI_MOSI_GPIO                   GPIOE
#define SPI_MOSI_PIN                    GPIO_Pin_12

#define SPI_MOSI_L                    			GPIO_ResetBits(SPI_MOSI_GPIO, SPI_MOSI_PIN); //Data低
#define SPI_MOSI_H                    			GPIO_SetBits(SPI_MOSI_GPIO, SPI_MOSI_PIN); //Data高

#define SPI_SCK_GPIO                   		GPIOE
#define SPI_SCK_PIN                    		GPIO_Pin_10
#define SPI_SCK_L                    			GPIO_ResetBits(SPI_SCK_GPIO, SPI_SCK_PIN); //CLK低
#define SPI_SCK_H                    			GPIO_SetBits(SPI_SCK_GPIO, SPI_SCK_PIN); //CLK高

#define SPI_CS_GPIO                   		GPIOE
#define SPI_CS_PIN                    		GPIO_Pin_11
#define SPI_CS_L                    			GPIO_ResetBits(SPI_CS_GPIO, SPI_CS_PIN); //RCK低
#define SPI_CS_H                    			GPIO_SetBits(SPI_CS_GPIO, SPI_CS_PIN); //RCK高

#endif
#define NUM_74HC595                    6

extern void drv_spi_init(void);

extern void Write_74HC595(unsigned char ChipNum,unsigned char *DataBuf);

#endif

