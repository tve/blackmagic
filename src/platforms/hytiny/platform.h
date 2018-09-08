/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Patching for Bluepill board by Dave Marples <dave@marples.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"
#include "version.h"

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/usb/usbd.h>

#define BOARD_IDENT       "Black Magic Probe (HyTiny), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU   "Black Magic (Upgrade) for HyTiny, (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_UPD   "Black Magic (DFU Upgrade) for HyTiny, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT         "Black Magic Firmware Upgrade (HyTiny, Firmware " FIRMWARE_VERSION ")"
#define UPD_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Kg"

/* Important pin mappings for STM32 Hytiny implementation:
 *
 *
 * LED0  = PA1
 * TX    = PA2
 * RX    = PA3
 * SWDIO = PA4
 * SWCLK = PA5
 * nSRST = PA6
 * SWO   = PB7  (usart1 pin swap)
 */

#undef PLATFORM_HAS_TRACESW1
#define TRACESWO_ASYNC 1
#define NUM_TRACE_PACKETS		(128)		/* This is an 8K buffer */

#ifdef ENABLE_DEBUG
#define PLATFORM_HAS_DEBUG 1
#define USBUART_DEBUG
extern bool debug_bmp;
int usbuart_debug_write(const char *buf, size_t len);
void dbg_outf(const char *fmt, ...);
# define DEBUG dbg_outf
#else
# define DEBUG(...)
#endif

/* Hardware definitions... */
#define TRST_PORT       GPIOA
#define TRST_PIN        GPIO6

#define TMS_PORT	    GPIOA
#define TMS_PIN		    GPIO4
#define SWDIO_PORT 	    TMS_PORT
#define SWDIO_PIN	    TMS_PIN

#define TCK_PORT	    GPIOA
#define TCK_PIN		    GPIO5
#define SWCLK_PORT 	    TCK_PORT
#define SWCLK_PIN	    TCK_PIN

#define TDO_PORT	    GPIOA
#define TDO_PIN		    GPIO10

#define TDI_PORT	    GPIOA
#define TDI_PIN		    GPIO11

#define SRST_PORT	    GPIOA
#define SRST_PIN	    GPIO6

#define VSENSE_PORT     GPIOA
#define VSENSE_PIN      GPIO9

/* These are fixed by the board design */
#define LED_PORT	    GPIOA
#define LED_IDLE_RUN    GPIO1

/* The 'force DFU' pin. We use BOOT1 cos that's on a jumper and it's ignored if BOOT0=0 */
//#define DFUFORCE_PORT   GPIOA
//#define DFUFORCE_PIN    GPIO0

/* Use PC14 for a "dummy" uart led. So we can observere at least with scope*/
#define LED_PORT_UART	GPIOC
#define LED_UART	    GPIO14

#define TMS_SET_MODE() \
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_10_MHZ, \
	              GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
#if 0
#define SWDIO_MODE_FLOAT() \
	gpio_set_mode(SWDIO_PORT, GPIO_MODE_INPUT, \
	              GPIO_CNF_INPUT_FLOAT, SWDIO_PIN);
#define SWDIO_MODE_DRIVE() \
	gpio_set_mode(SWDIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, \
	              GPIO_CNF_OUTPUT_PUSHPULL, SWDIO_PIN);
#else
# define SWD_CR   GPIO_CRL(SWDIO_PORT)
# define SWD_CR_SHIFT (4*4)
#define SWDIO_MODE_FLOAT() 	do { \
	uint32_t cr = SWD_CR; \
	cr  &= ~(0xf << SWD_CR_SHIFT); \
	cr  |=  (0x4 << SWD_CR_SHIFT); \
	SWD_CR = cr; \
} while(0)
#define SWDIO_MODE_DRIVE() 	do { \
	uint32_t cr = SWD_CR; \
	cr  &= ~(0xf << SWD_CR_SHIFT); \
	cr  |=  (0x1 << SWD_CR_SHIFT); \
	SWD_CR = cr; \
} while(0)
#endif

#define UART_PIN_SETUP() \
       gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_2_MHZ, \
                     GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN);


#define SWO_PIN_SETUP()						     \
        AFIO_MAPR |= AFIO_MAPR_USART1_REMAP;     \
        gpio_set_mode(SWOUSART_PORT, GPIO_MODE_INPUT, \
        GPIO_CNF_INPUT_FLOAT, SWOUSART_RX_PIN);


#define USB_DRIVER              st_usbfs_v1_usb_driver
#define USB_IRQ	                NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR	                usb_lp_can_rx0_isr

/* Interrupt priorities.  Low numbers are high priority.
 */
#define IRQ_PRI_SWO_DMA         (1 << 4)
#define IRQ_PRI_USB		        (2 << 4)
#define IRQ_PRI_USBUSART	    (3 << 4)
#define IRQ_PRI_USBUSART_TIM	(4 << 4)
#define IRQ_PRI_USB_VBUS        (14 << 4)

/* On F103, only USART1 is on AHB2 and can reach 4.5 MBaud at 72 MHz.*/
#define SWO_UART				USART1
#define SWO_UART_DR				USART1_DR
#define SWO_UART_CLK			RCC_USART1
#define SWO_UART_PORT			GPIOB
#define SWO_UART_RX_PIN			GPIO7

/* This DMA channel is set by the USART in use */
#define SWO_DMA_BUS				DMA1
#define SWO_DMA_CLK				RCC_DMA1
#define SWO_DMA_CHAN			DMA_CHANNEL5
#define SWO_DMA_IRQ				NVIC_DMA1_CHANNEL5_IRQ
#define SWO_DMA_ISR(x)			dma1_channel5_isr(x)

#define USBUSART                USART2
#define USBUSART_CR1            USART2_CR1
#define USBUSART_IRQ            NVIC_USART2_IRQ
#define USBUSART_CLK            RCC_USART2
#define USBUSART_PORT           GPIOA
#define USBUSART_TX_PIN         GPIO2
#define USBUSART_ISR            usart2_isr
#define USBUSART_TIM            TIM3
#define USBUSART_TIM_CLK_EN()   rcc_periph_clock_enable(RCC_TIM3)
#define USBUSART_TIM_IRQ        NVIC_TIM3_IRQ
#define USBUSART_TIM_ISR        tim3_isr

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)   {gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(x)

/* Use newlib provided integer only stdio functions */
#define sscanf                  siscanf
#define sprintf                 siprintf
#define vasprintf               vasiprintf
#define snprintf                sniprintf

#endif
