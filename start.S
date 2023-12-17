# Copyright 2023 Dual Tachyon
# https://github.com/DualTachyon
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

	.cpu cortex-m0
	.fpu softvfp
	.thumb

	.global Stack
	.global HandlerReset
	.global Main

	.global OVERLAY_Install
	.global BOARD_FLASH_Init
	.global BSS_Init

	.global SystickHandler
	.weak SystickHandler
	
	.weak HandlerNMI
	.global HandlerNMI

	.weak HandlerHardFault
	.global HandlerHardFault

	.weak HandlerSVCall
	.global HandlerSVCall

	.weak HandlerPendSV
	.global HandlerPendSV

	.weak SystickHandler
	.global SystickHandler

	.weak HandlerWWDT
	.global HandlerWWDT

	.weak HandlerIWDT
	.global HandlerIWDT
	
	.weak HandlerRTC
	.global HandlerRTC

	.weak HandlerDMA
	.global HandlerDMA

	.weak HandlerSARADC
	.global HandlerSARADC

	.weak HandlerTIMER_BASE0
	.global HandlerTIMER_BASE0

	.weak HandlerTIMER_BASE1
	.global HandlerTIMER_BASE1

	.weak HandlerTIMER_PLUS0
	.global HandlerTIMER_PLUS0

	.weak HandlerTIMER_PLUS1
	.global HandlerTIMER_PLUS1

	.weak HandlerPWM_BASE0
	.global HandlerPWM_BASE0

	.weak HandlerPWM_BASE1
	.global HandlerPWM_BASE1

	.weak HandlerPWM_PLUS0
	.global HandlerPWM_PLUS0

	.weak HandlerPWM_PLUS1
	.global HandlerPWM_PLUS1

	.weak HandlerUART0
	.global HandlerUART0

	.weak HandlerUART1
	.global HandlerUART1

	.weak HandlerUART2
	.global HandlerUART2

	.weak HandlerSPI0
	.global HandlerSPI0

	.weak HandlerSPI1
	.global HandlerSPI1

	.weak HandlerIIC0
	.global HandlerIIC0

	.weak HandlerIIC1
	.global HandlerIIC1

	.weak HandlerCMP
	.global HandlerCMP

	.weak HandlerTIMER_BASE2
	.global HandlerTIMER_BASE2

	.weak HandlerGPIOA5
	.global HandlerGPIOA5

	.weak HandlerGPIOA6
	.global HandlerGPIOA6

	.weak HandlerGPIOA7
	.global HandlerGPIOA7

	.weak HandlerGPIOB0
	.global HandlerGPIOB0

	.weak HandlerGPIOB1
	.global HandlerGPIOB1

	.weak HandlerGPIOC0
	.global HandlerGPIOC0

	.weak HandlerGPIOC1
	.global HandlerGPIOC1

	.weak HandlerGPIOA
	.global HandlerGPIOA

	.weak HandlerGPIOB
	.global HandlerGPIOB

	.weak HandlerGPIOC
	.global HandlerGPIOC

	.weak HandlerReset
	.global HandlerReset


	.section .text.isr

Stack:
	.long	0x20003FF0
Reset:
	.long	HandlerReset + 1
NMI:
	.long	HandlerNMI + 1
HardFault:
	.long	HandlerHardFault + 1
Reserved_10:
	.long	0
Reserved_14:
	.long	0
Reserved_18:
	.long	0
Reserved_1C:
	.long	0
Reserved_20:
	.long	0
Reserved_24:
	.long	0
Reserved_28:
	.long	0
SVCall:
	.long	HandlerSVCall + 1
Reserved_30:
	.long	0
Reserved_34:
	.long	0
PendSV:
	.long	HandlerPendSV + 1
SysTick:
	.long	SystickHandler + 1
WWDT:
	.long	HandlerWWDT + 1
IWDT:
	.long	HandlerIWDT + 1
RTC:
	.long	HandlerRTC + 1
DMA:
	.long	HandlerDMA + 1
SARADC:
	.long	HandlerSARADC + 1
TIMER_BASE0:
	.long	HandlerTIMER_BASE0 + 1
TIMER_BASE1:
	.long	HandlerTIMER_BASE1 + 1
TIMER_PLUS0:
	.long	HandlerTIMER_PLUS0 + 1
TIMER_PLUS1:
	.long	HandlerTIMER_PLUS1 + 1
PWM_BASE0:
	.long	HandlerPWM_BASE0 + 1
PWM_BASE1:
	.long	HandlerPWM_BASE1 + 1
PWM_PLUS0:
	.long	HandlerPWM_PLUS0 + 1
PWM_PLUS1:
	.long	HandlerPWM_PLUS1 + 1
UART0:
	.long	HandlerUART0 + 1
UART1:
	.long	HandlerUART1 + 1
UART2:
	.long	HandlerUART2 + 1
SPI0:
	.long	HandlerSPI0 + 1
SPI1:
	.long	HandlerSPI1 + 1
IIC0:
	.long	HandlerIIC0 + 1
IIC1:
	.long	HandlerIIC1 + 1
CMP:
	.long	HandlerCMP + 1
TIMER_BASE2:
	.long	HandlerTIMER_BASE2 + 1
GPIOA5:
	.long	HandlerGPIOA5 + 1
GPIOA6:
	.long	HandlerGPIOA6 + 1
GPIOA7:
	.long	HandlerGPIOA7 + 1
GPIOB0:
	.long	HandlerGPIOB0 + 1
GPIOB1:
	.long	HandlerGPIOB1 + 1
GPIOC0:
	.long	HandlerGPIOC0 + 1
GPIOC1:
	.long	HandlerGPIOC1 + 1
GPIOA:
	.long	HandlerGPIOA + 1
GPIOB:
	.long	HandlerGPIOB + 1
GPIOC:
	.long	HandlerGPIOC + 1

	.section .text

HandlerNMI:
	b	.

HandlerHardFault:
	b	.

HandlerSVCall:
	b	.

HandlerPendSV:
	b	.

SystickHandler:
	bx	lr

HandlerWWDT:
	b	.

HandlerIWDT:
	b	.

HandlerRTC:
	b	.

HandlerDMA:
	b	.

HandlerSARADC:
	b	.

HandlerTIMER_BASE0:
	b	.

HandlerTIMER_BASE1:
	b	.

HandlerTIMER_PLUS0:
	b	.

HandlerTIMER_PLUS1:
	b	.

HandlerPWM_BASE0:
	b	.

HandlerPWM_BASE1:
	b	.

HandlerPWM_PLUS0:
	b	.

HandlerPWM_PLUS1:
	b	.

HandlerUART0:
	b	.

HandlerUART1:
	b	.

HandlerUART2:
	b	.

HandlerSPI0:
	b	.

HandlerSPI1:
	b	.

HandlerIIC0:
	b	.

HandlerIIC1:
	b	.

HandlerCMP:
	b	.

HandlerTIMER_BASE2:
	b	.

HandlerGPIOA5:
	b	.

HandlerGPIOA6:
	b	.

HandlerGPIOA7:
	b	.

HandlerGPIOB0:
	b	.

HandlerGPIOB1:
	b	.

HandlerGPIOC0:
	b	.

HandlerGPIOC1:
	b	.

HandlerGPIOA:
	b	.

HandlerGPIOB:
	b	.

HandlerGPIOC:
	b	.

HandlerReset:
	ldr	r0, =0x20003FF0
	mov	sp, r0
	bl	DATA_Init
	bl	BSS_Init
#if defined(ENABLE_OVERLAY)
	bl	BOARD_FLASH_Init
#endif
	bl	Main
	b 	.
