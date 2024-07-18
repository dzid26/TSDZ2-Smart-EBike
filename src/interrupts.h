/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#define EXTI_HALL_A_IRQ  7              // ITC_IRQ_PORTE - Hall sensor A rise/fall detection
#define EXTI_HALL_B_IRQ  6              // ITC_IRQ_PORTD - Hall sensor B rise/fall detection
#define EXTI_HALL_C_IRQ  5              // ITC_IRQ_PORTC - Hall sensor C rise/fall detection
#define TIM1_CAP_COM_IRQHANDLER 12      // ITC_IRQ_TIM1_CAPCOM - PWM control loop (52us)
#define TIM4_OVF_IRQHANDLER 23          // ITC_IRQ_TIM4_OVF - TIM 4 overflow: 1ms counter
#define UART2_IRQHANDLER 21             // UART


// PWM cycle interrupt (called every 64us)
void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER);
// UART interrupt
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);
// TIM4 Overflow interrupt (called every 1ms)
void TIM4_IRQHandler(void) __interrupt(TIM4_OVF_IRQHANDLER);
// Hall Sensor Signal interrupt
void HALL_SENSOR_A_PORT_IRQHandler(void) __interrupt(EXTI_HALL_A_IRQ);
void HALL_SENSOR_B_PORT_IRQHandler(void) __interrupt(EXTI_HALL_B_IRQ);
void HALL_SENSOR_C_PORT_IRQHandler(void) __interrupt(EXTI_HALL_C_IRQ);


#endif
