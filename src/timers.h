/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include <stdint.h>

#include "stm8s.h"

void timers_init(void);

extern volatile uint8_t ui8_1ms_counter;

// ----------------------------------------------------------------------------
// CPU and timer clock frequencies
//
// F_CPU is the base for all timer clock derivations below.
// Each TIMx_PRESCALER_SEL is the chosen prescaler for that timer — change it
// here and both the frequency macro and the init call in timers.c update.
// ----------------------------------------------------------------------------

#define F_CPU               (HSI_VALUE / 1U)   // 16MHz, CPU clock

// TIM2: torque sensor pulse — timer clock = 8MHz
#define TIM2_PRESCALER_SEL  TIM2_PRESCALER_2
#define TIM2_CLK_HZ         (F_CPU / (1UL << ((uint8_t)TIM2_PRESCALER_SEL)))

// TIM3: Hall sensor timebase + TIME_DEBUG — timer clock = 250kHz, 4µs/tick
#define TIM3_PRESCALER_SEL  TIM3_PRESCALER_64
#define TIM3_FREQ_HZ        (F_CPU / (1UL << ((uint8_t)TIM3_PRESCALER_SEL)))
#define TIM3_TICK_US        ((uint8_t)(1000000UL / TIM3_FREQ_HZ))

// TIM4: 1ms counter — timer clock = 125kHz
#define TIM4_PRESCALER_SEL  TIM4_PRESCALER_128
#define TIM4_CLK_HZ         (F_CPU / (1UL << ((uint8_t)TIM4_PRESCALER_SEL)))

#endif /* TIMERS_H_ */
