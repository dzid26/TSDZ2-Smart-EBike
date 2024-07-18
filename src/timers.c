/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "timers.h"
#include "interrupts.h"

volatile uint8_t ui8_1ms_counter = 0;

#ifdef __CDT_PARSER__
#define __interrupt(x)
#endif

void timer2_init(void);
void timer3_init(void);
void timer4_init(void);

void timers_init(void) {
    timer2_init();
    timer3_init();
    timer4_init();
}

// 16bit Timer2 is used to create the pulse signal for excitation of the torque sensor circuit
// Pulse signal: period of 20us, Ton = 2us, Toff = 18us
void timer2_init(void) {
    uint16_t ui16_i;

    // target: 20µs period (50kHz), counter = TIM2_CLK_HZ × 20µs - 1 = 159
    TIM2_TimeBaseInit(TIM2_PRESCALER_SEL, 159);

    // pulse of 2us
    TIM2_OC2Init(TIM2_OCMODE_PWM1,
            TIM2_OUTPUTSTATE_ENABLE,
            16,
            TIM2_OCPOLARITY_HIGH);
    TIM2_OC2PreloadConfig(ENABLE);

    TIM2_ARRPreloadConfig(ENABLE);

    TIM2_Cmd(ENABLE);

    // IMPORTANT: this software delay is needed so timer2 work after this
    for (ui16_i = 0; ui16_i < (65000); ui16_i++) {
        ;
    }
}

// HALL sensor 16bit time counter (250 KHz, 4us period, 1deg resolution at max rotor speed of 660ERPS)
// Counter is used to measure the time between Hall sensors transitions.
// Hall sensor GPIO IRQ is used to read counter reference value at every Hall sensor transition
void timer3_init(void) {
    uint16_t ui16_i;

    // TIM3 Peripheral Configuration
    TIM3_DeInit();
    // 16-bit counter at 4µs/tick → max period = 65536 × 4µs ≈ 262ms
    TIM3_TimeBaseInit(TIM3_PRESCALER_SEL, 0xffff);
    TIM3_Cmd(ENABLE); // TIM3 counter enable

    // IMPORTANT: this software delay is needed so timer3 work after this
    for (ui16_i = 0; ui16_i < (65000); ui16_i++) {
        ;
    }
}

// 8bit TIM4 configuration used to generate a 1ms counter (Counter overflow every 1ms)
void timer4_init(void) {
    uint16_t ui16_i;

    TIM4_DeInit();
    // target: 1ms period (1kHz), counter = TIM4_CLK_HZ × 1ms - 1 = 124
    TIM4_TimeBaseInit(TIM4_PRESCALER_SEL, 125U-1U);
    ITC_SetSoftwarePriority(TIM4_OVF_IRQHANDLER, ITC_PRIORITYLEVEL_1); // 1 = lowest priority
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE); // Enable Update/Overflow Interrupt (see below TIM4_IRQHandler function)

    // IMPORTANT: this software delay is needed so timer3 work after this
    for (ui16_i = 0; ui16_i < (65000); ui16_i++) {
        ;
    }
    TIM4_Cmd(ENABLE); // TIM4 counter enable
}

// TIM4 Overflow Interrupt handler
INTERRUPT_HANDLER(TIM4_IRQHandler, TIM4_OVF_IRQHANDLER) {
    ui8_1ms_counter++;
    TIM4->SR1 = 0; // Reset interrupt flag
}

