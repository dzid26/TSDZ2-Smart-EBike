/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2021.
 *
 * Released under the GPL License, Version 3
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "config.h"
#include "common.h"

//#define TIME_DEBUG
//#define HALL_DEBUG

extern volatile uint8_t u8_isr_load_perc;

//#define FW_VERSION 15 // mspider65

/*---------------------------------------------------------
 NOTE: regarding motor rotor offset

 The motor rotor offset should be as close to 0 as
 possible. You can try to tune with the wheel in the air,
 full throttle and look at the batttery current. Adjust
 for the lowest battery current possible.
 ---------------------------------------------------------*/
#define MOTOR_ROTOR_OFFSET_ANGLE  (uint8_t)2
#define PHASE_ROTOR_ANGLE_30  (uint8_t)((uint8_t)21  + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_90  (uint8_t)((uint8_t)64  + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_150 (uint8_t)((uint8_t)107 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_210 (uint8_t)((uint8_t)149 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_270 (uint8_t)((uint8_t)192 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_330 (uint8_t)((uint8_t)235 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)

#define HALL_COUNTER_FREQ                                       250000U // 250KHz or 4us

// ----------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------
#define F_CPU                                           (HSI_VALUE / 1U)  //cpu frequency, based on HSI clock and 1x prescaler

//#define PWM_FREQ										18 // 18 Khz
#define PWM_FREQ										19 // 19 Khz

#if PWM_FREQ == 19
// PWM related values
#define PWM_COUNTER_MAX										420 // 16MHz / 840 = 19,047 KHz                                        107
#define MIDDLE_SVM_TABLE									107 // svm table 19 Khz
#define MIDDLE_PWM_COUNTER									107
// wheel speed parameters
#define OEM_WHEEL_SPEED_DIVISOR								384 // at 19 KHz
#else
// PWM related values
#define PWM_COUNTER_MAX										444 // 16MHz / 888 = 18,018 KHz                                    110
#define MIDDLE_SVM_TABLE									110 // svm table 18 Khz
#define MIDDLE_PWM_COUNTER									110
// wheel speed parameters
#define OEM_WHEEL_SPEED_DIVISOR								363 // at 18 KHz
#endif

#define MOTOR_TASK_FREQ									((uint16_t)(F_CPU / (PWM_COUNTER_MAX*2))) // 55.5us (PWM period) 18 Khz

/*---------------------------------------------------------
 NOTE: regarding duty cycle (PWM) ramping

 Do not change these values if not sure of the effects!

 A lower value of the duty cycle inverse step will mean
 a faster acceleration. Be careful not to choose too
 low values for acceleration.
 ---------------------------------------------------------*/

// ramp up/down PWM cycles count
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT			(uint8_t)(MOTOR_TASK_FREQ/98)
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN				(uint8_t)(MOTOR_TASK_FREQ/781)
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT		(uint8_t)(MOTOR_TASK_FREQ/260)
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN			(uint8_t)(MOTOR_TASK_FREQ/1953)
#define CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP				(uint8_t)(MOTOR_TASK_FREQ/78)
#define WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP			(uint8_t)(MOTOR_TASK_FREQ/78)
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT	(uint8_t)(MOTOR_TASK_FREQ/78)
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN		(uint8_t)(MOTOR_TASK_FREQ/390)

#define MOTOR_OVER_SPEED_ERPS								((MOTOR_TASK_FREQ/29) < 650 ?  (MOTOR_TASK_FREQ/29) : 650) // motor max speed | 29 points for the sinewave at max speed (less than MOTOR_TASK_FREQ/29)
#define MOTOR_SPEED_FIELD_WEAKENING_MIN						490

#define MOTOR_POLE_PAIRS	                                    8U 		// 1 * RPS = MOTOR_POLE_PAIRS * ERPS// cadence sensor

// foc angle multiplier
#if MOTOR_TYPE
// 36 volt motor
#define FOC_ANGLE_MULTIPLIER								30

//bemf 36V motor = 0.0806 V/(rad/s) = 0.5 V/(rev/s) = 0.0633 V/erps source:  https://avdweb.nl/solar-bike/hub-motor/efficiency-bldc-motor-tongsheng-tsdz2-and-astro-3205-compared
#define K_BEMF_X1000                                        63U

#else
// 48 volt motor
#define FOC_ANGLE_MULTIPLIER								39

//casainho said 48V motor has the same max speed (4000rpm) as 36V motor [with corresponding battery] and 4/3 more windings, so the BEMF factor can be scaled by 4/3:
//bemf 48V motor: 0.0633 * 48/36 = 0.0844 V/erps:
#define K_BEMF_X1000                                        84U
#endif

//Gearing - motor
/*------------------------------------------------------------------------------
The secondary has a 93T gear being driven by an 10T, 
so it is an 9.3:1 reduction (the primary “blue gear” was 4.5:1), 
for a total of 4.5 X 9.3 = 41.85:1
https://www.electricbike.com/tsdz2-750w-mid-drive-torque-sensing
---------------------------------------------------------------------------------*/
#define MOTOR_GEAR_RATIO_X8 335U // 41.85

//motor updates speed 6x8 per motor revolution (6 hal states)
//cadence updates speed with each of its hal sensor state transition (CADENCE_SENSOR_STATES) which is every 6x8x42/20/4 motor revolutions (about 24x slower)
#define MOTOR_EROTATIONS_EVERY_CADENCE_TICK ((uint8_t)(MOTOR_POLE_PAIRS * MOTOR_GEAR_RATIO_X8 / 8U / CADENCE_SENSOR_NUMBER_MAGNETS / CADENCE_SENSOR_STATES)) //motor has 6 hal sensor states


// cadence sensor
/*---------------------------------------------------------------------------
 NOTE: regarding the cadence sensor

 CADENCE_SENSOR_NUMBER_MAGNETS = 20, this is the number of magnets used for
 the cadence sensor. Was validated on August 2018 by Casainho and jbalat

 Cadence is calculated by counting how much time passes between two
 transitions. Depending on if all transitions are measured or simply
 transitions of the same kind it is important to adjust the calculation of
 pedal cadence.
*/
#define CADENCE_SENSOR_NUMBER_MAGNETS				20U
#define CADENCE_SENSOR_STATES                       4U      // There are two hal sensors and both can be On or Off

#define CADENCE_TICKS_STARTUP_RPM                   1U

#define CADENCE_RPM_TICK_NUM						(MOTOR_TASK_FREQ * (60U / CADENCE_SENSOR_NUMBER_MAGNETS))
#define CADENCE_RPS_TICK_NUM						(MOTOR_TASK_FREQ / CADENCE_SENSOR_NUMBER_MAGNETS)
#define CADENCE_COUNTER_RESET						1U
#define CADENCE_TICKS_STOP							(CADENCE_RPM_TICK_NUM + 1U) //add one to ensure the division with CADENCE_RPM_TICK_NUM gives 0RPM
// Wheel speed sensor
#define MAX_PLAUSIBLE_WHEEL_SPEED_X10				800U
#define WHEEL_SPEED_COUNTER_RESET					1U
#define WHEEL_SPEED_COUNTER_MAX						UINT16_MAX
#define WHEEL_SPEED_TICKS_STOP						UINT16_MAX


// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MAX_SPEED        ((uint16_t)((uint32_t)WHEEL_PERIMETER * MOTOR_TASK_FREQ / (MAX_PLAUSIBLE_WHEEL_SPEED_X10 / 10U) * 60U / 1000U * 60U / 1000U))// small value - fast rotation

#define PWM_DUTY_CYCLE_MAX									UINT8_MAX
#define PWM_DUTY_CYCLE_BITS                                 8U
#define PWM_DUTY_CYCLE_STARTUP								30    // Initial PWM Duty Cycle at motor startup

// ----------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------

/* Hall Sensors NOTE! - results after Hall sensor calibration experiment
Dai test sulla calibrazione dei sensori Hall risulta che Trise - Tfall = 21 e cioè 84 us
(1 Hall counter step = 4us).
Quindi gli stati 6,3,5 (fronte di salita) vengono rilevati con un ritardo di 84us maggiore
rispetto agli stati 2,1,4.
Quindi per gli stati 6,3,5 va sommato 21 (21x4us=84us) al contatore Hall usato per l'interpolazione,
visto che è partito con 84us di ritardo rispetto agli altri stati.
In questo modo il contatore Hall viene allineato allo stesso modo per tutti gli stati, ma sarà
comunque in ritardo di Tfall per tutti gli stati. Questo ritardo viene gestito con un ulteriore
offset da sommare al contatore per tutti gli stati.
Dai test effettuati risulta che Tfall vale circa 66us (16,5 step) a cui va sommato il ritardo fra							   
la lettura del contatore Hall e la scrittura dei registri PWM che è sempre uguale a mezzo
ciclo PWM (1/(19047*2) = 26,25us o 6,5 step).
Quindi l'offset per gli stati 2,1,4 vale 23 (16,5+6,5) mentre per gli stati 6,3,5
vale 44 (16,5+6,5+21).
I test effettuati hanno inoltre calcolato che il riferimento angolare corretto non è 10 ma 4 step
***************************************
Test effettuato il 21/1/2021
MOTOR_ROTOR_OFFSET_ANGLE:  10 -> 4
HALL_COUNTER_OFFSET_DOWN:  8  -> 23
HALL_COUNTER_OFFSET_UP:    29 -> 44
****************************************
*/

#define HALL_COUNTER_OFFSET_DOWN                (HALL_COUNTER_FREQ/MOTOR_TASK_FREQ/2 + 17)
#define HALL_COUNTER_OFFSET_UP                  (HALL_COUNTER_OFFSET_DOWN + 21)
#define FW_HALL_COUNTER_OFFSET_MAX              5 // 5*4=20us max time offset

#define MOTOR_ROTOR_INTERPOLATION_MIN_ERPS      10U
 
// adc torque offset gap value for error
#define ADC_TORQUE_SENSOR_OFFSET_THRESHOLD		30

// Torque sensor range values
#define ADC_TORQUE_SENSOR_RANGE					(uint16_t)(PEDAL_TORQUE_ADC_MAX - PEDAL_TORQUE_ADC_OFFSET)
#define ADC_TORQUE_SENSOR_RANGE_TARGET	  		160

// Torque sensor offset values
#if TORQUE_SENSOR_CALIBRATED
#define ADC_TORQUE_SENSOR_CALIBRATION_OFFSET    (uint16_t)(((6 * ADC_TORQUE_SENSOR_RANGE) / ADC_TORQUE_SENSOR_RANGE_TARGET) + 1)
#define ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ		(uint16_t)(((20 * ADC_TORQUE_SENSOR_RANGE) / ADC_TORQUE_SENSOR_RANGE_TARGET) + 1)
#define ADC_TORQUE_SENSOR_OFFSET_ADJ			(uint16_t)(((PEDAL_TORQUE_ADC_OFFSET_ADJ * ADC_TORQUE_SENSOR_RANGE) / ADC_TORQUE_SENSOR_RANGE_TARGET) + 1)
#else
#define ADC_TORQUE_SENSOR_CALIBRATION_OFFSET    6
#define ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ		20
#define ADC_TORQUE_SENSOR_OFFSET_ADJ			PEDAL_TORQUE_ADC_OFFSET_ADJ
#endif

// adc torque range parameters for remapping
#define ADC_TORQUE_SENSOR_DELTA_ADJ				(uint16_t)((ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ * 2) - ADC_TORQUE_SENSOR_CALIBRATION_OFFSET - ADC_TORQUE_SENSOR_OFFSET_ADJ)
#define ADC_TORQUE_SENSOR_RANGE_INGREASE_X100	(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET * 50) / ADC_TORQUE_SENSOR_RANGE)
#define ADC_TORQUE_SENSOR_ANGLE_COEFF			11
#define ADC_TORQUE_SENSOR_ANGLE_COEFF_X10		(uint16_t)(ADC_TORQUE_SENSOR_ANGLE_COEFF * 10)

#define ADC_TORQUE_SENSOR_RANGE_TARGET_MIN 		(uint16_t)((float)((ADC_TORQUE_SENSOR_RANGE_TARGET / 2) \
* (((ADC_TORQUE_SENSOR_RANGE_TARGET / 2) / ADC_TORQUE_SENSOR_ANGLE_COEFF + ADC_TORQUE_SENSOR_ANGLE_COEFF) / ADC_TORQUE_SENSOR_ANGLE_COEFF)))

#define ADC_TORQUE_SENSOR_RANGE_TARGET_MAX 		(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET_MIN * (100 + PEDAL_TORQUE_ADC_RANGE_ADJ)) / 100)

// parameters of the adc torque step for human power calculation
#define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100	34 // base adc step for remapping
#define WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION		24 // Kg
#define PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT		75 // % of torque sensor range with weight
#define ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT		(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET * PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT) / 100)

#define ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT			(uint16_t)(((((ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT \
* ADC_TORQUE_SENSOR_RANGE_TARGET_MIN) / ADC_TORQUE_SENSOR_RANGE_TARGET)	* (100 + PEDAL_TORQUE_ADC_RANGE_ADJ) / 100) \
* (ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT - ADC_TORQUE_SENSOR_CALIBRATION_OFFSET + ADC_TORQUE_SENSOR_OFFSET_ADJ \
- ((ADC_TORQUE_SENSOR_DELTA_ADJ * ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT) / ADC_TORQUE_SENSOR_RANGE_TARGET))) / ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT)

#define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100	(uint8_t)((uint16_t)(((WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION * 167) \
/ ((ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT * ADC_TORQUE_SENSOR_RANGE_TARGET_MAX) \
/ (ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - (((ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT) * 10) \
/ PEDAL_TORQUE_ADC_ANGLE_ADJ))) \
* PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100) / PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100))

// scale the torque assist target current
#define TORQUE_ASSIST_FACTOR_DENOMINATOR			120

// torque step mode
#define TORQUE_STEP_DEFAULT							0 // not calibrated
#define TORQUE_STEP_ADVANCED						1 // calibrated

// smooth start ramp
#define SMOOTH_START_RAMP_MIN						30

// adc current
//#define ADC_10_BIT_BATTERY_EXTRACURRENT				38  //  6 amps
#define ADC_10_BIT_BATTERY_EXTRACURRENT				50  //  8 amps
#define ADC_10_BIT_BATTERY_CURRENT_MAX				112	// 18 amps // 1 = 0.16 Amp
//#define ADC_10_BIT_BATTERY_CURRENT_MAX				124	// 20 amps // 1 = 0.16 Amp
//#define ADC_10_BIT_BATTERY_CURRENT_MAX				136	// 22 amps // 1 = 0.16 Amp
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX			187	// 30 amps // 1 = 0.16 Amp
/*---------------------------------------------------------
 NOTE: regarding ADC battery current max

 This is the maximum current in ADC steps that the motor
 will be able to draw from the battery. A higher value
 will give higher torque figures but the limit of the
 controller is 16 A and it should not be exceeded.
 ---------------------------------------------------------*/

// throttle ADC values
//#define ADC_THROTTLE_MIN_VALUE					47
//#define ADC_THROTTLE_MAX_VALUE					176

/*---------------------------------------------------------
 NOTE: regarding throttle ADC values

 Max voltage value for throttle, in ADC 8 bits step,
 each ADC 8 bits step = (5 V / 256) = 0.0195

 ---------------------------------------------------------*/

/* ---------------------------------------------------------------------------

 NOTE: regarding the torque sensor output values

 Torque (force) value needs to be found experimentaly.

 One torque sensor ADC 10 bit step is equal to 0.38 kg

 Force (Nm) = 1 Kg * 9.81 * 0.17 (0.17 = arm cranks size)
 --------------------------------------------------------------------------*/

// ADC battery voltage measurement
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X512		44
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000		87  // conversion value verified with a cheap power meter

// ADC battery voltage to be subtracted from the cut-off
#define DIFFERENCE_CUT_OFF_SHUTDOWN_10_BIT				100

/*---------------------------------------------------------
 NOTE: regarding ADC battery voltage measurement

 0.344 per ADC 8 bit step:

 17.9 V -->  ADC 8 bits value  = 52;
 40 V   -->  ADC 8 bits value  = 116;

 This signal is atenuated by the opamp 358.
 ---------------------------------------------------------*/

// ADC battery current measurement
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X512		80
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100		16  // 0.16A x 10 bit ADC step

// for oem display

// UART
#define UART_RX_BUFFER_LEN   						7
#define RX_CHECK_CODE					(UART_RX_BUFFER_LEN - 1)															
#define UART_TX_BUFFER_LEN							9
#define TX_CHECK_CODE					(UART_TX_BUFFER_LEN - 1)
#define TX_STX										0x43
#define RX_STX										0x59

// parameters for display data
#if UNITS_TYPE          // 1 mph and miles
#define OEM_WHEEL_FACTOR							900
#else                   // 0 = km/h and kilometer
#define OEM_WHEEL_FACTOR							1435
#endif
#define MILES										1

#define DATA_INDEX_ARRAY_DIM						6

// delay lights function (0.1 sec)
#define DELAY_LIGHTS_ON					 	DELAY_MENU_ON

// delay function status (0.1 sec)
#define DELAY_FUNCTION_STATUS			(uint8_t) (DELAY_MENU_ON / 2)

// delay torque sensor calibration (0.1 sec)
#define DELAY_DISPLAY_TORQUE_CALIBRATION			250

// display function status
#define FUNCTION_STATUS_OFF							1
#define FUNCTION_STATUS_ON				(uint8_t) (100 + DISPLAY_STATUS_OFFSET)
#define DISPLAY_STATUS_OFFSET						5

// assist level 
#define OFF											0
#define ECO											1
#define TOUR										2
#define SPORT										3
#define TURBO										4

// assist pedal level mask
#define ASSIST_PEDAL_LEVEL0							0x10
#define ASSIST_PEDAL_LEVEL01						0x80
#define ASSIST_PEDAL_LEVEL1							0x40
#define ASSIST_PEDAL_LEVEL2							0x02
#define ASSIST_PEDAL_LEVEL3							0x04
#define ASSIST_PEDAL_LEVEL4							0x08

#define ASSIST_PEDAL_LEVEL01_PERCENT				60

// assist mode
#define OFFROAD_MODE								0
#define STREET_MODE									1

// oem display fault & function code
#define CLEAR_DISPLAY								0
#define NO_FUNCTION									0
#define NO_FAULT									0
#define NO_ERROR                                  	0 

#define ERROR_OVERVOLTAGE							1 // E01 (E06 blinking for XH18)
#define ERROR_TORQUE_SENSOR                       	2 // E02
#define ERROR_CADENCE_SENSOR			          	3 // E03
#define ERROR_MOTOR_BLOCKED                       	4 // E04
#define ERROR_THROTTLE								5 // E05 (E03 blinking for XH18)
#define ERROR_OVERTEMPERATURE						6 // E06
#define ERROR_BATTERY_OVERCURRENT                 	7 // E07 (E04 blinking for XH18)
#define ERROR_SPEED_SENSOR							8 // E08
#define ERROR_WRITE_EEPROM  					  	9 // E09 shared (E08 blinking for XH18)
#define ERROR_MOTOR_CHECK                       	9 // E09 shared (E08 blinking for XH18)

// optional ADC function
#if ENABLE_TEMPERATURE_LIMIT && ENABLE_THROTTLE
#define OPTIONAL_ADC_FUNCTION                 		NOT_IN_USE
#elif ENABLE_TEMPERATURE_LIMIT
#define OPTIONAL_ADC_FUNCTION                 		TEMPERATURE_CONTROL
#elif ENABLE_THROTTLE && ENABLE_BRAKE_SENSOR
#define OPTIONAL_ADC_FUNCTION                 		THROTTLE_CONTROL
#else
#define OPTIONAL_ADC_FUNCTION                 		NOT_IN_USE
#endif

// temperature sensor type
#define LM35										0
#define TMP36										1

// throttle mode
#define DISABLED									0
#define PEDALING									1
#define W_O_P_6KM_H_ONLY							2
#define W_O_P_6KM_H_AND_PEDALING					3
#define UNCONDITIONAL								4

// wheel perimeter
#define WHEEL_PERIMETER_0							(uint8_t) (WHEEL_PERIMETER & 0x00FF)
#define WHEEL_PERIMETER_1							(uint8_t) ((WHEEL_PERIMETER >> 8) & 0x00FF)

// BATTERY PARAMETER
// battery low voltage cut off
#define BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0		(uint8_t) ((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF * 10) & 0x00FF)
#define BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1		(uint8_t) (((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF * 10) >> 8) & 0x00FF)
// battery voltage to be subtracted from the cut-off 8bit
#define DIFFERENCE_CUT_OFF_SHUTDOWN_8_BIT			26
// battery voltage for saving battery capacity at shutdown
#define BATTERY_VOLTAGE_SHUTDOWN_8_BIT			(uint8_t) (((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF * 250 / BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000)) - ((uint16_t) DIFFERENCE_CUT_OFF_SHUTDOWN_8_BIT))
#define BATTERY_VOLTAGE_SHUTDOWN_10_BIT			(uint16_t) (BATTERY_VOLTAGE_SHUTDOWN_8_BIT << 2)
// battery voltage reset SOC percentage
#define BATTERY_VOLTAGE_RESET_SOC_PERCENT_X10   (uint16_t)((float)LI_ION_CELL_RESET_SOC_PERCENT * (float)(BATTERY_CELLS_NUMBER * 10))
// battery SOC eeprom value saved (8 bit)
#define BATTERY_SOC								0
// battery SOC % threshold x10 (volts calc)
#define BATTERY_SOC_PERCENT_THRESHOLD_X10		150

// cell bars
#if ENABLE_VLCD6 || ENABLE_XH18
#define LI_ION_CELL_VOLTS_6_X100		(uint16_t)((float)LI_ION_CELL_OVERVOLT * 100)
#define LI_ION_CELL_VOLTS_5_X100		(uint16_t)((float)LI_ION_CELL_RESET_SOC_PERCENT * 100)
#define LI_ION_CELL_VOLTS_4_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_FULL * 100)
#define LI_ION_CELL_VOLTS_3_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_3_OF_4 * 100)
#define LI_ION_CELL_VOLTS_2_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_2_OF_4 * 100)
#define LI_ION_CELL_VOLTS_1_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_1_OF_4 * 100)
#define LI_ION_CELL_VOLTS_0_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_EMPTY * 100)
#define BATTERY_SOC_VOLTS_6_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_OVERVOLT * 10))
#define BATTERY_SOC_VOLTS_5_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_RESET_SOC_PERCENT * 10))
#define BATTERY_SOC_VOLTS_4_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_FULL * 10))
#define BATTERY_SOC_VOLTS_3_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_3_OF_4 * 10))
#define BATTERY_SOC_VOLTS_2_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_2_OF_4 * 10))
#define BATTERY_SOC_VOLTS_1_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_1_OF_4 * 10))
#define BATTERY_SOC_VOLTS_0_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_EMPTY * 10))
#else // ENABLE_VLCD5 or 850C
#define LI_ION_CELL_VOLTS_8_X100		(uint16_t)((float)LI_ION_CELL_OVERVOLT * 100)
#define LI_ION_CELL_VOLTS_7_X100		(uint16_t)((float)LI_ION_CELL_RESET_SOC_PERCENT * 100)
#define LI_ION_CELL_VOLTS_6_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_FULL * 100)
#define LI_ION_CELL_VOLTS_5_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_5_OF_6 * 100)
#define LI_ION_CELL_VOLTS_4_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_4_OF_6 * 100)
#define LI_ION_CELL_VOLTS_3_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_3_OF_6 * 100)
#define LI_ION_CELL_VOLTS_2_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_2_OF_6 * 100)
#define LI_ION_CELL_VOLTS_1_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_1_OF_6 * 100)
#define LI_ION_CELL_VOLTS_0_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_EMPTY * 100)
#define BATTERY_SOC_VOLTS_8_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_OVERVOLT * 10))
#define BATTERY_SOC_VOLTS_7_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_RESET_SOC_PERCENT * 10))
#define BATTERY_SOC_VOLTS_6_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_FULL * 10))
#define BATTERY_SOC_VOLTS_5_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_5_OF_6 * 10))
#define BATTERY_SOC_VOLTS_4_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_4_OF_6 * 10))
#define BATTERY_SOC_VOLTS_3_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_3_OF_6 * 10))
#define BATTERY_SOC_VOLTS_2_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_2_OF_6 * 10))
#define BATTERY_SOC_VOLTS_1_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_1_OF_6 * 10))
#define BATTERY_SOC_VOLTS_0_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_EMPTY * 10))
#endif

// assist level 0
#define TORQUE_ASSIST_LEVEL_0        0
#define CADENCE_ASSIST_LEVEL_0       0
#define EMTB_ASSIST_LEVEL_0          0
#define WALK_ASSIST_LEVEL_0          0
#define CRUISE_TARGET_SPEED_LEVEL_0  0

// power assist level
#define POWER_ASSIST_LEVEL_OFF       0
#define POWER_ASSIST_LEVEL_ECO       (uint8_t)(POWER_ASSIST_LEVEL_1 / 2)
#define POWER_ASSIST_LEVEL_TOUR      (uint8_t)(POWER_ASSIST_LEVEL_2 / 2)
#define POWER_ASSIST_LEVEL_SPORT     (uint8_t)(POWER_ASSIST_LEVEL_3 / 2)
#define POWER_ASSIST_LEVEL_TURBO     (uint8_t)(POWER_ASSIST_LEVEL_4 / 2)

// walk assist
#define WALK_ASSIST_THRESHOLD_SPEED		(uint8_t)(WALK_ASSIST_THRESHOLD_SPEED_X10 / 10)
#define WALK_ASSIST_WHEEL_SPEED_MIN_DETECT_X10	42
#define WALK_ASSIST_ERPS_THRESHOLD				20
#define WALK_ASSIST_ADJ_DELAY_MIN				4
#define WALK_ASSIST_ADJ_DELAY_STARTUP			10
#define WALK_ASSIST_DUTY_CYCLE_MIN              40
#define WALK_ASSIST_DUTY_CYCLE_STARTUP			50
#define WALK_ASSIST_DUTY_CYCLE_MAX              130
#define WALK_ASSIST_ADC_BATTERY_CURRENT_MAX     40


// cruise threshold (speed limit min km/h x10)
#define CRUISE_THRESHOLD_SPEED_X10				(CRUISE_THRESHOLD_SPEED * 10)
#define CRUISE_THRESHOLD_SPEED_X10_DEFAULT		80
#define CRUISE_OFFROAD_THRESHOLD_SPEED_X10		(uint8_t)CRUISE_THRESHOLD_SPEED_X10
#if CRUISE_THRESHOLD_SPEED_X10 < CRUISE_THRESHOLD_SPEED_X10_DEFAULT
#define CRUISE_STREET_THRESHOLD_SPEED_X10		(uint8_t)(CRUISE_THRESHOLD_SPEED_X10_DEFAULT)
#else	
#define CRUISE_STREET_THRESHOLD_SPEED_X10		(uint8_t)(CRUISE_THRESHOLD_SPEED_X10)
#endif	

// odometer compensation for displayed data (eeprom)
#define ODOMETER_COMPENSATION					0
// zero odometer compensation
#define ZERO_ODOMETER_COMPENSATION				100000000

#define ASSISTANCE_WITH_ERROR_ENABLED			0

#define AVAIABLE_FOR_FUTURE_USE					0 // EEPROM



static_assert(MOTOR_TYPE == 0 || MOTOR_TYPE == 1);

#endif // _MAIN_H_
