/**
 * @file Sensor.h
 *
 * This file contains the headers for sensor.cpp
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "gpio.h"
#include <iostream> 


/* for sensors */
#define pin_cs_sensor P9_13 // 0_31=31
#define pin_clk_sensor P9_12 // 1_28=60
#define pin_din_sensor  P9_11 // 0_30=30
#define pin_dout1_sensor P9_14 // 1_18=50
#define pin_dout2_sensor  P9_15 // 1_16=48
#define pin_dout3_sensor  P9_26 // 0_14=14
#define pin_dout4_sensor  P9_27 // 3_19=125

#define NUM_ADC 4
#define NUM_ADC_PORT 8

#define NUM_OF_POT_SENSOR 10
#define NUM_OF_PRES_SENSOR 22
#define NUM_OF_SENSOR NUM_OF_POT_SENSOR+NUM_OF_PRES_SENSOR

/* for analog input */
#define NUM_OF_AINS 7
#define AIO_NUM 7

/* INDEX PIN for Sensor */
#define PIN_POT1 0     // Pin for Potensiometer 1
#define PIN_POT2 1     // Pin for Potensiometer 1
#define PIN_POT3 2     // Pin for Potensiometer 1
#define PIN_POT4 3     // Pin for Potensiometer 1
#define PIN_POT5 4     // Pin for Potensiometer 1
#define PIN_POT6 5     // Pin for Potensiometer 1
#define PIN_POT7 6     // Pin for Potensiometer 1
#define PIN_POT8 7     // Pin for Potensiometer 1
#define PIN_POT9 0     // Pin for Potensiometer 1
#define PIN_POT10 1     // Pin for Potensiometer 1

#define PIN_PRES_1 1    // Pin for Pressure Sensor 1
#define PIN_PRES_2 2    // Pin for Pressure Sensor 2


PIN analog_pin[NUM_OF_AINS];
FILE* fd[AIO_NUM] = {};

/* Analog Input */
void initAIO(void);
void closeAIO(void);
uint32_t myAnalogRead(int i);

/**** SPI for sensors ****/
void set_DIN_SENSOR(bool value);
void set_CLK_SENSOR(bool value);
void set_CS_SENSOR(bool value);
int get_DOUT_SENSOR(int adc_num);

/* reading function */
unsigned long *read_sensor(unsigned long adc_num,unsigned long* sensorVal);
void read_sensor_all (int index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], double *Angle, double *Pressure);

/* ADC Converter */
double ADCtoPressure (unsigned long ADCValue);
double ADCtoAngle (unsigned long ADCValue, int Pot_ref);

void printSensorVal (int i, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT]);

#endif
