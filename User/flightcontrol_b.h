#ifndef __FLIGHTCONTROL_B_H
#define __FLIGHTCONTROL_B_H

#include "stm32f10x.h"                  // Device header
void initialize_pid(void);
float applyMovingAverage(uint16_t* history);
float mapPPMToAngle(uint16_t ppmValue, uint16_t minPPM, uint16_t maxPPM, float minAngle, float maxAngle);
void updatePPMHistory(uint8_t channel, uint16_t ppmValue);
void calculateAttitudeAngles(float* pitch, float* roll, float* yaw);
void updateAndCalculate(void);
uint16_t map_to_pwm(float pid_output, float output_min, float output_max);
void control_loop(void);
#endif
