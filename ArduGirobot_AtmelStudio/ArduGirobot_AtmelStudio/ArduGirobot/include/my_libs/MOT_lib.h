

#ifndef MYPWM_H_
#define MYPWM_H_

#include <Arduino.h>
#include "Configs.h"

#define FWD 0
#define REV 1

#define PD5Pin 5
#define PD6Pin 6
#define PB3Pin 11
#define PD3Pin 3

#define MOT0A PD6Pin
#define MOT0B PD5Pin

#define MOT1A PB3Pin
#define MOT1B PD3Pin



void Motor0_Stop();
void Motor1_Stop();

void Mot0_Init(void);
void Mot1_Init(void);

void Motor0_SetSpeed(uint8_t dir,uint8_t speed);
void Motor1_SetSpeed(uint8_t dir,uint8_t speed);

void Motors_SetSpeed(int leftMotorSpeed, int rightMotorSpeed);




#endif /* MYPWM_H_ */