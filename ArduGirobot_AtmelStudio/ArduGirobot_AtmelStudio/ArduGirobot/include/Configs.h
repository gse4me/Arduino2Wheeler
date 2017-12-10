/*
 * Configs.h
 *
 * Created: 12/7/2017 9:57:32 PM
 *  Author: vsand
 */


#ifndef CONFIGS_H_
#define CONFIGS_H_

//ENV
#define XBEE__BAUD_RATE		 115200
#define SERIAL__BAUD_RATE	 XBEE__BAUD_RATE
#define SERIAL__MY_BUF_SIZE  64


//PING
#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

//MOTOR DEFINES
#define MOT__USE_SLOW_DECAY     //motor drive type 

//ENCODER DEFINES
#define ENC__USE_DOUBLE_INT    //use 2 interrupts or 1 per encoder



//PID DEFINES
#define PID__MOTOR_SAMPLE_TIME 5

#ifdef MOT__USE_SLOW_DECAY
#define MOT0__KP 40
#define MOT0__KI 100
#define MOT0__KD 0
#define MOT0__OUT_MIN -255
#define MOT0__OUT_MAX 255
#define MOT0__SAMPLE_TIME PID__MOTOR_SAMPLE_TIME

#define MOT1__KP 40
#define MOT1__KI 100
#define MOT1__KD 0
#define MOT1__OUT_MIN -255
#define MOT1__OUT_MAX 255
#define MOT1__SAMPLE_TIME PID__MOTOR_SAMPLE_TIME


#define GIRO__TARGET_ANGLE -3 //setpoint
#define GIRO__KP 0.8
#define GIRO__KI 0.8
#define GIRO__KD 0
#define GIRO__OUT_MIN -35
#define GIRO__OUT_MAX 35
#define GIRO__SAMPLE_TIME PID__MOTOR_SAMPLE_TIME
#else


#endif

#endif /* CONFIGS_H_ */