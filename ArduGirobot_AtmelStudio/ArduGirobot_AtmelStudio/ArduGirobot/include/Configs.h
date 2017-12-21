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
#define SERIAL_LOG_INTERVAL  3


//PING
#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

//MOTOR DEFINES
#define MOT__USE_SLOW_DECAY     //motor drive type 


//PID DEFINES
#define PID__MOTOR_SAMPLE_TIME 5
//ENCODER DEFINES
#define ENC__USE_DOUBLE_INT    //use 2 interrupts or 1 per encoder
#define ENC__SAMPLE_TIME  PID__MOTOR_SAMPLE_TIME

#define EEPROM_PID_CONFIGS_ADDR 0
#define LOAD_PIDS_FROM_EEPROM 1

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

//------------------------ protocol ---------------//
#define ARD_START_PRINT_IDX ARD_PID1_INPUT
//------------------------- RECEIVE COMMANDS ----------------//

#define ARD_LOG             255

#define ARD_PID1_INPUT      1
#define ARD_PID1_OUTPUT     2
#define ARD_PID1_SETPOINT   3

#define ARD_PID2_INPUT      4
#define ARD_PID2_OUTPUT     5
#define ARD_PID2_SETPOINT   6

#define ARD_PID3_INPUT      7
#define ARD_PID3_OUTPUT     8
#define ARD_PID3_SETPOINT   9

//skip 10 -> it's newline
#define ARD_PID1_KP         11
#define ARD_PID1_KI         12
#define ARD_PID1_KD         13

#define ARD_PID2_KP         14
#define ARD_PID2_KI         15
#define ARD_PID2_KD         16

#define ARD_PID3_KP         17
#define ARD_PID3_KI         18
#define ARD_PID3_KD         19

#define ARD_NORMAL_LOOP_TIME 20
#define ARD_SERIAL_LOOP_TIME 21

//------------------------- TRANSMIT COMMANDS ---------------//
#define CUTE_PID1_KP        1
#define CUTE_PID1_KI        2
#define CUTE_PID1_KD        3

#define CUTE_PID2_KP        4
#define CUTE_PID2_KI        5
#define CUTE_PID2_KD        6

#define CUTE_PID3_KP        7
#define CUTE_PID3_KI        8
#define CUTE_PID3_KD        9

//avoid 10
#define CUTE_PID1_SETP      11
#define CUTE_PID2_SETP      12
#define CUTE_PID3_SETP      13

#define CUTE_P1_PRNT_ON         20
#define CUTE_P1_PRNT_OFF        21
#define CUTE_P2_PRNT_ON         22
#define CUTE_P2_PRNT_OFF        23
#define CUTE_P3_PRNT_ON         24
#define CUTE_P3_PRNT_OFF        25

#define CUTE_GIRO_TO_MOT_ON     26
#define CUTE_GIRO_TO_MOT_OFF    27
#define CUTE_GET_ALL_PID_CFGS   28

#define CUTE_SAVE_TO_EEPROM		30
#define CUTE_GET_UP 31

#define CUTE_CYCLE_TIME_PRINTS_ON 32
#define CUTE_CYCLE_TIME_PRINTS_OFF 33


#endif /* CONFIGS_H_ */