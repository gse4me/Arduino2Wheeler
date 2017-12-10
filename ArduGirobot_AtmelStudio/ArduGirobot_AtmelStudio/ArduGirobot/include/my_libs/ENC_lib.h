
#ifndef ENC_LIB_H_
#define ENC_LIB_H_

#include <Arduino.h>
#include "Configs.h"

extern long Mot0_Enc;
extern long Mot1_Enc;

extern uint8_t Mot0_Dir;
extern uint8_t Mot1_Dir;

void Enc_Init();


#endif /* ENC_LIB_H_ */