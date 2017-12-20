/*
 * Utils.h
 *
 * Created: 12/8/2017 11:27:58 PM
 *  Author: vsand
 */


#ifndef UTILS_H_
#define UTILS_H_

#include <EEPROM.h>

struct PIDConfig_e {
    double Kp;
    double Ki;
    double Kd;
    double Setpoint;
    double Input;
    double Output;

    PIDConfig_e(double kp, double ki, double kd, double setpoint) {
        Kp=kp;
        Kd=kd;
        Ki=ki;
        Setpoint=setpoint;
    }
} ;

inline void LOG(const char* x) {
    Serial.write(255);
    Serial.println(x);
}

void WritePidCfgToEEPROM(uint16_t addr,PIDConfig_e *cfg) {
    EEPROM.put(addr, cfg->Kp);
    addr+=sizeof(double);
    EEPROM.put(addr, cfg->Ki);
    addr+=sizeof(double);
    EEPROM.put(addr,cfg->Kd);
}

void ReadPidCfgFromEEPROM(uint16_t addr,PIDConfig_e *cfg) {
    EEPROM.get(addr, cfg->Kp);
    addr+=sizeof(double);
    EEPROM.get(addr, cfg->Ki);
    addr+=sizeof(double);
    EEPROM.get(addr,cfg->Kd);
}


#endif /* UTILS_H_ */