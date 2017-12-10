/*
 * Utils.h
 *
 * Created: 12/8/2017 11:27:58 PM
 *  Author: vsand
 */


#ifndef UTILS_H_
#define UTILS_H_


struct PIDConfig_e
{
    char Name[5];
    double Kp;
    double Ki;
    double Kd;
    double Setpoint;
    double Input;
    double Output;

    PIDConfig_e(char* name, double kp, double ki, double kd, double setpoint)
    {
        strcpy(Name, name);
        Kp=kp;
        Kd=kd;
        Ki=ki;
        Setpoint=setpoint;
    }
} ;


#endif /* UTILS_H_ */