#include <Arduino.h>
#include <MPU6050.h>

#include <string.h>
#include "Motors.h"
#include "NewPing.h"
#include "MPU6050Calib.h"
#include "PID_v1.h"
#include "Configs.h"

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

MPU6050 mpu;



int16_t Giro_AccY, Giro_AccZ, Giro_RotX;
volatile int motorPower, Giro_Rate;
volatile float Giro_AccAngle, Giro_Angle, Giro_FilteredAngle, Giro_PrevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;



unsigned long previousMilliEnc = 0;
unsigned long previousMilliSerialLog = 0;
const long intervalEnc = PID__MOTOR_SAMPLE_TIME;
const long intervalSerialLog = 100;

bool running=false;

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

//PIDConfig_e Mot0_PidCtl ("M0",20,220,0,0);
//PIDConfig_e Mot1_PidCtl ("M1",20,220,0,0);
#ifdef SLOW_DRAIN
//PIDConfig_e Mot0_PidCtl ("M0",30,100,0,0);
//PIDConfig_e Mot1_PidCtl ("M1",27,100,0,0);

PIDConfig_e Mot0_PidCtl ("M0",20,20,0,0);
PIDConfig_e Mot1_PidCtl ("M1",20,20,0,0);

#else
PIDConfig_e Mot0_PidCtl ("M0",40,220,0,0);
PIDConfig_e Mot1_PidCtl ("M1",40,220,0,0);
#endif

PIDConfig_e Giro_PidCtl ("Gi",1,0.5,0,targetAngle);

PID Mot0_Pid(&Mot0_PidCtl.Input, &Mot0_PidCtl.Output, &Mot0_PidCtl.Setpoint, Mot0_PidCtl.Kp, Mot0_PidCtl.Ki, Mot0_PidCtl.Kd, P_ON_E, DIRECT);
PID Mot1_Pid(&Mot1_PidCtl.Input, &Mot1_PidCtl.Output, &Mot1_PidCtl.Setpoint, Mot1_PidCtl.Kp, Mot1_PidCtl.Ki, Mot1_PidCtl.Kd, P_ON_E, DIRECT);
PID Giro_Pid(&Giro_PidCtl.Input, &Giro_PidCtl.Output, &Giro_PidCtl.Setpoint, Giro_PidCtl.Kp, Giro_PidCtl.Ki, Giro_PidCtl.Kd, P_ON_E, DIRECT);


/*


->SP M0/M1/Gi Kp Kd Ki

->GD M0/M1/Gi
<-DD: M0/M1/Gi Kp Kd Ki Setpoint

->SS M0/M1/M2 Setpoint

<-ENC: M0val M0dir M1val M2dir

*/


void PrintPidCfg(PIDConfig_e *cfg)
{
    Serial.print("DD: ");
    Serial.print(cfg->Name);
    Serial.print(" ");
    Serial.print(cfg->Kp);
    Serial.print(" ");
    Serial.print(cfg->Ki);
    Serial.print(" ");
    Serial.print(cfg->Kd);
    Serial.print(" ");
    Serial.print(cfg->Setpoint);
    Serial.println(" ");
}

void PrintEncoderValues()
{
    Serial.print("ENC: ");
    Serial.print(Mot0_PidCtl.Input);
    (Mot0_Dir==0xFFFF)?Serial.print(" 1 "):Serial.print(" 0 ");

    Serial.print(Mot1_PidCtl.Input);
    (Mot1_Dir==0xFFFF)?Serial.print(" 1 "):Serial.print(" 0 ");
    Serial.println(" ");
}


void PrintPidOutputValues()
{
    Serial.print("PID: ");
    Serial.print(Mot0_PidCtl.Output);

    Serial.print(" ");
    Serial.print(Mot1_PidCtl.Output);

    Serial.println(" ");



    Serial.print("SET: ");
    Serial.print(Mot0_PidCtl.Setpoint);

    Serial.print(" ");
    Serial.print(Mot1_PidCtl.Setpoint);

    Serial.println(" ");
}




const byte Ser_BuffSize = 80;
const char Ser_EndMarker = '\n';
char Ser_Buffer[Ser_BuffSize]; // an array to store the received data
boolean Ser_NewData = false;

void Ser_ReceiveData()
{
    static byte ndx = 0;
    char rc;

    // if (Serial.available() > 0) {
    while (Serial.available() > 0 && Ser_NewData == false)
        {
            rc = Serial.read();

            if (rc != Ser_EndMarker)
                {
                    Ser_Buffer[ndx] = rc;
                    ndx++;
                    if (ndx >= Ser_BuffSize)
                        {
                            ndx = Ser_BuffSize - 1;
                        }
                }
            else
                {
                    Ser_Buffer[ndx] = '\0'; // terminate the string
                    ndx = 0;
                    Ser_NewData = true;
                }
        }
}




PIDConfig_e *CfgPtr=NULL;
PID *PidPtr=NULL;

bool connectGiroToMot=true;

bool printRawGiro=false;
bool printMotPid=false;
bool printGiroPid=false;

void Ser_ParseData()
{
    if (Ser_NewData==false) return;

    char *pch;

    pch=strtok(Ser_Buffer, " ");

    if(strcmp(pch,"SP")==0) //Set Pid
        {
            Serial.println("Debug: SP Start");

            pch=strtok(NULL, " ");
            if(strcmp(pch,"M0")==0)
                {
                    CfgPtr = &Mot0_PidCtl;
                    PidPtr = &Mot0_Pid;
                }
            else if(strcmp(pch,"M1")==0)
                {
                    CfgPtr = &Mot1_PidCtl;
                    PidPtr = &Mot1_Pid;
                }
            else if(strcmp(pch,"Gi")==0)
                {
                    CfgPtr = &Giro_PidCtl;
                    PidPtr = &Giro_Pid;
                }

            pch=strtok(NULL, " ");
            CfgPtr->Kp=atof(pch);

            pch=strtok(NULL, " ");
            CfgPtr->Ki=atof(pch);

            pch=strtok(NULL, " ");
            CfgPtr->Kd=atof(pch);

            PidPtr->SetTunings(CfgPtr->Kp,CfgPtr->Ki,CfgPtr->Kd);

            Serial.println("Debug: SP Done");
        }
    else  if(strcmp(pch,"GD")==0)  //Get Pid Data
        {
            Serial.println("Debug: GD Start");

            pch=strtok(NULL, " ");

            if(strcmp(pch,"M0")==0)
                {
                    CfgPtr = &Mot0_PidCtl;
                }
            else if(strcmp(pch,"M1")==0)
                {
                    CfgPtr = &Mot1_PidCtl;
                }
            else if(strcmp(pch,"Gi")==0)
                {
                    CfgPtr = &Giro_PidCtl;
                }

            PrintPidCfg(CfgPtr);

            Serial.println("Debug: GD Done");
        }
    else if(strcmp(pch,"SS")==0)  //Set  setpoint
        {
            Serial.println("Debug: SS Start");
            pch=strtok(NULL, " ");

            if(strcmp(pch,"M0")==0)
                {
                    CfgPtr = &Mot0_PidCtl;
                    PidPtr = &Mot0_Pid;
                }
            else if(strcmp(pch,"M1")==0)
                {
                    CfgPtr = &Mot1_PidCtl;
                    PidPtr = &Mot0_Pid;
                }
            else if(strcmp(pch,"Gi")==0)
                {
                    CfgPtr = &Giro_PidCtl;
                    PidPtr = &Mot0_Pid;
                }

            pch=strtok(NULL, " ");

            CfgPtr->Setpoint=atof(pch);
            Serial.println("Debug: SS Done");

        }

    else if (strcmp(pch,"GC_ON")==0)
        {
            connectGiroToMot=true;
        }
    else if (strcmp(pch,"GC_OFF")==0)
        {
            connectGiroToMot=false;
            Mot0_PidCtl.Setpoint=0;
            Mot1_PidCtl.Setpoint=0;
        }
    else if (strcmp(pch,"GRP_ON")==0)
        {
            printRawGiro=true;
        }
    else if (strcmp(pch,"GRP_OFF")==0)
        {
            printRawGiro=false;
        }
    else if (strcmp(pch,"MP_ON")==0)
        {
            printMotPid=true;
        }
    else if (strcmp(pch,"MP_OFF")==0)
        {
            printMotPid=false;
        }
    else if (strcmp(pch,"GP_ON")==0)
        {
            printGiroPid=true;
        }
    else if (strcmp(pch,"GP_OFF")==0)
        {
            printGiroPid=false;
        }


    Ser_NewData=false;
}



double *Mot0_RpmPtr=&Mot0_PidCtl.Input;
double *Mot1_RpmPtr=&Mot1_PidCtl.Input;

void UpdateEncoderValues()
{
    *Mot0_RpmPtr=Mot0_Enc;
    *Mot1_RpmPtr=Mot1_Enc;

    Mot0_Enc=0;
    Mot1_Enc=0;
}


unsigned long Giro_currTime;
unsigned long Giro_loopTime;
unsigned long Giro_prevTime;

void Giro_ReadData()
{

    Giro_currTime = millis();
    Giro_loopTime = Giro_currTime - Giro_prevTime;
    Giro_prevTime = Giro_currTime;

    // read acceleration and gyroscope values
    Giro_AccY = mpu.getAccelerationY();
    Giro_AccZ = mpu.getAccelerationZ();
    Giro_RotX = mpu.getRotationX();


    Giro_AccAngle = atan2(Giro_AccY, Giro_AccZ)*RAD_TO_DEG;

    Giro_Rate = map(Giro_RotX, -32768, 32767, -250, 250);
    Giro_Angle = (float)Giro_Rate*Giro_loopTime/1000;

    Giro_FilteredAngle = 0.9934*(Giro_PrevAngle + Giro_Angle) + 0.0066*(Giro_AccAngle);

    Giro_PrevAngle = Giro_FilteredAngle;

    Giro_PidCtl.Input=-Giro_FilteredAngle;

}

void PrintGiroRawData()
{
    Serial.print("GRD: ");
    Serial.print(Giro_AccY);
    Serial.print(" ");
    Serial.print(Giro_AccZ);
    Serial.print(" ");
    Serial.print(Giro_RotX);

    Serial.println(" ");
}

void PrintGiroInputOutputValues()
{
    Serial.print("GIO: ");
    Serial.print(Giro_PidCtl.Input);
    Serial.print(" ");
    Serial.print(Giro_PidCtl.Output);

    Serial.println(" ");
}



void setup()
{
    Serial.begin(250000);
    Serial.println("Setup:Serial Initialized");


    //CalibSetup();
    //Sensor readings with offsets:	-3	-5	16388	0	2	-1
    //Your offsets:	501	1553	1320	150	-6	19
    //Data is printed as: acelX acelY acelZ giroX giroY giroZ
    mpu.initialize();
    mpu.setXAccelOffset(501);
    mpu.setYAccelOffset(1553);
    mpu.setZAccelOffset(1320);
    mpu.setXGyroOffset(150);
    mpu.setYGyroOffset(-6);
    mpu.setZGyroOffset(19);
    Serial.println("Setup: Mpu6050 Initialized");

    Mot0_Init();
    Mot1_Init();
    Motors_SetSpeed(0,0);
    Serial.println("Setup: Motors Initialized");

    Enc_Init();
    Serial.println("Setup: Encoders Initialized");

    PrintPidCfg(&Mot0_PidCtl);
    PrintPidCfg(&Mot1_PidCtl);
    PrintPidCfg(&Giro_PidCtl);

    Mot0_Pid.SetSampleTime(PID__MOTOR_SAMPLE_TIME);
    Mot0_Pid.SetOutputLimits(-255,255);
    Mot0_Pid.SetMode(AUTOMATIC);

    Mot1_Pid.SetSampleTime(PID__MOTOR_SAMPLE_TIME);
    Mot1_Pid.SetOutputLimits(-255,255);
    Mot1_Pid.SetMode(AUTOMATIC);

    Giro_Pid.SetSampleTime(PID__MOTOR_SAMPLE_TIME);

    Giro_Pid.SetMode(AUTOMATIC);

#ifndef DOUBLE_ENC_INT
    Giro_Pid.SetOutputLimits(-39,39); //min and max enc speed for a sample time of 10 ms
#else
    Giro_Pid.SetOutputLimits(-39,39); //min and max enc speed for a sample time of 10 ms
#endif

    Serial.println("Setup: Pids initialized");

    Serial.println("Setup: Done");
}

unsigned long loopTime;

float self_balance_pid_setpoint=0;

void loop()
{
    loopTime=micros();


    unsigned long currentMillis = millis();
    if (currentMillis - previousMilliEnc >= intervalEnc)
        {
            previousMilliEnc=currentMillis;
            UpdateEncoderValues();
        }


    Giro_ReadData();

    Giro_PidCtl.Input+=self_balance_pid_setpoint;
    Giro_Pid.Compute();

    if (connectGiroToMot==true)
        {
            Mot0_PidCtl.Setpoint=Giro_PidCtl.Output;
            Mot1_PidCtl.Setpoint=Giro_PidCtl.Output;
        }


    Mot0_Pid.Compute();
    Mot1_Pid.Compute();

    if  ( (running==false) &&
            (Giro_PidCtl.Input > targetAngle-3 ) &&
            (Giro_PidCtl.Input < targetAngle+3 )
        )
        {
            running = true;
        }




    if((connectGiroToMot==true) &&
            ((running==false) ||
             (Giro_PidCtl.Input > 50 || Giro_PidCtl.Input < -50) || //we fell
             ((Giro_PidCtl.Output >1) && (Giro_PidCtl.Output < 1))) //somewhat echilibrum
      )
        {
            Motors_SetSpeed(0,0);
            running=false;
            self_balance_pid_setpoint = 0;
        }
    else
        {
            Motors_SetSpeed(Mot0_PidCtl.Output,Mot1_PidCtl.Output);
            //if(Mot0_PidCtl.Output < -1)self_balance_pid_setpoint += 0.001;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
            //if(Mot0_PidCtl.Output > 1)self_balance_pid_setpoint -= 0.001;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
            //if(Mot1_PidCtl.Output < -1)self_balance_pid_setpoint += 0.001;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
            //if(Mot1_PidCtl.Output > 1)self_balance_pid_setpoint -= 0.001;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards

        }




    if (currentMillis - previousMilliSerialLog >= intervalSerialLog)
        {
            previousMilliSerialLog=currentMillis;

            if (printRawGiro==true)
                {
                    PrintGiroRawData();
                }

            if (printGiroPid==true)
                {
                    PrintGiroInputOutputValues();
                }

            if(printMotPid==true)
                {
                    PrintEncoderValues();
                    PrintPidOutputValues();
                }

            Ser_ReceiveData();
            Ser_ParseData();

            loopTime=micros()-loopTime;
            Serial.print("MC: ");
            Serial.println(loopTime);
        }


}
