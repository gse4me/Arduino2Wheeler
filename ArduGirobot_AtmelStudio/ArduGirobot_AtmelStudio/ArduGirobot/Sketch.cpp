#include <Arduino.h>
#include <MPU6050.h>
#include <string.h>

#include "MOT_lib.h"
#include "ENC_lib.h"

#include "NewPing.h"
#include "MPU6050Calib.h"
#include "PID_v1.h"
#include "Configs.h"
#include "Utils.h"

//------------------------------ SONAR --------------------------------------------------//
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
int distanceCm;

//----------------------------- REMOTE CONTROL --------------------------------//
bool RemoteDirUp=false;
bool RemoteDirDn=false;
bool RemoteDirRight=false;
bool RemoteDirLeft=false;
bool RemoteBtnZ=false;
bool RemoteBtnC=false;
uint16_t RemoteControlByte=0;
unsigned long RemoteCmdRecTime=0;

uint8_t turning_speed=6;

//---------------------------------------------------------------------------------------//

float self_balance_pid_setpoint=0;


//--------------------------- STATISTICS --------------------------------------//
unsigned long normalLoopTime=0;
unsigned long serialLoopTime=0;


//-------------------------- GIRO DATA -------------------------------------//
MPU6050 mpu;
int16_t Giro_AccY;
int16_t Giro_AccZ;
int16_t Giro_RotX;

volatile int motorPower;
volatile int Giro_Rate;
volatile float Giro_AccAngle;
volatile float Giro_Angle;
volatile float Giro_FilteredAngle;
volatile float Giro_PrevAngle=0;
volatile float error;
volatile float prevError=0;
volatile float errorSum=0;

//-------------------------- LOGGING FLAGS -----------------------------//
bool ConnectGiroToMot=true;
bool PrintPidCfgValues=true;
bool PrintP1CfgValues=true;
bool PrintP2CfgValues=true;
bool PrintP3CfgValues=true;
bool PrintCycleTimeStats=true;

//=-------------------- OTHER FLAGS -----------------------------------//
bool stateRunning=true;
bool stateFallen=false;
bool stateEquilibrum=true;

bool getUp=false;
//----------------------- PID STRUCTURES -----------------------------//
PIDConfig_e *Mot0_PidCtl;
PIDConfig_e *Mot1_PidCtl;
PIDConfig_e *Giro_PidCtl;

PID *Mot0_Pid;
PID *Mot1_Pid;
PID *Giro_Pid;


//------------------------------------------------------------------------- READ AND WRITE PIDS TO EEPROM -----------------------------------------------------------------------//
void getPidConfigsFromEEPROM() {
    uint16_t addr=EEPROM_PID_CONFIGS_ADDR;

    ReadPidCfgFromEEPROM(addr,Mot0_PidCtl);
    addr+=3*sizeof(double);
    ReadPidCfgFromEEPROM(addr,Mot1_PidCtl);
    addr+=3*sizeof(double);
    ReadPidCfgFromEEPROM(addr,Giro_PidCtl);
}


void putPidConfigToEEPROM() {

    uint16_t addr=EEPROM_PID_CONFIGS_ADDR;

    WritePidCfgToEEPROM(addr,Mot0_PidCtl);
    addr+=3*sizeof(double);
    WritePidCfgToEEPROM(addr,Mot1_PidCtl);
    addr+=3*sizeof(double);
    WritePidCfgToEEPROM(addr,Giro_PidCtl);
}


//-------------------------------------------------------------------------- PID INITIALIZATION ----------------------------------------------------------------------------------------------//
void initPidControls() {
    Mot0_PidCtl= new PIDConfig_e(MOT0__KP,MOT0__KI,MOT0__KD,0);
    Mot1_PidCtl= new PIDConfig_e(MOT1__KP,MOT1__KI,MOT1__KD,0);
    Giro_PidCtl= new PIDConfig_e(GIRO__KP,GIRO__KI,GIRO__KD,GIRO__TARGET_ANGLE);

#ifdef LOAD_PIDS_FROM_EEPROM
    getPidConfigsFromEEPROM();
#endif

    Mot0_Pid= new PID(&Mot0_PidCtl->Input, &Mot0_PidCtl->Output, &Mot0_PidCtl->Setpoint, Mot0_PidCtl->Kp, Mot0_PidCtl->Ki, Mot0_PidCtl->Kd, P_ON_E, DIRECT);
    Mot1_Pid= new PID(&Mot1_PidCtl->Input, &Mot1_PidCtl->Output, &Mot1_PidCtl->Setpoint, Mot1_PidCtl->Kp, Mot1_PidCtl->Ki, Mot1_PidCtl->Kd, P_ON_E, DIRECT);
    Giro_Pid= new PID(&Giro_PidCtl->Input, &Giro_PidCtl->Output, &Giro_PidCtl->Setpoint, Giro_PidCtl->Kp, Giro_PidCtl->Ki, Giro_PidCtl->Kd, P_ON_E, DIRECT);

    Mot0_Pid->SetSampleTime(MOT0__SAMPLE_TIME);
    Mot1_Pid->SetSampleTime(MOT1__SAMPLE_TIME);
    Giro_Pid->SetSampleTime(GIRO__SAMPLE_TIME);

    Mot0_Pid->SetOutputLimits(MOT0__OUT_MIN,MOT0__OUT_MAX);
    Mot1_Pid->SetOutputLimits(MOT1__OUT_MIN,MOT1__OUT_MAX);
    Giro_Pid->SetOutputLimits(GIRO__OUT_MIN,GIRO__OUT_MAX); //min and max enc speed for a sample time of 10 ms

    Mot0_Pid->SetMode(AUTOMATIC);
    Mot1_Pid->SetMode(AUTOMATIC);
    Giro_Pid->SetMode(AUTOMATIC);

}

//-------------------------------------------------------------------------------- Remote Control --------------------------------------------------------------------------------------//

void ParseRemoteControlByte(uint8_t ctrl) {
    if (ctrl & 1) {
        RemoteBtnC=true;
    }

    if (ctrl & 2) {
        RemoteBtnZ=true;
    }

    if (ctrl & 4) {
        RemoteDirLeft=true;
    }

    if (ctrl & 8) {
        RemoteDirRight=true;
    }

    if (ctrl & 16) {
        RemoteDirUp=true;
    }

    if (ctrl & 32) {
        RemoteDirDn=true;
    }
}

//-------------------------------------------------------------------------------- SERIAL DATA LOGGING --------------------------------------------------------------------------//
void Serial_PrintStats() {
    static uint8_t i=ARD_START_PRINT_IDX;
    static uint8_t old_i=ARD_START_PRINT_IDX;

    if (Serial.availableForWrite()<6) {
        return;
    }
    old_i=i;

    if (PrintP1CfgValues==true) {
        switch(i) {
        case ARD_PID1_INPUT:
            Serial.write(i++);
            Serial.println(Mot0_PidCtl->Input);
            break;
        case ARD_PID1_OUTPUT:
            Serial.write(i++);
            Serial.println(Mot0_PidCtl->Output);
            break;
        case ARD_PID1_SETPOINT:
            Serial.write(i);
            Serial.println(Mot0_PidCtl->Setpoint);
            i=ARD_START_PRINT_IDX;
            break;
        }
    }

    if (PrintP2CfgValues==true) {
        if (i==ARD_START_PRINT_IDX) {
            i=ARD_PID2_INPUT;
            return;
        }

        switch(i) {
        case ARD_PID2_INPUT:
            Serial.write(i++);
            Serial.println(Mot1_PidCtl->Input);
            break;
        case ARD_PID2_OUTPUT:
            Serial.write(i++);
            Serial.println(Mot1_PidCtl->Output);
            break;
        case ARD_PID2_SETPOINT:
            Serial.write(i);
            Serial.println(Mot1_PidCtl->Setpoint);
            i=ARD_START_PRINT_IDX;
            break;
        }
    }

    if (PrintP3CfgValues==true) {

        if (i==ARD_START_PRINT_IDX) {
            i=ARD_PID3_INPUT;
            return;
        }

        switch(i) {
        case ARD_PID3_INPUT:
            Serial.write(i++);
            Serial.println(Giro_PidCtl->Input);
            break;
        case ARD_PID3_OUTPUT:
            Serial.write(i++);
            Serial.println(Giro_PidCtl->Output);
            break;
        case ARD_PID3_SETPOINT:
            Serial.write(i);
            Serial.println(Giro_PidCtl->Setpoint);
            i=ARD_START_PRINT_IDX; //go to pid cfg
            break;
        }
    }

    if (PrintPidCfgValues==true) {
        if (i==ARD_START_PRINT_IDX) {
            i=ARD_PID1_KP;
            return;
        }

        switch(i) {
        case ARD_PID1_KP:
            Serial.write(i++);
            Serial.println(Mot0_PidCtl->Kp);
            break;
        case ARD_PID1_KI:
            Serial.write(i++);
            Serial.println(Mot0_PidCtl->Ki);
            break;
        case ARD_PID1_KD:
            Serial.write(i++);
            Serial.println(Mot0_PidCtl->Kd);
            break;
        case ARD_PID2_KP:
            Serial.write(i++);
            Serial.println(Mot1_PidCtl->Kp);
            break;
        case ARD_PID2_KI:
            Serial.write(i++);
            Serial.println(Mot1_PidCtl->Ki);
            break;
        case ARD_PID2_KD:
            Serial.write(i++);
            Serial.println(Mot1_PidCtl->Kd);
            break;
        case ARD_PID3_KP:
            Serial.write(i++);
            Serial.println(Giro_PidCtl->Kp);
            break;
        case ARD_PID3_KI:
            Serial.write(i++);
            Serial.println(Giro_PidCtl->Ki);
            break;
        case ARD_PID3_KD:
            Serial.write(i);
            Serial.println(Giro_PidCtl->Kd);
            i=ARD_START_PRINT_IDX; //go to start
            PrintPidCfgValues=false; // don't print them again unless requested
            break;
        }
    }

    if (PrintCycleTimeStats==true) {
        if (i==ARD_START_PRINT_IDX) {
            i=ARD_NORMAL_LOOP_TIME;
            return;
        }

        switch(i) {
        case ARD_NORMAL_LOOP_TIME:
            Serial.write(i++);
            Serial.println(normalLoopTime);
            break;
        case ARD_SERIAL_LOOP_TIME:
            Serial.write(i);
            Serial.println(serialLoopTime);
            i=ARD_START_PRINT_IDX; //go to pid cfg
            break;
        }
    }

    if (old_i==i) { // if we get stuck (something gets set to false
        i=ARD_START_PRINT_IDX;
    }
}

//---------------------------------------------------------------------------------  SERIAL READ --------------------------------------------------------------------------------//
const char Ser_EndMarker = '\n';
char Ser_Buffer[SERIAL__MY_BUF_SIZE]; // an array to store the received data
boolean Ser_NewData = false;

void Serial_LookForData() {
    static byte ndx = 0;
    char rc;

    // if (Serial.available() > 0) {
    while (Ser_NewData == false && Serial.available() > 0) {
        rc = Serial.read();

        if (rc != Ser_EndMarker) {
            Ser_Buffer[ndx] = rc;
            ndx++;
            if (ndx >= SERIAL__MY_BUF_SIZE) {
                ndx = SERIAL__MY_BUF_SIZE - 1;
            }
        } else {
            Ser_Buffer[ndx] = '\0'; // terminate the string
            ndx = 0;
            Ser_NewData = true;
        }
    }
}

//---------------------------------------------------------------- SERIAL CMD PROCESSING -----------------------------------------------------------------------------//
void Serial_ParseData() {
    if (Ser_NewData==false) return;

    uint8_t target=Ser_Buffer[0];
    char *val=Ser_Buffer+1;
    double dval;
    if (val[0]!='\n') {
        dval=atof(val);
    }

    switch(target) {
    case CUTE_PID1_KP:
        Mot0_PidCtl->Kp=dval;
        Mot0_Pid->SetTunings(Mot0_PidCtl->Kp,Mot0_PidCtl->Ki,Mot0_PidCtl->Kd);
        break;
    case CUTE_PID1_KI:
        Mot0_PidCtl->Ki=dval;
        Mot0_Pid->SetTunings(Mot0_PidCtl->Kp,Mot0_PidCtl->Ki,Mot0_PidCtl->Kd);
        break;
    case CUTE_PID1_KD:
        Mot0_PidCtl->Kd=dval;
        Mot0_Pid->SetTunings(Mot0_PidCtl->Kp,Mot0_PidCtl->Ki,Mot0_PidCtl->Kd);
        break;
    case CUTE_PID2_KP:
        Mot1_PidCtl->Kp=dval;
        Mot1_Pid->SetTunings(Mot1_PidCtl->Kp,Mot1_PidCtl->Ki,Mot1_PidCtl->Kd);
        break;
    case CUTE_PID2_KI:
        Mot1_PidCtl->Ki=dval;
        Mot1_Pid->SetTunings(Mot1_PidCtl->Kp,Mot1_PidCtl->Ki,Mot1_PidCtl->Kd);
        break;
    case CUTE_PID2_KD:
        Mot1_PidCtl->Kd=dval;
        Mot1_Pid->SetTunings(Mot1_PidCtl->Kp,Mot1_PidCtl->Ki,Mot1_PidCtl->Kd);
        break;
    case CUTE_PID3_KP:
        Giro_PidCtl->Kp=dval;
        Giro_Pid->SetTunings(Giro_PidCtl->Kp,Giro_PidCtl->Ki,Giro_PidCtl->Kd);
        break;
    case CUTE_PID3_KI:
        Giro_PidCtl->Ki=dval;
        Giro_Pid->SetTunings(Giro_PidCtl->Kp,Giro_PidCtl->Ki,Giro_PidCtl->Kd);
        break;
    case CUTE_PID3_KD:
        Giro_PidCtl->Kd=dval;
        Giro_Pid->SetTunings(Giro_PidCtl->Kp,Giro_PidCtl->Ki,Giro_PidCtl->Kd);
        break;
    case CUTE_PID1_SETP:
        Mot0_PidCtl->Setpoint=dval;
        break;
    case CUTE_PID2_SETP:
        Mot1_PidCtl->Setpoint=dval;
        break;
    case CUTE_PID3_SETP:
        Giro_PidCtl->Setpoint=dval;
        break;
    case CUTE_P1_PRNT_ON:
        PrintP1CfgValues=true;
        break;
    case CUTE_P1_PRNT_OFF:
        PrintP1CfgValues=false;
        break;
    case CUTE_P2_PRNT_ON:
        PrintP2CfgValues=true;
        break;
    case CUTE_P2_PRNT_OFF:
        PrintP2CfgValues=false;
        break;
    case CUTE_P3_PRNT_ON:
        PrintP3CfgValues=true;
        break;
    case CUTE_P3_PRNT_OFF:
        PrintP3CfgValues=false;
        break;
    case CUTE_GIRO_TO_MOT_ON:
        ConnectGiroToMot=true;
        break;
    case CUTE_GIRO_TO_MOT_OFF:
        ConnectGiroToMot=false;
        break;
    case CUTE_GET_ALL_PID_CFGS:
        PrintPidCfgValues=true;
        break;
    case CUTE_SAVE_TO_EEPROM:
        putPidConfigToEEPROM();
        break;
    case CUTE_GET_UP:
        getUp=true;
        break;
    case CUTE_CYCLE_TIME_PRINTS_ON:
        PrintCycleTimeStats=true;
        break;
    case CUTE_CYCLE_TIME_PRINTS_OFF:
        PrintCycleTimeStats=false;
        break;
    case REMOTE_CONTROL_BYTE:
        ParseRemoteControlByte(val[0]);
        break;
    default:
        break;

    }

    Ser_NewData=false;
}

//----------------------------------------------------------------------------- SERIAL TASK------------------------------------------------------------------------------------//
bool SerialCom() {
    static unsigned long previousMilliSerialLog=0;
    unsigned long currentMillis = millis();

    if (currentMillis - previousMilliSerialLog >= SERIAL_LOG_INTERVAL) {
        previousMilliSerialLog=currentMillis;

        Serial_LookForData();
        Serial_ParseData();
        Serial_PrintStats();
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------------ ENCODER VALUE UPDATE --------------------------------------//
void UpdateEncoderValues() {
    static unsigned long previousMilliEnc = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - previousMilliEnc >= ENC__SAMPLE_TIME) {
        previousMilliEnc=currentMillis;

        noInterrupts();
        Mot0_PidCtl->Input=Mot0_Enc;
        Mot1_PidCtl->Input=Mot1_Enc;

        Mot0_Enc=0;
        Mot1_Enc=0;
        interrupts();
    }
}

//--------------------------------------------------------------------------------- GIRO UPDATE ---------------------------------------------------------//

void Giro_ReadData() {
    static unsigned long Giro_prevTime=0;


    unsigned long Giro_currTime = millis();
    unsigned long Giro_loopTime = Giro_currTime - Giro_prevTime;
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

    Giro_PidCtl->Input=-Giro_FilteredAngle;

}



void setup() {
    Serial.begin(SERIAL__BAUD_RATE);
    LOG("Setup:Serial Initialized");


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
    LOG("Setup: Mpu6050 Initialized");

    Mot0_Init();
    Mot1_Init();
    Motors_SetSpeed(0,0);
    LOG("Setup: Motors Initialized");

    Enc_Init();
    LOG("Setup: Encoders Initialized");

    initPidControls();
    LOG("Setup: PID initialized");

    LOG("Setup: Done");
}


unsigned long cmdLife=30;
unsigned long cmdStartTime=0;
bool goUp=false;
bool goDn=false;
bool goNormal=false;
bool goLeft=false;
bool goRight=false;

void loop() {
    unsigned long enterTime=millis();
    //-------------------------------------------------------------

    if (!ConnectGiroToMot) { //motors controlled by pc
        Giro_ReadData();
        Giro_Pid->Compute();
        UpdateEncoderValues();
        Mot0_Pid->Compute();
        Mot1_Pid->Compute();
    } else { //motors controlled by giro -> we are running


        if (RemoteBtnZ == true) {
            getUp = true;
            RemoteBtnZ = false;
        }

        //Directions
        if(enterTime-cmdStartTime>cmdLife) {
            goUp=false;
            goDn=false;
            goLeft=false;
            goRight=false;

            goNormal=true;
        }


        if(RemoteDirUp == true) {
            RemoteDirUp = false;
            goNormal = false;
            goUp = true;
            cmdStartTime=enterTime;
        }
        if(RemoteDirDn == true) {
            RemoteDirDn = false;
            goNormal = false;
            goDn = true;
            cmdStartTime=enterTime;
        }

        if(RemoteDirLeft == true) {
            RemoteDirLeft = false;
            goLeft = true;
            cmdStartTime=enterTime;
        }
        if(RemoteDirRight == true) {
            RemoteDirRight = false;
            goRight = true;
            cmdStartTime=enterTime;
        }



        //target angle
        Giro_ReadData();
        if(goUp==true) {
            if(Giro_PidCtl->Setpoint > GIRO__TARGET_ANGLE - 19) Giro_PidCtl->Setpoint -= 0.1;
            //  if(Giro_PidCtl->Output > 20 * -1)Giro_PidCtl->Setpoint -= 0.05;
        }
        if(goDn == true) {
            if(Giro_PidCtl->Setpoint < GIRO__TARGET_ANGLE + 19) Giro_PidCtl->Setpoint += 0.1;
            // if(Giro_PidCtl->Output < 20) Giro_PidCtl->Setpoint += 0.05;
        }

        if(goNormal==true) {
            if(Giro_PidCtl->Setpoint > GIRO__TARGET_ANGLE + 3) Giro_PidCtl->Setpoint -= 0.1;
            else if(Giro_PidCtl->Setpoint < GIRO__TARGET_ANGLE-3)Giro_PidCtl->Setpoint += 0.1;
            else {
                Giro_PidCtl->Setpoint = GIRO__TARGET_ANGLE;
                goNormal=false;
            }

        }


        Giro_Pid->Compute();
        Mot0_PidCtl->Setpoint=Giro_PidCtl->Output;
        Mot1_PidCtl->Setpoint=Giro_PidCtl->Output;
        UpdateEncoderValues();

        if(goLeft==true) {
            // Mot0_PidCtl->Input -= turning_speed;
            Mot1_PidCtl->Input += turning_speed;
        }
        if(goRight==true) {
            Mot0_PidCtl->Input += turning_speed;
            // Mot1_PidCtl->Input -= turning_speed;
        }

        Mot0_Pid->Compute();
        Mot1_Pid->Compute();





        //decisions based on angle
        if (stateRunning==false || Giro_PidCtl->Input > 50 || Giro_PidCtl->Input < -50) { //we fell
            stateRunning=false;
            Mot0_PidCtl->Output=0;
            Mot1_PidCtl->Output=0;

            Mot0_Pid->SetMode(MANUAL);
            Mot1_Pid->SetMode(MANUAL);
            Giro_Pid->SetMode(MANUAL);
        }

        if (stateRunning==false && (Giro_PidCtl->Input > Giro_PidCtl->Setpoint -3)  && (Giro_PidCtl->Input < Giro_PidCtl->Setpoint +3)) {
            stateRunning = true;

            Mot0_Pid->SetMode(AUTOMATIC);
            Mot1_Pid->SetMode(AUTOMATIC);
            Giro_Pid->SetMode(AUTOMATIC);

            getUp=false;
        }

        if (stateRunning==true && getUp==true) {
            getUp=false;
        }

        if (getUp==true) {
            if (Giro_PidCtl->Input>0) {
                Mot0_PidCtl->Output=-255;
                Mot1_PidCtl->Output=-255;
            } else {
                Mot0_PidCtl->Output=255;
                Mot1_PidCtl->Output=255;
            }
        }
    }

    Motors_SetSpeed(Mot0_PidCtl->Output,Mot1_PidCtl->Output);

    //------------------------------------------------------------
    bool serialCycle=SerialCom();

    unsigned long exitTime=millis();
    if (serialCycle==true) {
        serialLoopTime=exitTime-enterTime;
    } else {
        normalLoopTime=exitTime-enterTime;
    }
}


