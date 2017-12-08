#include "Motors.h"
#include "pwm_basic.h"
#include "../Configs.h"

void Mot0_Init(void)
{
    PWM_0_init();

    pinMode(PD5Pin,OUTPUT);
    pinMode(PD6Pin,OUTPUT);
    Motor0_Stop();
}

void Mot1_Init(void)
{
    PWM_1_init();

    pinMode(PB3Pin,OUTPUT);
    pinMode(PD3Pin,OUTPUT);
    Motor1_Stop();
}

void Motor0_Stop()
{
    PWM_0_disable_output_ch1();
    PWM_0_disable_output_ch0();
    PORTD &= ~(1<<PORTD5);
    PORTD &= ~(1<<PORTD6);
}

void Motor1_Stop()
{
    PWM_1_disable_output_ch1();
    PWM_1_disable_output_ch0();
    PORTD &= ~(1<<PORTD3);
    PORTB &= ~(1<<PORTB3);
}


#ifdef SLOW_DRAIN
void Motor0_SetSpeed(uint8_t dir,uint8_t speed)
{
    if (dir==FWD)
        {
            PWM_0_enable_output_ch1();
            PWM_0_load_duty_cycle_ch1(255-speed);

            PWM_0_disable_output_ch0();
            PORTD |= (1<<PORTD6);
        }
    else if (dir==REV)
        {
            PWM_0_enable_output_ch0();
            PWM_0_load_duty_cycle_ch0(255-speed);

            PWM_0_disable_output_ch1();
            PORTD |= (1<<PORTD5);
        }
}
#else
void Motor0_SetSpeed(uint8_t dir,uint8_t speed)
{
    if (dir==FWD)
        {
            PWM_0_disable_output_ch1();
            PORTD &= ~(1<<PORTD5);

            PWM_0_enable_output_ch0();
            PWM_0_load_duty_cycle_ch0(speed);
        }
    else if (dir==REV)
        {
            PWM_0_disable_output_ch0();
            PORTD &= ~(1<<PORTD6);

            PWM_0_enable_output_ch1();
            PWM_0_load_duty_cycle_ch1(speed);
        }
}
#endif

#ifdef SLOW_DRAIN
void Motor1_SetSpeed(uint8_t dir,uint8_t speed)
{
    if (dir==FWD)
        {
            PWM_1_enable_output_ch1();
            PWM_1_load_duty_cycle_ch1(255-speed);

            PWM_1_disable_output_ch0();
            PORTB |= (1<<PORTB3);
        }
    else if (dir==REV)
        {
            PWM_1_enable_output_ch0();
            PWM_1_load_duty_cycle_ch0(255-speed);

            PWM_1_disable_output_ch1();
            PORTD |= (1<<PORTD3);
        }
}

#else
void Motor1_SetSpeed(uint8_t dir,uint8_t speed)
{
    if (dir==FWD)
        {
            PWM_1_disable_output_ch1();
            PORTD &= ~(1<<PORTD3);

            PWM_1_enable_output_ch0();
            PWM_1_load_duty_cycle_ch0(speed);
        }
    else if (dir==REV)
        {
            PWM_1_disable_output_ch0();
            PORTB &= ~(1<<PORTB3);

            PWM_1_enable_output_ch1();
            PWM_1_load_duty_cycle_ch1(speed);
        }
}
#endif

void Motors_SetSpeed(int leftMotorSpeed, int rightMotorSpeed)
{

    if(leftMotorSpeed >= 0)
        {
            Motor0_SetSpeed(REV,leftMotorSpeed);
        }
    else
        {
            Motor0_SetSpeed(FWD, abs(leftMotorSpeed));
        }

    if(rightMotorSpeed >= 0)
        {
            Motor1_SetSpeed(REV,rightMotorSpeed);
        }
    else
        {
            Motor1_SetSpeed(FWD,abs(rightMotorSpeed));
        }
}


void Enc_Init()
{
    pinMode(A1,INPUT_PULLUP);
    pinMode(A0,INPUT_PULLUP);
    pinMode(13,INPUT_PULLUP);//pinMode(13,INPUT_PULLUP); //led
    pinMode(12,INPUT_PULLUP);

    PCICR = (1 << PCIE0) | // Enable pin change interrupt 0 {PCINT[14:8]}
            (1 << PCIE1) ; // Enable pin change interrupt 1 {PCINT[7:0]}


    PCMSK0 = (1 << PCINT4); // Pin change enable mask 5
    PCMSK1 = (1 << PCINT9); // Pin change enable mask 9
#ifdef DOUBLE_ENC_INT
    PCMSK0 |= (1 << PCINT5); // Pin change enable mask 5
    PCMSK1 |= (1 << PCINT8); // Pin change enable mask 9
#endif
}


long Mot0_Enc=0;
long Mot1_Enc=0;

uint8_t Mot0_Dir=0;
uint8_t Mot1_Dir=0;

#ifndef DOUBLE_ENC_INT
ISR(PCINT0_vect)
{
    if(bitRead(PINB,5) == bitRead(PINB,4))
        {
            Mot1_Enc++;
            Mot1_Dir=((Mot1_Dir<<1) | 1);
        }
    else
        {
            Mot1_Enc--;
            Mot1_Dir=(Mot1_Dir<<1);
        }
}

ISR(PCINT1_vect)
{
    if(bitRead(PINC,1) == bitRead(PINC,0))
        {
            Mot0_Enc++;
            Mot0_Dir=((Mot0_Dir<<1) | 1);
        }
    else
        {
            Mot0_Enc--;
            Mot0_Dir=(Mot0_Dir<<1);
        }
}
#else

uint8_t Mot1_state=00;
uint8_t Mot1_p1Val;
uint8_t Mot1_p2Val;
uint8_t Mot1_tmpState;

uint8_t Mot0_state=00;
uint8_t Mot0_p1Val;
uint8_t Mot0_p2Val;
uint8_t Mot0_tmpState;



ISR(PCINT0_vect)
{
    Mot1_p1Val = bitRead(PINB,5);
    Mot1_p2Val = bitRead(PINB,4);
    Mot1_tmpState = Mot1_state & 3;

    if (Mot1_p1Val) Mot1_tmpState |= 8;
    if (Mot1_p2Val) Mot1_tmpState |= 4;

    Mot1_state = (Mot1_tmpState >> 2);
    switch (Mot1_tmpState)
        {
        case 1:
        case 7:
        case 8:
        case 14:
            Mot1_Enc++;
            return;
        case 2:
        case 4:
        case 11:
        case 13:
            Mot1_Enc--;
            return;
        case 3:
        case 12:
            Mot1_Enc += 2;
            return;
        case 6:
        case 9:
            Mot1_Enc -= 2;
            return;
        }
}

ISR(PCINT1_vect)
{
    Mot0_p1Val = bitRead(PINC,1);
    Mot0_p2Val = bitRead(PINC,0);
    Mot0_tmpState = Mot0_state & 3;

    if (Mot0_p1Val) Mot0_tmpState |= 4;
    if (Mot0_p2Val) Mot0_tmpState |= 8;

    Mot0_state = (Mot0_tmpState >> 2);
    switch (Mot0_tmpState)
        {
        case 1:
        case 7:
        case 8:
        case 14:
            Mot0_Enc++;
            return;
        case 2:
        case 4:
        case 11:
        case 13:
            Mot0_Enc--;
            return;
        case 3:
        case 12:
            Mot0_Enc += 2;
            return;
        case 6:
        case 9:
            Mot0_Enc -= 2;
            return;
        }
}
#endif