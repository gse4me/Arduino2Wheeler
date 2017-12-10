#include "MOT_lib.h"
#include "pwm_basic.h"


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


#ifdef MOT__USE_SLOW_DECAY
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

#ifdef MOT__USE_SLOW_DECAY
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

