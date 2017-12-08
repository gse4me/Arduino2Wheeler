
#ifndef _PWM_BASIC_H_INCLUDED
#define _PWM_BASIC_H_INCLUDED

#include <arduino.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef uint8_t PWM_0_register_t;

int8_t PWM_0_init(void);
void PWM_0_enable();
void PWM_0_disable();
void PWM_0_enable_output_ch0();
void PWM_0_disable_output_ch0();
void PWM_0_enable_output_ch1();
void PWM_0_disable_output_ch1();


void PWM_0_load_counter(PWM_0_register_t counter_value);
void PWM_0_load_duty_cycle_ch0(PWM_0_register_t duty_value);
void PWM_0_load_duty_cycle_ch1(PWM_0_register_t duty_value);



typedef uint8_t PWM_1_register_t;


int8_t PWM_1_init(void);
void PWM_1_enable();
void PWM_1_disable();
void PWM_1_enable_output_ch0();
void PWM_1_disable_output_ch0();
void PWM_1_enable_output_ch1();
void PWM_1_disable_output_ch1();
void PWM_1_load_counter(PWM_1_register_t counter_value);
void PWM_1_load_duty_cycle_ch0(PWM_1_register_t duty_value);
void PWM_1_load_duty_cycle_ch1(PWM_1_register_t duty_value);


#ifdef __cplusplus
}
#endif

#endif /* _PWM_BASIC_H_INCLUDED */
