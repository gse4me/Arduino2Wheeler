#include <pwm_basic.h>


int8_t PWM_0_init()
{

    /* Enable TC0 */
    PRR &= ~(1 << PRTIM0);

    TCCR0A = (1 << COM0A1) | (0 << COM0A0)   /* Clear OCA on Compare Match, set OCA on BOTTOM (non-inverting mode) */
             | (1 << COM0B1) | (0 << COM0B0) /* Clear OCB on Compare Match, set OCB on BOTTOM (non-inverting mode) */
             | (1 << WGM01) | (1 << WGM00);  /* TC8 Mode 3 Fast PWM */

    TCCR0B = 0                                          /* TC8 Mode 3 Fast PWM */
             | (0 << CS02) | (1 << CS01) | (0 << CS00); /* IO clock divided by 8 */

    TIMSK0 = 0 << OCIE0B   /* Output Compare B Match Interrupt Enable: disabled */
             | 0 << OCIE0A /* Output Compare A Match Interrupt Enable: disabled */
             | 1 << TOIE0; /* Overflow Interrupt Enable: enabled */

    OCR0A = 0; /* Output compare A: 50 */

    OCR0B = 0; /* Output compare B: 50 */

    return 0;
}

void PWM_0_enable()
{
}

void PWM_0_disable()
{
}

inline void PWM_0_enable_output_ch0()
{

    TCCR0A |= ((1 << COM0A1) | (0 << COM0A0));
}

inline void PWM_0_disable_output_ch0()
{

    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
}

inline void PWM_0_enable_output_ch1()
{

    TCCR0A |= ((1 << COM0B1) | (0 << COM0B0));
}

inline void PWM_0_disable_output_ch1()
{

    TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
}

inline void PWM_0_load_counter(PWM_0_register_t counter_value)
{
    TCNT0 = counter_value;
}

inline void PWM_0_load_duty_cycle_ch0(PWM_0_register_t duty_value)
{
    OCR0A = duty_value;
}

inline void PWM_0_load_duty_cycle_ch1(PWM_0_register_t duty_value)
{
    OCR0B = duty_value;
}


///////////////////////////////////////////////////////////////////


int8_t PWM_1_init()
{

    /* Enable TC2 */
    PRR &= ~(1 << PRTIM2);

    TCCR2A = (1 << COM2A1) | (0 << COM2A0)   /* Clear OCA on Compare Match, set OCA on BOTTOM (non-inverting mode) */
             | (1 << COM2B1) | (0 << COM2B0) /* Clear OCB on Compare Match, set OCB on BOTTOM (non-inverting mode) */
             | (1 << WGM21) | (1 << WGM20);  /* TC8 Mode 3 Fast PWM */

    TCCR2B = 0                                          /* TC8 Mode 3 Fast PWM */
             | (0 << CS22) | (1 << CS21) | (0 << CS20); /* IO clock divided by 8 */

    TIMSK2 = 0 << OCIE2B   /* Output Compare B Match Interrupt Enable: disabled */
             | 0 << OCIE2A ;/* Output Compare A Match Interrupt Enable: disabled */
    //| 1 << TOIE2; /* Overflow Interrupt Enable: enabled */

    OCR2A = 0; /* Output compare A: 0 */

    OCR2B = 0; /* Output compare B: 0 */

    //ASSR = (1 << EXCLK); // Enable external clock

    return 0;
}

void PWM_1_enable()
{
}

void PWM_1_disable()
{
}

inline void PWM_1_enable_output_ch0()
{

    TCCR2A |= ((1 << COM2A1) | (0 << COM2A0));
}

inline void PWM_1_disable_output_ch0()
{

    TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0));
}

inline void PWM_1_enable_output_ch1()
{

    TCCR2A |= ((1 << COM2B1) | (0 << COM2B0));
}

inline void PWM_1_disable_output_ch1()
{

    TCCR2A &= ~((1 << COM2B1) | (1 << COM2B0));
}

inline void PWM_1_load_counter(PWM_1_register_t counter_value)
{
    TCNT2 = counter_value;
}

inline void PWM_1_load_duty_cycle_ch0(PWM_1_register_t duty_value)
{
    OCR2A = duty_value;
}

inline void PWM_1_load_duty_cycle_ch1(PWM_1_register_t duty_value)
{
    OCR2B = duty_value;
}
