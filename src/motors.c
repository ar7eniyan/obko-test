#include "motors.h"

#include "gpio.h"
#include "stm32h7xx.h"
#include "stm32h743xx.h"

#include "tools.h"
#include <stdint.h>


void setup_motors(void)
{
    SET_RCC_xxxxEN(RCC->APB2ENR,
        RCC_APB2ENR_TIM15EN | RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM17EN);

    // Load the optimal prescaler value for motor control
    TIM15->PSC = 6000 - 1;
    TIM16->PSC = 6000 - 1;
    TIM17->PSC = 6000 - 1;
    // Enable ARR preloading, accroding to the RM
    TIM15->CR1 = TIM_CR1_ARPE;
    TIM16->CR1 = TIM_CR1_ARPE;
    TIM17->CR1 = TIM_CR1_ARPE;
    // Capture/Compare channel 1 is set by default to output, set mode to PWM
    // mode 1, enable CCR1 preloading as said in the RM.
    TIM15->CCMR1 = (0b110 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM16->CCMR1 = (0b110 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM17->CCMR1 = (0b110 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    // Setup break functionality on rear wheel timers, enable output
    // (disable for now, TODO: pull down break input)
    TIM15->BDTR = TIM_BDTR_MOE;
    TIM16->AF1 = TIM16_AF1_BKINE | TIM16_AF1_BKINP;
    TIM16->BDTR = TIM_BDTR_MOE; // | TIM_BDTR_BKP | TIM_BDTR_BKE | TIM_BDTR_OSSI;
    TIM17->AF1 = TIM17_AF1_BKINE | TIM17_AF1_BKINP;
    TIM17->BDTR = TIM_BDTR_MOE; // | TIM_BDTR_BKP | TIM_BDTR_BKE | TIM_BDTR_OSSI;
    // Enable channel 1 non-inversed output
    TIM15->CCER = TIM_CCER_CC1E;
    TIM16->CCER = TIM_CCER_CC1E;
    TIM17->CCER = TIM_CCER_CC1E;
    // Start the counter (for later, now it's stopped because of ARR == 0)
    TIM15->CR1 |= TIM_CR1_CEN;
    TIM16->CR1 |= TIM_CR1_CEN;
    TIM17->CR1 |= TIM_CR1_CEN;

    SET_RCC_xxxxEN(RCC->AHB4ENR,
        RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN);

    GPIOB->OTYPER |= GPIO_OTYPER_OT4_Pos | GPIO_OTYPER_OT5_Pos |
        GPIO_OTYPER_OT8_Pos | GPIO_OTYPER_OT9_Pos;
    MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5,
        (0b0001 << GPIO_AFRL_AFSEL4_Pos) | (0b0001 << GPIO_AFRL_AFSEL5_Pos));
    MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9,
        (0b0001 << GPIO_AFRH_AFSEL8_Pos) | (0b0001 << GPIO_AFRH_AFSEL9_Pos));
    MODIFY_REG(GPIOB->MODER,
        GPIO_MODER_MODE4 | GPIO_MODER_MODE5 |
        GPIO_MODER_MODE8 | GPIO_MODER_MODE9,
        (0b10 << GPIO_MODER_MODE4_Pos) | (0b10 << GPIO_MODER_MODE5_Pos) |
        (0b10 << GPIO_MODER_MODE8_Pos) | (0b10 << GPIO_MODER_MODE9_Pos));

    gpio_setup_pin(GPIOE, 5,
        GPIO_FLAGS_MODE_AF | GPIO_FLAGS_OTYPE_OD | GPIO_FLAGS_AFSEL_AF4);
}

void motor_steering_write(uint16_t period_ticks)
{
    // The counter goes from 0 to the ARR!
    TIM15->ARR = period_ticks - 1;
    TIM15->CCR1 = period_ticks / 2;
}

void motor_rear_left_write(uint16_t period_ticks)
{
    TIM16->ARR = period_ticks - 1;
    TIM16->CCR1 = period_ticks / 2;
}

void motor_rear_right_write(uint16_t period_ticks)
{
    TIM17->ARR = period_ticks - 1;
    TIM17->CCR1 = period_ticks / 2;
}

