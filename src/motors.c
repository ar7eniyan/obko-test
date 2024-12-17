#include "motors.h"

#include "gpio.h"
#include "stm32h7xx.h"
#include "stm32h743xx.h"

#include "tools.h"
#include <math.h>
#include <stdint.h>


// TODO: move all the constants to config.h
#define ANGULAR_VEL_TO_TIMER_PERIOD(omega) \
    ((2 * M_PI / 200 * 240e6 / 6000) / (omega))

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
    TIM15->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_MOE;
    TIM16->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_MOE;
    TIM17->BDTR = TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_MOE;
    // Enable channel 1 output (use complementary on TIM16 and TIM17 because
    // of pinout).
    TIM15->CCER = TIM_CCER_CC1E;
    TIM16->CCER = TIM_CCER_CC1NE;
    TIM17->CCER = TIM_CCER_CC1NE;
    // Start the counter (for later, now it's stopped because of ARR == 0)
    TIM15->CR1 |= TIM_CR1_CEN;
    TIM16->CR1 |= TIM_CR1_CEN;
    TIM17->CR1 |= TIM_CR1_CEN;

    // Configure GPIO blocks for motor drivers' pins: PUL
    gpio_setup_pin(GPIOB, 6,
        GPIO_FLAGS_MODE_AF | GPIO_FLAGS_OTYPE_OD | GPIO_FLAGS_AFSEL_AF1);
    gpio_setup_pin(GPIOB, 7,
        GPIO_FLAGS_MODE_AF | GPIO_FLAGS_OTYPE_OD | GPIO_FLAGS_AFSEL_AF1);
    gpio_setup_pin(GPIOE, 5,
        GPIO_FLAGS_MODE_AF | GPIO_FLAGS_OTYPE_OD | GPIO_FLAGS_AFSEL_AF4);

    // ALM
    gpio_setup_pin(GPIOB, 4,
        GPIO_FLAGS_MODE_AF | GPIO_FLAGS_AFSEL_AF1 | GPIO_FLAGS_PUPD_PD);
    gpio_setup_pin(GPIOB, 5,
        GPIO_FLAGS_MODE_AF | GPIO_FLAGS_AFSEL_AF1 | GPIO_FLAGS_PUPD_PD);

    // Enable break functionality after the break inputs are pulled down to 0V.
    // Regarding break polarity: the RM's point here is hard to understand,
    // but tests show that TIM_AF1_BKINP being set to 1 flips the polarity of
    // BKIN, so we don't enable it.
    TIM16->BDTR |= TIM_BDTR_BKE | TIM_BDTR_BKP;
    TIM16->AF1 = TIM16_AF1_BKINE;
    TIM17->BDTR |= TIM_BDTR_BKE | TIM_BDTR_BKP;
    TIM17->AF1 = TIM17_AF1_BKINE;

    // ENA
    gpio_setup_pin(GPIOB, 3, GPIO_FLAGS_MODE_OUT | GPIO_FLAGS_OTYPE_OD);
    gpio_setup_pin(GPIOE, 0, GPIO_FLAGS_MODE_OUT | GPIO_FLAGS_OTYPE_OD);
    gpio_setup_pin(GPIOE, 2, GPIO_FLAGS_MODE_OUT | GPIO_FLAGS_OTYPE_OD);

    // DIR
    gpio_setup_pin(GPIOD, 7, GPIO_FLAGS_MODE_OUT | GPIO_FLAGS_OTYPE_OD);
    gpio_setup_pin(GPIOE, 1, GPIO_FLAGS_MODE_OUT | GPIO_FLAGS_OTYPE_OD);
    gpio_setup_pin(GPIOE, 4, GPIO_FLAGS_MODE_OUT | GPIO_FLAGS_OTYPE_OD);
}

void motor_steering_drive(float angular_vel)
{
    // TODO: swap set/reset if needed
    // Set the rotation direction
    if (angular_vel < 0.0) {
        angular_vel *= -1.0;
        GPIOE->BSRR |= GPIO_BSRR_BS4;
    } else {
        GPIOE->BSRR |= GPIO_BSRR_BR4;
    }
    uint16_t timer_period_ticks = ANGULAR_VEL_TO_TIMER_PERIOD(angular_vel);
    // The counter goes from 0 to the ARR!
    TIM15->ARR = timer_period_ticks - 1;
    TIM15->CCR1 = timer_period_ticks / 2;
}

void motor_rear_left_drive(float angular_vel)
{
    // TODO: swap set/reset if needed
    // Set the rotation direction
    if (angular_vel < 0.0) {
        angular_vel *= -1.0;
        GPIOE->BSRR |= GPIO_BSRR_BS1;
    } else {
        GPIOE->BSRR |= GPIO_BSRR_BR1;
    }
    uint16_t timer_period_ticks = ANGULAR_VEL_TO_TIMER_PERIOD(angular_vel);
    // The counter goes from 0 to the ARR!
    TIM16->ARR = timer_period_ticks - 1;
    TIM16->CCR1 = timer_period_ticks / 2;
}

void motor_rear_right_drive(float angular_vel)
{
    // TODO: swap set/reset if needed
    // Set the rotation direction
    if (angular_vel < 0.0) {
        angular_vel *= -1.0;
        GPIOD->BSRR |= GPIO_BSRR_BS7;
    } else {
        GPIOD->BSRR |= GPIO_BSRR_BR7;
    }
    uint16_t timer_period_ticks = ANGULAR_VEL_TO_TIMER_PERIOD(angular_vel);
    // The counter goes from 0 to the ARR!
    TIM17->ARR = timer_period_ticks - 1;
    TIM17->CCR1 = timer_period_ticks / 2;
}

