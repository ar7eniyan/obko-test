#include "motors.h"

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
    // Setup break functionality
    TIM15->AF1 = TIM15_AF1_BKINE | TIM15_AF1_BKINP;
    TIM15->BDTR = TIM_BDTR_MOE | TIM_BDTR_BKP | TIM_BDTR_BKE | TIM_BDTR_OSSI;
    TIM16->AF1 = TIM16_AF1_BKINE | TIM16_AF1_BKINP;
    TIM16->BDTR = TIM_BDTR_MOE | TIM_BDTR_BKP | TIM_BDTR_BKE | TIM_BDTR_OSSI;
    TIM17->AF1 = TIM17_AF1_BKINE | TIM17_AF1_BKINP;
    TIM17->BDTR = TIM_BDTR_MOE | TIM_BDTR_BKP | TIM_BDTR_BKE | TIM_BDTR_OSSI;
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

    GPIOE->OTYPER |= GPIO_OTYPER_OT5;
    MODIFY_REG(GPIOE->AFR[0], GPIO_AFRL_AFSEL5, 0b0100 << GPIO_AFRL_AFSEL5_Pos);
    MODIFY_REG(GPIOE->MODER, GPIO_MODER_MODE5, (0b10 << GPIO_MODER_MODE5_Pos));
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

void setup_hrtim(void)
{
    SET_RCC_xxxxEN(RCC->APB2ENR, RCC_APB2ENR_HRTIMEN);
    // HRTIM frequency is the same as other timers' (240 Mhz here)

    // Timers A, B and C are used to output 3 PWM signals with different
    // frequencies and 50% duty cycle each (using the Half feature of CMP1).
    HRTIM1_TIMA->TIMxCR = HRTIM_TIMCR_CONT | HRTIM_TIMCR_HALF
        | (0b111 << HRTIM_TIMCR_CK_PSC_Pos);
    HRTIM1_TIMB->TIMxCR = HRTIM_TIMCR_CONT | HRTIM_TIMCR_HALF
        | (0b111 << HRTIM_TIMCR_CK_PSC_Pos);
    HRTIM1_TIMC->TIMxCR = HRTIM_TIMCR_CONT | HRTIM_TIMCR_HALF
        | (0b111 << HRTIM_TIMCR_CK_PSC_Pos);

    // Set/reset sources (set on period, reset on compare 1)
    HRTIM1_TIMA->SETx1R |= HRTIM_SET1R_PER;
    HRTIM1_TIMA->RSTx1R |= HRTIM_RST1R_CMP1;
    HRTIM1_TIMB->SETx1R |= HRTIM_SET1R_PER;
    HRTIM1_TIMB->RSTx1R |= HRTIM_RST1R_CMP1;
    HRTIM1_TIMC->SETx1R |= HRTIM_SET1R_PER;
    HRTIM1_TIMC->RSTx1R |= HRTIM_RST1R_CMP1;

    // Output configuration: IDLE and FAULT states are inactive, positive
    // polarity
    HRTIM1_TIMA->OUTxR = (0b10 << HRTIM_OUTR_FAULT1_Pos);
    HRTIM1_TIMB->OUTxR = (0b10 << HRTIM_OUTR_FAULT1_Pos);
    HRTIM1_TIMC->OUTxR = (0b10 << HRTIM_OUTR_FAULT1_Pos);

    // Motor drivers (Rear Left/Right, Steering) GPIO binding:
    // RL: PUL - PA9 (HRTIM_CHC1), ENA - PD3, DIR - PD5, ALM - PD4 (HRTIM_FLT3)
    // RR: PUL - PC8 (HRTIM_CHB1), ENA - PD7, DIR - PD6, ALM - PB3 (HRTIM_FLT4)
    // ST: PUL - PC6 (HRTIM_CHA1), ENA - PC7, DIR - PC9
    // Timer output GPIO binding:
    // CHA1 - AF1 on PC6, CHB1 - AF1 on PC8, CHC1 - AF2 on PA9.
    // Use GPIO settings: open drain output, low speed, no internal PU/PD.
    // External pullup is 5V on all pins outputting to the motor drivers. The
    // pullup is permanent, and a Zener diode is used between 5V and 3.3V rails
    // for safety during startup (3.3V stabilizes about 0.7 ms later).
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOCEN);

    GPIOA->OTYPER |= (1 << GPIO_OTYPER_OT6_Pos) | (1 << GPIO_OTYPER_OT8_Pos);
    MODIFY_REG(GPIOC->AFR[0], GPIO_AFRL_AFSEL6, 0b0001 << GPIO_AFRL_AFSEL6_Pos);
    MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL8, 0b0001 << GPIO_AFRH_AFSEL8_Pos);
    MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE6 | GPIO_MODER_MODE8,
        (0b10 << GPIO_MODER_MODE6_Pos) | (0b10 << GPIO_MODER_MODE8_Pos));

    GPIOA->OTYPER |= (1 << GPIO_OTYPER_OT9_Pos);
    MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL9, 0b0010 << GPIO_AFRH_AFSEL9_Pos);
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE9, 0b10 << GPIO_MODER_MODE9_Pos);

    // Enable the outputs (transition to RUN from IDLE)
    HRTIM1_COMMON->OENR |= HRTIM_OENR_TA1OEN | HRTIM_OENR_TB1OEN | HRTIM_OENR_TC1OEN;

    // Start the timers
    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN | HRTIM_MCR_TCCEN;
}

