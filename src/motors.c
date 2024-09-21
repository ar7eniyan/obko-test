#include "motors.h"

#include "stm32h7xx.h"
#include "stm32h743xx.h"

#include "tools.h"


void setup_hrtim(void)
{
    SET_RCC_xxxxEN(RCC->APB2ENR, RCC_APB2ENR_HRTIMEN);
    // HRTIM frequency is the same as other timers' (240 Mhz here)

    // Timers A, B and C are used to output 3 PWM signals with different
    // frequencies and 50% duty cycle each (using the Half feature of CMP1).
    HRTIM1_TIMA->TIMxCR = HRTIM_TIMCR_CONT | HRTIM_TIMCR_HALF
        | (0b101 << HRTIM_TIMCR_CK_PSC_Pos);
    HRTIM1_TIMB->TIMxCR = HRTIM_TIMCR_CONT | HRTIM_TIMCR_HALF
        | (0b101 << HRTIM_TIMCR_CK_PSC_Pos);
    HRTIM1_TIMC->TIMxCR = HRTIM_TIMCR_CONT | HRTIM_TIMCR_HALF
        | (0b101 << HRTIM_TIMCR_CK_PSC_Pos);

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

    // Timer output GPIO binding:
    // CHA1 - AF1 on PC6, CHB1 - AF1 on PC8, CHC1 - AF2 on PA9.
    // Use default GPIO settings except for AF: push-pull output type, low
    // speed, no pullup/pulldown.
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOCEN);

    MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE6 | GPIO_MODER_MODE8,
        (0b10 << GPIO_MODER_MODE6_Pos) | (0b10 << GPIO_MODER_MODE8_Pos));
    MODIFY_REG(GPIOC->AFR[0], GPIO_AFRL_AFSEL6, 0b0001 << GPIO_AFRL_AFSEL6_Pos);
    MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL8, 0b0001 << GPIO_AFRH_AFSEL8_Pos);

    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE9, 0b10 << GPIO_MODER_MODE9_Pos);
    MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL9, 0b0010 << GPIO_AFRH_AFSEL9_Pos);

    // Enable the outputs (transition to RUN from IDLE)
    HRTIM1_COMMON->OENR |= HRTIM_OENR_TA1OEN | HRTIM_OENR_TB1OEN | HRTIM_OENR_TC1OEN;

    // Start the timers
    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN | HRTIM_MCR_TCCEN;
}

