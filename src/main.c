#include "main.h"

void setup_clocks(void) {
    // Update flash latency to correspond with VOS0 and 480MHz
    FLASH->ACR = (0b10 << FLASH_ACR_WRHIGHFREQ_Pos) | (FLASH_ACR_LATENCY_4WS);
    __IO uint32_t tmpreg = FLASH->ACR;
    (void)tmpreg;

    // Scale Vcore up to VOS0:
    // Lock power supply settings
    PWR->CR3 &= ~PWR_CR3_SCUEN;
    // Scale up to VOS1
    while(~PWR->CSR1 & PWR_CSR1_ACTVOSRDY);
    PWR->D3CR = (0b11 << PWR_D3CR_VOS_Pos) | PWR_D3CR_VOSRDY;
    while(PWR->D3CR & PWR_D3CR_VOSRDY);
    // Enable SYSCFG periphiral, enable LDO regulator overdirve mode
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;
    while(~PWR->D3CR & PWR_D3CR_VOSRDY);

    // Enable High-speed External oscillator
    RCC->CR |= RCC_CR_HSEON;
    while (~RCC->CR & RCC_CR_HSERDY);

    // HSE as PLL source, prescaler 5 for PLL1
    RCC->PLLCKSELR = (0b10 << RCC_PLLCKSELR_PLLSRC_Pos) | (0b000101 << RCC_PLLCKSELR_DIVM1_Pos);
    // PLL1 range from 4MHz to 8MHz, P and Q outputs enabled
    RCC->PLLCFGR = RCC_PLLCFGR_PLL1RGE_2 | RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN;
    // PLL1 multiplication by 192, division by 2 on P and Q outputs
    RCC->PLL1DIVR = (191 << RCC_PLL1DIVR_N1_Pos) | (1 << RCC_PLL1DIVR_P1_Pos) | (1 << RCC_PLL1DIVR_Q1_Pos);
    // Enable PLL1 and wait until it's locked
    RCC->CR |= RCC_CR_PLL1ON;
    while (~RCC->CR & RCC_CR_PLL1RDY);

    // HPRE prescaler to 2, D1PPRE prescaler to 2, wait for clock propagation
    RCC->D1CFGR = (0b100 << RCC_D1CFGR_D1PPRE_Pos) | (0b1000 << RCC_D1CFGR_HPRE_Pos);
    // D2PPRE1 prescaler to 2, D2PPRE2 prescaler to 2
    // That means timer clocks will be two times faster than their bus clocks
    // (equal to AHB clocks, 240 Mhz in this case).
    // See Table 57 in RM for reference.
    RCC->D2CFGR = (0b100 << RCC_D2CFGR_D2PPRE1_Pos) | (0b100 << RCC_D2CFGR_D2PPRE2_Pos);
    // D3PPRE prescaler to 2
    RCC->D3CFGR = (0b100 << RCC_D3CFGR_D3PPRE_Pos);

    // TODO: Need to wait for 16 cycles of the slowest of all APB clocks
    for (int delay = 100; delay--; );

    // Route the PLL1 P output to the sys_ck
    RCC->CFGR = (0b011 << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0b011 << RCC_CFGR_SWS_Pos));
    // TODO: Clarify on the delay needed
    for (int delay = 100; delay--; );
    SystemCoreClockUpdate();
}

void vBlinkTask(void *pvParameters) {
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
    // 01 = GPIO output mode
    GPIOE->MODER &= ~GPIO_MODER_MODE3_1;
    GPIOE->MODER |= GPIO_MODER_MODE3_0;

    for (;;) {
        GPIOE->BSRR = GPIO_BSRR_BS3;
        vTaskDelay(pdMS_TO_TICKS(200));
        GPIOE->BSRR = GPIO_BSRR_BR3;
        vTaskDelay(pdMS_TO_TICKS(800));
    }
    vTaskDelete(NULL);
}

void vEthEchoTask(void *pvParameters) {
    char *tx_buf, *rx_buf;
    int rx_len;
    eth_setup(&tx_buf);

    for (;;) {
        if ((rx_len = eth_recv(&rx_buf)) != -1){
            // Copy starting with the SA (will be the DA of the outgoing
            // packet). Our address will be automatically inserted as the SA.
            memcpy(tx_buf, rx_buf + 6, rx_len - 6);
            eth_send(rx_len - 6, &tx_buf);
        }
    }
    vTaskDelete(NULL);
}

void vI2CTask(void *pvParameters) {
    char test_i2c_data[] = "Hello\n";

    for (;;) {
        i2c_master_transmit(I2C1, I2C_ENC_ADDR, test_i2c_data, sizeof(test_i2c_data));
        i2c_master_transmit(I2C3, I2C_ENC_ADDR, test_i2c_data, sizeof(test_i2c_data));
        i2c_master_transmit(I2C4, I2C_ENC_ADDR, test_i2c_data, sizeof(test_i2c_data));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

int main(void) {
    setup_clocks();
    setup_hrtim();
    setup_i2c();
    setup_uart();
    HRTIM1_TIMA->PERxR = 1200;
    HRTIM1_TIMB->PERxR = 1200;
    HRTIM1_TIMC->PERxR = 1200;

    xTaskCreate(vBlinkTask, "blink", 128, NULL, tskIDLE_PRIORITY + 5, NULL);
    xTaskCreate(vEthEchoTask, "echo", 128, NULL, tskIDLE_PRIORITY + 10, NULL);
    xTaskCreate(vI2CTask, "i2c_hello", 128, NULL, tskIDLE_PRIORITY + 15, NULL);

    vTaskStartScheduler();
    // The function above returns only if something calls vTaskEndScheduler().
    // The choice here is to treat as a bug and hang the CPU.
    for(;;);
    return 0;
}

// TODO: write them properly
void vApplicationTickHook(void) {}
void vApplicationIdleHook(void) {}
void vApplicationMallocFailedHook(void) {}
void vApplicationStackOverflowHook(void) {}

