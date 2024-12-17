#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h743xx.h"
#include "system_stm32h7xx.h"

#include "tools.h"
#include "motors.h"
#include "ethernet.h"
#include "i2c.h"
#include "uart.h"
#include "config.h"
#include "gpio.h"

void setup_clocks(void)
{
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
    RCC->PLLCKSELR = (0b10 << RCC_PLLCKSELR_PLLSRC_Pos) | (5 << RCC_PLLCKSELR_DIVM1_Pos);
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
/*
    // I2C clk mux.
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLL3RGE_2 |
                     RCC_PLLCFGR_PLL3VCOSEL);

    RCC->PLLCKSELR |= (25 << RCC_PLLCKSELR_DIVM3_Pos);
    RCC->PLL3DIVR = ((150 << RCC_PLL3DIVR_N3_Pos) |
                    (3 << RCC_PLL3DIVR_R3_Pos) |
                    (2 << RCC_PLL3DIVR_Q3_Pos));

    RCC->CR |= RCC_CR_PLL3ON;
    while((RCC->CR & RCC_CR_PLL3RDY) == 0);

    // PLL3R as clock sourse for I2C1
    RCC->D2CCIP2R &= ~RCC_D2CCIP2R_I2C123SEL_Msk;
    RCC->D2CCIP2R |= (0b01 << RCC_D2CCIP2R_I2C123SEL_Pos);
*/
    // 6. Настройка тактирования для I2C1 (PLL3 для 50 МГц)
    RCC->PLLCKSELR |= (25 << RCC_PLLCKSELR_DIVM3_Pos);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLL3RGE_2 | RCC_PLLCFGR_PLL3VCOSEL;

    RCC->PLL3DIVR = ((150 << RCC_PLL3DIVR_N3_Pos) |  // Умножитель 150
                     (3 << RCC_PLL3DIVR_R3_Pos)  |  // Делитель на выходе R = 4
                     (2 << RCC_PLL3DIVR_Q3_Pos));   // Делитель на выходе Q = 3
/*
    RCC->CR |= RCC_CR_PLL3ON;
    while (!(RCC->CR & RCC_CR_PLL3RDY)); // Ожидание готовности PLL3

    // Установка PLL3R как источник тактирования для I2C1
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_I2C123SEL_Msk) |
                    (0b01 << RCC_D2CCIP2R_I2C123SEL_Pos);

    // 7. Переключение системы на PLL1
    RCC->CFGR |= RCC_CFGR_SW_PLL1; // Использование PLL1 как системного такта
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL1); // Ожидание готовности PLL1
*/
    RCC->PLLCFGR |= RCC_PLLCFGR_DIVR3EN; // Включаем выход R для PLL3

    // 6. Включение PLL1 и PLL3
    RCC->CR |= RCC_CR_PLL1ON | RCC_CR_PLL3ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY)); // Ожидание готовности PLL1
    while (!(RCC->CR & RCC_CR_PLL3RDY)); // Ожидание готовности PLL3

    // 7. Настройка предделителей шин
    RCC->D1CFGR = RCC_D1CFGR_HPRE_DIV2 | // Делитель AHB = 2
                  RCC_D1CFGR_D1PPRE_DIV2; // Делитель APB3 = 2

    RCC->D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | // Делитель APB1 = 2
                  RCC_D2CFGR_D2PPRE2_DIV2;  // Делитель APB2 = 2

    RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2; // Делитель APB4 = 2

    // 8. Установка PLL3R как источник тактирования для I2C1
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_I2C123SEL_Msk) |
                    (0b01 << RCC_D2CCIP2R_I2C123SEL_Pos);

    // 9. Переключение системы на PLL1
    RCC->CFGR |= RCC_CFGR_SW_PLL1; // Использование PLL1 как системного такта
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL1); // Ожидание готовности PLL1


    // TODO: Need to wait for 16 cycles of the slowest of all APB clocks
    for (int delay = 100; delay--; );

    // Route the PLL1 P output to the sys_ck
    RCC->CFGR = (0b011 << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0b011 << RCC_CFGR_SWS_Pos));
    // TODO: Clarify on the delay needed
    for (int delay = 100; delay--; );
    SystemCoreClockUpdate();
}

void vBlinkTask(void *pvParameters)
{
    gpio_setup_pin(GPIOE, 3, GPIO_FLAGS_MODE_OUT);

    for (;;) {
        GPIOE->BSRR = GPIO_BSRR_BS3;
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIOE->BSRR = GPIO_BSRR_BR3;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void vEthEchoTask(void *pvParameters)
{
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

void vI2CReadEncoder(void *pvParameters)
{
    char test_i2c_data[] = "Hello";

    for (;;) {
        i2c_master_transmit(I2C1, I2C_ENC_ADDR, test_i2c_data, sizeof(test_i2c_data));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

void vEthPingTask(void *pvParameters)
{
    // 6 (DA) + 6 (SA) + 2 (EtherType) + 48 (ping msg)
    char *tx_buf, *rx_buf;
    int rx_len;
    eth_setup(&tx_buf);

    for (;;) {
        if ((rx_len = eth_recv(&rx_buf)) != -1){
            if (rx_len < 14 + 8) {
                continue;
            }
            if (memcmp(&rx_buf[14], "ping", 4) != 0 ||
                memcmp(&rx_buf[rx_len - 4], "ping", 4) != 0) {
                continue;
            }
            for (size_t i = 14; i < rx_len - 4; i++) {
                if (rx_buf[i] != 0x20) {
                    continue;
                }
            }
            memcpy(tx_buf, rx_buf + 6, rx_len - 6);
            memcpy(&tx_buf[8], "pong", 4);
            memcpy(&tx_buf[rx_len - 4 - 6], "pong", 4);
            eth_send(rx_len - 6, &tx_buf);
        }
    }
    vTaskDelete(NULL);
}

void __attribute__((__noreturn__)) panic(void)
{
    // Blink an on-board LED forever
    gpio_setup_pin(GPIOE, 3, GPIO_FLAGS_MODE_OUT);
    for(;;) {
        GPIOE->BSRR = GPIO_BSRR_BS3;
        // TODO: replace with hand-written delays not depending on the OS
        vTaskDelay(pdMS_TO_TICKS(50));
        GPIOE->BSRR = GPIO_BSRR_BR3;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void)
{
    setup_clocks();
    setup_motors();
    //setup_i2c();
    //setup_uart();
    // Somewhere around the minimum speed possible
    motor_steering_drive(0.02);
    motor_rear_left_drive(0.02);
    motor_rear_right_drive(0.02);

    xTaskCreate(vBlinkTask, "blink", 128, NULL, tskIDLE_PRIORITY + 5, NULL);
    xTaskCreate(vEthPingTask, "echo", 128, NULL, tskIDLE_PRIORITY + 10, NULL);
    //xTaskCreate(vI2CTask, "i2c_hello", 128, NULL, tskIDLE_PRIORITY + 15, NULL);
    //xTaskCreate(vI2CReadEncoder, "encoder_steering", 128, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();
    // The function above returns only if something calls vTaskEndScheduler().
    // The choice here is to treat as a bug and hang the CPU.
    panic();
    return 0;
}

// TODO: write them properly
void vApplicationTickHook(void) {}
void vApplicationIdleHook(void) {}
void vApplicationMallocFailedHook(void) {}
void vApplicationStackOverflowHook(void) {}

