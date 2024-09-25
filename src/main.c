#include <main.h>


void vBlinkTask(void *pvParameters)
{
    SET_RCC_xxxxEN(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
    // 01 = GPIO output mode
    GPIOE->MODER &= ~GPIO_MODER_MODE3_1;
    GPIOE->MODER |= GPIO_MODER_MODE3_0;

    for (;;) {
        GPIOE->BSRR = GPIO_BSRR_BS3;
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIOE->BSRR = GPIO_BSRR_BR3;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

int main(void)
{
    setup_clocks();
    setup_ethernet();
    setup_hrtim();
    setup_i2c();
    HRTIM1_TIMA->PERxR = 1200;
    HRTIM1_TIMB->PERxR = 1200;
    HRTIM1_TIMC->PERxR = 1200;

    xTaskCreate(vBlinkTask, "blink", 128, NULL, tskIDLE_PRIORITY + 5, NULL);

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

