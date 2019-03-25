#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1052.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"

#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t ledTaskHandle;
TaskHandle_t uartTaskHandle;

void ledTask(void* param) {

	const TickType_t delayMs = 1000 / portTICK_PERIOD_MS;

	while(1) {
	    GPIO_PinWrite(GPIO1, 8, 1);
	    vTaskDelay(delayMs);
	    GPIO_PinWrite(GPIO1, 8, 0);
	    vTaskDelay(delayMs);
	}
}

void uartTask(void* param) {

	const TickType_t delayMs = 1000 / portTICK_PERIOD_MS;
	char text[32] = {0};
	uint8_t i = 0;

	while(1) {
		sprintf(text, "HELLO TASK %d\r\n", i++);
	    LPUART_WriteBlocking(LPUART1, (uint8_t*)text, strlen(text));
	    vTaskDelay(delayMs);
	}
}

void main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    xTaskCreate(ledTask, "LED_TASK", 128, NULL, 1, &ledTaskHandle);
    xTaskCreate(uartTask, "UART_TASK", 128, NULL, 1, &uartTaskHandle);

    vTaskStartScheduler();
}
