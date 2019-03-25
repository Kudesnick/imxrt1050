/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    VisionSOM_RT_FreeRTOS_emWIN_HandsOn.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1052.h"

#include "fsl_lpuart.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t ledTaskHandle;

void GPIO1_INT3_IRQHandler(void) {
	GPIO_PortClearInterruptFlags(GPIO1, 1U << 3);

	BaseType_t higherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(ledTaskHandle, 1, eSetValueWithOverwrite, &higherPriorityTaskWoken);
	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void ledTask(void* param) {

	const TickType_t delayMs = 1000 / portTICK_PERIOD_MS;

	while(1) {
	    uint32_t notification = 0;
	    xTaskNotifyWait(0, UINT32_MAX, &notification, portMAX_DELAY);

	    GPIO_PinWrite(GPIO1, 8, 1);
	    vTaskDelay(delayMs);
	    GPIO_PinWrite(GPIO1, 8, 0);
	}
}

int main(void) {

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    printf("Hello World\n");

    xTaskCreate(ledTask, "LED_TASK", 128, ledTask, 1, &ledTaskHandle);

    vTaskStartScheduler();

    return 0 ;
}
