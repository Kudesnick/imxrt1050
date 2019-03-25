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
#include "fsl_flexio_i2c_master.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

TaskHandle_t measurementTaskHandle;
TaskHandle_t printTaskHandle;

QueueHandle_t queueHandle;

void printTask(void* param) {
	char textBuffer[32];
	while(1) {
		uint8_t temp = 0;
		uint32_t press = 0;

		xQueueReceive(queueHandle, &temp, portMAX_DELAY);

		sprintf(textBuffer, "T = %d, P = %d\n", temp, press);
		LPUART_WriteBlocking(LPUART1, (uint8_t*)textBuffer, strlen(textBuffer));
	}
}

void measurementTask(void* param) {

	uint8_t data[5];

	flexio_i2c_master_transfer_t transfer;
	transfer.data = data;
	transfer.dataSize = 1;
	transfer.flags = 0;
	transfer.slaveAddress = 0x60;
	transfer.subaddressSize = 1;
	transfer.direction = kFLEXIO_I2C_Write;

	data[0] = 0x39; // Oversampling 128, 512 ms
	transfer.subaddress = 0x26;
	FLEXIO_I2C_MasterTransferBlocking(&FlexIO_I2C_1_peripheralConfig, &transfer);

	transfer.direction = kFLEXIO_I2C_Read;
	transfer.subaddress = 0x01;
	transfer.dataSize = 5;

	const TickType_t delayMs = 500 / portTICK_PERIOD_MS;

	while(1) {
		vTaskDelay(delayMs);

		FLEXIO_I2C_MasterTransferBlocking(&FlexIO_I2C_1_peripheralConfig, &transfer);
		uint8_t temp = data[3];
		uint32_t press = (data[2] >> 6) | (data[1] << 2) | (data[0] << 10);

		xQueueSend(queueHandle, &temp, 0);
	}
}

int main(void) {

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    printf("Hello World\n");

    xTaskCreate(measurementTask, "MEAS", 128, NULL, 1, &measurementTaskHandle);
    xTaskCreate(printTask, "PRINT", 128, NULL, 1, &printTaskHandle);

    queueHandle = xQueueCreate(10, sizeof(uint8_t));

    vTaskStartScheduler();

    return 0 ;
}
