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
#include "fsl_semc.h"
#include "fsl_elcdif.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "logo.h"

#include "emwin_support.h"
#include "GUI.h"
#include "GRAPH.h"
#include "TEXT.h"
#include "IMAGE.h"
#include "BUTTON.h"
#include "DIALOG.h"

#define ACTIVE_AXIS_X 1
#define ACTIVE_AXIS_Y 2
#define ACTIVE_AXIS_Z 3

int activeAxis = ACTIVE_AXIS_X;

TaskHandle_t measurementTaskHandle;
TaskHandle_t guiTaskHandle;

QueueHandle_t queueHandle;

BUTTON_Handle buttonX;
BUTTON_Handle buttonY;
BUTTON_Handle buttonZ;

typedef struct {
	uint8_t temp;
	uint32_t press;
	int16_t accX;
	int16_t accY;
	int16_t accZ;
} QueueData_t;

void eventCallback(WM_MESSAGE * pMsg)
{
	if(pMsg->MsgId == WM_NOTIFY_PARENT) {
		if (pMsg->hWinSrc == buttonX) {
			int code = pMsg->Data.v;
			if (code == WM_NOTIFICATION_RELEASED) {
				activeAxis = ACTIVE_AXIS_X;
			}
		}
		else if (pMsg->hWinSrc == buttonY) {
			int code = pMsg->Data.v;
			if (code == WM_NOTIFICATION_RELEASED) {
				activeAxis = ACTIVE_AXIS_Y;
			}
		}
		else if (pMsg->hWinSrc == buttonZ) {
			int code = pMsg->Data.v;
			if (code == WM_NOTIFICATION_RELEASED) {
				activeAxis = ACTIVE_AXIS_Z;
			}
		}
		return;
	}

	WINDOW_Callback(pMsg);
}

void guiTask(void* param) {

	GUI_Init();

	GUI_Clear();

    int frameLeft = 20;
    int frameRight = 5;

	GRAPH_Handle tempGraph = GRAPH_CreateEx(0, LCD_GetYSize() / 2, LCD_GetXSize() / 2, LCD_GetYSize() / 2, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_GRAPH0);
	GRAPH_SetBorder(tempGraph, frameLeft, 5, frameRight, 5);

	const float tempFactor = 0.125;
	const int tempOffset = -80;

	GRAPH_SCALE_Handle tempScale = GRAPH_SCALE_Create(5, GUI_TA_LEFT, GRAPH_SCALE_CF_VERTICAL, 20);
	GRAPH_SCALE_SetFactor(tempScale, tempFactor);
	GRAPH_SCALE_SetOff(tempScale, tempOffset);
	GRAPH_SCALE_SetTextColor(tempScale, GUI_BLACK);
	GRAPH_AttachScale(tempGraph, tempScale);

	GRAPH_DATA_Handle tempData = GRAPH_DATA_YT_Create(GUI_BLUE, LCD_GetXSize() / 2 - frameLeft - frameRight, NULL, 0);
	GRAPH_DATA_YT_SetAlign(tempData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(tempGraph, tempData);

    frameLeft = 30;

	GRAPH_Handle pressGraph = GRAPH_CreateEx(LCD_GetXSize() / 2, LCD_GetYSize() / 2, LCD_GetXSize() / 2, LCD_GetYSize() / 2, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_GRAPH1);
	GRAPH_SetBorder(pressGraph, frameLeft, 5, frameRight, 5);

	const float pressFactor = 5.0;
	const int pressOffset = -100;

	GRAPH_SCALE_Handle pressScale = GRAPH_SCALE_Create(5, GUI_TA_LEFT, GRAPH_SCALE_CF_VERTICAL, 20);
	GRAPH_SCALE_SetFactor(pressScale, pressFactor);
	GRAPH_SCALE_SetOff(pressScale, pressOffset);
	GRAPH_SCALE_SetTextColor(pressScale, GUI_BLACK);
	GRAPH_AttachScale(pressGraph, pressScale);

	GRAPH_DATA_Handle pressData = GRAPH_DATA_YT_Create(GUI_GREEN, LCD_GetXSize() / 2 - frameLeft - frameRight, NULL, 0);
	GRAPH_DATA_YT_SetAlign(pressData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(pressGraph, pressData);

	WM_HWIN window =  WINDOW_CreateEx(0, 0, LCD_GetXSize(), LCD_GetYSize() / 2, WM_HBKWIN, WM_CF_SHOW, 0, GUI_ID_USER, NULL);
	WINDOW_SetBkColor(window,  GUI_WHITE);

	char* textBuffer = "     0 mg";

	TEXT_Handle accText = TEXT_CreateEx(500, 100, 150, 50, window, WM_CF_SHOW, TEXT_CF_RIGHT, GUI_ID_TEXT0, textBuffer);
	TEXT_SetFont(accText, GUI_LARGE_FONT);

	buttonX = BUTTON_CreateEx(670, 40, 100, 50, window, WM_CF_SHOW, 0, GUI_ID_BUTTON0);
	BUTTON_SetText(buttonX, "X");
	buttonY = BUTTON_CreateEx(670, 100, 100, 50, window, WM_CF_SHOW, 0, GUI_ID_BUTTON1);
	BUTTON_SetText(buttonY, "Y");
	buttonZ = BUTTON_CreateEx(670, 160, 100, 50, window, WM_CF_SHOW, 0, GUI_ID_BUTTON2);
	BUTTON_SetText(buttonZ, "Z");

	WM_SetCallback(window, eventCallback);

    GUI_BITMAP bitmap = {
    .XSize = 397,
    .YSize = 96,
    .BytesPerLine = 1191,
    .BitsPerPixel = 24,
    .pData = logoBitmap,
    .pPal = NULL,
    .pMethods = GUI_DRAW_BMP24,
    };

    IMAGE_Handle img = IMAGE_CreateEx(50, 70, 400, 100, window, WM_CF_SHOW, 0, GUI_ID_IMAGE0);
    IMAGE_SetBitmap(img,  &bitmap);
    GUI_Exec();

    while(1)
    {
    	QueueData_t queueData;
        xQueueReceive(queueHandle, &queueData, portMAX_DELAY);

        GRAPH_DATA_YT_AddValue(tempData, queueData.temp / tempFactor + tempOffset);
       	GRAPH_DATA_YT_AddValue(pressData, queueData.press / pressFactor + pressOffset);

       	int16_t acceleration = 0;
       	if (activeAxis == ACTIVE_AXIS_X) {
       		acceleration = queueData.accX;
       	} else if (activeAxis == ACTIVE_AXIS_Y) {
       		acceleration = queueData.accY;
       	} else if (activeAxis == ACTIVE_AXIS_Z) {
       		acceleration = queueData.accZ;
       	}

       	sprintf(textBuffer, "%d mg", acceleration);
       	TEXT_SetText(accText, textBuffer);
       	GUI_Exec();

    }
}

void measurementTask(void* param) {

	uint8_t data[16];

	flexio_i2c_master_transfer_t mlpTransfer;
	mlpTransfer.data = data;
	mlpTransfer.dataSize = 1;
	mlpTransfer.flags = 0;
	mlpTransfer.slaveAddress = 0x60;
	mlpTransfer.subaddressSize = 1;
	mlpTransfer.direction = kFLEXIO_I2C_Write;

	data[0] = 0x39; // Oversampling 128, 512 ms
	mlpTransfer.subaddress = 0x26;
	FLEXIO_I2C_MasterTransferBlocking(&FlexIO_I2C_1_peripheralConfig, &mlpTransfer);

	flexio_i2c_master_transfer_t mmaTransfer;
	mmaTransfer.data = data;
	mmaTransfer.dataSize = 1;
	mmaTransfer.flags = 0;
	mmaTransfer.slaveAddress = 0x1C;
	mmaTransfer.subaddressSize = 1;
	mmaTransfer.direction = kFLEXIO_I2C_Write;

	data[0] = 0x01;
	mmaTransfer.subaddress = 0x2A;
	FLEXIO_I2C_MasterTransferBlocking(&FlexIO_I2C_1_peripheralConfig, &mmaTransfer);

	mlpTransfer.direction = kFLEXIO_I2C_Read;
	mlpTransfer.subaddress = 0x01;
	mlpTransfer.dataSize = 5;

	mmaTransfer.direction = kFLEXIO_I2C_Read;
	mmaTransfer.subaddress = 0x01;
	mmaTransfer.dataSize = 6;

	const TickType_t delayMs = 200 / portTICK_PERIOD_MS;

	QueueData_t queueData;

	while(1) {
		vTaskDelay(delayMs);

		FLEXIO_I2C_MasterTransferBlocking(&FlexIO_I2C_1_peripheralConfig, &mlpTransfer);

		queueData.temp = data[3];
		queueData.press = (data[2] >> 6) | (data[1] << 2) | (data[0] << 10);
		queueData.press /= 100;

		FLEXIO_I2C_MasterTransferBlocking(&FlexIO_I2C_1_peripheralConfig, &mmaTransfer);
		queueData.accX = ((data[0] << 8) | (data[1]));
		queueData.accY = ((data[2] << 8) | (data[3]));
		queueData.accZ = ((data[4] << 8) | (data[5]));

		queueData.accX /= 16;
		queueData.accY /= 16;
		queueData.accZ /= 16;

		xQueueSend(queueHandle, &queueData, 0);

	}
}

void touchTimer(TimerHandle_t xTimer)
{
	uint8_t data[5] = {0};

	flexio_i2c_master_transfer_t touchTransfer;
	touchTransfer.data = data;
	touchTransfer.dataSize = 5;
	touchTransfer.flags = 0;
	touchTransfer.slaveAddress = 0x38;
	touchTransfer.subaddressSize = 0x01;
	touchTransfer.subaddress = 0x02;
	touchTransfer.direction = kFLEXIO_I2C_Read;
	FLEXIO_I2C_MasterTransferBlocking(&FlexIO_I2C_1_peripheralConfig, &touchTransfer);

	uint8_t points = data[0];
	uint32_t x = ((data[1] & 0x0F) << 8) | (data[2]);
	uint32_t y = ((data[3] & 0x0F) << 8) | (data[4]);

	static bool activeTouch = false;

	GUI_PID_STATE state;
	state.Layer = 0;
	state.Pressed = 0;
	state.x = x;
	state.y = y;

	if (points == 1) {
		activeTouch = true;
		state.Pressed = 1;
		GUI_PID_StoreState(&state);
	} else if (activeTouch) {
		activeTouch = false;
		state.Pressed = 0;
		GUI_PID_StoreState(&state);
	}
}

status_t BOARD_InitSEMC(void)
{
    semc_config_t config;
    semc_sdram_config_t sdramconfig;
    uint32_t clockFrq = CLOCK_GetFreq(kCLOCK_SemcClk);

    /* Initializes the MAC configure structure to zero. */
    memset(&config, 0, sizeof(semc_config_t));
    memset(&sdramconfig, 0, sizeof(semc_sdram_config_t));

    /* Initialize SEMC. */
    SEMC_GetDefaultConfig(&config);
    config.dqsMode = kSEMC_Loopbackdqspad;  /* For more accurate timing. */
    SEMC_Init(SEMC, &config);

    /* Configure SDRAM. */
    sdramconfig.csxPinMux = kSEMC_MUXCSX0;
    sdramconfig.address = 0x80000000;
    sdramconfig.memsize_kbytes = 32 * 1024; /* 32MB = 32*1024*1KBytes*/
    sdramconfig.portSize = kSEMC_PortSize16Bit;
    sdramconfig.burstLen = kSEMC_Sdram_BurstLen8;
    sdramconfig.columnAddrBitNum = kSEMC_SdramColunm_9bit;
    sdramconfig.casLatency = kSEMC_LatencyThree;
    sdramconfig.tPrecharge2Act_Ns = 18;   /* Trp 18ns */
    sdramconfig.tAct2ReadWrite_Ns = 18;   /* Trcd 18ns */
    sdramconfig.tRefreshRecovery_Ns = 72; /* Use the maximum of the (Trfc , Txsr). */
    sdramconfig.tWriteRecovery_Ns = 12;   /* 12ns */
    sdramconfig.tCkeOff_Ns = 42;  /* The minimum cycle of SDRAM CLK off state. CKE is off in self refresh at a minimum period tRAS.*/
    sdramconfig.tAct2Prechage_Ns = 42; /* Tras 42ns */
    sdramconfig.tSelfRefRecovery_Ns = 67;
    sdramconfig.tRefresh2Refresh_Ns = 60;
    sdramconfig.tAct2Act_Ns = 60;
    sdramconfig.tPrescalePeriod_Ns = 160 * (1000000000 / clockFrq);
    sdramconfig.refreshPeriod_nsPerRow = 64 * 1000000 / 8192; /* 64ms/8192 */
    sdramconfig.refreshUrgThreshold = sdramconfig.refreshPeriod_nsPerRow;
    sdramconfig.refreshBurstLen = 1;
    return SEMC_ConfigureSDRAM(SEMC, kSEMC_SDRAM_CS0, &sdramconfig, clockFrq);
}

void BOARD_InitLcd(void)
{
    GPIO_PinWrite(BOARD_INITPINS_LCD_RESET_GPIO, BOARD_INITPINS_LCD_RESET_PIN, 0);
    volatile uint32_t i = 0x1000U;
    while (i--)
    	;
    GPIO_PinWrite(BOARD_INITPINS_LCD_RESET_GPIO, BOARD_INITPINS_LCD_RESET_PIN, 1);
}

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    BOARD_InitSEMC();

    BOARD_InitLcd();

    xTaskCreate(measurementTask, "MEAS", 256, NULL, 1, &measurementTaskHandle);
    xTaskCreate(guiTask, "GUI", 1024, NULL, 1, &guiTaskHandle);
    TimerHandle_t timer = xTimerCreate("TOUCH", 150 / portTICK_PERIOD_MS, pdTRUE, NULL, touchTimer);
    xTimerStart(timer, 0);

    queueHandle = xQueueCreate(10, sizeof(QueueData_t));

    vTaskStartScheduler();

    return 0 ;
}

