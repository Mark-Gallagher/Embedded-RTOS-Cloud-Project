/*
 * Copyright 2016-2022 NXP
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
 *Mark Gallagher
 *Group A
 *
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "ESP8266.h"
#include "client.h"
#include "MQTT_client.h"

/* TODO: insert other definitions and declarations here */
void initTask(void *pvParameters);
void publishTask(void *pvParameters);
void SW2_Task(void *pvParameters);
void subscribeTask(void *pvParameters);
void volumeControlTask(void *pvParameters);
void currentTimeTask(void *pvParameters);
static void timerTask(void *pvParameters);
TaskHandle_t currentTime_TaskHandle = NULL;
TaskHandle_t timer_TaskHandle = NULL;
EventGroupHandle_t Time_EventGroup = NULL;
EventGroupHandle_t UART_EventGroup = NULL;
EventBits_t Event_bits;
EventBits_t UART_bits;
SemaphoreHandle_t SW2_Semaphore = NULL;
SemaphoreHandle_t VC_Semaphore = NULL;
SemaphoreHandle_t timer_Semaphore = NULL;
uint32_t timeSeconds = 63000;
uint8_t volumeValue = 40;
#define VAL 5
#define pauseBit (1<<0)		//Uart event bits
#define resumeBit (1<<1)	//Uart event bits
#define clearBit (1<<2)		//Uart event bits
#define Pit_Bit (1<<3)		//Time event bit


static uint32_t RTOS_RunTimeCounter;  //runtime counter, used for configGENERATE_RUNTIME_STATS

void FTM0_IRQHandler(void) {
   //Clear interrupt flag.
  FTM_ClearStatusFlags(FTM0, kFTM_TimeOverflowFlag);
  RTOS_RunTimeCounter++;  //increment runtime counter
}

void RTOS_AppConfigureTimerForRuntimeStats(void) {
  RTOS_RunTimeCounter = 0;
  NVIC_SetPriority(FTM0_IRQn, 2);
  EnableIRQ(FTM0_IRQn);
}

uint32_t RTOS_AppGetRuntimeCounterValueFromISR(void) {
  return RTOS_RunTimeCounter;
}
/*
 * End of real time statistics code
 */

/* PIT_Time interrupt handler */
void PIT_CurrentTime_Handler(void) {
//Pit timer prints time and sets PIT_Bit in event group and a task notification is sent using vTaskNotifyGiveFromISR api call
  uint32_t intStatus;
  static BaseType_t xHigherPriorityTaskWoken;
  //uint8_t ch = 0;
  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

  /* Place your code here */
  xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(Time_EventGroup, Pit_Bit, &xHigherPriorityTaskWoken);
  vTaskNotifyGiveFromISR(currentTime_TaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}
/* PIT_Timer interrupt handler */
void PIT_Timer_Handler(void) {
  uint32_t intStatus;
  static BaseType_t xHigherPriorityTaskWoken;

  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1, intStatus);

  /* Place your code here */
  	xHigherPriorityTaskWoken = pdFALSE;
  	xSemaphoreGiveFromISR(timer_Semaphore, &xHigherPriorityTaskWoken);
  	vTaskNotifyGiveFromISR(timer_TaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


/* UART0_RX_TX_IRQn interrupt handler */
void UART_Handler(void) {
  uint32_t intStatus;
  static BaseType_t xHigherPriorityTaskWoken;
  uint8_t ch;
  /* Reading all interrupt flags of status registers */
  intStatus = UART_GetStatusFlags(UART0_PERIPHERAL);

  /* Flags can be cleared by reading the status register and reading/writing data registers.
    See the reference manual for details of each flag.
    The UART_ClearStatusFlags() function can be also used for clearing of flags in case the content of data regsiter is not used.
    For example:
        status_t status;
        intStatus &= ~(kUART_RxOverrunFlag | kUART_NoiseErrorFlag | kUART_FramingErrorFlag | kUART_ParityErrorFlag);
        status = UART_ClearStatusFlags(UART0_PERIPHERAL, intStatus);
  */

  /* Place your code here */
   ch = UART_ReadByte(UART0);
   xHigherPriorityTaskWoken = pdFALSE;
   switch(ch){
   case 'p': xEventGroupSetBitsFromISR(UART_EventGroup, pauseBit, &xHigherPriorityTaskWoken); break;
   case 'r': xEventGroupSetBitsFromISR(UART_EventGroup, resumeBit, &xHigherPriorityTaskWoken); break;
   case 'c': xEventGroupSetBitsFromISR(UART_EventGroup, clearBit, &xHigherPriorityTaskWoken); break;
   default: printf("Invalid Entry");
   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    xTaskCreate(initTask, "Init Task", 300, NULL, 4, NULL);
    vTaskStartScheduler();

    while(1) {
    }
    return 0 ;
}

void initTask(void *pvParameters) {
	char ID[9];
	printf("Starting Init Task\n\r");
    while(1) {
    	if(espSendATCommand(AT, OK_Response, 2000) == TIMEOUT) {
    		printf("\n\r**** No Response, Check ESP8266 Connections\n\r");
    	}
    	else
    		break;
    }

    //Configure ESP8266 in station mode
    espSendATCommand(modeSetStation, OK_Response, 2000);
    //List available WiFi access points
    //espSendATCommand(listAPs, OK_Response, 20000);

    //Connect to WiFi access points
    //Enter your WiFi access point SSID and password here
      espAPConnect("AndroidHotspot4169", "5a21e7d21bfa");
   


    //Create a unique client ID based on micro ID
    memset(ID, 0, sizeof(ID));
    itoa(SIM->UIDL, ID, 10);

    //Connect to MQTT broker
    if(Client_Connect("io.adafruit.com", ID, 1, "Gallagher123", "aio_Fhas2723b5Pxvnpm2t1T6E31usNk") == MQTT_CONNECT_OK) {
    	printf("\n\n\rConnected to MQTT Broker, creating publish topic\n\n\r");
    	xTaskCreate(publishTask, "Publish Task", 300, NULL, 3, NULL);
    	xTaskCreate(SW2_Task, "SW2 Task", 100, NULL, 3, NULL);
    	xTaskCreate(subscribeTask, "Subscribe Task", 300, NULL, 2, NULL);
    	xTaskCreate(volumeControlTask, "Volume Control Task", 100, NULL, 3, NULL);
    	xTaskCreate(currentTimeTask, "Current Time Task", 300, NULL, 3, &currentTime_TaskHandle);
    	xTaskCreate(timerTask, "Timer Task", 300, NULL, 3, &timer_TaskHandle);
    	timer_Semaphore = xSemaphoreCreateBinary();
    	SW2_Semaphore = xSemaphoreCreateBinary();
    	VC_Semaphore = xSemaphoreCreateBinary();
    	UART_EventGroup = xEventGroupCreate();
    	Time_EventGroup = xEventGroupCreate();
    	vTaskDelete(NULL);							//Delete current Task(init task)
    }
    else {
    	printf("\n\n\r Could not connect to MQTT broker, stopping application\n\n\r");
    	while(1);
    }
	while(1) {
	}
}
static void timerTask(void *pvParameters){
	uint32_t myTime = 0;
	uint32_t timer_nv = 0;	//notification Value
	uint32_t timerSeconds = 0;
	EventBits_t UART_bits;
	printf("Starting Timer Task\n\r");
	printf("Timer: %d\n\r", myTime);
	while(1){
		if(xSemaphoreTakeFromISR(timer_Semaphore, 0) == pdTRUE){
			printf("Timer: %d\n\r", ++myTime);
		}

		UART_bits = xEventGroupWaitBits(UART_EventGroup, pauseBit | resumeBit | clearBit, pdTRUE, pdFALSE, 0);
		if(UART_bits & pauseBit){
			PIT_StopTimer(PIT, 0);
			printf("Timer Paused\n\r");
		}
		if(UART_bits & resumeBit){
			PIT_StartTimer(PIT, 0);
			printf("Timer Resumed\n\r");
		}
		if(UART_bits & clearBit){
			myTime = 0;
			printf("Timer Cleared\n\r");
		}
	}
	timer_nv = ulTaskNotifyTake(pdTRUE, 0);
	if(timer_nv > 0){
		timerSeconds++;
	}
}

void currentTimeTask(void *pvParameters){
	uint32_t time_nv= 0;
	//Wait for PIT to set event group bit PIT_BIT
	Event_bits = xEventGroupWaitBits(Time_EventGroup, Pit_Bit ,pdTRUE, pdFALSE, 0);
	printf("%02d:%02d:%02d\n\r",timeSeconds/3600,(timeSeconds%3600/60),timeSeconds%60);
	if(Event_bits & Pit_Bit){
		timeSeconds++;
		if(timeSeconds == 86400){
			timeSeconds = 0;
		}
	}
	time_nv = ulTaskNotifyTake(pdTRUE, 0);
	if(time_nv > 0){
		timeSeconds = 0;
	}
}

void volumeControlTask(void *pvParameters){
	while(1){
		if(GPIO_PinRead(GPIOA, 4) == 0) {
			xSemaphoreGive(VC_Semaphore);
			while(GPIO_PinRead(GPIOA, 4) == 0);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/*
Poll SW2 every 100ms and send semaphore to publish task when switch is pressed
*/
void SW2_Task(void *pvParameters) {
	while(1) {
		if(GPIO_PinRead(GPIOC, 6) == 0) {
			xSemaphoreGive(SW2_Semaphore);
			while(GPIO_PinRead(GPIOC, 6) == 0);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/*
 * Publish to SW2 data feed when semaphore is available
 */
void publishTask(void *pvParameters) {
	while(1) {
		if(xSemaphoreTake(SW2_Semaphore, portMAX_DELAY) == pdTRUE) {
			Client_PublishNumberData("Gallagher123/feeds/sw2-data-feed", rand()%51, 0);		//rand function generates a random number from 0 to 50
		}

		if(xSemaphoreTake(VC_Semaphore, portMAX_DELAY) == pdTRUE){
			Client_PublishNumberData("Gallagher123/feeds/volume-control-feed", volumeValue += VAL, 0);
		}
		if(xEventGroupGetBitsFromISR(Time_EventGroup)){
			Client_PublishNumberData("Gallagher123/feeds/current-time-feed",Event_bits , 0);
		}
		if(xEventGroupGetBitsFromISR(UART_EventGroup)){
			Client_PublishNumberData("Gallagher123/feeds/timer-display-feed", UART_bits, 0);
		}
		keep_Alive();	//this will keep the connection alive if no messages are published
	}
}

/*
 * On start up this task subscribes to the LED control feed
 * It then check for a published packet from subscribed feed
 * Published packet payload is then parsed
 */
void subscribeTask(void *pvParameters) {
	char publishTopic[40], publishPayload[20];
	uint8_t slider;
	Client_Subscribe("Gallagher123/feeds/led-control-feed", 1);
	Client_Subscribe("Gallagher123/feeds/volume-control-feed",2);
	Client_Subscribe("Gallagher123/feeds/current-time-feed", 3);
	while(1) {
		if(publishPacketCheck(publishTopic, publishPayload) == SUCCESS) {
			printf("\n\rPayload %s received from topic %s\n\n\r", publishPayload, publishTopic);
			if(strstr(publishTopic, "Gallagher123/feeds/led-control-feed")) {	//test for message from toggle switch block
				if(strstr(publishPayload, "LED ON")) {			//check for substring "LED ON"
					LED_RED_ON();
				}
				else if(strstr(publishPayload, "LED OFF")) {	//check for substring "LED OFF"
					LED_RED_OFF();
				}
				vTaskDelay(pdMS_TO_TICKS(100));
			}
			printf("\n\rPayload %s received from topic %s\n\n\r", publishPayload, publishTopic);
			if(strstr(publishTopic, "Gallagher123/feeds/volume-control-feed")) {
				if(strstr(publishPayload, "40")){
					slider = volumeValue += VAL;			//increments the volume by VAL
					printf("Volume Data %02d\n\r",slider);
				}
				else if(strstr(publishPayload, "50")){
					slider = volumeValue -= VAL;			//decrements the volume by VAL
					printf("Volume Data %02d\n\r",slider);
				}
				vTaskDelay(pdMS_TO_TICKS(500));
			}
			printf("\n\rPayload %s received from topic %s\n\n\r", publishPayload, publishTopic);
			if(strstr(publishTopic, "Gallagher123/feeds/current-time-feed")) {
				if(strstr(publishPayload, "%02d:%02d:%02d")){
					printf("%02d:%02d:%02d/n/r",timeSeconds/3600,(timeSeconds%3600/60),timeSeconds%60);
				}

				vTaskDelay(pdMS_TO_TICKS(500));
			}
		}
	}
}




