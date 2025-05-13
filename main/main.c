#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "CS816D.h"
#include "GC9A01.h"
#include "lvgl.h"
#define TAG "TOUCH_TASK_EXAMPLE"
// Queue configuration
#define TOUCH_QUEUE_LENGTH   10
#define TOUCH_ITEM_SIZE	  sizeof(ST_TOUCH_DATA)

ST_TOUCH_DATA st_TouchData;
ST_TOUCH_DATA st_TouchData_Received;

static QueueHandle_t touch_queue = NULL;

static void touch_read_task(void *pvParameters)
{
	while (1)
	{
		if(u1_CS816D_ReadTouch(&st_TouchData) == U1OK)
		{
			// Send touch data to the queue
			if (xQueueSend(touch_queue, &st_TouchData, portMAX_DELAY) != pdTRUE) 
			{
				ESP_LOGE(TAG, "Failed to send touch data to queue");
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void lcd_task(void * arg)
{
	uint16_t Color;
	for(;;)
	{
		if (xQueueReceive(touch_queue, &st_TouchData_Received, portMAX_DELAY) == pdTRUE) 
		{
			Color=rand();
			GC9A01_FillRect(0,0,239,239,Color);
			GC9A01_Update();
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
}

void app_main(void)
{
	CS816D_Init();
	GC9A01_Init();	// Create touch data queue

	touch_queue = xQueueCreate(TOUCH_QUEUE_LENGTH, TOUCH_ITEM_SIZE);
	if (touch_queue == NULL) {
		ESP_LOGE(TAG, "Failed to create touch queue");
		return;
	}

	TaskHandle_t LCDHandle;

	xTaskCreate(touch_read_task, "touch_read_task", 2048, NULL, 5, NULL);
	xTaskCreate(lcd_task,"lcd_task",2048,NULL,tskIDLE_PRIORITY,&LCDHandle);
	configASSERT(LCDHandle);
}