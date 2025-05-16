//==================================================================================================
//
//	File Name		 : UI.c
//	CPU Type			: ESP32-C3
//	Project Name		: ESP32_C3_LCD_128Inch
//
//	Description		 : UI driver for ESP32-C3 with 1.28 inch LCD using LVGL
//
//	History			 : Ver.0.01		2024.11.27 V.Vu	 New
//
//==================================================================================================
//==================================================================================================
//	Compile Option
//==================================================================================================

//==================================================================================================
//	#pragma section
//==================================================================================================

//==================================================================================================
//	Local Compile Option
//==================================================================================================

//==================================================================================================
//	Header File
//==================================================================================================
#include <stdio.h>
#include "UI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "CS816D.h"
#include "GC9A01.h"
#include "lvgl.h"
#include "esp_timer.h"
//==================================================================================================
//	Local define
//==================================================================================================
#define U4_DISPLAY_HEIGHT	((U4)GC9A01_Height)
#define U4_DISPLAY_WIDTH	((U4)GC9A01_Width)
//==================================================================================================
//	Local define I/O
//==================================================================================================

//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM 
//==================================================================================================
extern DMA_ATTR uint16_t U2_ScreenBuff[U4_DISPLAY_HEIGHT * U4_DISPLAY_WIDTH];
static lv_display_t *pst_DisplayDriver = NULL;
static lv_color_t *buf1 = NULL;
//==================================================================================================
//	Local ROM
//==================================================================================================
static const char *TAG = "UI";
//==================================================================================================
//	Local Function Prototype
//==================================================================================================
static void lvgl_tick_init(void);
static void lv_tick_task(void *arg);
static void display_Flush_Callback(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p);
static void create_ui(void);
//==================================================================================================
//	Source Code
//==================================================================================================
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			UIÅz
//	Name:			UI_Init
//	Function:		Initialize LVGL, display driver, and LCD buffer
//
//	Argument:		-
//	Return value:	-
//	Create:			2024.11.27 V.Vu	 New
//	Change:			-
//	Remarks:		Initialize LVGL, display driver, buffer, and tick timer
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void UI_Init(void)
{			
	GC9A01_Init();	// Init LCD driver
	lv_init();		// Initialize LVGL
	pst_DisplayDriver = lv_display_create(U4_DISPLAY_HEIGHT, U4_DISPLAY_WIDTH);		// Create display driver with LCD size
	lv_display_set_flush_cb(pst_DisplayDriver, display_Flush_Callback);				// Set flush callback for display driver

	// Allocate memory for display buffer
	buf1 = heap_caps_malloc(U4_DISPLAY_WIDTH * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

	// Check if buffer allocation is successful
	if (!buf1) {
		ESP_LOGE(TAG, "Failed to allocate LVGL buffer!");
		return;
	}
	// Set buffer for display driver
	lv_display_set_buffers(pst_DisplayDriver, buf1, NULL, U4_DISPLAY_WIDTH * 40 * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);	
	lvgl_tick_init();			// Initialize LVGL tick timer
	create_ui();				// Create basic UI (button)
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			ÅyUIÅz
//	Name:			UI_Job
//	Function:		Handle UI tasks (should be called in main loop)
//
//	Argument:		-
//	Return value:	-
//	Create:			2024.11.27 V.Vu	 New
//	Change:			-
//	Remarks:		 Call lv_timer_handler to update UI
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void UI_Job(void)
{
	// Handle LVGL tasks (update UI)
	lv_timer_handler();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			ÅyUIÅz
//	Name:			lvgl_tick_init
//	Function:		Initialize periodic timer for LVGL tick
//
//	Argument:		-
//	Return value:	-
//	Create:			2024.11.27 V.Vu	 New
//	Change:			-
//	Remarks:		 Create timer to call lv_tick_task every 1ms
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void lvgl_tick_init(void)
{
	// Configure parameters for periodic timer
	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &lv_tick_task,
		.name = "lvgl_tick"
	};

	esp_timer_handle_t periodic_timer;

	// Create periodic timer
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

	// Start timer with 1ms period
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000)); // 1000us = 1ms
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			ÅyUIÅz
//	Name:			lv_tick_task
//	Function:		Increase LVGL tick every 1ms
//
//	Argument:		arg - not used
//	Return value:	-
//	Create:			2024.11.27 V.Vu	 New
//	Change:			-
//	Remarks:		 Callback for LVGL timer
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void lv_tick_task(void *arg)
{
	// Increase LVGL tick (1ms)
	lv_tick_inc(1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			ÅyUIÅz
//	Name:			display_Flush_Callback
//	Function:		Send display area data from LVGL to LCD
//
//	Argument:		disp		- LVGL display driver
//					area		- area to update
//					color_p - color buffer
//	Return value:	-
//	Create:			2024.11.27 V.Vu	 New
//	Change:			-
//	Remarks:		 Called by LVGL when display area needs update
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void display_Flush_Callback(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
	// Get coordinates of area to update
	U4 au4_x1 = area->x1;
	U4 au4_y1 = area->y1;
	U4 au4_x2 = area->x2;
	U4 au4_y2 = area->y2;
	U4 au4_size;

	GC9A01_SetWindow(au4_x1, au4_y1, au4_x2, au4_y2);									// Set drawing window on LCD
	au4_size = (au4_x2 - au4_x1 + 1) * (au4_y2 - au4_y1 + 1) * sizeof(uint16_t);		// Calculate size of area to send
	lcd_data((uint8_t *)color_p, au4_size);												// Send color data to LCD
	lv_disp_flush_ready(disp);															// Notify LVGL that flushing is done
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			ÅyUIÅz
//	Name:			create_ui
//	Function:		Create basic user interface (button)
//
//	Argument:		-
//	Return value:	-
//	Create:			2024.11.27 V.Vu	 New
//	Change:			-
//	Remarks:		 Create a button at the center of the screen
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void create_ui(void)
{
	lv_obj_t * btn = lv_btn_create(lv_scr_act());			// Create a button on the main screen
	lv_obj_set_size(btn, 100, 50);							// Set button size
	lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);				// Align button to center of the screen
	
	lv_obj_t * label = lv_label_create(btn);				// Create a label on the button
	lv_label_set_text(label, "Click me!");					// Set label text
	lv_obj_center(label);									// Center the label on the button
}
