//==================================================================================================
//
//	File Name	:	CS816D.c
//	CPU Type	:	ESP32
//	IDE			:	ESP-IDF V5.3.1
//	Customer	
//	Version		:	Ver.0.01
//	Coding		:	v.Vu
//	History		:	09/05/2025
//	Outline		:
//
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
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "CS816D.h"
#include "ASTIdef.h"
//==================================================================================================
//	Local define
//==================================================================================================

#define	U1_TP_RST_PIN			(U1)1
#define	U1_TP_INT_PIN			(U1)0

#define I2C_MASTER_PORT          I2C_NUM_0
#define I2C_MASTER_SDA_IO        4
#define I2C_MASTER_SCL_IO        5
#define I2C_MASTER_FREQ_HZ       400000
#define I2C_ADDR_CST816D         0x1
//==================================================================================================
//	Local define I/O
//==================================================================================================

//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM
//==================================================================================================
static i2c_master_dev_handle_t dev_handle;
i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_dev_handle_t cst816d_dev = NULL;

U1 au1_FingerIndex;
U1 au1_data[4];
U1 au1_Gesture;
//==================================================================================================
//	Local ROM
//==================================================================================================
static const char *TAG = "CS816D";
//==================================================================================================
//	Local Function Prototype
//==================================================================================================


//==================================================================================================
//	Source Code
//==================================================================================================
static void cst816d_i2c_write_continous(U1 au1_Adr, U1 *apu1_Data, U1 au1_Len);
static void cst816d_i2c_read_continous(U1 au1_Adr, U1 *apu1_Data, U1 au1_Len);
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	CS816D_Init
//	Function:	Init I2C for CS816D
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void CS816D_Init(void)
{
	ESP_LOGI(TAG, "Initialize CS816D");
	// Configure I2C bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 4,
        .flags.enable_internal_pullup = 1,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

	// Configure master device
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_CST816D,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .flags.disable_ack_check = 0,
    };
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &cst816d_dev));

	// Configure INT and RST pins
	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << U1_TP_INT_PIN) | (1ULL << U1_TP_RST_PIN),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&io_conf));

	ESP_LOGI(TAG, "Config GPIO for CS816D");
	gpio_set_level(U1_TP_INT_PIN, U1HI); // Reset the CS816D
	vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10ms
	gpio_set_level(U1_TP_INT_PIN, U1LO); // Release reset
	vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10ms

	// gpio_set_level(U1_TP_RST_PIN, U1LO); // Reset the CS816D
	// vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10ms
	// gpio_set_level(U1_TP_RST_PIN, U1HI); // Release reset
	// vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10ms
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	CS816D_Read_Touch
//	Function:	Get position data from CS816D
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
U1 u1_CS816D_Read_Touch(U2 *apu2_x, U2 *apu2_y, U1 *apu1_Gesture)
{
	ESP_LOGI(TAG, "Read Touch Data");
	au1_FingerIndex = U1FALSE;

	cst816d_i2c_read_continous(0x02, &au1_FingerIndex, (U1)1);
	cst816d_i2c_read_continous(0x01, &au1_Gesture, (U1)1);
	if (!(au1_Gesture == U1_SlideUp || au1_Gesture == U1_SlideDown))
	{
		au1_Gesture = U1_None;
	}
	*apu1_Gesture = au1_Gesture;

	cst816d_i2c_read_continous(0x03,au1_data,4);
	*apu2_x = ((au1_data[0] & 0x0f) << 8) | au1_data[1];
	*apu2_y = ((au1_data[2] & 0x0f) << 8) | au1_data[3];


	return au1_FingerIndex;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	cst816d_i2c_write_continous
//	Function:	Wite multi byte via I2C
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void cst816d_i2c_write_continous(U1 au1_Adr, U1 *apu1_Data, U1 au1_Len)
{
	U1 au1_Tmp;
	au1_Tmp = (au1_Adr<<1);
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &au1_Tmp, 1, 1000 / portTICK_PERIOD_MS));
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, apu1_Data, au1_Len, 1000 / portTICK_PERIOD_MS));
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	u1_cst816d_i2c_read_continous
//	Function:	Read multi byte via I2C
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void cst816d_i2c_read_continous(U1 au1_Adr, U1 *apu1_Data, U1 au1_Len)
{
	esp_err_t ret = i2c_master_transmit_receive(cst816d_dev, &au1_Adr, 1, apu1_Data, au1_Len, 100/portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }
}
