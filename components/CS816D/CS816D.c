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
#include "Define.h"
//==================================================================================================
//	Local define
//==================================================================================================

#define	U1_TP_RST_PIN				((U1)1)
#define	U1_TP_INT_PIN				((U1)0)

#define I2C_MASTER_PORT				I2C_NUM_0
#define U1_I2C_MASTER_SDA_IO		((U1)4)
#define U1_I2C_MASTER_SCL_IO		((U1)5)
#define U4_I2C_MASTER_FREQ_HZ		((U4)400000)
#define U1_I2C_ADDR_CST816D			((U1)0x1)

// CST816D configuration
#define CST816D_ADDR 				((U1)0x15)
#define CST816D_REG_GESTURE			((U1)0x01)
#define CST816D_REG_FINGERNUM		((U1)0x02)
#define CST816D_REG_XPOS_H			((U1)0x03)
#define CST816D_REG_XPOS_L			((U1)0x04)
#define CST816D_REG_YPOS_H			((U1)0x05)
#define CST816D_REG_YPOS_L			((U1)0x06)
//==================================================================================================
//	Local define I/O
//==================================================================================================

//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM
//==================================================================================================
static i2c_master_dev_handle_t cst816d_dev;		// cst816d touch device handle
static i2c_master_bus_handle_t i2cbus_handle;	// I2C master bus handle
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
static U1 u1_cst816d_i2c_read_continous(U1 u1_RegAdd, U1 *apu1_Data, U2 au2_Len);
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
	// Initialize I2C bus
	i2c_master_bus_config_t i2cbus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_MASTER_PORT,
		.scl_io_num = U1_I2C_MASTER_SCL_IO,
		.sda_io_num = U1_I2C_MASTER_SDA_IO,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2cbus_config, &i2cbus_handle));

	// Add device to I2C bus
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = CST816D_ADDR,
		.scl_speed_hz = U4_I2C_MASTER_FREQ_HZ,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_cfg, &cst816d_dev));

	// Configure INT pin as input
	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << U1_TP_INT_PIN),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_NEGEDGE,
	};
	gpio_config(&io_conf);

	// Configure RST pin as output and perform reset
	gpio_config_t rst_conf = {
		.pin_bit_mask = (1ULL << U1_TP_RST_PIN),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&rst_conf);

	// Reset sequence
	gpio_set_level(U1_TP_RST_PIN, 0);
	vTaskDelay(10/portTICK_PERIOD_MS);
	gpio_set_level(U1_TP_RST_PIN, 1);
	vTaskDelay(50/portTICK_PERIOD_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	u1_CS816D_ReadTouch
//	Function:	Get position data from CS816D
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
U1 u1_CS816D_ReadTouch(ST_TOUCH_DATA *apst_TouchData)
{
	U1 au1_Data[6];
	U1 au1_Ret;
	au1_Ret = U1NG;
	if (gpio_get_level(U1_TP_INT_PIN) == U1OFF) 
	{
		au1_Ret = u1_cst816d_i2c_read_continous(CST816D_REG_GESTURE, au1_Data, sizeof(au1_Data));
		// Read all touch data registers
		if (au1_Ret == U1OK) 
		{
			apst_TouchData->u1_Gesture = au1_Data[0];
			apst_TouchData->u1_FingerNum = au1_Data[1];
			apst_TouchData->u2_X = ((au1_Data[2] & 0x0F) << 8) | au1_Data[3];
			apst_TouchData->u2_Y = ((au1_Data[4] & 0x0F) << 8) | au1_Data[5];
			
			ESP_LOGI(TAG, "Touch detected - X: %d, Y: %d", apst_TouchData->u2_X, apst_TouchData->u2_Y);
		}
	}
	return au1_Ret;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	u1_cst816d_i2c_read_continous
//	Function:	Read from I2C device using master API
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static U1 u1_cst816d_i2c_read_continous(U1 u1_RegAdd, U1 *apu1_Data, U2 au2_Len)
{
	S1 as1_Timeout;
	esp_err_t ret;
	as1_Timeout = -1; // Timeout in ms
	// Write register address first
	ret = i2c_master_transmit(cst816d_dev, &u1_RegAdd, 1, as1_Timeout);
	if (ret != ESP_OK) {
		return U1NG;
	}
	// Then read data
	ret = i2c_master_receive(cst816d_dev, apu1_Data, au2_Len, as1_Timeout);
	if (ret != ESP_OK) {
		return U1NG;
	}
	return U1OK;
}