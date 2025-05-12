#include <stdio.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define TAG "TOUCH_TASK_EXAMPLE"

// CST816D configuration
#define CST816D_ADDR 0x15
#define CST816D_REG_GESTURE   0x01
#define CST816D_REG_FINGERNUM 0x02
#define CST816D_REG_XPOS_H    0x03
#define CST816D_REG_XPOS_L    0x04
#define CST816D_REG_YPOS_H    0x05
#define CST816D_REG_YPOS_L    0x06

// Pin configuration
#define I2C_MASTER_SCL_IO    5
#define I2C_MASTER_SDA_IO    4
#define I2C_MASTER_INT_IO    0
#define I2C_MASTER_RST_IO    1
#define I2C_MASTER_FREQ_HZ   400000
#define I2C_MASTER_PORT      I2C_NUM_0

// Queue configuration
#define TOUCH_QUEUE_LENGTH   10
#define TOUCH_ITEM_SIZE      sizeof(touch_data_t)

// Touch data structure
typedef struct {
    uint8_t gesture;
    uint8_t finger_num;
    uint16_t x;
    uint16_t y;
} touch_data_t;

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static QueueHandle_t touch_queue = NULL;

// Initialize I2C master bus
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CST816D_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

// Initialize touch panel
static esp_err_t touch_panel_init(void)
{
    // Configure INT pin as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_INT_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    // Configure RST pin as output and perform reset
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_RST_IO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_conf);

    // Reset sequence
    gpio_set_level(I2C_MASTER_RST_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(I2C_MASTER_RST_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

// Read from I2C device using master API
static esp_err_t i2c_read_reg(uint8_t reg_addr, uint8_t *data, size_t len)
{
    // Write register address first
    esp_err_t ret = i2c_master_transmit(dev_handle, &reg_addr, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Then read data
    return i2c_master_receive(dev_handle, data, len, -1);
}

// Touch reading task
static void touch_read_task(void *pvParameters)
{
    touch_data_t touch_data;
    uint8_t raw_data[6];
    
    while (1) {
        // Check if touch is detected (INT pin is low)
        if (gpio_get_level(I2C_MASTER_INT_IO) == 0) {
            // Read all touch data registers
            if (i2c_read_reg(CST816D_REG_GESTURE, raw_data, sizeof(raw_data)) == ESP_OK) {
                touch_data.gesture = raw_data[0];
                touch_data.finger_num = raw_data[1];
                touch_data.x = ((raw_data[2] & 0x0F) << 8) | raw_data[3];
                touch_data.y = ((raw_data[4] & 0x0F) << 8) | raw_data[5];
                
                // Send data to queue
                if (xQueueSend(touch_queue, &touch_data, pdMS_TO_TICKS(10)) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send touch data to queue");
                }
                
                ESP_LOGI(TAG, "Touch detected - X: %d, Y: %d", touch_data.x, touch_data.y);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 20ms
    }
}

void app_main(void)
{
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Initialize touch panel
    ESP_ERROR_CHECK(touch_panel_init());
    ESP_LOGI(TAG, "Touch panel initialized successfully");
    
    // Create touch data queue
    touch_queue = xQueueCreate(TOUCH_QUEUE_LENGTH, TOUCH_ITEM_SIZE);
    if (touch_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create touch queue");
        return;
    }
    
    // Create touch reading task
    xTaskCreate(touch_read_task, "touch_read_task", 4096, NULL, 5, NULL);
    
    // Main loop to process touch data
    touch_data_t received_data;
    while (1) {
        if (xQueueReceive(touch_queue, &received_data, portMAX_DELAY) == pdTRUE) {
            // Process received touch data here
            ESP_LOGI(TAG, "Processed touch - Gesture: 0x%02X, Fingers: %d, X: %d, Y: %d",
                    received_data.gesture, received_data.finger_num, 
                    received_data.x, received_data.y);
            
            // Add your application logic here
        }
    }
}