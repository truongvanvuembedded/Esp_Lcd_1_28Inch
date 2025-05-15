#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "CS816D.h"
#include "GC9A01.h"
#include "lvgl.h"
#define TAG "TOUCH_TASK_EXAMPLE"


extern DMA_ATTR uint16_t ScreenBuff[GC9A01_Height * GC9A01_Width];
static void lv_tick_task(void *arg)
{
    lv_tick_inc(1); // Cộng 1ms cho LVGL
}

void lvgl_tick_init(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "lvgl_tick"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000)); // 1000us = 1ms
}

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;

    // Set drawing window
    GC9A01_SetWindow(x1, y1, x2, y2); // Bạn cần viết hàm này để gọi lcd_cmd và lcd_data phù hợp

    int32_t size = (x2 - x1 + 1) * (y2 - y1 + 1) * sizeof(uint16_t);
    lcd_data((uint8_t *)color_p, size); // color_p là buffer chứa pixel 16-bit (RGB565)

    lv_disp_flush_ready(disp); // Thông báo cho LVGL là vẽ xong
}

void lvgl_init(void)
{
	lv_init();
    static lv_display_t *disp_drv = NULL;

    // Khởi tạo màn hình (GC9A01 init)
    GC9A01_Init(); // Gọi các hàm bạn đã viết để reset, init LCD

    // Tạo display driver LVGL
    disp_drv = lv_display_create(GC9A01_Width, GC9A01_Height);

    // Gắn hàm flush
    lv_display_set_flush_cb(disp_drv, my_disp_flush);

    // Nếu dùng buffer ngoài (PSRAM), bạn có thể set buffer tại đây
    // static lv_color_t *buf1 = NULL;
    // buf1 = heap_caps_malloc(GC9A01_Width * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    // lv_display_set_buffers(disp_drv, buf1, NULL, GC9A01_Width * 40 * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
// Dùng DMA hoặc Internal RAM tuỳ theo nhu cầu
	static lv_color_t *buf1 = NULL;
	buf1 = heap_caps_malloc(GC9A01_Width * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
	if (!buf1) {
		ESP_LOGE(TAG, "Failed to allocate LVGL buffer!");
		return;
	}
	lv_display_set_buffers(disp_drv, buf1, NULL, GC9A01_Width * 40 * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Nếu bạn điều khiển độ sáng bằng PWM thì init LEDC ở đây (tùy chọn)
}



// void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
//     // Thiết lập vùng cần cập nhật
//     GC9A01_SetWindow(area->x1, area->y1, area->x2, area->y2);
    
//     // Sao chép dữ liệu từ bộ đệm LVGL sang bộ đệm màn hình
//     uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
//     for(uint32_t i = 0; i < size; i++) {
//         ScreenBuff[(area->y1 + (i / (area->x2 - area->x1 + 1))) * GC9A01_Width + area->x1 + (i % (area->x2 - area->x1 + 1))] = 
//             color_p[i].full;
//     }
    
//     // Cập nhật toàn bộ màn hình (nếu cần)
//     GC9A01_Update();
    
//     // Báo cho LVGL biết flush đã hoàn thành
//     lv_disp_flush_ready(disp_drv);
// }


// void lvgl_task(void * arg)
// {
//     while(1) {
//         lv_timer_handler();
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

void create_ui()
{
    // Tạo một button mới
    lv_obj_t * btn = lv_btn_create(lv_scr_act()); // Tạo button trên màn hình hiện tại
    lv_obj_set_size(btn, 100, 50);                // Đặt kích thước: rộng 100px, cao 50px
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);     // Căn giữa màn hình
    
    // Thêm nhãn (label) vào button
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Click me!");        // Đặt text hiển thị
    lv_obj_center(label);                         // Căn giữa label trong button
}

void app_main(void)
{
	lvgl_init();
	 lvgl_tick_init();
	create_ui();
	while(1) {
	lv_timer_handler();
	vTaskDelay(pdMS_TO_TICKS(10));
}
}