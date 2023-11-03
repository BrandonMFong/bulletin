/**
 * author: brando
 * date: 11/3/23
 */

#include <stdbool.h>
#include "esp_lcd_panel_io.h"
#include "lvgl.h"

void example_lvgl_demo_ui(lv_disp_t *disp);
bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void lcd_init(void);

