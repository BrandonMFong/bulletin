/**
 * author: brando
 * date: 11/3/23
 */

#ifndef LCDUTIL_H
#define LCDUTIL_H

#include <stdbool.h>
#include "esp_lcd_panel_io.h"
#include "lvgl.h"

void lcd_init(void);
int BNLcdPrint(const char * line);

#endif // LCDUTIL_H

