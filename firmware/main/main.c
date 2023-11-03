/**
 * author: brando
 * date: 11/3/23
 */

#include "bulletin.h"
#include "uartutil.h"
#include "lcdutil.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
	lcd_init();
    xTaskCreate(BNEchoTask, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}

