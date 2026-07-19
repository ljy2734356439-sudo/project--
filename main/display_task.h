/*
 * display_task.h
 * 屏幕显示任务（当前为占位版本）
 *
 * 现状说明：ILI9341的SPI驱动和LVGL界面还没开始写，
 * 这个任务先用日志打印代替屏幕显示，确保多任务架构先跑通，
 * 后续把内部实现换成LVGL的画图代码即可，其他任务不需要改动。
 */

#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

void display_task(void *pvParameters);

#endif // DISPLAY_TASK_H
