/*
 * button_task.h
 * 物理按键任务：轮询两个按键(手动开始/停止、手动重新标定)，做软件去抖动，
 * 检测到有效按下时往 g_button_event_queue 发事件
 */

#ifndef BUTTON_TASK_H
#define BUTTON_TASK_H

void button_task(void *pvParameters);

#endif // BUTTON_TASK_H
