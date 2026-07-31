#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "can.h"
#include <stdbool.h>

bool app_tasks_start(void);
bool app_can_send(const can_frame_t *frame);
bool app_can_receive(can_frame_t *frame);

#endif
