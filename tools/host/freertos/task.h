#pragma once
#include "freertos/FreeRTOS.h"
void host_task_stub(void *fn, const char *name, unsigned stack, void *arg,
                    unsigned prio, void *handle, int core);
#define xTaskCreatePinnedToCore(fn, name, stack, arg, prio, handle, core) \
    host_task_stub((void *)(fn), name, stack, arg, prio, handle, core)
#define vTaskDelay(x) ((void)(x))
