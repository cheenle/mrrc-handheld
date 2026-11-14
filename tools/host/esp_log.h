#pragma once
#include <stdio.h>
#define ESP_LOGI(t, ...) do { printf("I %s: ", t); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ESP_LOGW(t, ...) ESP_LOGI(t, __VA_ARGS__)
#define ESP_LOGE(t, ...) ESP_LOGI(t, __VA_ARGS__)
