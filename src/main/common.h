/**
 * @file common.h
 * @brief Common header file for shared includes and definitions.
 *
 * This file includes a set of standard libraries and project-specific headers
 * that are used across multiple files. It helps to reduce redundant include
 * statements and provides a central place for common definitions.
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "cJSON.h"

#endif // COMMON_H
