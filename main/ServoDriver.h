#pragma once

#include "driver/ledc.h"
#include "esp_err.h"
#include "soc/gpio_num.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_MAX_CHANNELS 8

typedef struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
} servo_channel_t;

typedef struct {
    ledc_timer_t timer;
    ledc_mode_t speed_mode;
    uint32_t freq_hz;
    uint32_t resolution;

    servo_channel_t channels[SERVO_MAX_CHANNELS];
    uint8_t channel_count;
} servo_driver_t;

/* Driver (timer) */
esp_err_t servo_driver_init(servo_driver_t *drv);

/* Channel (servo) */
esp_err_t servo_attach(
    servo_driver_t *drv,
    gpio_num_t gpio,
    ledc_channel_t channel
);

/* Control */
esp_err_t servo_write_us(
    servo_driver_t *drv,
    ledc_channel_t channel,
    uint32_t pulse_us
);

#ifdef __cplusplus
}
#endif