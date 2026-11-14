#include "ServoDriver.h"

esp_err_t servo_driver_init(servo_driver_t *drv)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = drv->speed_mode,
        .timer_num        = drv->timer,
        .duty_resolution  = drv->resolution,
        .freq_hz          = drv->freq_hz,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    drv->channel_count = 0;
    return ledc_timer_config(&timer_conf);
}

esp_err_t servo_attach(
    servo_driver_t *drv,
    gpio_num_t gpio,
    ledc_channel_t channel
)
{
    if (drv->channel_count >= SERVO_MAX_CHANNELS)
        return ESP_ERR_INVALID_ARG;

    ledc_channel_config_t chan_conf = {
        .gpio_num   = gpio,
        .speed_mode = drv->speed_mode,
        .channel    = channel,
        .timer_sel  = drv->timer,
        .duty       = 0,
        .hpoint     = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&chan_conf));

    drv->channels[drv->channel_count++] = (servo_channel_t){
        .gpio = gpio,
        .channel = channel
    };

    return ESP_OK;
}

esp_err_t servo_write_us(
    servo_driver_t *drv,
    ledc_channel_t channel,
    uint32_t pulse_us
)
{
    uint32_t period_us = 1000000 / drv->freq_hz;
    uint32_t max_duty  = (1 << drv->resolution) - 1;

    uint32_t duty = (pulse_us * max_duty) / period_us;

    ESP_ERROR_CHECK(ledc_set_duty(
        drv->speed_mode,
        channel,
        duty
    ));

    return ledc_update_duty(
        drv->speed_mode,
        channel
    );
}
