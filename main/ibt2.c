/*
 * ibt2.c - Minimal IBT-2 (BTS7960) H-bridge driver for ESP32-C3, ESP-IDF v6.x
 */
#include "ibt2.h"
#include "esp_check.h"

static const char *TAG = "ibt2";

/* ESP32-C3 LEDC only has low-speed mode. */
#define IBT2_SPEED_MODE    LEDC_LOW_SPEED_MODE
#define IBT2_DEFAULT_RES   LEDC_TIMER_10_BIT
#define IBT2_DEFAULT_FREQ  20000

static esp_err_t set_duty(ledc_channel_t ch, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(IBT2_SPEED_MODE, ch, duty), TAG, "set_duty");
    return ledc_update_duty(IBT2_SPEED_MODE, ch);
}

esp_err_t ibt2_init(ibt2_t *motor, const ibt2_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(motor && cfg, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(cfg->l_channel != cfg->r_channel, ESP_ERR_INVALID_ARG,
                        TAG, "L and R must use different LEDC channels");

    motor->cfg = *cfg;
    if (motor->cfg.freq_hz == 0) {
        motor->cfg.freq_hz = IBT2_DEFAULT_FREQ;
    }
    if (motor->cfg.duty_resolution == 0) {
        motor->cfg.duty_resolution = IBT2_DEFAULT_RES;
    }
    ESP_RETURN_ON_FALSE(motor->cfg.duty_resolution < LEDC_TIMER_BIT_MAX, ESP_ERR_INVALID_ARG,
                        TAG, "duty_resolution too high");
    /* LEDC duty range is [0, 2^res]; 2^res = fully on. */
    motor->duty_full = 1U << motor->cfg.duty_resolution;

    /* Enable pin: output, start LOW so the bridge stays off while we set up PWM. */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << motor->cfg.en_gpio,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "en gpio");
    ESP_RETURN_ON_ERROR(gpio_set_level(motor->cfg.en_gpio, 0), TAG, "en low");

    /* One timer shared by both PWM channels. */
    ledc_timer_config_t tcfg = {
        .speed_mode      = IBT2_SPEED_MODE,
        .duty_resolution = motor->cfg.duty_resolution,
        .timer_num       = motor->cfg.timer,
        .freq_hz         = motor->cfg.freq_hz,
        .clk_cfg         = motor->cfg.clk_cfg,   /* 0 == LEDC_AUTO_CLK */
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tcfg), TAG, "timer");

    ledc_channel_config_t ccfg = {
        .speed_mode = IBT2_SPEED_MODE,
        .timer_sel  = motor->cfg.timer,
        .duty       = 0,
        .hpoint     = 0,
    };

    ccfg.channel  = motor->cfg.l_channel;
    ccfg.gpio_num = motor->cfg.l_pwm_gpio;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ccfg), TAG, "L channel");

    ccfg.channel  = motor->cfg.r_channel;
    ccfg.gpio_num = motor->cfg.r_pwm_gpio;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ccfg), TAG, "R channel");

    /* Both PWM at 0 -> enabling now gives a braked, stationary motor. */
    return ibt2_enable(motor, true);
}

esp_err_t ibt2_set(ibt2_t *motor, ibt2_dir_t dir, uint8_t speed_pct)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "null arg");
    if (speed_pct > 100) {
        speed_pct = 100;
    }
    return ibt2_set_duty(motor, dir, (motor->duty_full * speed_pct) / 100);
}

esp_err_t ibt2_set_duty(ibt2_t *motor, ibt2_dir_t dir, uint32_t duty)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "null arg");
    if (duty > motor->duty_full) {
        duty = motor->duty_full;
    }

    ledc_channel_t active = (dir == IBT2_DIR_FORWARD) ? motor->cfg.r_channel : motor->cfg.l_channel;
    ledc_channel_t idle   = (dir == IBT2_DIR_FORWARD) ? motor->cfg.l_channel : motor->cfg.r_channel;

    /* Drop the opposite side first, then drive the active side. */
    ESP_RETURN_ON_ERROR(set_duty(idle, 0), TAG, "idle side");
    return set_duty(active, duty);
}

esp_err_t ibt2_set_signed(ibt2_t *motor, int8_t speed)
{
    if (speed >= 0) {
        return ibt2_set(motor, IBT2_DIR_FORWARD, (uint8_t)speed);
    }
    /* int8_t min is -128; widen before negating, ibt2_set clamps to 100. */
    return ibt2_set(motor, IBT2_DIR_REVERSE, (uint8_t)(-(int)speed));
}

esp_err_t ibt2_brake(ibt2_t *motor)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(set_duty(motor->cfg.l_channel, 0), TAG, "L off");
    return set_duty(motor->cfg.r_channel, 0);
}

esp_err_t ibt2_enable(ibt2_t *motor, bool enable)
{
    ESP_RETURN_ON_FALSE(motor, ESP_ERR_INVALID_ARG, TAG, "null arg");
    return gpio_set_level(motor->cfg.en_gpio, enable ? 1 : 0);
}
