/*
 * ibt2.h - Minimal IBT-2 (BTS7960) H-bridge driver for ESP32-C3, ESP-IDF v6.x
 *
 * Wiring assumed:
 *   R_EN + L_EN tied together -> en_gpio   (shared enable)
 *   L_PWM                     -> l_pwm_gpio
 *   R_PWM                     -> r_pwm_gpio
 *
 * Truth table (EN high):
 *   R_PWM = PWM, L_PWM = 0  -> forward
 *   R_PWM = 0,   L_PWM = PWM -> reverse
 *   R_PWM = 0,   L_PWM = 0  -> brake (both low-side FETs on)
 * EN low -> outputs high-impedance (coast / free-wheel)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IBT2_DIR_FORWARD = 0,   /* drives R_PWM */
    IBT2_DIR_REVERSE = 1,   /* drives L_PWM */
} ibt2_dir_t;

/*
 * LEDC setup. The driver uses exactly one timer and two channels. Pick ones
 * your other LEDC users don't touch.
 *
 * freq_hz * 2^duty_resolution must not exceed the LEDC source clock
 * (80 MHz APB on the C3), e.g. 20 kHz -> max 11 bit, 1 kHz -> max 14 bit.
 * ledc_timer_config() returns an error from ibt2_init() if it can't be met.
 *
 * On the ESP32-C3 all LEDC timers share ONE clock source. If your other
 * timers use a specific clk_cfg, set the same one here.
 */
typedef struct {
    gpio_num_t        en_gpio;          /* shared R_EN/L_EN pin */
    gpio_num_t        l_pwm_gpio;       /* L_PWM pin */
    gpio_num_t        r_pwm_gpio;       /* R_PWM pin */

    ledc_timer_t      timer;            /* LEDC timer reserved for this motor */
    ledc_channel_t    l_channel;        /* LEDC channel for L_PWM */
    ledc_channel_t    r_channel;        /* LEDC channel for R_PWM (must differ from l_channel) */
    uint32_t          freq_hz;          /* PWM frequency, 0 = 20 kHz */
    ledc_timer_bit_t  duty_resolution;  /* PWM resolution, 0 = 10 bit */
    ledc_clk_cfg_t    clk_cfg;          /* clock source, 0 = LEDC_AUTO_CLK */
} ibt2_config_t;

/* Handle - allocate it yourself (static or on the stack of a long-lived task). */
typedef struct {
    ibt2_config_t cfg;
    uint32_t      duty_full;     /* LEDC duty value meaning 100 % (= 2^duty_resolution) */
} ibt2_t;

/* Configure GPIO + LEDC. Motor ends up enabled and braked (speed 0). */
esp_err_t ibt2_init(ibt2_t *motor, const ibt2_config_t *cfg);

/* Set direction and speed. speed_pct is 0..100 (clamped). */
esp_err_t ibt2_set(ibt2_t *motor, ibt2_dir_t dir, uint8_t speed_pct);

/* Set direction and raw duty, 0..ibt2_duty_max(motor) (clamped).
 * Use this to get the full benefit of a higher resolution. */
esp_err_t ibt2_set_duty(ibt2_t *motor, ibt2_dir_t dir, uint32_t duty);

/* Raw duty value that means 100 %. */
static inline uint32_t ibt2_duty_max(const ibt2_t *motor) { return motor->duty_full; }

/* Signed shortcut: -100..100. Positive = forward, negative = reverse, 0 = brake. */
esp_err_t ibt2_set_signed(ibt2_t *motor, int8_t speed);

/* Both PWM outputs low with EN high -> active brake. */
esp_err_t ibt2_brake(ibt2_t *motor);

/* Drive the shared enable pin. false = outputs off (coast). */
esp_err_t ibt2_enable(ibt2_t *motor, bool enable);

#ifdef __cplusplus
}
#endif
