/*
 * main.c - CRSF -> IBT-2 motor + 3 servos, ESP32-C3, ESP-IDF v6.x
 *
 *   ch4 < 1700 us   -> disarmed, brake
 *   ch5 rising past 1700 us -> forward 100 % for 100 ms, then brake (ignores endstop)
 *   ch3 1600..2000  -> forward  0..75 % (blocked while endstop pressed)
 *   ch3 1400..1600  -> brake (EN high, both PWM low)
 *   ch3 1000..1400  -> reverse 20..0 %
 *   ch6/ch7/ch8     -> servos, pulse width passed straight through
 *   no frame for FAILSAFE_MS -> brake (servos hold last position)
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "CrsfDecoder.h"
#include "ibt2.h"
#include "ServoDriver.h"

/* --- pins: change to match your board --- */
#define CRSF_RX_GPIO    20
#define CRSF_TX_GPIO    21
#define MOTOR_EN_GPIO   6
#define MOTOR_L_GPIO    0
#define MOTOR_R_GPIO    1
#define ENDSTOP_GPIO    5           /* switch to GND, low = pressed */
#define SERVO6_GPIO     4
#define SERVO7_GPIO     7
#define SERVO8_GPIO     8

/* LEDC: motor uses timer 0 + ch 0/1, servos use timer 1 + ch 2/3/4 */
#define SERVO6_CH       LEDC_CHANNEL_2
#define SERVO7_CH       LEDC_CHANNEL_3
#define SERVO8_CH       LEDC_CHANNEL_4

#define DEADBAND_LO     1400
#define DEADBAND_HI     1600
#define STICK_SPAN      400         /* 1600->2000 and 1400->1000 */
#define ARM_THRESHOLD   1700        /* ch4 must be at or above this to drive */
#define BUMP_THRESHOLD  1700        /* ch5 rising past this triggers a bump */
#define BUMP_MS         20
#define FWD_LIMIT_PCT   75
#define REV_LIMIT_PCT   20
#define FAILSAFE_MS     100

/* CRSF raw 11-bit (172..1811) -> microseconds (~988..2012), 992 == 1500 us */
static inline int crsf_to_us(unsigned raw)
{
    return 1500 + (((int)raw - 992) * 5) / 8;
}

/* Map 0..STICK_SPAN of stick travel to 0..max duty, clamped */
static inline uint32_t stick_to_duty(int travel, uint32_t max)
{
    uint32_t d = (uint32_t)travel * max / STICK_SPAN;
    return d > max ? max : d;
}

void app_main(void)
{
    /* UART for the receiver */
    const uart_config_t ucfg = {
        .baud_rate  = CRSF_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(CRSF_PORT, &ucfg));
    ESP_ERROR_CHECK(uart_set_pin(CRSF_PORT, CRSF_TX_GPIO, CRSF_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CRSF_PORT, 256, 0, 0, NULL, 0));

    /* Length 1 is required for xQueueOverwrite() in the decoder */
    ChannelQueue    = xQueueCreate(1, sizeof(crsf_channels_t));
    LinkStatusQueue = xQueueCreate(1, sizeof(crsf_link_status_t));

    /* Endstop input with internal pull-up */
    const gpio_config_t es = {
        .pin_bit_mask = 1ULL << ENDSTOP_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&es));

    /* Motor: starts enabled and braked */
    static ibt2_t motor;
    const ibt2_config_t mcfg = {
        .en_gpio    = MOTOR_EN_GPIO,
        .l_pwm_gpio = MOTOR_L_GPIO,
        .r_pwm_gpio = MOTOR_R_GPIO,
        .timer      = LEDC_TIMER_0,
        .l_channel  = LEDC_CHANNEL_0,
        .r_channel  = LEDC_CHANNEL_1,
        /* freq, resolution, clk left 0 -> 20 kHz, 10 bit, auto */
    };
    ESP_ERROR_CHECK(ibt2_init(&motor, &mcfg));

    /* Servos: 50 Hz, 14 bit (~1.2 us steps). No pulses until the first frame. */
    static servo_driver_t servos = {
        .timer      = LEDC_TIMER_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .freq_hz    = 50,
        .resolution = LEDC_TIMER_14_BIT,
    };
    ESP_ERROR_CHECK(servo_driver_init(&servos));
    ESP_ERROR_CHECK(servo_attach(&servos, SERVO6_GPIO, SERVO6_CH));
    ESP_ERROR_CHECK(servo_attach(&servos, SERVO7_GPIO, SERVO7_CH));
    ESP_ERROR_CHECK(servo_attach(&servos, SERVO8_GPIO, SERVO8_CH));

    xTaskCreate(crsf_capture_task, "crsf_rx", 4096, NULL, 10, NULL);

    const uint32_t full    = ibt2_duty_max(&motor);
    const uint32_t fwd_max = full * FWD_LIMIT_PCT / 100;
    const uint32_t rev_max = full * REV_LIMIT_PCT / 100;
    bool bump_prev = true;          /* switch must be seen low before the first bump */
    crsf_channels_t ch;

    while (1) {
        if (xQueueReceive(ChannelQueue, &ch, pdMS_TO_TICKS(FAILSAFE_MS)) != pdTRUE) {
            ibt2_brake(&motor);             /* signal lost */
            continue;
        }

        /* Servos: straight passthrough */
        servo_write_us(&servos, SERVO6_CH, (uint32_t)crsf_to_us(ch.ch6));
        servo_write_us(&servos, SERVO7_CH, (uint32_t)crsf_to_us(ch.ch7));
        servo_write_us(&servos, SERVO8_CH, (uint32_t)crsf_to_us(ch.ch8));

        int  throttle = crsf_to_us(ch.ch3);
        bool armed    = crsf_to_us(ch.ch4) >= ARM_THRESHOLD;
        bool bump     = crsf_to_us(ch.ch5) >  BUMP_THRESHOLD;
        bool endstop  = gpio_get_level(ENDSTOP_GPIO) == 0;

        bool bump_edge = bump && !bump_prev;
        bump_prev = bump;

        if (!armed) {
            ibt2_brake(&motor);             /* disarmed */
        } else if (bump_edge) {
            ibt2_set_duty(&motor, IBT2_DIR_FORWARD, full);  /* ignores endstop */
            vTaskDelay(pdMS_TO_TICKS(BUMP_MS));
            ibt2_brake(&motor);
        } else if (throttle > DEADBAND_HI && !endstop) {
            ibt2_set_duty(&motor, IBT2_DIR_FORWARD,
                          stick_to_duty(throttle - DEADBAND_HI, fwd_max));
        } else if (throttle < DEADBAND_LO) {
            ibt2_set_duty(&motor, IBT2_DIR_REVERSE,
                          stick_to_duty(DEADBAND_LO - throttle, rev_max));
        } else {
            ibt2_brake(&motor);             /* neutral, or forward blocked by endstop */
        }
    }
}