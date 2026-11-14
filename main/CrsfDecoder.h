#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <math.h>

#ifndef CRSF_PORT
#define CRSF_PORT      UART_NUM_1
#endif
#define CRSF_BAUDRATE  420000
#define CRSF_CRC_POLY 0xD5

typedef struct __attribute__((packed)) {
    unsigned ch0  : 11;
    unsigned ch1  : 11;
    unsigned ch2  : 11;
    unsigned ch3  : 11;
    unsigned ch4  : 11;
    unsigned ch5  : 11;
    unsigned ch6  : 11;
    unsigned ch7  : 11;
    unsigned ch8  : 11;
    unsigned ch9  : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} crsf_channels_t ;

typedef struct __attribute__((packed)) {
    int8_t uplinkRssi1;
    int8_t uplinkRssi2;
    uint8_t uplinkLq;
    int8_t uplinkSnr;
    uint8_t activeAntenna;
    uint8_t rfMode;
    uint8_t uplinkTxPower;
    int8_t downlinkRssi;
    int8_t downlinkLq;
    int8_t downlinkSnr;
} crsf_link_status_t;

extern QueueHandle_t ChannelQueue;
extern QueueHandle_t LinkStatusQueue;

extern crsf_channels_t CrsfChannels;
extern crsf_link_status_t CrsfLinkStatus;

void crsf_capture_task(void *args);

void channel_queue_receive_task(void *args);

void link_status_receive_task(void *arg);

void crsf_send_device_info(void);

void crsf_send_telemetry(double voltageRaw, double currentRaw, double airspeedRaw);

#ifdef __cplusplus
}
#endif