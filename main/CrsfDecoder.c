#include "CrsfDecoder.h"

crsf_channels_t CrsfChannels;
crsf_link_status_t CrsfLinkStatus;
QueueHandle_t ChannelQueue;
QueueHandle_t LinkStatusQueue;

static uint8_t crsf_calc_crc(uint8_t type, const uint8_t *payload, uint8_t length) {
    uint8_t crc = 0x00;
    uint8_t payload_len = length - 2;
    crc ^= type;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80)
            crc = (crc << 1) ^ CRSF_CRC_POLY;
        else
            crc <<= 1;
    }
    for (uint8_t i = 0; i < payload_len; i++) {
        crc ^= payload[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRSF_CRC_POLY;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void crsf_capture_task(void *arg) {
    uint8_t byte;
    uint8_t length = 0;
    uint8_t type = 0;
    uint8_t payload[64];

    enum {
        WAIT_ADDR,
        WAIT_LEN,
        WAIT_TYPE,
        WAIT_PAYLOAD,
        WAIT_CRC
    } state = WAIT_ADDR;

    int payload_len = 0;
    int payload_index = 0;

    while (1) {
        if (uart_read_bytes(CRSF_PORT, &byte, 1, portMAX_DELAY) != 1) {
            continue;
        }

        switch (state) {

            case WAIT_ADDR:
                if (byte == 0xC8) {
                    state = WAIT_LEN;
                }
                break;

            case WAIT_LEN:
                length = byte;
                if (length < 2 || length > 62) {
                    state = WAIT_ADDR;
                } else {
                    state = WAIT_TYPE;
                }
                break;

            case WAIT_TYPE:
                if (byte != 0x16 && byte != 0x14) {
                    state = WAIT_ADDR;
                    break;
                }
                type = byte;
                payload_len = length - 2;
                payload_index = 0;
                state = (payload_len == 0) ? WAIT_CRC : WAIT_PAYLOAD;
                break;

            case WAIT_PAYLOAD:
                payload[payload_index++] = byte;
                if (payload_index >= payload_len) {
                    state = WAIT_CRC;
                }
                break;

            case WAIT_CRC: {
                uint8_t crc = byte;
                if (crc == crsf_calc_crc(type, payload, length)) {
                    if (type == 0x16 && length == 24) {
                        crsf_channels_t tmp;
                        memcpy(&tmp, payload, sizeof(tmp));
                        xQueueOverwrite(ChannelQueue, &tmp);
                    } else if (type == 0x14 && length == 12) {
                        crsf_link_status_t tmp;
                        memcpy(&tmp, payload, sizeof(tmp));
                        xQueueOverwrite(LinkStatusQueue, &tmp);
                    }
                }
                state = WAIT_ADDR;
                break;
            }

        } // switch
    } // while
}

void channel_queue_receive_task(void *args) {
    crsf_channels_t tempChannelData;
    while (1) {
        if (xQueueReceive(ChannelQueue, &tempChannelData, portMAX_DELAY)) {
            CrsfChannels = tempChannelData;
        }
    }
}

void link_status_receive_task(void *arg) {
    crsf_link_status_t tempLinkStatus;
    while (1) {
        if (xQueueReceive(LinkStatusQueue, &tempLinkStatus, portMAX_DELAY)) {
            CrsfLinkStatus = tempLinkStatus;
        }
    }
}

static void crsf_send(uint8_t type, const uint8_t *payload, uint8_t length) {
    uint8_t frame[64];
    uint8_t idx = 0;

    frame[idx++] = 0xEC;
    frame[idx++] = length + 2;
    frame[idx++] = type;

    memcpy(&frame[idx], payload, length);
    idx += length;
    frame[idx++] = crsf_calc_crc(type, payload, length + 2);
    uart_write_bytes(CRSF_PORT, (const char *)frame, idx);
}

void crsf_send_device_info(void) {
    uint8_t payload[] = {0, 0x0A, 0, 0x01, 0, 0x01, 0, 0, 0, 0, 0, 0, 0};
    crsf_send(0x1E, payload, sizeof(payload));
}

void crsf_send_telemetry(double voltageRaw, double currentRaw, double airspeedRaw) {
    int16_t voltage = (int16_t)lround(voltageRaw*10);
    int16_t current = (int16_t)lround(currentRaw*10);
    uint32_t airspeed = (uint32_t)lround(airspeedRaw*100);
    if (airspeed > 0xFFFFFF) {
        airspeed = 0xFFFFFF;
    }
    uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    payload[0] = (uint8_t)(voltage >> 8);
    payload[1] = (uint8_t)(voltage & 0xFF);
    payload[2] = (uint8_t)(current >> 8);
    payload[3] = (uint8_t)(current & 0xFF);
    payload[4] = (uint8_t)(airspeed >> 16) & 0xFF;
    payload[5] = (uint8_t)(airspeed >> 8) & 0xFF;
    payload[6] = (uint8_t)(airspeed & 0xFF);
    crsf_send(0x08, payload, sizeof(payload));
}