#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define UART_NUM UART_NUM_2 //UART port 2
#define TX_PIN (GPIO_NUM_17) // TX pin
#define RX_PIN (GPIO_NUM_16) // RX pin
#define BAUD_RATE 115200

#define millis() (unsigned long) (esp_timer_get_time() / 1000)
uint32_t _timeout;

#define MSP_MAX_SUPPORTED_CHANNELS 16

typedef struct {
  uint16_t channelValue[MSP_MAX_SUPPORTED_CHANNELS];
} msp_rc_t;


typedef struct {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;
} RCData;

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
}

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// bool msp_recv(uint16_t *messageID, void *payload, uint16_t maxSize, uint16_t *recvSize);

void msp_send(uint16_t messageID, void *payload, uint16_t size)
{
    uint8_t flag = 0;
    int msg_size = 9; 
    uint8_t crc = 0;
    uint8_t tmp_buf[2];

    msg_size += (int)size;

    uart_write_bytes(UART_NUM, "$", 1);
    uart_write_bytes(UART_NUM, "X", 1);
    uart_write_bytes(UART_NUM, "<", 1);

    crc = crc8_dvb_s2(crc, flag);
    uart_write_bytes(UART_NUM, &flag, 1);

    memcpy(tmp_buf, &messageID, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    uart_write_bytes(UART_NUM, tmp_buf, 2);

    memcpy(tmp_buf, &size, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    uart_write_bytes(UART_NUM, tmp_buf, 2);

    uint8_t *payloadPtr = (uint8_t*)payload;
    for (uint8_t i = 0; i < size; ++i) {
        uint8_t b = *(payloadPtr++);
        crc = crc8_dvb_s2(crc, b);
        uart_write_bytes(UART_NUM, &b, 1); 
    }

    uart_write_bytes(UART_NUM, &crc, 1); 
}

bool msp_recv(uint16_t *messageID, void *payload, uint16_t maxSize, uint16_t *recvSize) {
    uint32_t t0 = millis();
    uint8_t tmp_buf[2];

    while (1) {
        char header[3];
        int header_size = uart_read_bytes(UART_NUM, (uint8_t *)header, 3, 20 / portTICK_PERIOD_MS); // timeout  20ms
        if (header_size != 3 || header[0] != '$' || header[1] != 'X' || header[2] != '>') {
            continue; 
        }
        uint8_t checksumCalc = 0;
        uint8_t flag;
        uart_read_bytes(UART_NUM, &flag, 1, portMAX_DELAY);
        checksumCalc = crc8_dvb_s2(checksumCalc, flag);

        // read message ID (type)
        uart_read_bytes(UART_NUM, tmp_buf, 2, portMAX_DELAY);
        *messageID = (tmp_buf[1] << 8) | tmp_buf[0];
        checksumCalc = crc8_dvb_s2(checksumCalc, tmp_buf[0]);
        checksumCalc = crc8_dvb_s2(checksumCalc, tmp_buf[1]);

        // read payload size
        uart_read_bytes(UART_NUM, tmp_buf, 2, portMAX_DELAY);
        *recvSize = (tmp_buf[1] << 8) | tmp_buf[0];
        checksumCalc = crc8_dvb_s2(checksumCalc, tmp_buf[0]);
        checksumCalc = crc8_dvb_s2(checksumCalc, tmp_buf[1]);

        // read payload
        uint8_t *payloadPtr = (uint8_t*)payload;
        uint16_t idx = 0;
        while (idx < *recvSize) {
            uint8_t b;
            int bytes_read = uart_read_bytes(UART_NUM, &b, 1, portMAX_DELAY);
            if (bytes_read == 1) {
                checksumCalc = crc8_dvb_s2(checksumCalc, b);
                if (idx < maxSize) {
                    *(payloadPtr++) = b;
                }
                ++idx;
            } else {
                break; 
            }
        }

        // zero remaining bytes if recvSize < maxSize
        for (; idx < maxSize; ++idx) {
            *(payloadPtr++) = 0;
        }

        uint8_t checksum;
        uart_read_bytes(UART_NUM, &checksum, 1, portMAX_DELAY);
        if (checksumCalc == checksum) {
            return true;
        }
    }

    return false; 
}
bool msp_waitFor(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize) {
    uint16_t recvMessageID;
    uint16_t recvSizeValue;
    uint32_t t0 = millis();
    while (millis() - t0 < _timeout)
        if (msp_recv(&recvMessageID, payload, maxSize, (recvSize ? recvSize : &recvSizeValue)) && messageID == recvMessageID)
            return true;

    return false;
}

bool msp_request(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize) {
    msp_send(messageID, NULL, 0);

    return msp_waitFor(messageID, payload, maxSize, recvSize);
}


bool msp_command(uint16_t messageID, void * payload, uint16_t size, bool waitACK)
{
    msp_send(messageID, payload, size);
    if (waitACK)
    {
        msp_waitFor(messageID, NULL, 0, NULL);
    }
    return true;
}

void app_main(void) 
{
    init_uart();

    RCData rcData;
    while(1){
        msp_rc_t rc;

        bool res = msp_request(105, &rc, sizeof(rc), NULL);
        uart_wait_tx_done(UART_NUM, 20);
    
        if (res) {
            
            uint16_t roll     = rc.channelValue[0];
            uint16_t pitch    = rc.channelValue[1];
            uint16_t yaw      = rc.channelValue[2];
            uint16_t throttle = rc.channelValue[3];
            ESP_LOGD("TAG_MSP", "MSP Request RC: Roll=%u, Pitch=%u, Yaw=%u, Throttle=%u",
                    roll, pitch, yaw, throttle);
        }
        ESP_LOGI("MSP", "Requesting RC %d", res);

        rcData.throttle = rand() % 1000;
        rcData.roll = rand() % 2000;
        rcData.pitch = rand() % 2000;
        rcData.yaw = rand() % 2000;
        rcData.aux1 = rand() % 2000;
        rcData.aux2 = rand() % 2000;
        rcData.aux3 = rand() % 2000;
        rcData.aux4 = rand() % 2000;

        res = msp_command(200, &rcData, sizeof(RCData), true);
        uart_wait_tx_done(UART_NUM, portMAX_DELAY);

        ESP_LOGI("MSP", "SendRC %d", res);

        
    // vTaskDelay(450 / portTICK_PERIOD_MS);
    }
}
