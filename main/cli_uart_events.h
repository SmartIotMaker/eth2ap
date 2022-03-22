#ifndef CLI_UART_EVENT_H
#define CLI_UART_EVENT_H

#define  WIFI_PARAM_NUM 3
typedef struct {
    uint8_t ssid[32];      /**< SSID of target AP. */
    uint8_t password[64];  /**< Password of target AP. */
}wifi_sta_param_t;

void cli_init(void);

extern wifi_sta_param_t g_wifi_sta_param[WIFI_PARAM_NUM];

#endif