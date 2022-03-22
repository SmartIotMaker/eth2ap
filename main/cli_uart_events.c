/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "FreeRTOS_CLI.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "cli_uart_events.h"

static const char *TAG = "cli_uart_events";
#define WIFI_NVS_NAME "wifi_param"
#define KEY_NVS_WIFIP "KeyWifip"

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (1)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;



wifi_sta_param_t g_wifi_sta_param[WIFI_PARAM_NUM];

static BaseType_t SetAPParamerHandler( char * pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char * pcCommandString )
{
    BaseType_t ssid_str_len,index_str_len,password_str_len;
    char *pssidstr,*ppasswordstr,*pindexstr;
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    uint8_t wifi_index = 0;
    size_t wifiplength = sizeof(g_wifi_sta_param);

    pindexstr = (char*)FreeRTOS_CLIGetParameter(pcCommandString,1,&index_str_len);
    pssidstr = (char*)FreeRTOS_CLIGetParameter(pcCommandString,2,&ssid_str_len);
    ppasswordstr = (char*)FreeRTOS_CLIGetParameter(pcCommandString,3,&password_str_len);
    pindexstr[index_str_len] = 0;
    pssidstr[ssid_str_len] = 0;
    ppasswordstr[password_str_len] = 0;

    wifi_index = atoi(pindexstr);
    if(wifi_index <1 || wifi_index >WIFI_PARAM_NUM) {
        sprintf(pcWriteBuffer,"\r\nwifi_index(%d) must be range from 1 to %d\r\n",wifi_index,WIFI_PARAM_NUM);
    }
    else if(ssid_str_len > 31) {
        sprintf(pcWriteBuffer,"\r\nlength of ssid string must less than 31\r\n");
    }
    else if(password_str_len > 63) {
        sprintf(pcWriteBuffer,"\r\nlength of password string must less than 63\r\n");
    }
    else{
        //save to nvs_flash
        esp_err_t err = nvs_open(WIFI_NVS_NAME, NVS_READWRITE, &handle);
        if (err != ESP_OK) {
            sprintf(pcWriteBuffer,"\r\nopen %s fail\r\n",WIFI_NVS_NAME);
        }
        else {
            err = nvs_get_blob(handle, KEY_NVS_WIFIP, (void*)&g_wifi_sta_param, &wifiplength);
            if (err != ESP_OK) {
                sprintf(pcWriteBuffer,"\r\nget key %s value fail\r\n",KEY_NVS_WIFIP);
            }
            else{
                memcpy(g_wifi_sta_param[wifi_index-1].ssid,pssidstr,strlen(pssidstr)+1);
                memcpy(g_wifi_sta_param[wifi_index-1].password,ppasswordstr,strlen(ppasswordstr)+1);
                err = nvs_set_blob(handle, KEY_NVS_WIFIP, (void*)&g_wifi_sta_param, sizeof(g_wifi_sta_param));
                if (err != ESP_OK) {
                    sprintf(pcWriteBuffer,"\r\nset wifi param to nvs fail\r\n");
                }
                else {
                    err = nvs_commit(handle);
                    if (err != ESP_OK) {
                        sprintf(pcWriteBuffer,"\r\nset wifi param to %s nvs fail\r\n",pindexstr);
                    }
                    else{
                        sprintf(pcWriteBuffer,"\r\nsave wifi param to %s nvs OK,restart to be valid\r\n",pindexstr);
                    }
                    
                }
            }
            nvs_close(handle);
        }
    }
    return pdFALSE;
}

static BaseType_t GetAPParamerHandler( char * pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char * pcCommandString )
{
    BaseType_t index_str_len;
    char *pindexstr;
    uint8_t wifi_index = 0;

    pindexstr = (char*)FreeRTOS_CLIGetParameter(pcCommandString,1,&index_str_len);
    pindexstr[index_str_len]=0;
    wifi_index = atoi(pindexstr);
    if(wifi_index == 0){
        int len = sprintf(pcWriteBuffer,"\r\n");
        for(int i=0;i<WIFI_PARAM_NUM;i++){
            pcWriteBuffer+=len;
            len = sprintf(pcWriteBuffer,"wifi[%d].ssid= %s wifi[%d].password= %s\r\n",
                        i+1,g_wifi_sta_param[i].ssid,i+1,g_wifi_sta_param[i].password);
        }
    }
    else{
        sprintf(pcWriteBuffer,"\r\nwifi[%d].ssid= %s wifi[%d].password= %s\r\n",
        wifi_index,g_wifi_sta_param[wifi_index-1].ssid,
        wifi_index,g_wifi_sta_param[wifi_index-1].password);
    }

    return pdFALSE;
}

static const CLI_Command_Definition_t xSetWifiParam =
{
    "SetAPParamer",
    "\r\nSetAPParamer:set wifi ssid\r\n  SetAPParamer {wifi_index} {ssidstring} {password}\r\n    wifi_index:range from 1 to 3\r\n    ssidstring:ssid\r\n    password:passowd\r\n",
    SetAPParamerHandler,
    3
};

static const CLI_Command_Definition_t xGetWifiParam =
{
    "GetAPParamer",
    "\r\nGetAPParamer:get wifi ssid\r\n  GetAPParamer {wifi_index}\r\n    wifi_index:range from 1 to 3 or 0 for all\r\n",
    GetAPParamerHandler,
    1
};

static esp_err_t RecoverWifiParam()
{
    size_t wifiplength=sizeof(g_wifi_sta_param);
    nvs_handle_t handle;

    esp_err_t err = nvs_open(WIFI_NVS_NAME, NVS_READWRITE, &handle);

    strcpy((char*)g_wifi_sta_param[0].ssid,CONFIG_EXAMPLE_WIFI_SSID);
    strcpy((char*)g_wifi_sta_param[0].password,CONFIG_EXAMPLE_WIFI_PASSWORD);
    g_wifi_sta_param[1]= g_wifi_sta_param[0];
    g_wifi_sta_param[2]= g_wifi_sta_param[0];

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "err(%s):open %s fail",esp_err_to_name(err),WIFI_NVS_NAME);
        return err;
    }
    else {
        err = nvs_get_blob(handle, KEY_NVS_WIFIP, (void*)&g_wifi_sta_param, &wifiplength);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "err(%s):get key %s value fail,set to default",esp_err_to_name(err),KEY_NVS_WIFIP);
            err = nvs_set_blob(handle, KEY_NVS_WIFIP, (void*)&g_wifi_sta_param, sizeof(g_wifi_sta_param));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "set wifi param to nvs fail");
            }
            else {
                err = nvs_commit(handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "err(%s):set wifi param to nvs fail",esp_err_to_name(err));
                }
                else{
                    ESP_LOGI(TAG, "err(%s):set default wifi param to nvs ok",esp_err_to_name(err));
                }
            }
        }
        nvs_close(handle);
    }

    return err;
}


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    uint32_t RevNum = 0;
    bzero(dtmp, RD_BUF_SIZE);

    RecoverWifiParam();

    FreeRTOS_CLIRegisterCommand( &xSetWifiParam ) ;
    FreeRTOS_CLIRegisterCommand( &xGetWifiParam ) ;

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            
            //ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    if((RevNum +event.size)<RD_BUF_SIZE){
                        uart_read_bytes(EX_UART_NUM, dtmp+RevNum, event.size, portMAX_DELAY);
                        for(int i=0; i<event.size; i++){
                            if(dtmp[RevNum+i] == '\b'){
                                uart_write_bytes(EX_UART_NUM, "\033[1D\033[K", strlen("\033[1D\033[K"));
                            }
                            else{
                                uart_write_bytes(EX_UART_NUM, (const char*) &dtmp[RevNum+i], 1);
                            }
                            
                        }
                        
                        RevNum += event.size;
                        uart_flush_input(EX_UART_NUM);
                    }
                    else{
                        ESP_LOGI(TAG, "cli buffer overflow,dump it");
                        uart_flush_input(EX_UART_NUM);
                        xQueueReset(uart0_queue);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    //ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        if((RevNum + pos)<RD_BUF_SIZE){
                            uart_read_bytes(EX_UART_NUM, dtmp+RevNum, pos, 100 / portTICK_PERIOD_MS);
                            RevNum += pos;
                            uint8_t pat[PATTERN_CHR_NUM + 1];
                            memset(pat, 0, sizeof(pat));
                            uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);

                            //remove backspace and delete
                            uint8_t* s = dtmp;
                            while(*s != 0) {
                                if(*s == '\b' || *s == 0x7f ){
                                    if(s == dtmp){
                                        RevNum -= 1;
                                        memcpy(s, s+1, RevNum);
                                        
                                    }
                                    else{
                                        RevNum -= 2;
                                        memcpy(s-1, s+1, RevNum-(s-dtmp)+1);
                                        s--;
                                    }
                                    dtmp[RevNum]=0;
                                }
                                else{
                                    s++;
                                }
                            }

                            //process command
                            char *outp = FreeRTOS_CLIGetOutputBuffer();
                            //ESP_LOGI(TAG, "remove date(%d): %s",RevNum, dtmp);
                            uart_write_bytes(EX_UART_NUM, (const char*) "\r\n", 2);
                            FreeRTOS_CLIProcessCommand((char*)dtmp,outp,configCOMMAND_INT_MAX_OUTPUT_SIZE);
                            uart_write_bytes(EX_UART_NUM, (const char*) outp, strlen(outp));
                            uart_write_bytes(EX_UART_NUM, (const char*) "\r\n\r\n$ ", 6);
                            bzero(dtmp, RD_BUF_SIZE);
                            FreeRTOS_CLIClearOutputBuffer();
                            RevNum = 0;
                            uart_flush_input(EX_UART_NUM);
                            xQueueReset(uart0_queue);
                        }
                        else{
                            ESP_LOGI(TAG, "cli buffer overflow,dump it");
                            uart_flush_input(EX_UART_NUM);
                            xQueueReset(uart0_queue);
                        }
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void cli_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '\r', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
