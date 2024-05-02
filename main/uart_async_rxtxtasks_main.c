/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

static const int RX_BUF_SIZE = 1024;
gptimer_handle_t gptimer = NULL;
int counter = 0;

//Флаги каналов
bool channel1LockFlag = false;
bool channel2LockFlag = false;
bool channel3LockFlag = false;
bool channel4LockFlag = false;
bool channel5LockFlag = false;

//Каналов 5
#define MAX_CHANNEL_NUMBER 5
//Пины UART
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

// Функция обратного вызова таймера
static bool IRAM_ATTR gptimer_alarm_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
  
  counter++;
  if(counter > 1000){
    counter = 0 ;
    // В обработчиках используется ESP_DRAM_LOGx()
    ESP_DRAM_LOGW("timer0", "Timer ready! ");
  }
  //ESP_DRAM_LOGW("timer0", "Hardware timer alarm! Count: %d", counter);


  // Если вы вызывали функции FreeRTOS (например `xQueueSendFromISR`) из обработчика прерываний, 
  // вам необходимо вернуть значение true или false на основе возвращаемого значения аргумента pxHigherPriorityTaskWoken. 
  // Если возвращаемое значение `pxHigherPriorityTaskWoken` любых вызовов FreeRTOS равно pdTRUE, то вы должны вернуть true; в противном случае вернуть false.
  // ---
  // В данном простейшем случае мы не отправляли ничего в очереди, поэтому можно вернуть false
  return false;
}

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);    

    const gptimer_config_t timer_config ={
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000
    };
    gptimer_new_timer(&timer_config, &gptimer);

  // Подключаем функцию обратного вызова
    gptimer_event_callbacks_t cb_config = {
        .on_alarm = gptimer_alarm_callback,
    };
    gptimer_register_event_callbacks(gptimer, &cb_config, NULL);

    // Задаем начальное значение счетчика таймера (не обязательно)
    gptimer_set_raw_count(gptimer, 0);

    // Задаем параметры счетчика таймера
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000,                   // Конечное значение счетчика = 0.001 секунды
        .reload_count = 0,                        // Значение счетчика при автосбросе
        .flags.auto_reload_on_alarm = true,       // Автоперезапуск счетчика таймера разрешен
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);

    // Разрешаем прерывания для данного таймера
    gptimer_enable(gptimer);

    // Запускаем таймер
    //gptimer_start(gptimer);
    ESP_LOGI("main", "Hardware timer created");

}



//int sendData(const char* logName, const char* data)
//{
//    const int len = strlen(data);
//    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
//    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
//    return txBytes;
//}
//
//static void tx_task(void *arg)
//{
//    static const char *TX_TASK_TAG = "TX_TASK";
//    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//    while (1) {
//        sendData(TX_TASK_TAG, "Hello world");
//        vTaskDelay(2000 / portTICK_PERIOD_MS);
//    }
//}

void task_channel(void *arg){
    
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            if (data[0]==0x02 && data[1]==0x33 &&data[2]==0x01 && data[9]==0x33 && data[10]==0x03)
            {
                ESP_LOGI(RX_TASK_TAG, "Success get message");

                if(data[3] > 0 && data[3] <= MAX_CHANNEL_NUMBER){
                    ESP_LOGI(RX_TASK_TAG, "Channel %d select", data[3]);

                    if(data[4] == 1){
                        gptimer_stop(gptimer);
                        ESP_LOGI(RX_TASK_TAG, "Timer stop");
                    }
                    else if(data[4] == 2){
                        gptimer_start(gptimer);
                        ESP_LOGI(RX_TASK_TAG, "Timer start");
                    }
                    else{
                        ESP_LOGI(RX_TASK_TAG, "Command error");
                    }
                }
                else{
                    ESP_LOGI(RX_TASK_TAG, "Channel %d not found", data[3]);
                }

                
                
                
            }
            
        }
    }
    free(data);
}

void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);


}
