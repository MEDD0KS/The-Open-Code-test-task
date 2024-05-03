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

struct channel_task_args
{
    uint8_t channel;
    uint16_t _unlock_time;
    uint16_t _flash_time;
};//args[5] = { {1, 1, 1}, {2, 1, 1}, {3, 1, 1}, {4, 1, 1}, {5, 1, 1}  };

static uint32_t flash_time_timer_value[5] = {1 , 1 , 1, 1 , 1};
static uint16_t unique_counter[5] = {0, 0, 0, 0 , 0};
static bool pin_state_flag[5] = {false, false, false, false, false};

bool timer_flag = false;
//Флаги каналов
bool channelLockFlag[5] = {false, false, false, false, false};
//Каналов 5
#define MAX_CHANNEL_NUMBER 5
//Пины UART
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

#define CHANNEL_1_LED_PIN (GPIO_NUM_10)
#define CHANNEL_1_LOCK_PIN (GPIO_NUM_11)
#define CHANNEL_2_LED_PIN (GPIO_NUM_12)
#define CHANNEL_2_LOCK_PIN (GPIO_NUM_13)
#define CHANNEL_3_LED_PIN (GPIO_NUM_18)
#define CHANNEL_3_LOCK_PIN (GPIO_NUM_19)
#define CHANNEL_4_LED_PIN (GPIO_NUM_20)
#define CHANNEL_4_LOCK_PIN (GPIO_NUM_21)
#define CHANNEL_5_LED_PIN (GPIO_NUM_22)
#define CHANNEL_5_LOCK_PIN (GPIO_NUM_23)

static gpio_num_t lock_pins_array[5] = {CHANNEL_1_LOCK_PIN, CHANNEL_2_LOCK_PIN, CHANNEL_3_LOCK_PIN, CHANNEL_4_LOCK_PIN, CHANNEL_5_LOCK_PIN};
static gpio_num_t led_pins_array[5] = {CHANNEL_1_LED_PIN, CHANNEL_2_LED_PIN, CHANNEL_3_LED_PIN, CHANNEL_4_LED_PIN, CHANNEL_5_LED_PIN};

// Прерывание таймера
static bool IRAM_ATTR gptimer_alarm_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{ 
    //vPortEnterCritical();    

    if(channelLockFlag[0]){                
        if (unique_counter[0] == flash_time_timer_value[0]/2){
            ESP_DRAM_LOGW("timer0", "Hardware timer channel 1 alarm! Count: %d", unique_counter[0]);
            pin_state_flag[0] = !pin_state_flag[0];
            gpio_set_level(CHANNEL_1_LED_PIN, pin_state_flag[0]);            
            unique_counter[0] = 0;
        }
        unique_counter[0]++;        
    }
    if(channelLockFlag[1]){
        if (unique_counter[1] == flash_time_timer_value[1]/2){
            ESP_DRAM_LOGW("timer0", "Hardware timer channel 2 alarm! Count: %d", unique_counter[1]);
            pin_state_flag[1] = !pin_state_flag[1];
            gpio_set_level(led_pins_array[1], pin_state_flag[1]);            
            unique_counter[1] = 0;
        }
        unique_counter[1]++;   
    }
    if(channelLockFlag[2]){
        if (unique_counter[2] == flash_time_timer_value[2]/2){
            pin_state_flag[2] = !pin_state_flag[2];
            gpio_set_level(led_pins_array[2], pin_state_flag[2]);            
            unique_counter[2] = 0;
        }
        unique_counter[2]++;   
    }
    if(channelLockFlag[3]){
        if (unique_counter[3] == flash_time_timer_value[3]/2){
            pin_state_flag[3] = !pin_state_flag[3];
            gpio_set_level(led_pins_array[3], pin_state_flag[3]);            
            unique_counter[3] = 0;
        }
        unique_counter[3]++;   
    }
    if(channelLockFlag[4]){
        if (unique_counter[4] == flash_time_timer_value[4]/2){
            pin_state_flag[4] = !pin_state_flag[4];
            gpio_set_level(led_pins_array[4], pin_state_flag[4]);            
            unique_counter[4] = 0;
        }
        unique_counter[4]++;   
    }
    //vPortExitCritical(); 
  //ESP_DRAM_LOGW("timer0", "Hardware timer alarm! Count: %d", counter);
  return false;
}

void init(void) {

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<CHANNEL_1_LED_PIN)|(1ULL<<CHANNEL_1_LOCK_PIN)|
                            (1ULL<<CHANNEL_2_LED_PIN)|(1ULL<<CHANNEL_2_LOCK_PIN)|
                            (1ULL<<CHANNEL_3_LED_PIN)|(1ULL<<CHANNEL_3_LOCK_PIN)|
                            (1ULL<<CHANNEL_4_LED_PIN)|(1ULL<<CHANNEL_4_LOCK_PIN)|
                            (1ULL<<CHANNEL_5_LED_PIN)|(1ULL<<CHANNEL_5_LOCK_PIN);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

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

    gptimer_start(gptimer);
}

void task_channel(void *arg){    

    static const char *CHANNEL_TASK_TAG = "CHANNEL_TASK";
    esp_log_level_set(CHANNEL_TASK_TAG, ESP_LOG_INFO);

    struct channel_task_args *param = (struct channel_task_args*) arg; 

    if(channelLockFlag[param->channel - 1]) {
        ESP_LOGI(CHANNEL_TASK_TAG, "Channel already work");
    }
    else{        
        ESP_LOGI(CHANNEL_TASK_TAG, "channel: %d", param->channel);
        ESP_LOGI(CHANNEL_TASK_TAG, "unlock_time: %d", param->_unlock_time);
        ESP_LOGI(CHANNEL_TASK_TAG, "flash_time: %d", param->_flash_time);

        ESP_LOGI(CHANNEL_TASK_TAG, "Task created%d", param->channel);


        flash_time_timer_value[param->channel - 1] = param->_flash_time * 2;
        channelLockFlag[param->channel - 1] = true;      

        //if(gptimer_get)
        //gptimer_start(gptimer);

        gpio_set_level(lock_pins_array[param->channel - 1], 1);        

        uint8_t curr_channel = param->channel;

        vTaskDelay(param->_unlock_time / portTICK_PERIOD_MS);

        //ESP_LOGI(CHANNEL_TASK_TAG, "Task finished %d", param->channel);

        //vPortEnterCritical();    

        ESP_LOGI(CHANNEL_TASK_TAG, "Task finished %d", curr_channel);

        channelLockFlag[curr_channel-1] = false;

        //ESP_LOGI(CHANNEL_TASK_TAG, "State %d", channelLockFlag[param->channel - 1]);

        gpio_set_level(lock_pins_array[curr_channel-1], 0);

        gpio_set_level(led_pins_array[curr_channel-1], 0);

        //vPortExitCritical(); 

        //gptimer_stop(gptimer);

        
    }

    

    vTaskDelete(NULL);
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


                        //gptimer_stop(gptimer);
                        //ESP_LOGI(RX_TASK_TAG, "Timer stop");
                    }
                    else if(data[4] == 2){
                        
                        unlock_time = data[5] + ((int16_t)data[6]<<8);
                        flash_time = data[7] + ((int16_t)data[8]<<8);
                        ESP_LOGI(RX_TASK_TAG, "Unlock_time: %d", unlock_time);
                        ESP_LOGI(RX_TASK_TAG, "flash_time: %d", flash_time);

                        //uint16_t args[] = {(uint16_t)data[3], unlock_time, flash_time};
                        const struct channel_task_args args = {.channel = data[3], ._flash_time = flash_time, ._unlock_time = unlock_time};      
                        //char *task_name = "task_channel_";
                        //sprintf(task_name, "%d", data[3]);
                        //const char *task_name_wtf = task_name;
                         //args[data[3]].channel = data[3]; args[data[3]]._flash_time = flash_time; args[data[3]]._unlock_time = unlock_time;
                        //ESP_LOGI(RX_TASK_TAG, task_name);

                        xTaskCreate(task_channel, "task_channel", 1024*2, (void*)&args, configMAX_PRIORITIES-1, NULL);
                        //gptimer_start(gptimer);
                        //ESP_LOGI(RX_TASK_TAG, "Timer start");
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