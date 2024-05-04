/* Тестовое задание для компании "Открытый код"
    Задание приведено в файле task-embedded-2
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

//структура таймера
gptimer_handle_t gptimer = NULL;

//структура для передачи данных в задачу
struct channel_task_args
{
    uint8_t channel;
    uint16_t _unlock_time;
    uint16_t _flash_time;
};

//поля для таймера
static uint32_t flash_time_timer_value[5] = {1, 1, 1, 1, 1}; //поле значений для того до скольки считать 
static uint16_t unique_counter[5] = {0, 0, 0, 0, 0}; //счетчик таймера для каждого канала
static bool pin_state_flag[5] = {false, false, false, false, false}; //состояние пина светодиода

//Флаги каналов
bool channelLockFlag[5] = {false, false, false, false, false};
//Каналов 5
#define MAX_CHANNEL_NUMBER 5
//Пины UART
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

//Пины управляемых каналов 
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

//Вспомогательные массивы с управляемыми пинами
static gpio_num_t lock_pins_array[5] = {CHANNEL_1_LOCK_PIN, CHANNEL_2_LOCK_PIN, CHANNEL_3_LOCK_PIN, CHANNEL_4_LOCK_PIN, CHANNEL_5_LOCK_PIN};
static gpio_num_t led_pins_array[5] = {CHANNEL_1_LED_PIN, CHANNEL_2_LED_PIN, CHANNEL_3_LED_PIN, CHANNEL_4_LED_PIN, CHANNEL_5_LED_PIN};

// Прерывание таймера
static bool IRAM_ATTR gptimer_alarm_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{ 
    //Проверка флагов для каждого канала
    if(channelLockFlag[0]){   
        //Проверка заданного времени и блинк по достижении заданного счета             
        if (unique_counter[0] == flash_time_timer_value[0]/2){
            //ESP_DRAM_LOGW("timer0", "Hardware timer channel 1 alarm! Count: %d", unique_counter[0]);
            pin_state_flag[0] = !pin_state_flag[0];
            gpio_set_level(led_pins_array[0], pin_state_flag[0]);            
            unique_counter[0] = 0;
        }
        unique_counter[0]++;        
    }
    if(channelLockFlag[1]){
        if (unique_counter[1] == flash_time_timer_value[1]/2){
            //ESP_DRAM_LOGW("timer0", "Hardware timer channel 2 alarm! Count: %d", unique_counter[1]);
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
  return false;
}

void init(void) {

    static const char *INIT_TAG = "INIT";
    esp_log_level_set(INIT_TAG, ESP_LOG_INFO);

    //Инициализация пинов управления
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<CHANNEL_1_LED_PIN)|(1ULL<<CHANNEL_1_LOCK_PIN)|
                            (1ULL<<CHANNEL_2_LED_PIN)|(1ULL<<CHANNEL_2_LOCK_PIN)|
                            (1ULL<<CHANNEL_3_LED_PIN)|(1ULL<<CHANNEL_3_LOCK_PIN)|
                            (1ULL<<CHANNEL_4_LED_PIN)|(1ULL<<CHANNEL_4_LOCK_PIN)|
                            (1ULL<<CHANNEL_5_LED_PIN)|(1ULL<<CHANNEL_5_LOCK_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    ESP_LOGI(INIT_TAG, "Hardware gpio init");

    //Инициализация UART    
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);    

    ESP_LOGI(INIT_TAG, "UART created, speed: %d", uart_config.baud_rate);

    //Инициализация таймера
    const gptimer_config_t timer_config ={
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // частота счета 1 МГц
    };
    gptimer_new_timer(&timer_config, &gptimer);

  // Добавление callback таймера в обработку
    gptimer_event_callbacks_t cb_config = {
        .on_alarm = gptimer_alarm_callback,
    };
    gptimer_register_event_callbacks(gptimer, &cb_config, NULL);   

    // Задаем параметры счетчика таймера
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 500,                   // Конечное значение счетчика = 0.5 милисекунды
        .reload_count = 0,                        // Значение счетчика при сбросе
        .flags.auto_reload_on_alarm = true,       // Автоперезапуск таймера
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);

    gptimer_set_raw_count(gptimer, 0);
    // Разрешаем прерывания для данного таймера
    gptimer_enable(gptimer);

    // Запускаем таймер   
    gptimer_start(gptimer);

    ESP_LOGI(INIT_TAG, "Hardware timer created");
}

void task_channel(void *arg){    

    //Инфо в логировании
    static const char *CHANNEL_TASK_TAG = "CHANNEL_TASK";
    esp_log_level_set(CHANNEL_TASK_TAG, ESP_LOG_INFO);

    //Перенос аргументов в локальные поля
    struct channel_task_args *param = (struct channel_task_args*) arg; 
    uint8_t curr_channel = param->channel;
    uint16_t curr_unlock = param->_unlock_time;
    uint16_t curr_flash = param->_flash_time;
    
    //Инфо
    ESP_LOGI(CHANNEL_TASK_TAG, "channel: %d", curr_channel);
    ESP_LOGI(CHANNEL_TASK_TAG, "unlock_time: %d", curr_unlock);
    ESP_LOGI(CHANNEL_TASK_TAG, "flash_time: %d", curr_flash);

    ESP_LOGI(CHANNEL_TASK_TAG, "Task created%d", curr_channel);

    //Запись в поле времени до скольки считать
    flash_time_timer_value[curr_channel - 1] = curr_flash * 2;
    //Блокирование канала
    channelLockFlag[curr_channel - 1] = true;      
    //Открытие замка
    gpio_set_level(lock_pins_array[curr_channel - 1], 1);        
    //Ожидание времени
    vTaskDelay(curr_unlock / portTICK_PERIOD_MS);

    //Завершение задачи
    ESP_LOGI(CHANNEL_TASK_TAG, "Task finished %d", curr_channel);
    //Закрытие замка и отключение светодиода
    gpio_set_level(lock_pins_array[curr_channel-1], 0);
    gpio_set_level(led_pins_array[curr_channel-1], 0);
    unique_counter[curr_channel - 1] = 0;
    //Снятие флага занятости
    channelLockFlag[curr_channel-1] = false;

    vTaskDelete(NULL);
}

static void rx_task(void *arg)
{
    //Инфо в логировании
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    //Выделение памяти для приема
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    //цикл применика
    while (1) {
        //количество поступивших байт и занесениеих в выделенную память data
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);        
        if (rxBytes > 0) {
            data[rxBytes] = 0;

            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            //Проверяем посылку на соответствии информационному пакету
            //0. Экранирующий символ
            //1-2. Заголовок
            //9-10. Окончание
            if (data[0]==0x02 && data[1]==0x33 &&data[2]==0x01 && data[9]==0x33 && data[10]==0x03)
            {
                ESP_LOGI(RX_TASK_TAG, "Success get message");
                //ID канала 1-5, если иное - игнорируем
                if(data[3] > 0 && data[3] <= MAX_CHANNEL_NUMBER){
                    ESP_LOGI(RX_TASK_TAG, "Channel %d select", data[3]);
                    //Проверяем доступность канала, если занят - посылка игнорируется
                    if(!channelLockFlag[data[3] - 1]) {
                        //проверка команды откртытия, иные команды не указаны
                        if(data[4] == 2){           
                            //Примем структуру LSB для данных размером 2 байта  
                            uint16_t unlock_time = data[5] + ((int16_t)data[6]<<8);
                            uint16_t flash_time = data[7] + ((int16_t)data[8]<<8);
                            //Время не должно быть равно 0
                            if(unlock_time != 0 || flash_time != 0){
                                ESP_LOGI(RX_TASK_TAG, "Unlock_time: %d", unlock_time);
                                ESP_LOGI(RX_TASK_TAG, "flash_time: %d", flash_time);

                                //Создание задачи и передача в нее полученных параметров
                                const struct channel_task_args args = {.channel = data[3], ._flash_time = flash_time, ._unlock_time = unlock_time};      
                                xTaskCreate(task_channel, "task_channel", 1024*2, (void*)&args, configMAX_PRIORITIES-1, NULL);
                            }    
                            else ESP_LOGI(RX_TASK_TAG, "Time cannot be 0");
                        }
                        else ESP_LOGI(RX_TASK_TAG, "Command error");
                    }
                    else ESP_LOGI(RX_TASK_TAG, "Channel already work");                  
                }
                else ESP_LOGI(RX_TASK_TAG, "Channel %d not found", data[3]);
            }            
        }
    }
    free(data);
}

void app_main(void)
{
    //Инициализация
    init();
    //Создание задачи приемника UART
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}