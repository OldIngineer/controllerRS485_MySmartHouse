/*
ПРОГРАММА ДЛЯ КОНТРОЛЛЕРА ИСПОЛНЯЮЩЕГО УСТРОЙСТВА ПОДКЛЮЧЕНОГО К СЕТИ RS485
НА ОСНОВЕ МОДУЛЯ ESP32-WROOM-32D, дополненная:
- управлением других устройств по беспроводной связи BLE;
- драйвером цифрового датчика температуры и влажности DHT11
- драйвером аналогового емкостного датчика влажности земли "Capacitive Soil Moisture Sensor v1.2"
*/
//ссылки от примера "controller_vhci_ble_adv"

#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"

//ссылки от моего проекта "controller_RS485"
#include "esp_system.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"

//подключение управления сторожевым таймером 
#include "esp_task_wdt.h"

//драйвер датчика DHT11/DHT22
#include "dht.h"

//драйвер АЦП
#include "driver/adc.h"
#include "esp_adc_cal.h"

//Для обмена по сети RS485 используется UART2
#define port_rs485      (UART_NUM_2)
// CTS не используется в полудуплексном режиме RS485
#define CTS   (UART_PIN_NO_CHANGE)
//  RTS для полудуплексного режима RS485 управляет DE / ~ RE
#define rts    (GPIO_NUM_18)//UART2 default
#define TX     (GPIO_NUM_17)//UART2 default
#define RX     (GPIO_NUM_16)//UART2 default
//размер буфера обмена
#define uart_buf_size   (127)
// таймаут чтения пакета данных
#define PACKET_READ_TICS        (5 / portTICK_RATE_MS)
//задержка посылки сообщений рекламы (адвертайзинга) BLE
//при передаче общей команды: формируется ввиде интервала
//умноженного на номер узла, что исключает сбой от конфликта 
//принимаемых данных "controller_ble" от соседних узлов
#define DELAY_PACKET_ADVERT     (20 / portTICK_RATE_MS)
#define LED    (GPIO_NUM_12)//вывод управления светодиодом, низкий уровень
#define bit_0_number    (GPIO_NUM_4)//нулевой разряд адреса ВУ
#define bit_1_number    (GPIO_NUM_15)//первый разряд адреса ВУ
#define bit_2_number    (GPIO_NUM_33)//второй разряд адреса ВУ
#define bit_3_number    (GPIO_NUM_32)//третий разряд адреса ВУ
#define in1 (GPIO_NUM_22)//СИГНАЛ ОТ ДАТЧИКА ДВИЖЕНИЯ
#define in2 (GPIO_NUM_19)
#define in3 (GPIO_NUM_21)
#define in4 (GPIO_NUM_5)
#define out1    (GPIO_NUM_14)
#define out2    (GPIO_NUM_27)
#define out3    (GPIO_NUM_13)
#define out4    (GPIO_NUM_26)
#define out5    (GPIO_NUM_25)
#define out6    (GPIO_NUM_2)
#define SENSOR_TYPE (DHT_TYPE_DHT11)// тип датчика DHT11
#define bus_dht (GPIO_NUM_23)//вывод управления датчиком DHT11
#define bus_ahg (ADC1_CHANNEL_7)//ВЫВОД сигнала с датчика влажности земли (GPIO_NUM_35)
#define adc_atten (ADC_ATTEN_DB_11)//величина исмеряемого сигнала от 150 до 2450 мВ
#define PERIOD_AVD 360 //цикл посылки сообщений рекламы (PERIOD_AVD*0,625)мс
const TickType_t PERIOD_SEND_BLE = 1000 /  portTICK_PERIOD_MS; //время передачи рекламы BLE
char name_const[] = "MySmartHouse_";//постоянная часть передаваемого имени в рекламе
int size_name_const = 14; //количество символов в строке 14+1
uint8_t number;//задаваемый с помощью перемычек номер ВУ,
                // считывается при начальном запуске
uint8_t tmp = 0;//значение температуры
uint8_t php = 0;//значение влажности
uint8_t php_ground = 0;//значение влажности почвы
//================================================================
//Определяет таймаут прерывания TOUT,
// равный времени передачи одного символа (~ 11 бит) при текущей скорости передачи.
// Если время истекло, запускается прерывание UART_RXFIFO_TOUT_INT
#define READ_TOUT   (2)//???
/*Настройка параметров передачи по сети, в соответствии с "мастером"
скорость обмена 19200 бит/сек;
длина слова банных - 8 бит;
паритет четности;
количество стоп битов - 1;
режим аппаратного управления потоком -
? включить аппаратное управление потоком (UART_HW_FLOWCTRL_CTS_RTS), 
отключить (UART_HW_FLOWCTRL_DISABLE);
порог UART HW RTS - 122
*/
#define BAUD_RATE   (19200)
//инициализация UART
static void installing_uart(const int uart_num, gpio_num_t RTS, gpio_num_t TXD,
gpio_num_t RXD)
{
    uart_config_t uart_config = {
    .baud_rate = BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_EVEN,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};
// конфигурация UART параметров
ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
// Выбор UART выводов
ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD, RXD, RTS, CTS));
// Устанавливаем драйвер UART (нам здесь не нужна очередь событий)
ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buf_size * 2,
 uart_buf_size * 2, 0, NULL, 0));
// Устанавливаем полудуплексный режим RS485 c управлением от вывода RTS
ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
// Устанавливаем таймаут чтения. 
ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, READ_TOUT));
}
/*=========================ОТНОСИТСЯ К BLE============================*/
static const char *tag = "BLE_ADV";
char *adv_name="NO"; //адрес, где располагается имя устройства
int com_ble; //объявление переменной где хранится последняя команда передаваемая по BLE
bool out_in = 0;

#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/* Поле группы опкодов команды HCI (OGF)  */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)

#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

#define BD_ADDR_LEN     (6)                     /*Длина адреса устройства */
typedef uint8_t bd_addr_t[BD_ADDR_LEN];         /*Адрес устройства */

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};

static uint8_t hci_cmd_buf[128];//размер буфера интерфейса

/*
 * @brief: функция обратного вызова контроллера BT, используется для 
 * уведомления верхнего уровня о том, что
 * контроллер готов к приему команды
 */
static void controller_rcv_pkt_ready(void)
{
    //printf("controller rcv pkt ready\n");
}

/*
 * @brief: функция обратного вызова контроллера BT для передачи 
 * пакета данных в верхний
 * контроллер готов к приему команды
 */
static int host_rcv_pkt(uint8_t *datas, uint16_t lens)
{
    //printf("host rcv pkt: ");
    for (uint16_t i = 0; i < lens; i++) {
        //printf("%02x", datas[i]);
    }
    //printf("\n");
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

static uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}

static uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

static uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
        uint8_t adv_type, uint8_t addr_type_own,
        uint8_t addr_type_dir, bd_addr_t direct_bda,
        uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}


static uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
    uint16_t adv_intv_min = PERIOD_AVD; // (*х0,625)интервал рекламы
    uint16_t adv_intv_max = PERIOD_AVD; // (*х0,625)
    uint8_t adv_type = 0; // подключаемая ненаправленная реклама (ADV_IND)
    uint8_t own_addr_type = 0; // Адрес публичного устройства
    uint8_t peer_addr_type = 0; //Адрес публичного устройства
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; //Обработка всех подключений и сканирование

    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                  adv_intv_min,
                  adv_intv_max,
                  adv_type,
                  own_addr_type,
                  peer_addr_type,
                  peer_addr,
                  adv_chn_map,
                  adv_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}
//формирование данных объявления
static void hci_cmd_send_ble_set_adv_data(void)
{
    uint8_t name_len = (uint8_t)strlen(adv_name);
    //формирование заголовка рекламы:
    //1б.-размер данных -2 байта;
    //2б.-тип передаваемых данных - Flags (0x01)$
    //3б.- сами данные (Flags):
        //0х02 - General Discoverable Mode (общий режим обнаружения) +
        //0х04 - BR/EDR Not Supported (невозможность поддержки 
            //классического протокола Bluetooth) только BLE = 0x06;
    //4б.- размер данных (определится позже);
    //5б. - тип передаваемых данных - полное локальное имя (Complete Local Name) 0x09;


    uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09};
    uint8_t adv_data_len;

    adv_data[3] = name_len + 1;
    for (int i = 0; i < name_len; i++) {
        adv_data[5 + i] = (uint8_t)adv_name[i];
    }
    adv_data_len = 5 + name_len;

    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

/*
 * @brief: отправить команды HCI для выполнения рекламы BLE;
 */
void bleAdvtTask()//параметры отсутствуют
{
    int cmd_cnt = 0;//счетчик цикла подготовки отправки данных, вначале 0
    bool send_avail = false;//признак готовности к отправке, вначале false
    esp_vhci_host_register_callback(&vhci_host_cb);//регистрация обратного
    //вызова контроллера
    //printf("BLE advt task start\n");// " Запуск задачи BLE advt \ n "
    while (cmd_cnt<4) {// цикл формирования сообщения\рекламы
         vTaskDelay(100 / portTICK_PERIOD_MS);//задержка
        send_avail = esp_vhci_host_check_send_available();//используется для 
        //активной проверки, может ли хост отправлять пакет контроллеру или нет.
        //Возвращение: истина для готовности к отправке,
        // ложь означает невозможность отправки пакета
        if (send_avail) {
            switch (cmd_cnt) {
            case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
            case 1: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
            case 2: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;
            case 3: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;
            }
        }
        //printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d\n", send_avail, cmd_cnt);
         
    }
    //получение и отображение уровня мощности передатчика
       //int tx_power = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
       //printf("BLE TX POWER: %d\n", tx_power);
       vTaskDelay(PERIOD_SEND_BLE);//пауза 850мс, при интервале посылки 
       //сообщения в 300 мс формируется 3 посылки
       //выключить BLE
       hci_cmd_send_reset();
}
//==================================================================================

//инициализируем используемые выводы
static void installing_gpio()
{
    //УСТАНОВКА ВЫВОДА СВЕТОДИОДА
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED, 0);//зажигание светодиода
    //УСТАНОВКА ВЫВОДОВ ЗАДАНИЯ номера ВУ
     gpio_reset_pin(bit_0_number);
    gpio_set_direction(bit_0_number, GPIO_MODE_INPUT);
     gpio_reset_pin(bit_1_number);
    gpio_set_direction(bit_1_number, GPIO_MODE_INPUT);
     gpio_reset_pin(bit_2_number);
    gpio_set_direction(bit_2_number, GPIO_MODE_INPUT);
     gpio_reset_pin(bit_3_number);
    gpio_set_direction(bit_3_number, GPIO_MODE_INPUT);
    //ПОДКЛЮЧИТЬ ПОДТЯГИВАЮЩИЙ РЕЗИСТОР К ПЛЮСУ
    gpio_pullup_en(bit_0_number);
    gpio_pullup_en(bit_1_number);
    gpio_pullup_en(bit_2_number);
    gpio_pullup_en(bit_3_number);
    //УСТАНОВКА ВЫВОДОВ ДАТЧИКОВ
     gpio_reset_pin(in1);
     gpio_set_direction(in1,GPIO_MODE_INPUT);
     gpio_reset_pin(in2);
     gpio_set_direction(in2,GPIO_MODE_INPUT);
     gpio_reset_pin(in3);
     gpio_set_direction(in3,GPIO_MODE_INPUT);
     gpio_reset_pin(in4);
     gpio_set_direction(in4,GPIO_MODE_INPUT);
     //ПОДКЛЮЧИТЬ ПОДТЯГИВАЮЩИЙ РЕЗИСТОР К ПЛЮСУ
     gpio_pullup_en(in1);
     gpio_pullup_en(in2);
     gpio_pullup_en(in3);
     gpio_pullup_en(in4);
     //УСТАНОВКА ВЫХОДОВ УПРАВЛЕНИЯ
    gpio_reset_pin(out1);
    gpio_set_direction(out1,GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(out1, 0);
    gpio_reset_pin(out2);
    gpio_set_direction(out2,GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(out2, 0);
    gpio_reset_pin(out3);
    gpio_set_direction(out3,GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(out3, 0);
    gpio_reset_pin(out4);
    gpio_set_direction(out4,GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(out4, 0);
    gpio_reset_pin(out5);
    gpio_set_direction(out5,GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(out5, 0);
    gpio_reset_pin(out6);
    gpio_set_direction(out6,GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(out6, 0);
    //КОНФИГУРАЦИЯ АЦП ДЛЯ ИЗМЕРЕНИЯ СИГНАЛА ОТ ДАТЧИКА ВЛАЖНОСТИ ЗЕМЛИ
    adc1_config_width(ADC_WIDTH_BIT_9);//задание чтения АЦП с точностью 9 бит
    adc1_config_channel_atten(bus_ahg, adc_atten);//задание входа и пределов измерения
}
//инициализируем BLE
static void installing_ble()
{
    // Инициализировать NVS - используется для хранения данных калибровки PHY 
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGI(tag, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        //printf("Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return;//"Ошибка освобождения классической памяти контроллера Bluetooth: % s"
    }

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        //printf( "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;//« Ошибка инициализации контроллера Bluetooth: % s »
    }
    
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGI(tag, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        //printf("Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;//« Ошибка включения контроллера Bluetooth: % s »
    } 
    //printf("Bluetooth controller installing\n");
}


//п/п выполнения команды чтения датчиков ВУ
static void request_sensor(int len, uint8_t *data)
{
    // Определяем сигналы от датчиков
     uint8_t bit_d_0=0; uint8_t bit_d_1=0; uint8_t bit_d_2=0; uint8_t bit_d_3=0;
    if(!(gpio_get_level(in1))){bit_d_0 = 1;}
    if(gpio_get_level(in2)==0){bit_d_1 = 2;}
    if(gpio_get_level(in3)==0){bit_d_2 = 4;}
    if(gpio_get_level(in4)==0){bit_d_3 = 8;}
    uint8_t in = bit_d_0 + bit_d_1 + bit_d_2 + bit_d_3; 
    // определяем состояние выходов
    bit_d_0=0; bit_d_1=0; bit_d_2=0; bit_d_3=0; uint8_t bit_d_4=0; uint8_t bit_d_5=0;
    if(gpio_get_level(out1)==1){bit_d_0 = 1;}
    if(gpio_get_level(out2)==1){bit_d_1 = 2;}
    if(gpio_get_level(out3)==1){bit_d_2 = 4;}
    if(gpio_get_level(out4)==1){bit_d_3 = 8;}
    if(gpio_get_level(out5)==1){bit_d_4 = 16;}
    if(gpio_get_level(out6)==1){bit_d_5 = 32;} 
    uint8_t out = bit_d_0 + bit_d_1 + bit_d_2 + bit_d_3 +bit_d_4 + bit_d_5;
    //формирование ответа
            *data = number; *(data+1) = 3; *(data+2) = 0;
            *(data+3) = 6; *(data+4) = out;  
           *(data+5) = in;
            *(data+6) = tmp; *(data+7) = php;//температура, влажность
            *(data+8) = php_ground; *(data+9) = 0;//пока 0
            *(data+10) = 1; *(data+11) = 1; 
            //посылка ответа
            uart_write_bytes(port_rs485, (const char *) data, len+4);
    //измерение влажности почвы
    int adc_read = adc1_get_raw(bus_ahg);//вызов п/п чтения АЦП
    if (adc_read<32) {//исключить помехи менее 150 мВ
       php_ground = 0;//датчик отсутствует
       printf("Could not read data from sensor php_ground\n"); 
    } else {
    adc_read = 255-adc_read/2;//инвертировать  и привести к байту значение сигнала
    if (adc_read < 10) {
        php_ground = 1;
        printf("Humidity ground: %d\n", php_ground);
    } else {
        php_ground = adc_read;//значение влажности почвы
        printf("Humidity ground: %d\n", php_ground);
        }
    }
    //получение новых данных от датчиков температуры, влажности
    int16_t temperature, humidity;
    if(dht_read_data(SENSOR_TYPE, bus_dht, &humidity, &temperature) == ESP_OK) {
        tmp = temperature/10;//значение температуры
        php = humidity/10;//значение влажности 
        printf("Humidity: %d, Temp: %d\n", php, tmp);
    }    
        else {
            tmp = 0;
            php = 0; 
            printf("Could not read data from sensor DHT11\n");
        }
}
//п/п выполнения команды управления исполнительными устройствами
static void command_to_devices(int len, uint8_t *data)
{
  //формирование команд на выходах IO
   uint8_t out = *(data+5);
   uint8_t level;
   if(*(data+4)&64){level=1;} else {level=0;}
   if(out&1){gpio_set_level(out1, level);}
   if(out&2){gpio_set_level(out2, level);}
   if(out&4){gpio_set_level(out3, level);}
   if(out&8){gpio_set_level(out4, level);}
   if(out&16){gpio_set_level(out5, level);}
   if(out&32){gpio_set_level(out6, level);}
   //формирование ответа
   uart_write_bytes(port_rs485, (const char *) data, len);
}
//п/п выполнения широковещательной команды
static void broadcast_command(int len, uint8_t *data)
{
    //формирование команд на выходах IO
   uint8_t out = *(data+5);
   uint8_t level;
   if(*(data+4)&64){level=1;} else {level=0;}
   if(out&8){gpio_set_level(out4, level);}//вкл\выкл сирена
   if(out&2){gpio_set_level(out2, level);}//вкл\выкл свет
   //передача широковещательной команды устройствам BLE
   char name[size_name_const];
  //формирование команды отправляемой через BLE
        if(*(data+4)&64)
        {
            com_ble = 256;
        } else com_ble = 0;
        com_ble = com_ble + *(data+5);
    //перевод команды в символы
        char message[5];
        message[4] = '\0';
        message[3] = (com_ble%10) + '0';
        message[2] = ((com_ble/10)%10) + '0';
        message[1] = (com_ble/100) + '0';
        message[0] = '_'; 
        strcpy(name, name_const); //копирование строки 
    adv_name = name; //назначение адреса имени - адрес копированной строки
    strcat(adv_name, "00");//соединение строк с записью в 1 строку(номер 0)
    strcat(adv_name,message);//соединение строк с записью в 1 строку
        //printf(adv_name);
        //printf("\n");
    //задерка формирования пакета адвертайзинга(рекламы)
    //для исключения кофликта приема команды устройством BLE от соседних узлов
        vTaskDelay(DELAY_PACKET_ADVERT * number);
        bleAdvtTask();//вызов п\п формирования "рекламы"
}
//п\п передачи команды для устройств BLE зашифрованной в имени (рекламе)
static void event_comBLE(int len, uint8_t *data)
{
    char name[size_name_const];
    //перевод номера ВУ в символы
        char number_ch[3];
        number_ch[2] = '\0';
        number_ch[1] = (number % 10) + '0';//остаток от деления на 10 - мл.цифра
        number_ch[0] = (number/10) +'0';//разделить на 10 - ст.цифра
    //формирование команды отправляемой через BLE
        if(*(data+4)&64)
        {
            com_ble = 256;
        } else com_ble = 0;
        com_ble = com_ble + *(data+5);
    //перевод команды в символы
        char message[5];
        message[4] = '\0';
        message[3] = (com_ble%10) + '0';
        message[2] = ((com_ble/10)%10) + '0';
        message[1] = (com_ble/100) + '0';
        message[0] = '_';
    strcpy(name, name_const); //копирование строки 
    adv_name = name; //назначение адреса имени - адрес копированной строки
    strcat(adv_name,number_ch);//соединение строк с записью в 1 строку
    strcat(adv_name,message);//соединение строк с записью в 1 строку
        //printf(adv_name);
        //printf("\n"); 
    //формирование ответа
    uart_write_bytes(port_rs485, (const char *) data, len);  
    bleAdvtTask();//вызов п\п формирования "рекламы" 
}

static void uart_task(void *arg)
{ 
   esp_task_wdt_init(60, false);//период сторожевого таймера задач 60сек     
   installing_uart(port_rs485, rts, TX, RX);
   installing_gpio();
   installing_ble();
    // Выделяем в памяти ячейки под буферы для UART
 uint8_t* data = (uint8_t*) malloc(uart_buf_size); 
    // Определяем заданный с помощью перемычек номер ВУ 
    uint8_t bit_0=0; uint8_t bit_1=0; uint8_t bit_2=0; uint8_t bit_3=0;
    if(gpio_get_level(bit_0_number)){bit_0 = 1;}
    if(gpio_get_level(bit_1_number)){bit_1 = 2;}
    if(gpio_get_level(bit_2_number)){bit_2 = 4;}
    if(gpio_get_level(bit_3_number)){bit_3 = 8;}
    number = bit_0 + bit_1 + bit_2 + bit_3;
    while(1) //бесконечный цикл
    {
       //Считываем данные из UART, len=число байтов, прочитанных из UART FIFO
        int len = uart_read_bytes(port_rs485, data, uart_buf_size, PACKET_READ_TICS);
        if (len == 8)//если полученная команда правильной длины           
        {  
           esp_task_wdt_init(60, false);//период сторожевого таймера задач 60сек            
           if(*data==number)// если команда для этого устройства
           {
               //формирование мигания светодиода при опросах
               if(gpio_get_level(LED)){gpio_set_level(LED, 0);}
               else{gpio_set_level(LED, 1);}
               if(*(data+1)==3)//если команда запроса датчиков ВУ
               {
                uart_flush(port_rs485);//обнуляем UART RX buffer
                request_sensor(len, data);//вызов п/п  
               }
               if(*(data+1)==6)//если команда записи команд исп.устройств
               {
                uart_flush(port_rs485);//обнуляем UART RX buffer
                if(*(data+2)==1)
                {
                    event_comBLE(len, data);//событие - передача команды по BLE
                }
                if(*(data+2)==0)
                {
                    command_to_devices(len, data);//вызов п/п упр.исп.устр. непосред.
                }
               }   
           }
           if(*data==0)//если команда широковещательной рассылки
           {
               if(*(data+1)==6)//если команда записи команд исп.устройств
               {
               uart_flush(port_rs485);//обнуляем UART RX buffer
               broadcast_command(len, data);//вызов п/п 
               }   
           }
        }
      /*  
        //=================отладка=========================================
        if(gpio_get_level(in2)==0) //если подан нулевой имп. на IO19
        {
            if(out_in==0) //включить свет и сирену
            {
                com_ble = 266;
                out_in = 1;
            } else //выключить свет и сирену
            {
                com_ble = 10;
                out_in = 0;
            }
            //событие - передача команды по BLE
             char name[size_name_const];
                //перевод номера в символы
                char number_ch[3];
                number_ch[2] = '\0';
                number_ch[1] = (number % 10) + '0';//остаток от деления на 10 - мл.цифра
                number_ch[0] = (number/10) +'0';//разделить на 10 - ст.цифра
                //перевод команды в символы
                char message[5];
                message[4] = '\0';
                message[3] = (com_ble%10) + '0';
                message[2] = ((com_ble/10)%10) + '0';
                message[1] = (com_ble/100) + '0';
                message[0] = '_';
            strcpy(name, name_const); //копирование строки 
            adv_name = name; //назначение адреса имени - адрес копированной строки
            //strcat(adv_name, "00");//для широковещательной рассылки
            strcat(adv_name,number_ch);//соединение строк с записью в 1 строку
            strcat(adv_name,message);//соединение строк с записью в 1 строку
            printf(adv_name);
            printf("\n");    
        bleAdvtTask();//вызов п\п формирования "рекламы" 
        }
        //================================================================
       */
    }
    vTaskDelete(NULL);
}


void  app_main ( void )
{   
    //esp_task_wdt_init(60, false);//период сторожевого таймера задач 60сек
    //esp_task_wdt_add(NULL);//подписать текущую задачу на сторожевой таймер
     /*========================================================================
    Указание ОС на создание задачи(цикл):
    поддержка обмена по RS485 в качестве ведомого, 
    - функция (1-й параметр);
    - наименование задачи(2-й параметр);
    - размер стека 2048(3-й параметр);
    - указатель задачи ???(4-й параметр)
    - с приоритетом наивысшим, следующий configMAX_PRIORITIES-1(5-й параметр);
    - используется для возврата дескриптора, по которому можно ссылаться 
    на созданную задачу (6-й параметр)
     */
    xTaskCreate(uart_task, "uart_task", 2048, NULL, configMAX_PRIORITIES, NULL);
}