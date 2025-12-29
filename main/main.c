#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include <utils.h>

const char* Sname = "Shaleen's Heart";
const char* Mname = "Mumma's Heart";

static void ble_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
bool adc_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *data, void *user_data);
void monitor_queue(void*);
void trigger_haptics(void*);
void alter_pulse(TimerHandle_t timer);

adc_continuous_handle_t adc_handle;

uint8_t haptic_turn = 0;
clock_t long_p_tim = 0;
TimerHandle_t pulse_stop_timer = NULL;

static uint8_t char1_str[] = {0x11,0x22,0x33};

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, 
    .p_manufacturer_data =  NULL, 
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} gatts_profile = {};

typedef struct {
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

uint8_t match(char* odata, char* ndata, int olen, int nlen) {
    if(olen != nlen) return 0;

    for(int i = 0; i < olen; i++) {
        if(odata[i] != ndata[i]) return 0;
    }

    return 1;
}

enum PENDANT_CMD {
    BEAT = 1,
    TAP,
    LONG_PRESS,
    LONG_STOP
};

struct pendant_cmd_t {
    int cmd;
    struct pendant_cmd_t* next;
}* pendant_cmd_queue = NULL;

void add_pendant_cmd(struct pendant_cmd_t** queue, int cmd) {
    struct pendant_cmd_t* new_cmd = malloc(sizeof(struct pendant_cmd_t));
    new_cmd->cmd = cmd;
    new_cmd->next = NULL;

    if(queue && *queue) {
        struct pendant_cmd_t* current = *queue;
        while(current->next) {
            current = current->next;
        }    
        current->next = new_cmd;
    } else {
        *queue = new_cmd;
    }
}

int pop_pendant_cmd(struct pendant_cmd_t** queue) {
    if(queue && *queue) {
        struct pendant_cmd_t* temp = (*queue)->next;
        int cmd = (*queue)->cmd;
        free(*queue);
        *queue = temp;
        return cmd;
    } else {
        return -1;
    }
}

void show_btaddr(uint8_t* addr) {
    for(int i = 0; i < 6; i++) {
        printf("%x", addr[i]);
    }
}

static void ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            break;
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            break;
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        break;

    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        break;

    default:
        break;
    }
}

static void ble_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {

    case ESP_GATTS_REG_EVT:
        gatts_profile.gatts_if = gatts_if;
        gatts_profile.service_id.is_primary = true;
        gatts_profile.service_id.id.inst_id = 0x00;
        gatts_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
        gatts_profile.service_id.id.uuid.uuid.uuid16 = 0xff;

        //config adv data
        if(match((char*)esp_bt_dev_get_address(), "\x9c\x13\x9e\xab\xde\x12", 6, 6)) {
            esp_ble_gap_set_device_name(Sname);
        } else {
            esp_ble_gap_set_device_name(Mname);
        }

        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= adv_config_flag;
        esp_ble_gap_config_adv_data(&scan_rsp_data);
        adv_config_done |= scan_rsp_config_flag;

        esp_ble_gatts_create_service(gatts_if, &gatts_profile.service_id, 1);
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        if(param->write.handle == gatts_profile.descr_handle && param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }

        if(param->write.handle == gatts_profile.char_handle) {
            if(match((char *)param->write.value, (char *)".beat", param->write.len, 5)) {
                xTaskCreate(trigger_haptics, "h_trig_task", 4096, (void*)2, 3, NULL);
            } else if (match((char *)param->write.value, (char *)".tap", param->write.len, 4)) {
                xTaskCreate(trigger_haptics, "h_trig_task", 4096, (void*)1, 3, NULL);
            } else if (match((char *)param->write.value, (char *)".long", param->write.len, 5)) {
                long_p_tim = clock();
                if(pulse_stop_timer == NULL) {
                    pulse_stop_timer = xTimerCreate("alter_p_timer", pdMS_TO_TICKS(900), pdFALSE, NULL, alter_pulse);
                    xTimerReset(pulse_stop_timer, portMAX_DELAY);
                };
            } else if (match((char *)param->write.value, (char *)".long_stop", param->write.len, 10)) {
                haptic_turn = 2;
                long_p_tim = 0;
                if(pulse_stop_timer) {
                    if(xTimerIsTimerActive(pulse_stop_timer) != pdFALSE) {
                        xTimerStop(pulse_stop_timer, portMAX_DELAY);
                        xTimerDelete(pulse_stop_timer, portMAX_DELAY);
                        pulse_stop_timer = NULL;
                    }
                }
                haptic_turn = 0;
                gpio_set_level(GPIO_NUM_6, 0);
                gpio_set_level(GPIO_NUM_7, 0);
            }
        }
        
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        gatts_profile.service_handle = param->create.service_handle;
        gatts_profile.char_uuid.len = ESP_UUID_LEN_16;
        gatts_profile.char_uuid.uuid.uuid16 = 0xff01;

        esp_ble_gatts_start_service(gatts_profile.service_handle);
        esp_err_t add_char_ret = esp_ble_gatts_add_char(
            gatts_profile.service_handle, &gatts_profile.char_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            &gatts_demo_char1_val, NULL
        );
        break;
        
    case ESP_GATTS_ADD_CHAR_EVT: {
        
        gatts_profile.char_handle = param->add_char.attr_handle;
        gatts_profile.descr_uuid.len = ESP_UUID_LEN_16;
        gatts_profile.descr_uuid.uuid.uuid16 = 0x3333;

        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
            gatts_profile.service_handle, &gatts_profile.descr_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL
        );
        break;
    }
    
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gatts_profile.descr_handle = param->add_char_descr.attr_handle;
        break;
    
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        printf("GATT Server connected to ");
        show_btaddr(param->connect.remote_bda);
        printf("\n");
        gatts_profile.conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
   
    default:
        break;
    }
}

void setup_adc() {
    adc_continuous_handle_cfg_t h1_cfg = {
        .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV,
        .max_store_buf_size = 1024,
        .flags.flush_pool = 1,
    };

    adc_continuous_new_handle(&h1_cfg, &adc_handle);

    adc_continuous_config_t adc_cfg = {
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .sample_freq_hz = 5000,
        .pattern_num = 1,
    };

    adc_digi_pattern_config_t adc_pattern[2] = {0};

    for (int i = 0; i < 1; i++) {
        uint8_t adc_unit, adc_channel;
        adc_continuous_io_to_channel(i? 2 : 3, (adc_unit_t*)&adc_unit, (adc_channel_t*)&adc_channel); 
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = adc_channel;
        adc_pattern[i].unit = adc_unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    };

    adc_cfg.adc_pattern = adc_pattern;
    adc_continuous_config(adc_handle, &adc_cfg);

    adc_continuous_evt_cbs_t adc_cbs = {
        .on_conv_done = adc_cb,
    };

    adc_continuous_register_event_callbacks(adc_handle, &adc_cbs, NULL);
}

void setup_ble() {
    esp_err_t ret;

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ret = esp_bluedroid_init();
    ret = esp_bluedroid_enable();
    ret = esp_ble_gatts_register_callback(ble_gatts_cb);
    ret = esp_ble_gap_register_callback(ble_gap_cb);
    ret = esp_ble_gatts_app_register(0);
    esp_ble_gatt_set_local_mtu(500);
}

void alter_pulse(TimerHandle_t timer) {
    if(clock() - long_p_tim > 5100) {
        long_p_tim = 0;
        if(pulse_stop_timer) {
            if(xTimerIsTimerActive(pulse_stop_timer) != pdFALSE) {
                xTimerStop(pulse_stop_timer, portMAX_DELAY);
                xTimerDelete(pulse_stop_timer, portMAX_DELAY);
                pulse_stop_timer = NULL;
                return;
            }
        }
    } 
    if(haptic_turn == 1) {
        gpio_set_level(GPIO_NUM_7, 1);
        gpio_set_level(GPIO_NUM_6, 0);
        haptic_turn = 0;
    } else if (haptic_turn == 0) {
        gpio_set_level(GPIO_NUM_7, 0);
        gpio_set_level(GPIO_NUM_6, 1);
        haptic_turn = 1;
    } else if (haptic_turn == 2) {
        gpio_set_level(GPIO_NUM_7, 0);
        gpio_set_level(GPIO_NUM_6, 0);
    }
}

void trigger_haptics(void* htyp) {
    int cmd_type = 1;

    if(htyp) {
        cmd_type = (int)htyp;
    }

    if(cmd_type) {
        gpio_set_level(GPIO_NUM_6, 1);
        gpio_set_level(GPIO_NUM_7, 1);
    } else {
        gpio_set_level(GPIO_NUM_6, 0);
        gpio_set_level(GPIO_NUM_7, 0);
        vTaskDelete(NULL);
    }

    int delay_time = 180;

    if(cmd_type == 1) {
        delay_time = 180;
    } else if (cmd_type == 2) {
        delay_time = 120;
    };

    vTaskDelay(pdMS_TO_TICKS(delay_time));

    gpio_set_level(GPIO_NUM_6, 0);
    gpio_set_level(GPIO_NUM_7, 0);

    vTaskDelete(NULL);
}

void app_main(void)
{
    gpio_reset_pin(GPIO_NUM_6);
    gpio_reset_pin(GPIO_NUM_7);

    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);

    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    setup_ble();
    setup_adc();
    xTaskCreate(monitor_queue, "monitor_qtask", 4096, NULL, 3, NULL);
    adc_continuous_start(adc_handle);
}

void monitor_queue(void*) {
    while (1)
    {
        if(pendant_cmd_queue) {
            int cmd = pop_pendant_cmd(&pendant_cmd_queue);
            printf("adc: %d\n", cmd);
            switch (cmd)
            {
            case BEAT: {
                printf("beat!\n");
                esp_ble_gatts_send_indicate(gatts_profile.gatts_if, gatts_profile.conn_id, gatts_profile.char_handle, 5, (uint8_t*)".beat", false);
                break;
            }

            case TAP: {
                esp_ble_gatts_send_indicate(gatts_profile.gatts_if, gatts_profile.conn_id, gatts_profile.char_handle, 4, (uint8_t*)".tap", false);

                break;
            }

            case LONG_PRESS: {
                esp_ble_gatts_send_indicate(gatts_profile.gatts_if, gatts_profile.conn_id, gatts_profile.char_handle, 5, (uint8_t*)".long", false);
                break;
            }

            case LONG_STOP: {
                esp_ble_gatts_send_indicate(gatts_profile.gatts_if, gatts_profile.conn_id, gatts_profile.char_handle, 10, (uint8_t*)".long_stop", false);
                break;
            }
            
            default:
                break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
}

uint8_t beat_inc = 0;
int beat_count = 0;
int false_beat_count = 0;
clock_t last_beat_time = 0;
clock_t last_trig_beat = 0;

uint8_t force_start = 0;
clock_t force_rls = 0;
clock_t force_st_time = 0;
clock_t last_long_force = 0;

bool adc_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *data, void *user_data)
{
    adc_digi_output_data_t* final_data = (adc_digi_output_data_t*)(data->conv_frame_buffer);

    if(clock() - last_beat_time >= 15000) {
        beat_count = 0;
        false_beat_count = 0;
    }

    if(beat_count >= 10) {
        if((false_beat_count / beat_count) > .75) {
            beat_count = 0;
            false_beat_count = 0;
        }
    }
    
    for(int i = 0; i < data->size;) {
        uint16_t final_val = final_data->type2.data;

        if(final_data->type2.channel >= SOC_ADC_MAX_CHANNEL_NUM) {
            i++;
            if(i >= data->size) {
                break;
            } else {
                final_data++;
            }
            continue;
        }

        int gpio_num = -1;
        adc_continuous_channel_to_io(ADC_UNIT_1, final_data->type2.channel, &gpio_num);

        switch (gpio_num)
        {
        case 3: {
            // add_pendant_cmd(&pendant_cmd_queue, final_val);
            if(!beat_inc) {
                if(final_val > 2000) {
                    if(clock() - last_beat_time > 300) {
                        beat_inc = 1;
                        last_beat_time = clock();
                        beat_count++;
                        if(last_beat_time - last_trig_beat > 300) {

                        } else {
                            false_beat_count++;
                        }
                        if(beat_count >= 10) {
                            if((false_beat_count / beat_count) <= .75) {
                                add_pendant_cmd(&pendant_cmd_queue, BEAT);
                            }
                        }
                    } else {
                        last_trig_beat = clock();
                    }
                }
            } else {
                if(final_val < 1000) {
                    beat_inc = 0;
                }
            }
            break;
        }

        case 2: {
            if(!force_start) {
                if(final_val > 100) {
                    force_start = 1;
                    force_st_time = clock();
                }
            } else {
                if(final_val > 400 && clock() - force_start >= 800) {
                    if((last_long_force && clock() - last_long_force > 3000) || !last_long_force) {
                        add_pendant_cmd(&pendant_cmd_queue, LONG_PRESS);
                        last_long_force = clock();
                    }
                }
                else if(final_val < 100) {
                    force_rls = clock();
                    if(clock() - force_st_time <= 500) {
                        add_pendant_cmd(&pendant_cmd_queue, TAP);
                    } else {
                        add_pendant_cmd(&pendant_cmd_queue, LONG_STOP);
                    }
                    last_long_force = 0;
                    force_start = 0;
                    force_st_time = 0;
                }
            }
            break;
        }

        case 4: {

            break;
        }
        
        default: {
            break;
        }
        }

        i++;
        if(i >= data->size) {
            break;
        }
        final_data++;
    }

    return false;
}