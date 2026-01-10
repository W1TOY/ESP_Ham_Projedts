#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "protocol_examples_common.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "esp_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sys.h"
#include "lwip/err.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "readpower.h"
#include "driver/gpio.h"
#include "chanmemory.h"

#include "driver/pulse_cnt.h"
#include "freqcnt.h"


static const char *TAG = "MY_AUTO_TUNER_1";
#include "wifipassword.h"   // define following two variables in the wifipassword.h
//#define MY_ESP_WIFI_SSID      "mywifi_ssid"
//#define MY_ESP_WIFI_PASS      "password_for_mywifi"

#define MY_ESP_MAXIMUM_RETRY  5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

int32_t antenna = 0;
int32_t mem_val_l[4] = {0};
int32_t mem_val_c[4] = {0};
int32_t mem_val_p[4] = {0};
int mem_calibrated[4] = {0};
int mem_online[4] = {0};
int mem_error[4] = {0};
int mem_from_flash[4] = {0};
static QueueHandle_t gpio_evt_queue = NULL;
int gpio_button[4] = { 0};
int gpio_lights[4] = {0};
int last_antenna = 0;
int32_t last_freq = 0;
int process_keys = 1;
int relay_setting_delay = 0;

void process_short_button(int button_num);
void process_long_button(int button_num);

//***************************************************************************** 
// Wifi setup and reconnect
//***************************************************************************** 
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MY_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = MY_ESP_WIFI_SSID,
            .password = MY_ESP_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 MY_ESP_WIFI_SSID, MY_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 MY_ESP_WIFI_SSID, MY_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

//***************************************************************************** 
// MQTT handler
//***************************************************************************** 

esp_mqtt_client_handle_t client = NULL;

void publish_selected_antenna(int ant_num)
{
    if (ant_num == 0) {
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT1", "on", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT2", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT3", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT4", "off", 0, 0, 0);
    }
    else if (ant_num == 1) {
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT1", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT2", "on", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT3", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT4", "off", 0, 0, 0);
    }
    else if (ant_num == 2) {
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT1", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT2", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT3", "on", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT4", "off", 0, 0, 0);
    }
    else if (ant_num == 3) {
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT1", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT2", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT3", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT4", "on", 0, 0, 0);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    int tune_result = 0;
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        esp_mqtt_client_subscribe(client, "MY_AUTO_TUNER_1/Bypass", 0);
        esp_mqtt_client_subscribe(client, "MY_AUTO_TUNER_1/Tune", 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "off", 0, 0, 0);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuning", "off", 0, 0, 0);
        
        esp_mqtt_client_subscribe(client, "MY_AUTO_TUNER_1/ANT_SELECT", 0);
        esp_mqtt_client_subscribe(client, "MY_AUTO_TUNER_1/Beacon", 0);

        publish_selected_antenna(antenna);

        vTaskDelay(pdMS_TO_TICKS(500));
        break;
    case MQTT_EVENT_DATA:
//        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
//        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
//        printf("DATA=%.*s\r\n", event->data_len, event->data);

        if (event->topic_len < 1) break;

        if ( strncmp(event->topic, "MY_AUTO_TUNER_1/ANT_SELECT", strlen("MY_AUTO_TUNER_1/ANT_SELECT")) == 0) {
            if (strncmp(event->data, "on", 2) == 0)
            {
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/ANT_SELECT", "off", 0, 0, 0);
                int button_num = antenna;
                button_num += 1;
                if (button_num >= 4) button_num = 0;
                process_short_button(button_num);
                publish_selected_antenna(antenna);            
            }
        }
        else if ( strncmp(event->topic, "MY_AUTO_TUNER_1/Tune", strlen("MY_AUTO_TUNER_1/Tune")) == 0) {
            if (strncmp(event->data, "on", 2) == 0)
            {
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tune", "off", 0, 0, 0);
                process_long_button(antenna);
            }
        }
        else if ( strncmp(event->topic, "MY_AUTO_TUNER_1/Bypass", strlen("MY_AUTO_TUNER_1/Bypass")) == 0) {
            if (strncmp(event->data, "on", 2) == 0)
            {
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Bypass", "off", 0, 0, 0);
                process_short_button(antenna);
            }
        }

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;

    case  MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DIsconnected");
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

//***************************************************************************** 
// Start MQTT 
//***************************************************************************** 

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://10.1.1.180",
        .credentials.username = "hamqtt",
        .credentials.authentication.password="931121",
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = 5000,
        .network.timeout_ms = 5000
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}


//***************************************************************************** 
// To send heart beat beacons to MQTT server and local LED to indicate that this system is operational
//***************************************************************************** 

int beacon_status = 0;
void vTaskPeriodic(void *pvParameters)
{
    for (;;) {
        if (client == NULL) continue;
        if (beacon_status == 0) {
            beacon_status = 1;
            esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Beacon", "on", 0, 0, 0);
//            gpio_set_level(GPIO_ONBOARD_LED, 0);

        }
        else {
            beacon_status = 0;
            esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Beacon", "off", 0, 0, 0);
//            gpio_set_level(GPIO_ONBOARD_LED, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
//***************************************************************************** 
// to blink error light to server
//***************************************************************************** 

void vTaskErrorDisplay(void *pvParameters)
{
    int error_status = 0;
    for (;;) {
        if (mem_error[antenna] ==1) {
            if (error_status ==0) {
                error_status = 1;
                gpio_set_level(gpio_lights[antenna], 1);
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "on", 0, 0, 0);            }
            else {
                error_status = 0;
                gpio_set_level(gpio_lights[antenna], 0);
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "off", 0, 0, 0);            
            }
            vTaskDelay(pdMS_TO_TICKS(100));

        }
        else if (mem_error[antenna] ==2) {
            if (error_status ==0) {
                error_status = 1;
                gpio_set_level(gpio_lights[antenna], 1);
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "on", 0, 0, 0);            }
            else {
                error_status = 0;
                gpio_set_level(gpio_lights[antenna], 0);
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "off", 0, 0, 0);            
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (mem_error[antenna] ==3) {
            if (error_status ==0) {
                error_status = 1;
                gpio_set_level(gpio_lights[antenna], 1);
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "on", 0, 0, 0);            }
            else {
                error_status = 0;
                gpio_set_level(gpio_lights[antenna], 0);
                esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "off", 0, 0, 0);            
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }


        if (client == NULL) continue;
    }
}

void vTaskSettlingDisplay(void *pvParameters)
{
    // number of blinks every 5 sec
    for (;;) {

        if (relay_setting_delay == 0)
        {
            gpio_set_level(gpio_lights[3], 1);
        } 
        if (relay_setting_delay > 0) 
        {
            for (int i=0; i<relay_setting_delay; i++)
            {
                gpio_set_level(gpio_lights[3], 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(gpio_lights[3], 0);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
//***************************************************************************** 
// GPIO Button handler, control LED lights on each button based MQTT message
//***************************************************************************** 
static void IRAM_ATTR gpio_isr_handler_0(void* arg) { uint32_t button_num = (uint32_t) arg; xQueueSendFromISR(gpio_evt_queue, &button_num, NULL);}
void turn_on_antenna_light(int ant_num)
{
    for (int i=0; i<4; i++) gpio_set_level(gpio_lights[i], 0);
    if (ant_num >= 0)
    {
        gpio_set_level(gpio_lights[ant_num], 1);
        printf("Anttena[%d], light is turned on \n",ant_num);
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "on", 0, 0, 0);    
    }
    else {
        printf("Anttena[%d], light is turned off \n",ant_num);
       esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuned", "off", 0, 0, 0);
    }
}

//***************************************************************************** 
// load relay values from memory (when antenna switch switched)
//***************************************************************************** 
void load_lcvalues(int ant_num)
{
    set_LC((uint32_t)mem_val_p[ant_num], (uint32_t)mem_val_l[ant_num],(uint32_t)mem_val_c[ant_num]);
    printf("Antenna %d, loaded l = %ld, c = %ld, p =  %ld \n", ant_num, mem_val_l[ant_num], mem_val_c[ant_num], mem_val_p[ant_num]);
}

//***************************************************************************** 
// Turn the light ON if this antenna is tuned already
//***************************************************************************** 
void make_chan_online_if_calibrated(int ant_num)
{
    turn_on_antenna_light(-1);
    if (mem_calibrated[ant_num] == 1) {
        turn_on_antenna_light(ant_num);
        mem_online[ant_num] = 1;
        load_lcvalues(ant_num);                 
        printf("Anttena[%d], LC values are loaded \n",ant_num);
        mem_error[ant_num] = 0;     // selected but no calibration data

    }
    else {
        mem_online[ant_num] = 0;
        mem_error[ant_num] = 3;     // selected but no calibration data
        bypass_tuning_LC();
        printf("Anttena[%d], LC values are bypassed \n",ant_num);
    }
}
//***************************************************************************** 
// Toggle the light (based on short key)
//***************************************************************************** 
void toggle_chan_online_if_calibrated(int ant_num)
{
    turn_on_antenna_light(-1);
    if (mem_calibrated[ant_num] == 1) {
        if (mem_online[antenna] ==1 )
        {
            mem_online[ant_num] = 0;
            turn_on_antenna_light(-1);
            bypass_tuning_LC();
            printf("Anttena[%d], LC values are bypassed \n",ant_num);
            mem_error[ant_num] = 3;     // selected but no calibration data
        }
        else {
            mem_online[ant_num] = 1;
            turn_on_antenna_light(ant_num);
            load_lcvalues(ant_num);                 
            printf("Anttena[%d], LC values are loaded \n",ant_num);
            mem_error[ant_num] = 0;     // selected but no calibration data

        }
    }
    else {
        turn_on_antenna_light(-1);
        mem_online[antenna] = 0;                                 
        bypass_tuning_LC();
        printf("Anttena[%d], LC values are bypassed \n",ant_num);
        mem_error[ant_num] = 3;     // selected but no calibration data
    }
}

//***************************************************************************** 
// Use 4th button to change the relay setting time
//***************************************************************************** 
void change_relay_setting_delay()
{
    relay_setting_delay++;
    if (relay_setting_delay > 2) relay_setting_delay = 0;

    if (relay_setting_delay == 0)
    {
        relay_delay_coarse = 30;
        relay_delay_fine = 30;
    }
    else if (relay_setting_delay == 1)
    {
        relay_delay_coarse = 50;
        relay_delay_fine = 50;
    }
    else if (relay_setting_delay == 2)
    {
        relay_delay_coarse = 15;
        relay_delay_fine = 15;
    }

}

//***************************************************************************** 
// function to processo short key (button is pressed shorter than one sec)
// 1. If new antenna is selected
//     - load relay values if previously calibrated, and LED on (ONLINE)
//     - or init LC relays and off (OFFLINE)
// 2. If same antenna, then toggle ONLINE / OFFLINE
//***************************************************************************** 
void process_short_button(int button_num)
{
    if (button_num == 3) {
        change_relay_setting_delay();
        return;
    }
    if (button_num != antenna) {
        antenna = button_num;
        make_chan_online_if_calibrated(antenna);
        publish_selected_antenna(antenna);  
        last_freq = -1;
    }
    else {
        toggle_chan_online_if_calibrated(antenna);
    }
//    mem_error[antenna] = 0;
}
//***************************************************************************** 
// function to processo long key (button is pressed for longer than one sec)
// 1. If new antenna is selected, then it works same as short key
// 2. If the sam antenna,
//     - Read frequency and load LC values from memory if found in memory
//     - Perform full tuning if not found in memory
//     - Perform full tuning if long key is pressed again, and save the results to memory
//***************************************************************************** 
void process_long_button(int button_num)
{
    if (button_num == 3) {
        change_relay_setting_delay();
        return;
    }

    printf("Processing long button ... \n");
    if (button_num != antenna) {    // if calibrated, load the calibrated LC values from the memory
        antenna = button_num;
        make_chan_online_if_calibrated(antenna);
        publish_selected_antenna(antenna);          
        last_freq = -1;
    }
    else {  // if not calibrated, or long button is pressed again
        double swr = 0;
        printf("Set tuning mode.. \n");
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuning", "on", 0, 0, 0);    
        mem_val_l[antenna] = 0;
        mem_val_c[antenna] = 0;
        mem_val_p[antenna] = 0;
        int tready = set_tuning_mode();
        if (tready == 0)
        {
            int sresult = 0;
            sresult = search_lowest_SWR(&mem_val_l[antenna], &mem_val_c[antenna], &mem_val_p[antenna], &swr, 0);
            if (sresult != -1) {
                printf("Calibrated, antenna = %ld, l= %ld, c = %ld, p = %ld \n", antenna, mem_val_l[antenna], mem_val_c[antenna], mem_val_p[antenna]);
                mem_calibrated[antenna] = 1;
                make_chan_online_if_calibrated(antenna);
                mem_error[antenna] = 0;
            }
            else {
                mem_calibrated[antenna] = 0;
                make_chan_online_if_calibrated(antenna);
                mem_error[antenna] = 1;             //tuning failure 
            }
        }
        else if (tready == 1)
        {
            printf("Tuning fails because of too litte power \n");
            mem_calibrated[antenna] = 0;
            make_chan_online_if_calibrated(antenna);
            mem_error[antenna] = 1;     //tuning failure           
            mem_error[antenna] = 1;             //tuning failure 
        }
        else {
            printf("Tuning fails because of too much power \n");
            mem_error[antenna] = 1;            
            mem_calibrated[antenna] = 0;
            make_chan_online_if_calibrated(antenna);
            mem_error[antenna] = 1;             //tuning failure 
        }
        close_tuning_mode();
        esp_mqtt_client_publish(client, "MY_AUTO_TUNER_1/Tuning", "off", 0, 0, 0);    
    }

}
//***************************************************************************** 
// Task to read GPIO and call the short or long key process function
//***************************************************************************** 
static void gpio_task_example(void* arg) // change from interrupt to polling, evern 100 msec
{
    uint32_t button_num;
    int gpio_level = 0;
    int button_status = 0;
    int short_count = 0;
    for (;;)
    {
        for (;;) {
            for (button_num=0; button_num<4; button_num++) {
                gpio_level = gpio_get_level(gpio_button[button_num]);
                if (gpio_level == 0) {
                    button_status = 1;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            if (button_status == 1) {
                button_status = 0;
                short_count = 0;
                for (;;) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    gpio_level = gpio_get_level(gpio_button[button_num]);
                    if (gpio_level == 1) {
                        printf("Short key on channel %ld \n", button_num);
                        if (process_keys ==1) process_short_button(button_num);
                        break;
                    }
                    short_count++;
                    if (short_count > 10) break;
                }
                if (gpio_level == 0) {
                    printf("Long key on channel %ld \n", button_num);
                    if (process_keys ==1) process_long_button(button_num);
                }
                for (;;)
                {
                    gpio_level = gpio_get_level(gpio_button[button_num]);
                    if (gpio_level == 1) break;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
    }

}

//***************************************************************************** 
// Main
//***************************************************************************** 


void app_main(void)
{
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    
    ESP_LOGI(TAG, "Initializing channel memory..");
    //erase_init_chan_mem();    // to erase all memories manually if required
    init_chan_mem();    
    //test_chan_memmory();

    ESP_LOGI(TAG, "Initializing pulse counter..");
    app_setup_pcnt();

    ESP_LOGI(TAG, "Initializing tuning board..");
    init_adc_tunerboard();

    antenna = 0;
    for (int i=0; i<4; i++)
    {
        mem_val_l[i] = 0;
        mem_val_c[i] = 0;
        mem_val_p[i] = 0;
        mem_calibrated[i] = 0;
        mem_online[i] = 0;
        mem_error[i] = 0;
        mem_from_flash[i] = 0;
    }
    gpio_intr_enable((gpio_num_t)GPIO_BUTTON_0);
    gpio_intr_enable((gpio_num_t)GPIO_BUTTON_1);
    gpio_intr_enable((gpio_num_t)GPIO_BUTTON_2);
    gpio_intr_enable((gpio_num_t)GPIO_BUTTON_3);

    gpio_set_intr_type((gpio_num_t)GPIO_BUTTON_0, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)GPIO_BUTTON_1, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)GPIO_BUTTON_2, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)GPIO_BUTTON_3, GPIO_INTR_NEGEDGE);

    gpio_button[0] = GPIO_BUTTON_0;
    gpio_button[1] = GPIO_BUTTON_1;
    gpio_button[2] = GPIO_BUTTON_2;
    gpio_button[3] = GPIO_BUTTON_3;

    gpio_lights[0] = GPIO_BUTTON_LIGHT_0;
    gpio_lights[1] = GPIO_BUTTON_LIGHT_1;
    gpio_lights[2] = GPIO_BUTTON_LIGHT_2;
    gpio_lights[3] = GPIO_BUTTON_LIGHT_3;

    #define ESP_INTR_FLAG_DEFAULT 0

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add((gpio_num_t)GPIO_BUTTON_0, gpio_isr_handler_0, (void*) 0);
    gpio_isr_handler_add((gpio_num_t)GPIO_BUTTON_1, gpio_isr_handler_0, (void*) 1);
    gpio_isr_handler_add((gpio_num_t)GPIO_BUTTON_2, gpio_isr_handler_0, (void*) 2);
    gpio_isr_handler_add((gpio_num_t)GPIO_BUTTON_3, gpio_isr_handler_0, (void*) 3);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 4096, NULL, tskIDLE_PRIORITY, NULL);


    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    wifi_init_sta();
    mqtt_app_start();

    //*****************************************************
    // Unit tests
    //*****************************************************
//#define BUTTON_TEST
//#define RF_RELAY_FREQUENCY_TEST
//#define PTT_RELAY_TEST


    //*****************************************************
    // Relay test
    //*****************************************************
//    test_relay_lc_scan();
//    check_fwd_ref_power();
//    test_tuning();
//    check_signal_level();

    //*****************************************************
    // Button test
    //*****************************************************
#ifdef  BUTTON_TEST
    process_keys = 0;
    for (;;)
    {
        int gpio_input_level = 0;
        printf("\n");                
        gpio_input_level = gpio_get_level((gpio_num_t)GPIO_BUTTON_0);
        printf("Button input = %d, ", gpio_input_level);
        gpio_input_level = !gpio_input_level;
        gpio_set_level((gpio_num_t)GPIO_BUTTON_LIGHT_0, gpio_input_level);

        gpio_input_level = gpio_get_level((gpio_num_t)GPIO_BUTTON_1);
        printf(" %d, ", gpio_input_level);
        gpio_input_level = !gpio_input_level;
        gpio_set_level((gpio_num_t)GPIO_BUTTON_LIGHT_1, gpio_input_level);


        gpio_input_level = gpio_get_level((gpio_num_t)GPIO_BUTTON_2);
        printf(" %d, ", gpio_input_level);
        gpio_input_level = !gpio_input_level;
        gpio_set_level((gpio_num_t)GPIO_BUTTON_LIGHT_2, gpio_input_level);

        gpio_input_level = gpio_get_level((gpio_num_t)GPIO_BUTTON_3);
        printf(" %d \n", gpio_input_level);
        gpio_input_level = !gpio_input_level;
        gpio_set_level((gpio_num_t)GPIO_BUTTON_LIGHT_3, gpio_input_level);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
    //*****************************************************
    // Frequency and power measurement test, RF bypass relay trest
    //*****************************************************
#ifdef  RF_RELAY_FREQUENCY_TEST
    int32_t freq_measured = 0;
    for (;;) {
        gpio_set_level(GPIO_RF_BYPASS, 1); // tuning mode
        set_LC(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        freq_measured = measure_frequency();
        printf("frequency measured = %ld \n", freq_measured);
        gpio_set_level(GPIO_RF_BYPASS, 0); // tuning mode
        vTaskDelay(pdMS_TO_TICKS(2000));
    } 
#endif
    //*****************************************************
    // PTT relay test
    //*****************************************************
#ifdef PTT_RELAY_TEST
    for (;;) {
        gpio_set_level(GPIO_PTT_SWITCH, 1); // disconnect PTT pass thru
        printf("PTT disconnected \n");
        vTaskDelay(pdMS_TO_TICKS(5000));
        gpio_set_level(GPIO_PTT_SWITCH, 0); // disconnect PTT pass thru
        printf("PTT connected \n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    } 

#endif

    //*****************************************************
    // create tasks
    //*****************************************************

    xTaskCreate( vTaskPeriodic, "Periodic task", 4096, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
    xTaskCreate( vTaskErrorDisplay, "Error display task", 4096, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
    xTaskCreate( vTaskSettlingDisplay, "Setting Delay task", 4096, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );


}
