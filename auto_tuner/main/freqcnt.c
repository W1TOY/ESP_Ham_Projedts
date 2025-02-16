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
#include "esp_mac.h"
#include "mqtt_client.h"
#include "readpower.h"
#include "driver/gpio.h"
#include "chanmemory.h"

#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "readpower.h"

// GPIO to enable pulse counting, it will be set up to enable coutning when when the signal on this pin is active
#define GPIO_PCNT_ENABLE_INPUT 35
#define GPIO_PCNT_ENABLE_PIN_MASK  (1ULL<<GPIO_PCNT_ENABLE_INPUT)

// GPIO input to read pulses
#define GPIO_PCNT_INPUT 36


// GPIO to generate the pulse to start counting, this pin should be connected to GPIO_PCNT_ENABLE_INPUT externally
#define GPIO_PCNT_ENABLE_OUTPUT 45

// General PCNT values
#define FCNT_LIMIT  32000       // maximum pulse counts, if number of pulses exceeds this value, then overflow count will be incremented

static QueueHandle_t gpio_evt_queue = NULL;
int pulse_count = 0;            // varaible to read instantaneous pulse count
int32_t overflow_cnts_0 = 0;
int32_t total_cnts_0 = 0;
int32_t measured_freqs_0 = 0;
int gpio_num_input_freq_0 = GPIO_PCNT_INPUT;
int gpio_num_enable_count_0 = GPIO_PCNT_ENABLE_INPUT;
pcnt_unit_handle_t pcnt_units_0;
pcnt_channel_handle_t pcnt_chans_0;

// call back function when number of counted pulses exceeds threshold, overflow count will be incremented
static bool example_pcnt_on_reach_0(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) { overflow_cnts_0++; return true; }

//  initialize pulse count system
void init_freq_pcnt()
{

    pcnt_unit_config_t unit_configs_0;
    pcnt_chan_config_t chan_configS_0;
    int watch_points[] = {-FCNT_LIMIT, FCNT_LIMIT};

    pcnt_event_callbacks_t cbs_0;
    cbs_0.on_reach = example_pcnt_on_reach_0;

    QueueHandle_t queue_0;

    unit_configs_0.high_limit = FCNT_LIMIT;
    unit_configs_0.low_limit = -FCNT_LIMIT;
    unit_configs_0.intr_priority = 0;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_configs_0, &pcnt_units_0));

    chan_configS_0.level_gpio_num = gpio_num_enable_count_0;
    chan_configS_0.edge_gpio_num = gpio_num_input_freq_0;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units_0, &chan_configS_0, &pcnt_chans_0));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chans_0, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chans_0, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chans_0, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    for (size_t n = 0; n < sizeof(watch_points) / sizeof(watch_points[0]); n++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_units_0, watch_points[n]));
    }
    queue_0 = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_units_0, &cbs_0, queue_0));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_units_0));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_units_0));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_units_0));


}

// this function is called from main to set up the pulse counter
void app_setup_pcnt(void)
{
    setup_gpio_output(GPIO_PCNT_ENABLE_OUTPUT);
    gpio_set_level(GPIO_PCNT_ENABLE_OUTPUT, 0);
    init_freq_pcnt();

    gpio_pullup_en(GPIO_PCNT_INPUT);
    gpio_pulldown_en(GPIO_PCNT_INPUT);
//    gpio_pullup_dis(GPIO_PCNT_INPUT);
//    gpio_pulldown_dis(GPIO_PCNT_INPUT);

}

// functino to measure the pulse counts
int32_t measure_frequency(void)
{
    int i = 0;
    int32_t mfreq = 0;

    int count_ms = 200;                         // read number of pulses for this duration
    int mult_to_real_freq = 1000 / count_ms;    // multiply this to get actual frequency 

    
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_units_0));  // clear the values that contains the number of counted pulses
    overflow_cnts_0 = 0;                                   // clear the number of counts that counter exceedes maximum limit


    gpio_set_level(GPIO_PCNT_ENABLE_OUTPUT, 0);             // GPIO output to enable pulse counting, it is supposed to count when the level is high, but somehow reversed
    vTaskDelay(pdMS_TO_TICKS(count_ms));                    // wait for counting
    gpio_set_level(GPIO_PCNT_ENABLE_OUTPUT, 1);             // GPIO output to stop pulse counting

    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_units_0, &pulse_count));    
    mfreq = overflow_cnts_0 * FCNT_LIMIT + pulse_count;
    mfreq = mfreq * mult_to_real_freq;

    printf("Raw measured freq = %ld \n", mfreq);

    if (mfreq  <  1790000)  mfreq = 0;
    if ((mfreq >  2100000) && (mfreq <  3490000)) mfreq =  3500000;
    if ((mfreq >  4100000) && (mfreq <  6900000)) mfreq =  7000000;
    if ((mfreq >  7400000) && (mfreq < 10000000)) mfreq = 10000000;
    if ((mfreq > 10200000) && (mfreq < 13900000)) mfreq = 14000000;
    if ((mfreq > 14400000) && (mfreq < 17900000)) mfreq = 18000000;
    if ((mfreq > 18200000) && (mfreq < 20900000)) mfreq = 21000000;
    if ((mfreq > 22500000) && (mfreq < 24000000)) mfreq = 24000000;
    if ((mfreq > 25100000) && (mfreq < 27900000)) mfreq = 28000000;
    if (mfreq  > 29710000) mfreq = 50000000;

    return mfreq;
}
