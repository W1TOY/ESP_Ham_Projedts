
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
//************************************************************** 
// channel memory
//************************************************************** 
char freq_key_string[12];
nvs_handle_t my_handle;
esp_err_t err;


//************************************************************** 
// make a key based on frequency and antenna selection
// this key is used to save / restore values from nv memory
//************************************************************** 
void get_key_freq(int32_t freq, int32_t nant, char* freq_key)
{
    freq = (freq & FREQ_STEP_MASK) + nant;
    sprintf(freq_key, "F%9ld", freq);
}

//************************************************************** 
// erase flash, this functions is called only manually when needed
//************************************************************** 
void erase_init_chan_mem()
{
    printf("\n");
    printf("Erasing channel memory \n");
    ESP_ERROR_CHECK(nvs_flash_erase());
    esp_err_t err = nvs_flash_init();       // should be initailzied again
    ESP_ERROR_CHECK( err );
    err = nvs_open("storage", NVS_READWRITE, &my_handle);

}
//************************************************************** 
// initialize without erasing 
//************************************************************** 
void init_chan_mem()
{
    esp_err_t err = nvs_flash_init();       // should be initailzied again
    ESP_ERROR_CHECK( err );
    err = nvs_open("storage", NVS_READWRITE, &my_handle);

}

//************************************************************** 
// read and write L, C, and cap position value from flash
//************************************************************** 
void extract_lcvalues(int32_t key_val, int32_t* mem_l, int32_t* mem_c, int32_t* mem_p)
{
    *mem_l = (int32_t)((key_val & 0x000000FF) >> 0);
    *mem_c = (int32_t)((key_val & 0x0000FF00) >> 8);
    *mem_p = (int32_t)((key_val & 0x000F0000) >> 16);
}
void pack_lcvalues(int32_t* key_val,  int32_t mem_l, int32_t mem_c, int32_t mem_p)
{
    *key_val = (int32_t)mem_l;
    *key_val |= (int32_t)mem_c << 8;
    *key_val |= (int32_t)mem_p << 16;
}
int read_chan_mem(int32_t freq, int32_t selected_ant, int32_t* mem_l, int32_t* mem_c, int32_t* mem_p) 
{
    int32_t key_val = 0;
    printf("Read key from memory, freq = %ld, antenna = %ld \n", freq, selected_ant);
    get_key_freq(freq, selected_ant, &freq_key_string[0]);
    err = nvs_get_i32(my_handle, freq_key_string, &key_val);
    switch (err) {
        case ESP_OK:
            extract_lcvalues(key_val, mem_l, mem_c, mem_p);
            printf("Read Key from memory: freq = %ld, ant = %ld, l=%ld, c=%ld, p=%ld \n", freq, selected_ant, *mem_l, *mem_c, *mem_p);
            return 0;
        case ESP_ERR_NVS_NOT_FOUND:
            *mem_l = -1;
            *mem_c = -1;
            *mem_p = -1;
            printf("Key not found: freq = %ld, ant = %ld, l=%ld, c=%ld, p=%ld \n", freq, selected_ant, *mem_l, *mem_c, *mem_p);
            return -1;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
            return -2;
    }
}
int save_chan_mem(int32_t freq, int32_t selected_ant, int32_t mem_l, int32_t mem_c, int32_t mem_p) 
{
    int32_t key_val = 0;
    printf("Save key to memory,   freq = %ld, antenna = %ld \n", freq, selected_ant);
    get_key_freq(freq, selected_ant, &freq_key_string[0]);
    pack_lcvalues(&key_val,  mem_l, mem_c, mem_p);
    err = nvs_set_i32(my_handle, freq_key_string, key_val);
    err = nvs_commit(my_handle);
    return 0;
}

//************************************************************** 
// Test functions
//************************************************************** 
void test_chan_memmory()
{
    int32_t freq = 14100000;
    int32_t selected_ant = 1;
    int32_t mem_l = 1;
    int32_t mem_c = 2;
    int32_t mem_p = 0;

    freq = 14100000;
    selected_ant = 1;
    read_chan_mem(freq, selected_ant, &mem_l, &mem_c, &mem_p) ;
    mem_l = 2; mem_c = 3;mem_p = 0;
    save_chan_mem(freq, selected_ant, mem_l, mem_c, mem_p) ;

    freq = 24100000;
    selected_ant = 1;
    read_chan_mem(freq, selected_ant, &mem_l, &mem_c, &mem_p) ;
    mem_l = 9; mem_c = 8;mem_p = 1;
    save_chan_mem(freq, selected_ant, mem_l, mem_c, mem_p) ;

    freq = 34100000;
    selected_ant = 2;
    read_chan_mem(freq, selected_ant, &mem_l, &mem_c, &mem_p) ;
    mem_l = 4; mem_c = 5;mem_p = 1;
    save_chan_mem(freq, selected_ant, mem_l, mem_c, mem_p) ;

    freq = 14110000;
    selected_ant = 1;
    read_chan_mem(freq, selected_ant, &mem_l, &mem_c, &mem_p) ;

    freq = 24100000;
    selected_ant = 2;
    read_chan_mem(freq, selected_ant, &mem_l, &mem_c, &mem_p) ;
    mem_l = 6; mem_c = 7;mem_p = 1;
    save_chan_mem(freq, selected_ant, mem_l, mem_c, mem_p) ;

    freq = 24100000 - 1000;
    selected_ant = 1;
    read_chan_mem(freq, selected_ant, &mem_l, &mem_c, &mem_p) ;

    freq = 24100000 - 1000;
    selected_ant = 0;
    read_chan_mem(freq, selected_ant, &mem_l, &mem_c, &mem_p) ;

}



