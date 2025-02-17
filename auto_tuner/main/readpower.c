/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "readpower.h"

char pline[128];
int l_value = 0;
int c_value = 0;
double SWR = 1.0;
double pwr_fwd = 1.0;
double pwr_ref = 1.0;
double ref_coeff = 1.0;;
double swr = 1.0;

int status_tuning = 0;
int status_bypassed = 0;
int warning_high_swr = 0;

int print_search_debug = 1;
int relay_delay = 20;

int donotstop = 0;
char xinput;



// coarse search
int i_ind = 0;
int i_cap = 0;
int lowest_ind = 0;
int lowest_cap = 0;
int lowest_pos = 0;

int lowest_ind_A = 0;
int lowest_cap_A = 0;
int lowest_ind_B = 0;
int lowest_cap_B = 0;


// ADC instantiation
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};
adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
};

//**************************************************************************** 
// Function to read reflected power
// reflected power is used as swr as ref power is minized without knowing actual swr
//**************************************************************************** 
int32_t check_value_l = 0;
int32_t check_value_c = 0;
int32_t check_value_p = 0;

double read_ref()
{
    int aread = 0;
    double aref = 0;
    for (int i=0; i< NUM_ADC_READ; i++) { 
      ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_REF, &aread)); 
      aref += aread;
//      printf("each read = %d \n", aread);
    }
    aref = aref / (double)NUM_ADC_READ;
    return aref;
}

double get_swr()
{
    // some unknown problem with this particular relay settings at high frequency, so avoid it.
//    if (check_value_l == 0 && check_value_c == 0 && check_value_p == 1)
//    {
//      return 10000.0;
//    }
    return read_ref();
}

//**************************************************************************** 
// Instead of waiting fixed period of time after relay swithing
// this function measured swr continusously and stop reading if value becomes consistent within 5%
// hoping that this saves total tuning time
//**************************************************************************** 

double get_swr_adaptive_settling()
{
    // bypass adaptive algorithm
    vTaskDelay(pdMS_TO_TICKS(RELAY_SETTLING_TIME));
    return get_swr();

    // wait for min time
    vTaskDelay(pdMS_TO_TICKS(RELAY_SETTLING_TIME_MIN));   // wait for minimum time first
    double ref_n = 0;
    double ref_p = -1000;
    double ref_pp = -2000;
    int delay_delta = 2;
    for (int delay = 0; delay < RELAY_SETTLING_TIME; delay+=delay_delta)
    {
        vTaskDelay(pdMS_TO_TICKS(delay_delta));
        ref_n = get_swr();
        double ref_delta_1 = ref_n - ref_p;
        double ref_delta_2 = ref_n - ref_pp;
        if (ref_n == 0) ref_n = 1;

        if (ref_delta_1 < 0) ref_delta_1 = - ref_delta_1;
        if (ref_delta_2 < 0) ref_delta_2 = - ref_delta_2;
        if ((ref_delta_1 / ref_n) < 0.05 && (ref_delta_2 / ref_n) < 0.05 ) {
            break;
        }
        ref_pp = ref_p;
        ref_p = ref_n;

    }

    return ref_n;
}



//*************************************************************************** 
// Function to intialize the tuning board
//*************************************************************************** 
void setup_gpio_input(int pinnum)
{
    gpio_reset_pin((gpio_num_t)pinnum);
    gpio_intr_disable((gpio_num_t)pinnum);
    gpio_input_enable((gpio_num_t)pinnum);
}

void setup_gpio_output(int pinnum)
{
  gpio_set_drive_capability((gpio_num_t)pinnum, GPIO_DRIVE_CAP_DEFAULT);
  gpio_set_direction((gpio_num_t)pinnum, GPIO_MODE_OUTPUT);
/*  
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << pinnum;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
*/  
}

void init_adc_tunerboard()
{

    setup_gpio_input(GPIO_BUTTON_0);
    setup_gpio_input(GPIO_BUTTON_1);
    setup_gpio_input(GPIO_BUTTON_2);
    setup_gpio_input(GPIO_BUTTON_3);

    gpio_pullup_en((gpio_num_t)GPIO_BUTTON_0);    
    gpio_pullup_en((gpio_num_t)GPIO_BUTTON_1);    
    gpio_pullup_en((gpio_num_t)GPIO_BUTTON_2);    
    gpio_pullup_en((gpio_num_t)GPIO_BUTTON_3);    

    setup_gpio_output(GPIO_BUTTON_LIGHT_0);
    setup_gpio_output(GPIO_BUTTON_LIGHT_1);
    setup_gpio_output(GPIO_BUTTON_LIGHT_2);
    setup_gpio_output(GPIO_BUTTON_LIGHT_3);

    // Turn all lights on at start up
    gpio_set_level(GPIO_BUTTON_LIGHT_0, 1);
    gpio_set_level(GPIO_BUTTON_LIGHT_1, 1);
    gpio_set_level(GPIO_BUTTON_LIGHT_2, 1);
    gpio_set_level(GPIO_BUTTON_LIGHT_3, 1);


//    setup_gpio_output(GPIO_ONBOARD_LED);
//    gpio_set_level(GPIO_ONBOARD_LED, 0);

    setup_gpio_output(GPIO_RF_BYPASS);
    gpio_set_level(GPIO_RF_BYPASS, 0);

    setup_gpio_output(GPIO_PTT_SWITCH);
    gpio_set_level(GPIO_PTT_SWITCH, 0);

    setup_gpio_output(GPIO_OUT_EN_N);
    gpio_set_level(GPIO_OUT_EN_N, 1);

    setup_gpio_output(GPIO_SRCK);
    gpio_set_level(GPIO_SRCK, 0);

    setup_gpio_output(GPIO_RCK);
    gpio_set_level(GPIO_RCK, 0);

    setup_gpio_output(GPIO_SER_IN);
    gpio_set_level(GPIO_SER_IN, 0);

    // adc init
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_REF, &config));
}


//*************************************************************************** 
// function to send L/C/P values to shift register
//*************************************************************************** 
void shift_one_bit(uint32_t bit_in)
{
    if (bit_in == 0)
      gpio_set_level(GPIO_SER_IN, 0);
    else 
      gpio_set_level(GPIO_SER_IN, 1);

    gpio_set_level(GPIO_SRCK, 1);
    gpio_set_level(GPIO_SRCK, 0);

}
void clear_shift_registers()
{
  for (int i=0; i<16; i++)
  {
    shift_one_bit(0);
  }
}

void disable_shift_register_output()
{
  gpio_set_level(GPIO_OUT_EN_N, 1);
}

void enable_shift_register_output()
{
  gpio_set_level(GPIO_RCK, 1);
  gpio_set_level(GPIO_RCK, 0);
  gpio_set_level(GPIO_OUT_EN_N, 0);

}

void send_value_to_shift_registers(uint32_t valueRelay)
{
  for (int i=0; i<16; i++)
  {
    uint32_t bit_mask = 1;
    bit_mask = bit_mask << i;
    bit_mask = bit_mask & valueRelay;
    shift_one_bit(bit_mask);
  }
}

//*************************************************************************** 
// Functions to set LC relays
//*************************************************************************** 

void clear_all_LC_relays()
{
    uint32_t valueR = 0;
    send_value_to_shift_registers(valueR);
}
void set_LC_relays(uint32_t vRelay)
{

}
void set_LC(uint32_t cposition, uint32_t valueL, uint32_t valueC)
{
  check_value_l = valueL;   // save tese values for hack, avoid certain relay settings if problemnatic
  check_value_c = valueC;
  check_value_p = cposition;


  disable_shift_register_output();
  uint32_t valueRelay = 0;
  if ((valueC & 0x40) != 0) valueRelay |= 0x02; 
  if ((valueC & 0x20) != 0) valueRelay |= 0x04; 
  if ((valueC & 0x10) != 0) valueRelay |= 0x08; 
  if ((valueC & 0x08) != 0) valueRelay |= 0x10; 
  if ((valueC & 0x04) != 0) valueRelay |= 0x20; 
  if ((valueC & 0x02) != 0) valueRelay |= 0x40; 
  if ((valueC & 0x01) != 0) valueRelay |= 0x80; 

  if ((cposition & 0x01) != 0) valueRelay |= 0x100; 

  if ((valueL & 0x40) != 0) valueRelay |= 0x0200; 
  if ((valueL & 0x20) != 0) valueRelay |= 0x0400; 
  if ((valueL & 0x10) != 0) valueRelay |= 0x0800; 
  if ((valueL & 0x08) != 0) valueRelay |= 0x1000; 
  if ((valueL & 0x04) != 0) valueRelay |= 0x2000; 
  if ((valueL & 0x02) != 0) valueRelay |= 0x4000; 
  if ((valueL & 0x01) != 0) valueRelay |= 0x8000; 

  send_value_to_shift_registers(valueRelay);
  enable_shift_register_output();

}

//*************************************************************************** 
// Relay test functions
//*************************************************************************** 

void vibrate_relay(uint32_t valueRelay)
{
  for (int i=0; i<64; i++)
  {
    disable_shift_register_output();
    send_value_to_shift_registers(0);
    enable_shift_register_output();
    vTaskDelay(pdMS_TO_TICKS(50));
    disable_shift_register_output();
    send_value_to_shift_registers(valueRelay);
    enable_shift_register_output();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void vibrate_relay_lc(uint32_t cposition, uint32_t valueL, uint32_t valueC)
{
  for (int i=0; i<32; i++)
  {

    disable_shift_register_output();
    set_LC(0, 0, 0);
    enable_shift_register_output();
    vTaskDelay(pdMS_TO_TICKS(50));
    disable_shift_register_output();
    set_LC(cposition, valueL, valueC);
    enable_shift_register_output();
    vTaskDelay(pdMS_TO_TICKS(50));
  }

}

void test_relay_lc_scan()
{
  for (int i=0; i<7; i++)
  {
    printf("Turning on the relay C%d \n", i);
    vibrate_relay_lc(0, 0, 1 << i);
  } 
  printf("Turning on C position \n");
  vibrate_relay_lc(1, 0, 0);
  for (int i=0; i<7; i++)
  {
    printf("Turning on the relay L%d \n", i);
    vibrate_relay_lc(0, 1 << i, 0);
  } 


}
void test_relay_scan()
{
  for (int i=0; i<16; i++)
  {
    printf("Turning on the bit %d \n", i);
    vibrate_relay(1<<i);
  }

}




//*************************************************************************** 
// Tuning functions
//*************************************************************************** 
int check_tuning_power()
{
  printf("Reading ref power \n");
  double ref_power = get_swr();
  printf("Power for cheking = %f \n", ref_power);
  if (ref_power >= (double)TUNING_POWER_LIMIT)
    return -1;
  if (ref_power < (double)TUNING_POWER_LOWER_LIMIT)
    return 1;
  return 0;

}

void bypass_tuning_LC()
{
//  gpio_set_level(GPIO_RF_BYPASS, 0); // tuning mode
  set_LC(0, 0, 0);
}

int set_tuning_mode()
{
  printf("Setting relays...\n");
  gpio_set_level(GPIO_PTT_SWITCH, 1); // disconnect PTT Switch
  gpio_set_level(GPIO_RF_BYPASS, 1); // tuning mode
  set_LC(0, 0, 0);

  printf("Wait for relay settling...\n");
  vTaskDelay(pdMS_TO_TICKS(200));  
  if (check_tuning_power() < 0) {
    gpio_set_level(GPIO_PTT_SWITCH, 0); // disconnect PTT Switch
    gpio_set_level(GPIO_RF_BYPASS, 0); // tuning mode
    return -1;
  }
  else if (check_tuning_power() > 0) {  // tuning is not required
    gpio_set_level(GPIO_PTT_SWITCH, 0); // disconnect PTT Switch
    gpio_set_level(GPIO_RF_BYPASS, 0); // tuning mode
    return 1;
  }
  return 0;
}
int close_tuning_mode()
{
  gpio_set_level(GPIO_PTT_SWITCH, 0); // disconnect PTT Switch
  gpio_set_level(GPIO_RF_BYPASS, 0); // tuning mode
  return 0;
}

//*************************************************************************** 
// Function to search lowest ref power
//*************************************************************************** 
int search_lowest_SWR(int32_t* val_l, int32_t* val_c, int32_t* val_p, double* val_swr)
{

  double lowest_swr = 1e6;
  double swr = 1e6;
  double swr_A = 1e6;
  double swr_B = 1e6;

  double lowest_swr_A = 1e6;
  double lowest_swr_B = 1e6;

  int cposition = 0;

  int debug_display = 1;
  //---------------------------------------------------------------------
  // first step, coarse, find lowest setting, and try capacitor on both radio side and antenna side
  // cap position is searched only at the first step
  //---------------------------------------------------------------------
  for (i_ind = 0; i_ind < RANGE_L; i_ind += STEP_COARSE_SEARCH)
  {
    for (i_cap = 0; i_cap < RANGE_L; i_cap += STEP_COARSE_SEARCH)
    {
      set_LC(cposition, i_ind, i_cap);
      swr_A = get_swr_adaptive_settling();
      if (debug_display == 1) {
        sprintf(pline, "Searching, L = %d, C = %d, SWRA = %f \n", i_ind, i_cap, swr_A);
        printf(pline);
      }

      if (swr_A < lowest_swr_A) {
        lowest_ind_A = i_ind;
        lowest_cap_A = i_cap;
        lowest_swr_A = swr_A;
      }
    }
  }

  cposition =1;
  for (i_ind = 0; i_ind < RANGE_L; i_ind += STEP_COARSE_SEARCH)
  {
    for (i_cap = 0; i_cap < RANGE_L; i_cap += STEP_COARSE_SEARCH)
    {
      set_LC(cposition, i_ind, i_cap);
      swr_B = get_swr_adaptive_settling();
      if (debug_display == 1) {
        sprintf(pline, "Searching, L = %d, C = %d, SWRB = %f \n", i_ind, i_cap, swr_B);
        printf(pline);
      }
      if (swr_B < lowest_swr_B) {
        lowest_ind_B = i_ind;
        lowest_cap_B = i_cap;
        lowest_swr_B = swr_B;
      }
    }
  }
  if (debug_display == 1) {
    sprintf(pline, "A Coarse search results, L = %d, C = %d, SWR = %f \n", lowest_ind_A, lowest_cap_A, lowest_swr_A);
    printf(pline);
    sprintf(pline, "B Coarse search results, L = %d, C = %d, SWR = %f \n", lowest_ind_B, lowest_cap_B, lowest_swr_B);
    printf(pline);
  }

  if (lowest_swr_A > lowest_swr_B)
  {
    cposition =1;
    lowest_ind = lowest_ind_B;
    lowest_cap = lowest_cap_B;
    lowest_swr = lowest_swr_B;
  }
  else 
  {
    cposition =0;
    lowest_ind = lowest_ind_A;
    lowest_cap = lowest_cap_A;
    lowest_swr = lowest_swr_A;

  }
  set_LC(cposition, lowest_ind, lowest_cap);
  swr = get_swr_adaptive_settling();

  if (debug_display == 1) {
    sprintf(pline, "Coarse search results, L = %d, C = %d, SWR = %f, current SWR = %f \n", lowest_ind, lowest_cap, lowest_swr, swr);
    printf(pline);
  }

  // interim search
  int fine_ind_start = lowest_ind - STEP_COARSE_SEARCH;
  if (fine_ind_start < 0) fine_ind_start = 0;

  int fine_ind_stop = lowest_ind + STEP_COARSE_SEARCH;
  if (fine_ind_stop > RANGE_L) fine_ind_stop = RANGE_L;

  int fine_cap_start = lowest_cap - STEP_COARSE_SEARCH;
  if (fine_cap_start < 0) fine_cap_start = 0;

  int fine_cap_stop = lowest_cap + STEP_COARSE_SEARCH;
  if (fine_cap_stop > RANGE_C) fine_cap_stop = RANGE_C;

  //---------------------------------------------------------------------
  // second step, coarse, 
  //---------------------------------------------------------------------

  for (i_ind = fine_ind_start; i_ind < fine_ind_stop; i_ind += STEP_MEDIUM_SEARCH)
  {
    for (i_cap = fine_cap_start; i_cap < fine_cap_stop; i_cap += STEP_MEDIUM_SEARCH)
    {
      set_LC(cposition, i_ind, i_cap);
      swr = get_swr_adaptive_settling();
      if (debug_display == 1) {
        sprintf(pline, "Searching, L = %d, C = %d, SWR = %f \n", i_ind, i_cap, swr);
        printf(pline);
      }
      if (swr < lowest_swr) {
        lowest_ind = i_ind;
        lowest_cap = i_cap;
        lowest_swr = swr;

      }
    }
  }
  set_LC(cposition, lowest_ind, lowest_cap);
  swr = get_swr_adaptive_settling();

  if (debug_display == 1) {
    sprintf(pline, "Coarse stage 2 search results, L = %d, C = %d, SWR = %f, current SWR = %f \n", lowest_ind, lowest_cap, lowest_swr, swr);
    printf(pline);
  }


  // fine search
  fine_ind_start = lowest_ind - STEP_MEDIUM_SEARCH;
  if (fine_ind_start < 0) fine_ind_start = 0;

  fine_ind_stop = lowest_ind + STEP_MEDIUM_SEARCH;
  if (fine_ind_stop > RANGE_L) fine_ind_stop = RANGE_L;

  fine_cap_start = lowest_cap - STEP_MEDIUM_SEARCH;
  if (fine_cap_start < 0) fine_cap_start = 0;

  fine_cap_stop = lowest_cap + STEP_MEDIUM_SEARCH;
  if (fine_cap_stop > RANGE_C) fine_cap_stop = RANGE_C;


  for (i_ind = fine_ind_start; i_ind < fine_ind_stop; i_ind += 1)
  {
    for (i_cap = fine_cap_start; i_cap < fine_cap_stop; i_cap += 1)
    {
      set_LC(cposition, i_ind, i_cap);
      swr = get_swr_adaptive_settling();
      if (debug_display == 1) {
          sprintf(pline, "Fine Searching, L = %d, C = %d, SWR = %f \n", i_ind, i_cap, swr);
          printf(pline);
      }
      if (swr < lowest_swr) {
        lowest_ind = i_ind;
        lowest_cap = i_cap;
        lowest_swr = swr;

      }
    }
  }

  set_LC(cposition, lowest_ind, lowest_cap);
  swr = get_swr_adaptive_settling();

  if (debug_display == 1) {
    sprintf(pline, "Fine search results, L = %d, C = %d, SWR = %f, current SWR = %f \n", lowest_ind, lowest_cap, lowest_swr, swr);
    printf(pline);
  }

  *val_l = lowest_ind;
  *val_c = lowest_cap;
  *val_p = cposition;
   
  return 0;
}