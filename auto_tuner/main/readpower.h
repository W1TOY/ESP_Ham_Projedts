#ifndef READPOWER_H
#define READPOWER_H

#include <stdint.h>

#define NUM_ADC_READ 16                 // number of ADC read for averaging
#define ADC_CHANNEL_REF ADC_CHANNEL_0   // this becomes GPIO36 in ESP32




//#define GPIO_ONBOARD_LED 2          // on board blue LED for heart beat

#define GPIO_RF_BYPASS  38           // GPIO to control relay for switching between tuning and operation
#define GPIO_PTT_SWITCH 37          // Relay to control PTT loop through - to disconnect PTT while tuning

#define GPIO_OUT_EN_N 4            // these four signals are for controlling the Shift Register TPIC6B595
#define GPIO_SRCK     5
#define GPIO_RCK      6
#define GPIO_SER_IN   7

#define GPIO_BUTTON_0 17            // four buttons on the front panel
#define GPIO_BUTTON_1 18
#define GPIO_BUTTON_2 8
#define GPIO_BUTTON_3 3

#define GPIO_BUTTON_LIGHT_0 9      // LED on each button
#define GPIO_BUTTON_LIGHT_1 10
#define GPIO_BUTTON_LIGHT_2 11
#define GPIO_BUTTON_LIGHT_3 12

#define NUM_L 7     // number of L relay
#define NUM_C 7     // number of C relay
#define RANGE_L 128 // 2^7
#define RANGE_C 128
#define STEP_COARSE_SEARCH 16   // first step out of three step, L/C values are incremeneted by this
#define STEP_MEDIUM_SEARCH 4    // second step out of three step, L/C values are incremeneted by this
#define RELAY_SETTLING_TIME 10  // Waiting time before reading ref power after relay switching
#define RELAY_SETTLING_TIME_MIN 5  // Waiting time before reading ref power after relay switching

#define TUNING_POWER_LIMIT 4000         // tuning won't start if power is higher than this
#define TUNING_POWER_LOWER_LIMIT 1     // or if lower than this

void setup_gpio_output(int pinnum);
double read_fwd();
double read_ref();
double get_swr();
void test_read_just_ref_method();
void test_swr_bypass_method();
void init_adc_tunerboard();
int search_lowest_SWR(int32_t* val_l, int32_t* val_c, int32_t* val_p, double* val_swr);
void send_value_to_shift_registers(uint32_t valueRelay);
void clear_all_LC_relays();
void set_LC_relays(uint32_t vRelay);
void set_LC(uint32_t cposition, uint32_t valueL, uint32_t valueC);
void test_relay_find();
void enable_shift_register_output();
void clear_shift_registers();
void disable_shift_register_output();
void test_relay_scan();
void vibrate_relay_lc(uint32_t cposition, uint32_t valueL, uint32_t valueC);
void vibrate_relay(uint32_t valueRelay);
void test_relay_lc_scan();
void test_relay_scan();
int search_lowest_SWR_A();
void check_signal_level();
int check_tuning_power();
void bypass_tuning_LC();
void setup_gpio_output(int pinnum);
int set_tuning_mode();
int close_tuning_mode();
#endif