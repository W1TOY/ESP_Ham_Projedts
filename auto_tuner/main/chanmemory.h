#ifndef CHANMEMORY_H
#define CHANMEMORY_H

#define NUM_ANT_CHANNELS 4
#define NUM_CHAN_MEMS    1000
#define NUM_CHAN_MEM_DIGIT 7
#define FREQ_MATCH_RANGE 20000

#define FREQ_STEP_MASK 0xFFFFC000       // 16 KHz mask

void erase_init_chan_mem();
void init_chan_mem();
int read_chan_mem(int32_t freq, int32_t selected_ant, int32_t* mem_l, int32_t* mem_c, int32_t* mem_p) ;
int save_chan_mem(int32_t freq, int32_t selected_ant, int32_t mem_l, int32_t mem_c, int32_t mem_p) ;
void test_chan_memmory();
#endif
