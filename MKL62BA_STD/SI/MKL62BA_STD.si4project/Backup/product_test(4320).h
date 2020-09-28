#ifndef __PRODUCT__TEST__H
#define __PRODUCT__TEST__H

#include "stdint.h"

#define TEST_IO		15	
uint8_t get_product_flag(void);
void product_test_process(void);
void check_test_io(void);
void product_recv_lora_data(uint8_t *buf,uint8_t len);
uint8_t get_product_state(void);
#endif
