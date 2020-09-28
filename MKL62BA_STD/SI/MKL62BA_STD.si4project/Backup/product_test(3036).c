#include "product_test.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "Platform_bleInit.h"
#include "adv_init.h"
#include "lora_app.h"
#include "gps.h"
#include "Driver_Lis3dh_Ex.h"
#include "in_flash_manage.h"
#include "system_time.h"
#include "battery_driver.h"
#include "key_led.h"
#include "user.h"
#include "radio.h"



#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            3000//0xFFFFFFFF //RX continuous

void send_lora_config(void)
{
    uint8_t bufTx[10] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x00};
    uint8_t cnt = 0;
    Radio.SetModem(MODEM_LORA);

    Radio.SetChannel(902300000);
    USER_RTT( "###### ===== Radio.SetTxConfig ==== ######\r\n\r\n");
    Radio.SetTxConfig(MODEM_LORA, 22, 0, 0, 10, 1,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    USER_RTT( "###### ===== Radio.Send Data size %d ==== ######\r\n\r\n", sizeof(bufTx));
    for(cnt = 0; cnt < sizeof(bufTx); cnt++)
        USER_RTT( "%02x ", bufTx[cnt]);
    USER_RTT( "\r\n", bufTx);
    /* Send bufTx once*/
    Radio.Send(bufTx, sizeof(bufTx));

//    nrf_delay_ms(2000);
}

void recv_lora_config(void)
{
    Radio.SetModem(MODEM_LORA);

    Radio.SetChannel(902300000);
    USER_RTT( "###### ===== Radio.SetRxConfig ==== ######\r\n\r\n");
    /* Rx continuous */
    Radio.SetRxConfig(MODEM_LORA, 0, 10,
                      1, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    /* Send bufTx once*/
    Radio.Rx(RX_TIMEOUT_VALUE);
    USER_RTT( "###### ===== Set Radio.Rx ==== ######\r\n\r\n");
}

static uint8_t product_test_flag = 0;
void check_test_io(void)
{
	static uint8_t state = 0;
	static uint32_t timer;
	switch(state)
	{
		case 0:
			nrf_gpio_cfg_input(TEST_IO,NRF_GPIO_PIN_PULLUP);
			state  = 1;
			break;
		case 1:
			if(nrf_gpio_pin_read(TEST_IO)==0)
			{
				state = 2;
				USER_RTT("test io is low\r\n");
				timer = get_sys_time();
			}
			break;
		case 2:
			if(nrf_gpio_pin_read(TEST_IO)==1)
			{
				USER_RTT("test io is high\r\n");
				state = 1;
			}
			else
			{
				if(timepassed(timer,ONE_TIME_SLICE))
				{
					USER_RTT("enter product  \r\n");
					product_test_flag = 1;
					state = 3;
				}
			}
			break;
		case 3:
			break;
	}
}
static uint8_t product_success_flag = 0;
uint8_t get_product_state(void)
{
	return product_success_flag;
}
AxesRaw_t test;
uint8_t get_product_flag(void)
{
	return product_test_flag;
}
static uint8_t lora_recv_data[40]; 
static uint8_t lora_recv_len = 0;
void product_recv_lora_data(uint8_t *buf,uint8_t len)
{
	lora_recv_len = len;
	memcpy(lora_recv_data,buf,lora_recv_len);
}
void product_test_process(void)
{
	static  uint8_t state = 0;
	static uint32_t timer;
	static uint8_t flag = 0;
	static uint8_t lora_resend_num = 0;
	switch(state)
	{
		case 0:
			set_led_state(LED_STATE_PRODUCT_PROCESS);
			GpsInit();
			USER_RTT("product gps start\r\n");
			state = 1;
			timer = get_sys_time();
			break;
		case 1:
			task_gps_parser();
			if(get_gps_state())
			{
				GpsStop();
				state = 2;
				I2C_LIS3DH_Init();
				USER_RTT("product lis3dh start\r\n");
				timer = get_sys_time();
			}
			else
			{
				if(timepassed(timer,5000/TIME_SLICE))
				{
					state = 6 ;
				}
			}
			break;
		case 2:

			LIS3DH_Get_AccValue(&test);
			if(test.AXIS_X!=0 || test.AXIS_Y!=0 ||test.AXIS_Z!=0)
			{
				state = 3;
				timer = get_sys_time();
				lora_init();
				motor_init();
				USER_RTT("product lora start\r\n");
			}
			else
			{
				if(timepassed(timer,5000/TIME_SLICE))
				{
					state = 6;
				}
			}
			break;
		case 3:
			if(flag == 0 )
			{	flag = 1;
				USER_RTT("product lora send data\r\n");
				send_lora_config();
				//motor_open();
				nrf_gpio_pin_set(MOTOR_PIN);
				stu_last_rev.len = 0;
				timer = get_sys_time(); 
				lora_resend_num ++; 
				
			}
			if(timepassed(timer,500/TIME_SLICE) && flag == 1)
			{
				recv_lora_config();
				//motor_close();
				nrf_gpio_pin_clear(MOTOR_PIN);
				timer = get_sys_time(); 
				state = 4;
				flag = 0;
			}
			break;
		case 4:
			if(lora_recv_len)
			{
				USER_RTT("product lora recv data:");
				for(uint8_t i=0 ; i<lora_recv_len ; i++)
				{
					USER_RTT(" %02x",lora_recv_data[i]);
				}
				USER_RTT("\r\n");
				state = 5;
				
			}
			else
			{
				if(timepassed(timer,3000/TIME_SLICE))
				{
					
					timer = get_sys_time(); 
					USER_RTT("lora_resend_num is %d\r\n",lora_resend_num);
					if(lora_resend_num<3)
						state = 3;
					else
						state = 6;
				}
				
			}
			
	
			break;
		case 5:
			Radio.Sleep();
			state = 7;
			product_success_flag = 1;
			USER_RTT("product  success\r\n");
			set_led_state(LED_STATE_PRODUCT_SUCCESS);
			break;
		case 6:
			Radio.Sleep();
			state = 7;
			USER_RTT("product  false\r\n");
			set_led_state(LED_STATE_PRODUCT_FALSE);
			break;
		case 7:
			break;
	}
}
