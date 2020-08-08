
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <stddef.h>
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_serial.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_util.h"

int bot_id = 2;
static nrf_esb_payload_t        rx_payload;

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER


uint8_t arr[32];
void nrf_store_data(uint8_t data[],uint8_t arr[])
{
	
}
static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}

void uart_transmit(uint8_t arr[],int l);
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                // Set LEDs identical to the ones on the PTX.
                //int l=bot_id*5+1;
                nrf_gpio_pin_write(LED_1, !(rx_payload.data[1]%8>0 && rx_payload.data[1]%8<=4));
                nrf_gpio_pin_write(LED_2, !(rx_payload.data[1]%8>1 && rx_payload.data[1]%8<=5));
                //nrf_gpio_pin_write(LED_4, !(rx_payload.data[1]%8>3));
               // nrf_store_data(rx_payload.data,arr);
                //uart_transmit(arr,0);
                if(rx_payload.length==32)
             {    //nrf_delay_us(500000);
                for(int i=0;i<32;i++)
                {
                arr[i]=rx_payload.data[i];
                }
                
                uart_transmit(arr,bot_id);

            }
                NRF_LOG_DEBUG("Receiving packet: %02x", rx_payload.data[1]);
            }
            break;
    }
}
uint32_t esb_init(void)
{
	uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.selective_auto_ack       = false;
    nrf_esb_config.payload_length			= 32;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(err_code);

    return err_code;

}
void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    nrf_gpio_range_cfg_output(8, 15);
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_init(BSP_INIT_BUTTONS);
}
////////////////      UART          /////////
NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      RX_PIN_NUMBER, TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_9600,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 8
#define SERIAL_FIFO_RX_SIZE 8

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, NULL, sleep_handler);


NRF_SERIAL_UART_DEF(serial_uart, 0);
///////////////////////////////////////////
void uart_initialize(void)
{
	ret_code_t ret;
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);
  ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    // Initialize LEDs and buttons.
    //bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);

}

void uart_transmit(uint8_t arr[],int l)
{
	ret_code_t ret;
	uint8_t a[6];
	a[0]=255;
	for(int i=1;i<6;i++)
	{
		a[i]=arr[l+i-1];
	}
	ret=nrf_serial_write(&serial_uart,a, sizeof(a), NULL, 0);
       // bsp_board_led_on(1);
        APP_ERROR_CHECK(ret);
        
}

int main()
{
ret_code_t err_code;

    gpio_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

     err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);

    uart_initialize();
    while(true)
    {
   

    }


}