#ifndef __RADIO_H__

#define __RADIO_H__

#include <stdint.h>
#include <time.h>

typedef struct
{
    uint8_t sf;
    uint8_t cr;
    uint16_t bw;
    uint32_t freq;
    int16_t snr;
    int16_t rssi;
    uint64_t second;
    uint32_t nanosecond;
    uint8_t len;
    uint8_t buf[256];
} rx_info_t;


#define LORA_STATUS_CLEAR         0x10
#define LORA_STATUS_HDR_INVALID   0x08
#define LORA_STATUS_RX_RUNNING    0x04
#define LORA_STATUS_SIGNAL_SYNCED 0x02
#define LORA_STATUS_SIGNAL_DETECT 0x01

int lora_init();
void lora_cleanup();
void lora_set_standby();
void lora_set_sleep();

int lora_config(int sf, int cr, int bw);

int lora_rx_single(rx_info_t *data, int timeout_symbols);
int lora_rx_continuous(rx_info_t *data);
int lora_rx_continuous_stop();
int lora_tx(uint8_t *data, uint8_t len);

int lora_get_current_rssi();
int lora_set_frequency(long freq);
long lora_get_frequency();

void dump_hex(void *data, int len);

#endif
