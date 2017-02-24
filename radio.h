#ifndef __RADIO_H__

#define __RADIO_H__

#include <stdint.h>

typedef struct
{
    uint8_t buf[128];
    int len;
    int snr;
    int rssi;
    int cr;
    unsigned int ms;    // milliseconds
} rx_info_t;

#define LORA_STATUS_CLEAR         0x10
#define LORA_STATUS_HDR_INVALID   0x08
#define LORA_STATUS_RX_RUNNING    0x04
#define LORA_STATUS_SIGNAL_SYNCED 0x02
#define LORA_STATUS_SIGNAL_DETECT 0x01

void lora_init();
void lora_cleanup();
int lora_config(int sf, int cr, int bw);

int lora_rx_single(rx_info_t *data, int timeout_symbols);
int lora_rx_continuous_start();
int lora_rx_continuous_get(rx_info_t *data);
int lora_rx_continuous_stop();
int lora_tx(uint8_t *data, uint8_t len);

int lora_get_current_rssi();
void lora_set_frequency(long freq);
long lora_get_frequency();

void dump_hex(void *data, int len);

#endif
