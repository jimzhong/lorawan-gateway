#ifndef __RADIO_H__

#define __RADIO_H__

#include <stdint.h>
#include <time.h>

typedef enum
{
    RADIO_IDLE = 0,
    RADIO_RX_RUNNING,
    RADIO_TX_RUNNING,
    RADIO_CAD
} radio_state_t;

typedef struct
{
    uint8_t sf;
    uint8_t cr;
    uint16_t bw;
    uint32_t freq;
    int16_t snr;
    int16_t rssi;
    uint32_t second;
    uint32_t nanosecond;
    uint8_t len;
    uint8_t buf[256];
} rx_info_t;


int lora_init(int spi_ch, int spi_freq, int nss, int rst);
void lora_cleanup();

int lora_set_txpower(int txpower);
int lora_set_preamble_len(int prelen);
int lora_set_sync_word(uint8_t sw);
int lora_set_sf_cr_bw_crc(int sf, int cr, int bw, int crcon);
int lora_config(int sf, int cr, int bw, int crcon, int txpower, int prelen, int syncword);

int lora_rx_single(rx_info_t *data, int timeout_symbols, int invert_iq);
int lora_rx_continuous(void (*callback)(rx_info_t data), int invert_iq);
int lora_rx_continuous_stop();
int lora_tx(uint8_t *data, uint8_t len, int invert_iq);

int lora_get_current_rssi();
int lora_set_frequency(long freq);
long lora_get_frequency();

void dump_hex(void *data, int len);

#endif
