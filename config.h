#ifndef __CONFIG_H__
#define __CONFIG_H__

#define CONFIG_LORA_MAX_RX_LENGTH 64
#define CONFIG_LORA_IS_GATEWAY 1

#define CONFIG_LORA_PREAMBLE_LENGTH 8
#define CONFIG_LORA_SYNC_WORD 0x12

#define PIN_NSS     6
#define PIN_DIO0    2
#define PIN_RST     3
#define PIN_DIO2    5
#define PIN_DIO1    4

#define SPI_FREQ    500000
#define SPI_CHANNEL 0

#endif