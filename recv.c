#include "radio.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define PIN_NSS     6
#define PIN_RST     3

volatile int stopping = 0;
long freq = 438000000L;

void callback(rx_info_t data)
{
    printf("len=%d\n", data.len);
}

int main()
{
    rx_info_t data;

    if(!lora_init(0, 1000000, PIN_NSS, PIN_RST))
    {
        printf("SX1278 Not Present\n");
        return 1;
    }
    printf("Inited\n");
    lora_config(8, 45, 250, 17, 10, 0x12, 1);
    lora_set_frequency(freq);
    printf("Freq=%ld\n", lora_get_frequency());
    lora_rx_continuous(callback, 1);

    return 0;
}
