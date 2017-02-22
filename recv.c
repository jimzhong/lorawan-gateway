#include "radio.h"
#include <stdio.h>

int main()
{
    uint8_t rxbuf[128];
    int len;

    lora_init();
    lora_config(8, 45, 125, 8, 0x34);

    len = lora_rx_single(rxbuf, 1000);
    printf("%d\n", len);
    if (len > 0)
    {
        dump_hex(rxbuf, len);
    }

    lora_cleanup();
    return 0;
}
