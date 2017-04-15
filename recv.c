#include "radio.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

volatile int stopping = 0;
long freq = 434000000L;


int main()
{
    rx_info_t data;
    int len;

    // signal(SIGINT, stop);
    if(!lora_init()){
        printf("Error\n");
        return 1;
    }
    printf("Inited\n");
    lora_config(7, 45, 250, 6, 0x12, 0);
    // lora_set_frequency(freq);
    // printf("Freq set\n");
    lora_set_txpower(15);
    printf("Freq=%ld\n", lora_get_frequency());

    for(;;);

    while (1)
    {
        lora_rx_continuous(&data);
        fprintf(stderr, "SNR=%d, RSSI=%d, CR=%d, TM=%ld\n", data.snr, data.rssi, data.cr, data.second);
        dump_hex(data.buf, data.len);
    }

    return 0;
}
