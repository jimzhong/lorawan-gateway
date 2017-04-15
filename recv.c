#include "radio.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

volatile int stopping = 0;
long freq = 434000000L;

/*
int main()
{
    int size;
    if (!lora_begin(freq))
    {
        printf("Error");
        return 1;
    }
    lora_setSignalBandwidth(250000);
    lora_setSyncWord(0x12);
    lora_setSpreadingFactor(7);
    lora_setPreambleLength(8);
    lora_setCodingRate4(5);

    for(;;)
    {
        size = lora_parsePacket();
        if (size > 0)
        {
            printf("Received packet.\n");
            while (lora_available())
            {
                printf("%c", lora_read());
            }
            printf("\nRSSI=%d\n", lora_packetRssi());
        }
        else
        {
            //printf("No packet.\n");
        }
    }
}
*/


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
    lora_set_frequency(freq);
    printf("Freq set\n");
    lora_set_txpower(15);
    printf("Freq=%ld\n", lora_get_frequency());

    while (1)
    {
        lora_rx_continuous(&data);
        fprintf(stderr, "SNR=%d, RSSI=%d, CR=%d, TM=%ld\n", data.snr, data.rssi, data.cr, data.second);
        dump_hex(data.buf, data.len);
    }

    return 0;
}
