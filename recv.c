#include "radio.h"
#include <stdio.h>
#include <signal.h>

volatile int stopping = 0;
long freq = 436000000;

void stop()
{
    lora_rx_continuous_stop();
    lora_cleanup();
    exit(0);
}


int main()
{
    rx_info_t data;
    int len;

    signal(SIGINT, stop);

    lora_init();
    lora_config(11, 46, 125);
    lora_set_frequency(freq);
    lora_set_txpower(10);
    printf("Freq=%ld\n", lora_get_frequency());
    lora_rx_continuous_start();

    while (!stopping)
    {
        lora_rx_continuous_get(&data);
        fprintf(stderr, "SNR=%d, RSSI=%d, CR=%d, TM=%ld\n", data.snr, data.rssi, data.cr, data.ms);
        dump_hex(data.buf, data.len);
    }

    return 0;
}
