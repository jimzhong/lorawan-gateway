#include "radio.h"
#include <stdio.h>
#include <signal.h>

volatile int stopping = 0;

void stop()
{
    stopping = 1;
}


int main()
{
    rx_info_t data;
    int len;

    signal(SIGINT, stop);

    lora_init();
    lora_config(11, 46, 125);

    lora_rx_continuous_start(100);

    while (!stopping)
    {
        lora_rx_continuous_get(&data);
        fprintf(stderr, "SNR=%d, RSSI=%d, CR=%d, TM=%ld\n", data.snr, data.rssi, data.cr, data.ms);
        dump_hex(data.buf, data.len);
    }

    lora_rx_continuous_stop();

    lora_cleanup();
    return 0;
}
