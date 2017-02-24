#include "radio.h"
#include "wiringPi.h"
#include <stdio.h>
#include <stdlib.h>

uint8_t packet[] = {0x5a, 0x0e, 0xfa};

volatile int running = 1;

void stop()
{
    running = 0;
    lora_rx_continuous_stop();
    lora_cleanup();
    exit(0);
}

int main()
{
    lora_init();
    lora_config(11, 46, 125);
    lora_set_txpower(15);
    while (running)
    {
        lora_tx(packet, 3);
        delay(5000);
    }
    lora_cleanup();
    return 0;
}
