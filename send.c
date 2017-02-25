#include "radio.h"
#include "wiringPi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t packet[] = {0xde, 0xad, 0xbe, 0xef};
uint8_t buf[10];

volatile int running = 1;
long freq = 436000000;

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
    lora_config(8, 46, 250);
    lora_set_frequency(freq);
    printf("Freq=%ld\n", lora_get_frequency());
    lora_set_txpower(10);
    while (running)
    {
        memcpy(buf, packet, 4)
        lora_tx(buf, 4);
        delay(1000);
    }
    lora_cleanup();
    return 0;
}
