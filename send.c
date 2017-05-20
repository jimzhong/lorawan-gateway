#include "radio.h"
#include "wiringPi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t buf[40];

long freq = 438000000;

#define PIN_NSS     6
#define PIN_RST     3

#define SYNCWORD    0x56
#define PRELEN      10

int main(int argc, char ** argv)
{
    int i;
    int sf;

    sf = atoi(argv[1]);
    lora_init(0, 500000, PIN_NSS, PIN_RST);

    lora_config(sf, 46, 250, 1, 17, PRELEN, SYNCWORD);
    lora_set_frequency(freq);
    printf("Freq = %ld\n", lora_get_frequency());

    for (i = 1; i <= 20; i++)
    {
        memset(buf, i, sizeof(buf));
        lora_tx(buf, 40, 1);
        delay(1000);
        printf("Packet %d\n", i);
    }
    lora_cleanup();
    return 0;
}
