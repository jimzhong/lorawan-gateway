#include "radio.h"
#include "wiringPi.h"
#include <stdio.h>

int main()
{
    lora_init();
    lora_config(11, 45, 125);
    lora_set_txpower(15);
    while (1)
    {
        lora_tx("Hello world", 11);
        delay(200);
    }
    lora_cleanup();
    return 0;
}
