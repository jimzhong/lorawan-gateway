#include "radio.h"
#include <stdio.h>

int main()
{
    lora_init();
    lora_config(8, 45, 125, 8, 0x34);

    while (1)
    {
        lora_tx("Hello world", 11);
        delay(200);
    }
    lora_cleanup();
    return 0;
}
