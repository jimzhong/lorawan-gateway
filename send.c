#include "radio.h"
#include <stdio.h>

int main()
{
    lora_init();
    lora_config(8, 45, 125, 8, 0x34);

    lora_tx("Hello world", 11);
    lora_cleanup();
    return 0;
}
