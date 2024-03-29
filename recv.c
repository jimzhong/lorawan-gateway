#include "radio.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define PIN_NSS     6
#define PIN_RST     3

volatile int stopping = 0;
long freq = 438000000L;

typedef struct
{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t fixtype;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t speed;
    int32_t heading;
} location_t;

void callback(rx_info_t data)
{
    printf("len=%d\n", data.len);
}

void location_decoder(rx_info_t data)
{
    location_t *loc;
    loc = (location_t *)data.buf;
    printf("RSSI = %d CR = %d LEN = %d\n", data.rssi, data.cr, data.len);
    printf("Latitude = %.7lf\n", (double)(loc->lat) * 1e-7);
    printf("Longitude = %.7lf\n", (double)(loc->lon) * 1e-7);
    printf("Height = %.2lf meters\n", (double)(loc->height) * 1e-3);
    printf("Speed = %.2f m/s\n", (double)(loc->speed) * 1e-3);
}

int main()
{
    rx_info_t data;

    if(lora_init(0, 500000, PIN_NSS, PIN_RST))
    {
        printf("SX1278 Not Present\n");
        return 1;
    }
    printf("Inited\n");
    lora_config(8, 46, 250, 1, 17, 10, 0x12);
    lora_set_frequency(freq);
    printf("Freq=%ld\n", lora_get_frequency());
    lora_rx_continuous(location_decoder, 0);

    return 0;
}
