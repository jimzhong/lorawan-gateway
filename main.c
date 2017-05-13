#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <assert.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include <wiringPi.h>

#include "radio.h"
#include "network.h"

#define PIN_NSS     6
#define PIN_RST     3

#define SYNCWORD    0x56
#define PRELEN      10

int sockfd;

void rx_callback(rx_info_t data)
{
    struct tm t;
    time_t timestamp;
    timestamp = time(NULL);
    localtime_r(&timestamp, &t);
    printf("[%02d:%02d:%02d] LEN = %d SNR = %d RSSI = %d CR = %d\n", t.tm_hour, \
    t.tm_min, t.tm_sec, data.len, data.snr, data.rssi, data.cr);
    send_to_server(sockfd, &data, sizeof(rx_info_t));
}

int main(int argc, char **argv)
{
    int port;
    int cr, sf, bw;
    long freq;

    if (argc < 7)
    {
        printf("Usage: %s hostname port frequency spreading_factor coding_rate bandwidth\n", argv[0]);
        exit(-1);
    }

    port = atoi(argv[2]);
    freq = atol(argv[3]);
    sf = atoi(argv[4]);
    cr = atoi(argv[5]);
    bw = atoi(argv[6]);

    wiringPiSetup();

    if (port == 0)
    {
        printf("Cannot parse port.\n");
        exit(-1);
    }
    // initialize socket
    sockfd = connect_to_server(argv[1], port);

    if(lora_init(0, 500000, PIN_NSS, PIN_RST))
    {
        printf("SX1278 Not Present\n");
        return 1;
    }
    lora_config(sf, cr, bw, 1, 17, PRELEN, SYNCWORD);
    lora_set_frequency(freq);
    lora_rx_continuous(rx_callback, 0);

    return 0;
}
