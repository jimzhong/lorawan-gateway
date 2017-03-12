#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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

#include "heap.h"
#include "radio.h"
#include "network.h"

#define MAX_EVENTS 32
#define DEFAULT_CLOCK CLOCK_REALTIME
#define RXTX_LOCK_NUMBER 2

// BUF_LENGTH must >= sizeof(tx_request_t)
#define BUF_LENGTH 1024
#define TX_QUEUE_LENGTH 20
#define TX_QUEUE_MAX_DELAY_SEC 5    // how many seconds of delay allowed for a tx request

typedef struct
{
    int sf;
    int bw;
    int cr;
    int txpower;
    unsigned long txfreq;
    struct timespec tp;
    int len;
    uint8_t buf[256];
} tx_request_t;

int volatile running = 1;
int sockfd;
int timerfd[TX_QUEUE_LENGTH];
tx_request_t *tx_queue[TX_QUEUE_LENGTH] = {};    // NULL slots are usable

int min(int a, int b)
{
    if (a < b)
        return a;
    return b;
}

void exit_handler()
{
    running = 0;
    lora_cleanup();
}

PI_THREAD (lora_rx_task)
{
    int len;
    rx_info_t data;
    while(running)
    {
        // piLock(RXTX_LOCK_NUMBER);
        len = lora_rx_continuous(&data);
        // piUnlock(RXTX_LOCK_NUMBER);
        if (len > 0)
        {
            send_to_server(sockfd, &data, sizeof(rx_info_t));
        }
        else
        {
            usleep(1000);   // if len < 0, rx is canncelled, let tx first
        }
    }
    return 0;
}

// set a timer to expire at tp absolute time
void timer_set_expire_at(int fd, struct timespec tp)
{
    struct itimerspec new_value;
    new_value.it_value = tp;
    new_value.it_interval.tv_sec = 0;
    new_value.it_interval.tv_nsec = 0;

    if (timerfd_settime(fd, TFD_TIMER_ABSTIME, &new_value, NULL) == -1)
    {
        perror("timerfd_settime");
        exit(-1);
    }
}

void timer_cancel(int fd)
{
    struct timespec tp;
    tp.tv_sec = 0;
    tp.tv_nsec = 0;
    timer_set_expire_at(fd, tp);
}

void epoll_register_readable(int fd)
{
    struct epoll_events ev;
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) == -1)
    {
        perror("epoll_ctl");
        exit(-1);
    }
}

void queue_tx_request(tx_info_t *req)
{
    int i;
    fprintf(stderr, "Queuing a tx request of %d bytes\n", data->len);
    for (i = 0; i < TX_QUEUE_LENGTH; i++)
    {
        if (tx_queue[i] == NULL)    // find the first empty slot
        {
            // copy data into new slot
            tx_queue[i] = malloc(sizeof(tx_request_t));
            assert(tx_queue[i] != NULL);
            memmove(tx_queue[i], req, sizeof(tx_request_t));
            // start the corresponding timer
            timer_set_expire_at(timerfd[i], req->tp);
            fprintf(stderr, "Queued a TX request of %d bytes.\n", data->len);
            return;
        }
    }
    // should never reach here
    fprintf(stderr, "TX queue is full!!!\n");
}

void handle_timer_expiration(int fd)
{
    int i;
    long old_freq;
    tx_request_t *data;

    fprintf(stderr, "Handling expiration of fd %d\n", fd);
    for (i = 0; i < TX_QUEUE_LENGTH; i++)
    {
        if (timerfd[i] == fd)
        {
            data = tx_queue[i];
            tx_queue[i] = NULL;
            timer_cancel(fd);
            assert(data != NULL);
            lora_rx_continuous_stop();
            // if (data->txpower != 0)
            // {
            //     lora_set_txpower(data->txpower);
            // }
            // if (data->txfreq != 0)
            // {
            //     old_freq = lora_get_frequency();
            //     lora_set_frequency(data->txfreq);
            // }
            // lora_config(tx_queue[i]->sf, tx_queue[i]->cr, tx_queue[i]->bw);
            lora_tx(data->buf, data->len);
            fprintf(stderr, "Sent a packet of %d bytes\n", data->len);
            // if (data->txfreq != 0)
            // {
            //     lora_set_frequency(old_freq)
            // }
            free(data);
            return;
        }
    }
    // should never reach here
    fprintf(stderr, "timerfd not found.\n");
}


int main(int argc, char **argv)
{
    int i;
    int port;
    int cr, sf, bw;
    long freq;
    // epoll related argument
    int nfds;
    int epfd;
    struct epoll_events ev;
    struct epoll_events events[MAX_EVENTS];
    // receive buffer and length
    char buf[BUF_LENGTH];
    int len;
    tx_request_t data;

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
    if (lora_init() == -1)
    {
        printf("SX1278 not found\n");
        exit(-1);
    }
    if (lora_config(sf, cr, bw) == -1)
    {
        printf("Wrong sf/cr/bw.\n");
        lora_cleanup();
        exit(-1);
    }
    if (lora_set_frequency(freq) == -1)
    {
        printf("Wrong frequency.\n");
        lora_cleanup();
        exit(-1);
    }

    // initialize socket
    sockfd = connect_to_server(argv[1], port);
    // initialize timers for each slot of the tx queue
    for (i = 0; i < TX_QUEUE_LENGTH; i++)
    {
        timerfd[i] = timerfd_create(DEFAULT_CLOCK, 0);
    }

    epfd = epoll_create1(0);
    if (epfd == -1)
    {
        perror("epoll_create1");
        exit(-1);
    }
    // register sockfd to epoll
    epoll_register_readable(sockfd);
    // register timerfds
    for (i = 0; i < TX_QUEUE_LENGTH; i++)
    {
        epoll_register_readable(timerfd[i]);
    }

    if (piThreadCreate(lora_rx_task) != 0)
    {
        printf("RX thread creation failed.\n");
        lora_cleanup();
        exit(-1);
    }

    while(running)
    {
        nfds = epoll_wait(epfd, events, MAX_EVENTS, 200);
        if (nfds == -1)
        {
            perror("epoll_wait");
            exit(-1);
        }
        fprintf(stderr, "%d fds are ready\n", nfds);
        for (i = 0; i < nfds; i++)
        {
            if (events[i].data.fd == sockfd)
            {
                len = recv_from_server(sockfd, buf, BUF_LENGTH);
                if (len > 0)
                {
                    queue_tx_request(buf);
                }
            }
            else
            {
                // some timer has expired
                handle_timer_expiration(events[i].data.fd);
            }
        }
    }

    close(epfd);
    close(sockfd);
    return 0;
}



static void daemonize(void)
{
  int maxfd;
  int i;

  /* fork #1: exit parent process and continue in the background */
  if ((i = fork()) < 0) {
    perror("couldn't fork");
    exit(2);
  } else if (i > 0) {
    _exit(0);
  }

  /* fork #2: detach from terminal and fork again so we can never regain
   * access to the terminal */
  setsid();
  if ((i = fork()) < 0) {
    perror("couldn't fork #2");
    exit(2);
  } else if (i > 0) {
    _exit(0);
  }

  /* change to root directory and close file descriptors */
  chdir("/");
  maxfd = getdtablesize();
  for (i = 0; i < maxfd; i++) {
    close(i);
  }

  /* use /dev/null for stdin, stdout and stderr */
  open("/dev/null", O_RDONLY);
  open("/dev/null", O_WRONLY);
  open("/dev/null", O_WRONLY);
}
