#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "heap.h"
#include "radio.h"
#include "network.h"

#define BUF_LENGTH 1024

int volatile running = 1;

int sockfd;


void exit_handler()
{
    running = 0;
}

void lora_rx_task()
{
    while(running)
    {

    }
}

int main(int argc, char **argv)
{
    int port;
    char buf[BUF_LENGTH];

    if (argc < 3)
    {
        printf("Usage: %s hostname port\n", argv[0]);
        exit(-1);
    }
    port = atoi(argv[2]);
    if (port == 0)
    {
        printf("Cannot parse port.\n");
        exit(-1);
    }
    sockfd = connect_to_server(argv[1], port);
    send_to_server(sockfd, "123", 3);
    recv_from_server(sockfd, buf, BUF_LENGTH);

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
