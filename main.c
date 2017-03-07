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

#define BUF_LENGTH 1024

int volatile running = 1;

pthread_t lora_thread_info;

struct sockaddr_in server_addr = {};
int sockfd;


void exit_handler()
{
    running = 0;
}



int hostname_to_ip(char *hostname , char *ip)
{
	struct addrinfo hints, *servinfo, *p;
	struct sockaddr_in *h;
	int rv;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC; // use AF_INET6 to force IPv6
	hints.ai_socktype = SOCK_STREAM;

	if ( (rv = getaddrinfo( hostname , "http" , &hints , &servinfo)) != 0)
	{
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return 1;
	}

	// loop through all the results and connect to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next)
	{
		h = (struct sockaddr_in *) p->ai_addr;
		strcpy(ip , inet_ntoa( h->sin_addr ) );
	}

	freeaddrinfo(servinfo); // all done with this structure
	return 0;
}


void network_receive_task()
{
    char buf[BUF_LENGTH];
    int len;
    while(running)
    {
        len = recvfrom(sockfd, buf, BUF_LENGTH, BUF_LENGTH, NULL, NULL);
        printf("Received %d bytes", len);
    }
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
    char server_ip_string[INET_ADDRSTRLEN];

    if (argc < 3)
    {
        printf("Usage: %s hostname port\n", argv[0]);
        exit(-1);
    }
    sockfd = socket(AF_INET, SOCK_DGRAM|SOCK_NONBLOCK, 0);
    if (sockfd == -1)
    {
        perror("socket");
        exit(-1);
    }
    if (hostname_to_ip(argv[1], server_ip_string) != 0)
    {
        printf("Name resolution failed.\n");
        exit(-1);
    }
    // store this IP address in sa:
    if (inet_pton(AF_INET, server_ip_string, &(server_addr.sin_addr)) != 1)
    {
        printf("Cannot parse IP address.\n");
        exit(-1);
    }
    port = atoi(argv[2]);
    if (port == 0)
    {
        printf("Cannot parse port.\n");
        exit(-1);
    }
    printf("Server at %s:%d\n", server_ip_string, port);
    server_addr.sin_port = htons(port);

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
        perror("connect");
        exit(-1);
    }

    if(sendto(sockfd, "Hello", 5, 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
        perror("send");
        exit(-1);
    }

    // lora_init();
    // lora_config(11, 46, 125);
    //
    // for(;;)
    // {
    //
    // }
    //
    // pthread_join(&lora_rx_thread_info, NULL);
    // pthread_join(&lora_tx_thread_info, NULL);
    // lora_cleanup();
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
