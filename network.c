#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "network.h"

void send_to_server(int sockfd, void *buf, int len)
{
    int ret;
    ret = send(sockfd, buf, len, 0);
    // ret = sendto(sockfd, buf, len, 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
    if (ret == -1)
    {
        perror("send");
    }
    else
    {
        printf("Sent %d bytes to server.\n", ret);
    }
}

int recv_from_server(int sockfd, void *buf, int maxlen)
{
    int len;
    len = recv(sockfd, buf, maxlen, 0);
    if (len == -1)
    {
        perror("recv");
    }
    else
    {
        printf("Received %d bytes from server.\n", len);
    }
    return len;
}

static int hostname_to_ip(char *hostname , char *ip)
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


int connect_to_server(char *hostname, int port)
{
    int sockfd;
    struct sockaddr_in server_addr = {};
    char server_ip_string[INET_ADDRSTRLEN];

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        perror("socket");
        exit(-1);
    }
    if (hostname_to_ip(hostname, server_ip_string) != 0)
    {
        exit(-1);
    }
    // store this IP address in sa:
    if (inet_pton(AF_INET, server_ip_string, &(server_addr.sin_addr)) != 1)
    {
        printf("Cannot parse IP address.\n");
        exit(-1);
    }

    // printf("Server at %s:%d\n", server_ip_string, port);
    server_addr.sin_port = htons(port);
    server_addr.sin_family = AF_INET;

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
        perror("connect");
        exit(-1);
    }
    return sockfd;
}
