#ifndef __NETWORK_H__
#define __NETWORK_H__

void send_to_server(int sockfd, void *buf, int len);
int recv_from_server(int sockfd, void *buf, int maxlen);
int connect_to_server(char *hostname, int port);

#endif
