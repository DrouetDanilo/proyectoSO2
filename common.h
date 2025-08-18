// common.h
#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <time.h>
#include <errno.h>

#define MAX_MSG 256
#define HOST "127.0.0.1"

typedef enum {
    MSG_HELLO,
    MSG_COMMAND,      // commands from CC to drone
    MSG_STATUS,       // status from drone to CC
    MSG_ARTILLERY,    // from artillery to CC or drone
} msg_type_t;

typedef struct {
    msg_type_t type;
    int swarm_id;
    int truck_id;
    int drone_id;
    char text[200];
} msg_t;

int make_udp_socket();
int send_msg(int sock, int port, msg_t *m);
int recv_msg(int sock, msg_t *m, struct sockaddr_in *from);

int port_for_center(int base);
int port_for_truck(int base, int truck_id);
int port_for_drone(int base, int drone_global_id);
int port_for_artillery(int base);

#endif

