// common.c
#include "common.h"

int make_udp_socket(){
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s<0){ perror("socket"); exit(1); }
    int on = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    return s;
}

int send_msg(int sock, int port, msg_t *m){
    struct sockaddr_in to; memset(&to,0,sizeof(to));
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = inet_addr(HOST);
    to.sin_port = htons(port);
    char buf[MAX_MSG];
    // simple serialization: type|swarm|drone|text
    int n = snprintf(buf,sizeof(buf),"%d|%d|%d|%s", (int)m->type, m->swarm_id, m->drone_id, m->text);
    int res = sendto(sock, buf, n, 0, (struct sockaddr*)&to, sizeof(to));
    if(res<0){ /*perror("sendto");*/ }
    return res;
}

int recv_msg(int sock, msg_t *m, struct sockaddr_in *from){
    char buf[MAX_MSG];
    socklen_t fromlen = sizeof(struct sockaddr_in);
    int r = recvfrom(sock, buf, sizeof(buf)-1, 0, (struct sockaddr*)from, &fromlen);
    if(r<=0) return r;
    buf[r]=0;
    // parse
    int t, s, d;
    char txt[200];
    if(sscanf(buf,"%d|%d|%d|%199[^\n]", &t, &s, &d, txt) >= 3){
        m->type = (msg_type_t)t;
        m->swarm_id = s;
        m->drone_id = d;
        if(strlen(txt)) strncpy(m->text, txt, sizeof(m->text)-1);
    }
    return r;
}

int port_for_center(int base){ return base + 1; }
int port_for_truck(int base, int truck_id){ return base + 100 + truck_id; }
int port_for_drone(int base, int drone_global_id){ return base + 1000 + drone_global_id; }
int port_for_artillery(int base){ return base + 2; }

