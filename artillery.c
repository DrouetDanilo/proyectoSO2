// artillery.c
#include "common.h"
#include <math.h>

int BASE_PORT = 40000;
int W = 30;           // probabilidad de dar al dron
int DEFENSE_RADIUS = 50; // radio de defensa

// calcula distancia euclidiana entre dron y base (0,0 por simplicidad)
double distance_from_base(int x, int y) {
    return sqrt(x*x + y*y);
}

int main(int argc, char **argv){
    if(argc<2){ fprintf(stderr,"Usage: artillery params.txt\n"); exit(1); }
    char *params = argv[1];
    FILE *f = fopen(params,"r");
    if(f){
        char line[200];
        while(fgets(line,sizeof(line),f)){
            if(line[0]=='#') continue;
            char key[80]; int val;
            if(sscanf(line,"%[^=]=%d",key,&val)==2){
                if(strcmp(key,"W")==0) W=val;
                if(strcmp(key,"BASE_PORT")==0) BASE_PORT=val;
                if(strcmp(key,"DEFENSE_RADIUS")==0) DEFENSE_RADIUS=val;
            }
        }
        fclose(f);
    }

    srand(time(NULL));

    int sock = make_udp_socket();
    int aport = port_for_artillery(BASE_PORT);
    struct sockaddr_in addr; memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(HOST);
    addr.sin_port = htons(aport);
    if(bind(sock,(struct sockaddr*)&addr,sizeof(addr))<0){
        perror("bind artillery");
        exit(1);
    }
    printf("[ARTILLERY] listening port %d W=%d%% R=%d\n", aport, W, DEFENSE_RADIUS);

    msg_t m; struct sockaddr_in from;
    while(1){
        if(recv_msg(sock,&m,&from)<=0){ usleep(100000); continue; }

        // Solo nos interesan mensajes de STATUS con coordenadas
        if(m.type==MSG_STATUS && strncmp(m.text,"POS",3)==0){
            int x,y;
            if(sscanf(m.text,"POS %d %d",&x,&y)==2){
                double d = distance_from_base(x,y);
                if(d <= DEFENSE_RADIUS){
                    printf("[ARTILLERY] Drone %d (swarm %d) in defense zone (%.1f)\n",
                           m.drone_id, m.swarm_id, d);

                    // Intentos ilimitados mientras esté dentro
                    if((rand()%100) < W){
                        // HIT
                        msg_t hit; memset(&hit,0,sizeof(hit));
                        hit.type = MSG_ARTILLERY;
                        hit.drone_id = m.drone_id;
                        snprintf(hit.text,sizeof(hit.text),"HIT_BY_ARTILLERY");
                        int dport = port_for_drone(BASE_PORT, m.drone_id);
                        send_msg(sock, dport, &hit);

                        // inform center
                        msg_t inf; memset(&inf,0,sizeof(inf));
                        inf.type = MSG_ARTILLERY;
                        snprintf(inf.text,sizeof(inf.text),"DRONE %d SHOT_DOWN", m.drone_id);
                        send_msg(sock, port_for_center(BASE_PORT), &inf);

                        printf("[ARTILLERY] Drone %d destroyed!\n", m.drone_id);
                    } else {
                        // MISS
                        msg_t inf; memset(&inf,0,sizeof(inf));
                        inf.type = MSG_ARTILLERY;
                        snprintf(inf.text,sizeof(inf.text),"DRONE %d SURVIVED_DEFENSE", m.drone_id);
                        send_msg(sock, port_for_center(BASE_PORT), &inf);

                        printf("[ARTILLERY] Shot missed drone %d\n", m.drone_id);
                    }
                }
            }
        }
    }
    return 0;
}


