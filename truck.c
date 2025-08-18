// truck.c
#include "common.h"

// truck <params_path> <truck_id>
int BASE_PORT = 40000;
int ASSEMBLY_SIZE = 5;
char *params_path;

int main(int argc, char **argv){
    if(argc<3){ fprintf(stderr,"Usage: truck params.txt <truck_id>\n"); exit(1); }
    params_path = argv[1];
    int truck_id = atoi(argv[2]);

    // read base port from params quickly (minimal parsing)
    FILE *f = fopen(params_path,"r");
    if(f){
        char line[200];
        while(fgets(line,sizeof(line),f)){
            if(line[0]=='#') continue;
            char key[80]; int val;
            if(sscanf(line,"%[^=]=%d",key,&val)==2){
                if(strcmp(key,"BASE_PORT")==0) BASE_PORT=val;
                if(strcmp(key,"ASSEMBLY_SIZE")==0) ASSEMBLY_SIZE=val;
            }
        }
        fclose(f);
    }

    int truck_port = port_for_truck(BASE_PORT, truck_id);
    int center_port = port_for_center(BASE_PORT);
    int sock = make_udp_socket();

    // announce truck ready
    msg_t m; memset(&m,0,sizeof(m));
    m.type = MSG_ARTILLERY;
    m.truck_id = truck_id;
    snprintf(m.text,sizeof(m.text),"TRUCK_READY %d", truck_id);
    send_msg(sock, center_port, &m);

    // spawn ASSEMBLY_SIZE drones
    for(int i=0;i<ASSEMBLY_SIZE;i++){
        pid_t pid = fork();
        if(pid==0){
            char gid_s[16], ppath[256], tid[16];
            int global_id = truck_id * 100 + i + 1; // global unique (simple)
            snprintf(gid_s,sizeof(gid_s),"%d", global_id);
            snprintf(ppath,sizeof(ppath),"%s",params_path);
            snprintf(tid,sizeof(tid),"%d",truck_id);
            execl("./drone","drone", ppath, gid_s, tid, (char*)NULL);
            perror("execl drone");
            exit(1);
        } else if(pid<0){
            perror("fork drone");
        }
    }

    // truck listens for commands from center (e.g., REASSIGN_ONE_TO)
    struct sockaddr_in addr; memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(HOST);
    addr.sin_port = htons(truck_port);
    if(bind(sock,(struct sockaddr*)&addr,sizeof(addr))<0){
        perror("bind truck");
        exit(1);
    }
    printf("[TRUCK %d] iniciado puerto %d\n", truck_id, truck_port);
    msg_t rcv; struct sockaddr_in from;
    while(1){
        if(recv_msg(sock,&rcv,&from)<=0) { usleep(100000); continue; }
        if(rcv.type==MSG_COMMAND){
            printf("[TRUCK %d] CMD: %s\n",truck_id, rcv.text);
            if(strncmp(rcv.text,"REASSIGN_ONE_TO",15)==0){
                // choose one drone and send command to that drone's port (simple: pick first existing)
                // For simplicity we broadcast to all potential drone global ids in this truck
                for(int i=0;i<50;i++){
                    int gid = truck_id*100 + i + 1;
                    int dport = port_for_drone(BASE_PORT, gid);
                    msg_t cmd; memset(&cmd,0,sizeof(cmd));
                    cmd.type = MSG_COMMAND;
                    cmd.swarm_id = -1;
                    cmd.drone_id = gid;
                    snprintf(cmd.text,sizeof(cmd.text),"GO_TO_SWARM %s", rcv.text + 16);
                    send_msg(sock, dport, &cmd);
                }
            } else if(strncmp(rcv.text,"TAKEOFF",7)==0){
                // broadcast TAKEOFF to all drones of this truck
                for(int i=0;i<ASSEMBLY_SIZE;i++){
                    int gid = truck_id*100 + i + 1;
                    int dport = port_for_drone(BASE_PORT, gid);
                    msg_t cmd; memset(&cmd,0,sizeof(cmd));
                    cmd.type = MSG_COMMAND;
                    cmd.swarm_id = truck_id;
                    cmd.drone_id = gid;
                    snprintf(cmd.text,sizeof(cmd.text),"TAKEOFF");
                    send_msg(sock, dport, &cmd);
                }
            }
        }
    }
    return 0;
}

