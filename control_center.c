// control_center.c (versión modificada con artillería)
#include "common.h"
#include <semaphore.h>
#include <math.h>

#define MAX_SWARMS 100
#define MAX_DRONES_PER_SWARM 5

// Zonas de defensa
double B = 20.0;   // inicio zona defensa
double A = 50.0;   // fin zona defensa

typedef struct {
    int swarm_id;
    int truck_pid;
    int truck_id;
    int drone_global_ids[MAX_DRONES_PER_SWARM];
    int active_count;   // drones vivos
    int assembled;      // enjambre listo para TAKEOFF
} swarm_t;

char *params_path = NULL;
int BASE_PORT = 40000;
int NUM_TARGETS = 2;
int W = 30, Q = 5, Z = 5;
int ASSEMBLY_SIZE = 5;
int RANDOM_SEED = 0;

swarm_t swarms[MAX_SWARMS];
int center_sock;

// Semáforos
sem_t sem_swarms;        // protege swarms[]
sem_t sem_reassign_line; // zona crítica de reasignación exclusiva

void load_params(const char *path) {
    FILE *f = fopen(path, "r");
    if(!f) { perror("open params"); exit(1); }
    char line[200];
    while(fgets(line,sizeof(line),f)) {
        if(line[0]=='#' || strlen(line)<3) continue;
        char key[80]; int val;
        if(sscanf(line,"%[^=]=%d", key, &val)==2) {
            if(strcmp(key,"NUM_TARGETS")==0) NUM_TARGETS=val;
            if(strcmp(key,"W")==0) W=val;
            if(strcmp(key,"Q")==0) Q=val;
            if(strcmp(key,"Z")==0) Z=val;
            if(strcmp(key,"ASSEMBLY_SIZE")==0) ASSEMBLY_SIZE=val;
            if(strcmp(key,"BASE_PORT")==0) BASE_PORT=val;
            if(strcmp(key,"RANDOM_SEED")==0) RANDOM_SEED=val;
        }
    }
    fclose(f);
}

void spawn_trucks_and_drones() {
    for(int i=0;i<NUM_TARGETS;i++){
        pid_t pid = fork();
        if(pid==0){
            char tid[16], ppath[256];
            snprintf(tid,sizeof(tid),"%d",i);
            snprintf(ppath,sizeof(ppath),"%s",params_path);
            execl("./truck","truck", ppath, tid, (char*)NULL);
            perror("execl truck");
            exit(1);
        } else if(pid>0) {
            sem_wait(&sem_swarms);
            swarms[i].swarm_id = i;
            swarms[i].truck_pid = pid;
            swarms[i].truck_id = i;
            swarms[i].active_count = ASSEMBLY_SIZE;
            swarms[i].assembled = 0;
            for(int j=0;j<ASSEMBLY_SIZE;j++) swarms[i].drone_global_ids[j]=0;
            sem_post(&sem_swarms);
        } else {
            perror("fork truck");
        }
    }
}

void print_status() {
    sem_wait(&sem_swarms);
    printf("=== CENTER STATUS ===\n");
    for(int i=0;i<NUM_TARGETS;i++){
        printf("Swarm %d: active=%d assembled=%d drones:",i,
               swarms[i].active_count, swarms[i].assembled);
        for(int j=0;j<ASSEMBLY_SIZE;j++) printf(" %d", swarms[i].drone_global_ids[j]);
        printf("\n");
    }
    sem_post(&sem_swarms);
}

void reassign_one_from(int donor_id, int target_id){
    if(donor_id<0 || donor_id>=NUM_TARGETS) return;
    if(target_id<0 || target_id>=NUM_TARGETS) return;
    if(donor_id==target_id) return;

    sem_wait(&sem_reassign_line);
    sem_wait(&sem_swarms);
    int can_donate = (swarms[donor_id].active_count > 0);
    int needs = (swarms[target_id].active_count < ASSEMBLY_SIZE);
    sem_post(&sem_swarms);

    if(can_donate && needs){
        msg_t cmd; memset(&cmd,0,sizeof(cmd));
        cmd.type = MSG_COMMAND;
        cmd.swarm_id = donor_id;
        snprintf(cmd.text,sizeof(cmd.text),"REASSIGN_ONE_TO %d", target_id);
        int truck_port = port_for_truck(BASE_PORT, donor_id);
        send_msg(center_sock, truck_port, &cmd);

        sem_wait(&sem_swarms);
        swarms[target_id].active_count++;
        swarms[donor_id].active_count--;
        if(swarms[target_id].active_count > ASSEMBLY_SIZE)
            swarms[target_id].active_count = ASSEMBLY_SIZE;
        if(swarms[donor_id].active_count < 0)
            swarms[donor_id].active_count = 0;
        sem_post(&sem_swarms);

        printf("[CENTER] Reassigned 1 drone from %d to %d\n", donor_id, target_id);
    }
    sem_post(&sem_reassign_line);
}

void reconform_from_neighbors(int target_id) {
    int step = 1;
    while(step < NUM_TARGETS) {
        int l = target_id - step;
        int r = target_id + step;
        if(l>=0){
            sem_wait(&sem_swarms);
            int need = (swarms[target_id].active_count < ASSEMBLY_SIZE);
            sem_post(&sem_swarms);
            if(!need) return;
            reassign_one_from(l, target_id);
        }
        if(r<NUM_TARGETS){
            sem_wait(&sem_swarms);
            int need = (swarms[target_id].active_count < ASSEMBLY_SIZE);
            sem_post(&sem_swarms);
            if(!need) return;
            reassign_one_from(r, target_id);
        }
        step++;
    }
}

int all_drones_finished(){
    int finished = 1;
    sem_wait(&sem_swarms);
    for(int i=0;i<NUM_TARGETS;i++){
        if(swarms[i].active_count>0){
            finished = 0;
            break;
        }
    }
    sem_post(&sem_swarms);
    return finished;
}

// Función para disparar artillería a drones en zona de defensa
void artillery_fire(int swarm_id, int drone_id, double x) {
    if(x >= B && x < A) {
        if(rand()%100 < W) { // impacto exitoso
            msg_t hit; memset(&hit,0,sizeof(hit));
            hit.type = MSG_ARTILLERY;
            hit.swarm_id = swarm_id;
            hit.drone_id = drone_id;
            snprintf(hit.text,sizeof(hit.text),"HIT");
            int drone_port = port_for_drone(BASE_PORT, drone_id);
            send_msg(center_sock, drone_port, &hit);

            sem_wait(&sem_swarms);
            swarms[swarm_id].active_count--;
            if(swarms[swarm_id].active_count<0) swarms[swarm_id].active_count=0;
            sem_post(&sem_swarms);

            printf("[CENTER] Drone %d destruido por artillería (swarm %d)\n", drone_id, swarm_id);
        }
    }
}

void *listener_thread(void *arg) {
    (void)arg;
    struct sockaddr_in from;
    msg_t m;
    while(1){
        if(recv_msg(center_sock,&m,&from)<=0) {
            usleep(100000);
            continue;
        }

        if(m.type==MSG_HELLO) {
            int gid = m.drone_id;
            int sid = m.swarm_id;

            sem_wait(&sem_swarms);
            int found = 0;
            for(int j=0;j<ASSEMBLY_SIZE;j++){
                if(swarms[sid].drone_global_ids[j]==gid) { found=1; break; }
            }
            if(!found){
                for(int j=0;j<ASSEMBLY_SIZE;j++){
                    if(swarms[sid].drone_global_ids[j]==0){
                        swarms[sid].drone_global_ids[j]=gid;
                        break;
                    }
                }
            }
            sem_post(&sem_swarms);

            printf("[CENTER] HELLO drone %d (swarm %d): %s\n", gid, sid, m.text);
        }
        else if(m.type==MSG_STATUS) {
            printf("[CENTER] STATUS swarm:%d drone:%d -> %s\n", m.swarm_id, m.drone_id, m.text);

            // Procesar destrucción automática o pérdida de enlace
            if(strstr(m.text,"DETONATED") || strstr(m.text,"FUEL_ZERO_AUTODESTRUCT") || strstr(m.text,"LOST_LINK")){
                sem_wait(&sem_swarms);
                swarms[m.swarm_id].active_count--;
                if(swarms[m.swarm_id].active_count<0) swarms[m.swarm_id].active_count=0;
                sem_post(&sem_swarms);
            }
            else if(strstr(m.text,"IN_ASSEMBLY")){
                int count = 0;
                sem_wait(&sem_swarms);
                for(int j=0;j<ASSEMBLY_SIZE;j++)
                    if(swarms[m.swarm_id].drone_global_ids[j]!=0) count++;
                if(count==ASSEMBLY_SIZE){
                    swarms[m.swarm_id].assembled = 1;
                }
                int assembled_now = swarms[m.swarm_id].assembled;
                sem_post(&sem_swarms);

                if(assembled_now){
                    printf("[CENTER] Swarm %d assembled and ready -> TAKEOFF\n", m.swarm_id);
                    msg_t cmd; memset(&cmd,0,sizeof(cmd));
                    cmd.type = MSG_COMMAND;
                    cmd.swarm_id = m.swarm_id;
                    snprintf(cmd.text,sizeof(cmd.text),"TAKEOFF");
                    int truck_port = port_for_truck(BASE_PORT, m.swarm_id);
                    send_msg(center_sock, truck_port, &cmd);
                }
            }
            else if(strstr(m.text,"IN_REASSEMBLY")){
                sem_wait(&sem_swarms);
                int need = (swarms[m.swarm_id].active_count < ASSEMBLY_SIZE);
                sem_post(&sem_swarms);
                if(need){
                    printf("[CENTER] Swarm %d en re-ensamblaje -> reconformando...\n", m.swarm_id);
                    reconform_from_neighbors(m.swarm_id);
                }
            }
            else if(strstr(m.text,"POS")) {
                // Mensaje ejemplo: "POS 25.0 0.0"
                double x_pos, y_pos;
                if(sscanf(m.text,"POS %lf %lf",&x_pos,&y_pos)==2){
                    artillery_fire(m.swarm_id, m.drone_id, x_pos);
                }
            }
        }
        else if(m.type==MSG_ARTILLERY){
            printf("[CENTER] ARTILLERY MSG: %s\n", m.text);
        }
    }
    return NULL;
}

int main(int argc, char **argv){
    if(argc<2){ printf("Uso: control_center params.txt\n"); exit(1); }
    params_path = argv[1];
    load_params(params_path);

    sem_init(&sem_swarms, 0, 1);
    sem_init(&sem_reassign_line, 0, 1);

    srand(RANDOM_SEED ? RANDOM_SEED : time(NULL));
    center_sock = make_udp_socket();
    int center_port = port_for_center(BASE_PORT);
    struct sockaddr_in addr; memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(HOST);
    addr.sin_port = htons(center_port);
    if(bind(center_sock,(struct sockaddr*)&addr,sizeof(addr))<0){
        perror("bind center");
        exit(1);
    }
    printf("[CENTER] Iniciado en puerto %d\n", center_port);

    spawn_trucks_and_drones();

    pthread_t lt;
    pthread_create(&lt,NULL,listener_thread,NULL);

    while(1){
        sleep(5);
        print_status();

        // Terminar si todos los drones ya completaron misión o fueron destruidos
        if(all_drones_finished()){
            printf("[CENTER] Todos los drones terminaron. Saliendo...\n");
            break;
        }
    }

    pthread_cancel(lt);
    pthread_join(lt,NULL);
    sem_destroy(&sem_swarms);
    sem_destroy(&sem_reassign_line);
    close(center_sock);
    return 0;
}

