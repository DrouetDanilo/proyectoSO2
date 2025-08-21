// control_center.c (con asignación aleatoria de blancos)
#include "common.h"
#include <semaphore.h>
#include <math.h>

#define MAX_SWARMS 100
#define MAX_DRONES_PER_SWARM 5

typedef struct {
    int swarm_id;
    int truck_pid;
    int truck_id;
    int drone_global_ids[MAX_DRONES_PER_SWARM];
    int active_count;   // drones vivos
    int assembled;      // enjambre listo para TAKEOFF
    int target_id;      // ID del blanco asignado (0, 1, 2, ...)
    double target_x;    // Coordenada X del blanco
    double target_y;    // Coordenada Y del blanco
    int target_destroyed; // Estado del blanco: 0=entero, 1=destruido
} swarm_t;

char *params_path = NULL;
int BASE_PORT = 40000;
int NUM_TARGETS = 2;
int NUM_SWARMS = 2;  // Número de enjambres/trucks
int W = 30, Q = 5, Z = 5;
int ASSEMBLY_SIZE = 5;
int RANDOM_SEED = 0;
double C = 100.0; // Posición X base de los blancos

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
        char key[80]; int val; double dval;
        if(sscanf(line,"%[^=]=%d", key, &val)==2) {
            if(strcmp(key,"NUM_TARGETS")==0) NUM_TARGETS=val;
            if(strcmp(key,"NUM_SWARMS")==0) NUM_SWARMS=val;
            if(strcmp(key,"W")==0) W=val;
            if(strcmp(key,"Q")==0) Q=val;
            if(strcmp(key,"Z")==0) Z=val;
            if(strcmp(key,"ASSEMBLY_SIZE")==0) ASSEMBLY_SIZE=val;
            if(strcmp(key,"BASE_PORT")==0) BASE_PORT=val;
            if(strcmp(key,"RANDOM_SEED")==0) RANDOM_SEED=val;
        }
        else if(sscanf(line,"%[^=]=%lf", key, &dval)==2) {
            if(strcmp(key,"C")==0) C=dval;
        }
    }
    fclose(f);
}

void assign_random_targets() {
    // Si hay más enjambres que blancos, algunos enjambres atacarán el mismo blanco
    // Si hay más blancos que enjambres, algunos blancos quedarán sin atacar
    
    printf("[CENTER] Asignando %d enjambres a %d blancos disponibles\n", NUM_SWARMS, NUM_TARGETS);
    
    for(int i = 0; i < NUM_SWARMS; i++) {
        // Asignar blanco aleatoriamente (puede repetirse)
        swarms[i].target_id = rand() % NUM_TARGETS;
        swarms[i].target_x = C;
        swarms[i].target_y = rand() % 101; // 0 a 100
        swarms[i].target_destroyed = 0;
        
        printf("[CENTER] Swarm %d asignado a Blanco %d en (%.1f, %.1f)\n", 
               i, swarms[i].target_id, swarms[i].target_x, swarms[i].target_y);
    }
}

void spawn_trucks_and_drones() {
    for(int i=0;i<NUM_SWARMS;i++){
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
    
    // Asignar blancos aleatorios después de crear los enjambres
    assign_random_targets();
}

void print_status() {
    sem_wait(&sem_swarms);
    printf("=== CENTER STATUS ===\n");
    for(int i=0;i<NUM_SWARMS;i++){
        printf("Swarm %d: active=%d assembled=%d target=%d(%.1f,%.1f)%s drones:",i,
               swarms[i].active_count, swarms[i].assembled, 
               swarms[i].target_id, swarms[i].target_x, swarms[i].target_y,
               swarms[i].target_destroyed ? "[DESTRUIDO]" : "[ENTERO]");
        for(int j=0;j<ASSEMBLY_SIZE;j++) printf(" %d", swarms[i].drone_global_ids[j]);
        printf("\n");
    }
    sem_post(&sem_swarms);
}

void send_target_to_truck(int swarm_id) {
    msg_t cmd; memset(&cmd,0,sizeof(cmd));
    cmd.type = MSG_COMMAND;
    cmd.swarm_id = swarm_id;
    snprintf(cmd.text,sizeof(cmd.text),"TARGET %.1f %.1f %d", 
             swarms[swarm_id].target_x, swarms[swarm_id].target_y, swarms[swarm_id].target_id);
    int truck_port = port_for_truck(BASE_PORT, swarm_id);
    send_msg(center_sock, truck_port, &cmd);
}

// Elimina el dron del arreglo y actualiza el conteo
void remove_drone_from_swarm(int swarm_id, int drone_id) {
    for(int j=0; j<ASSEMBLY_SIZE; j++) {
        if(swarms[swarm_id].drone_global_ids[j] == drone_id) {
            swarms[swarm_id].drone_global_ids[j] = 0;
            swarms[swarm_id].active_count--;
            if(swarms[swarm_id].active_count < 0) swarms[swarm_id].active_count = 0;
            break;
        }
    }
}

// Busca un dron activo en el enjambre donante y lo reasigna
void reassign_one_from(int donor_id, int target_id){
    if(donor_id<0 || donor_id>=NUM_SWARMS) return;
    if(target_id<0 || target_id>=NUM_SWARMS) return;
    if(donor_id==target_id) return;

    sem_wait(&sem_reassign_line);
    sem_wait(&sem_swarms);

    // Solo reasignar si el destino NO está completo y el donante tiene drones activos
    if(swarms[donor_id].active_count > 0 && swarms[target_id].active_count < ASSEMBLY_SIZE) {
        // Buscar el primer dron activo en el donante
        int drone_id = 0;
        for(int j=0; j<ASSEMBLY_SIZE; j++) {
            if(swarms[donor_id].drone_global_ids[j] != 0) {
                drone_id = swarms[donor_id].drone_global_ids[j];
                swarms[donor_id].drone_global_ids[j] = 0;
                swarms[donor_id].active_count--;
                break;
            }
        }
        // Asignar el dron al destino
        if(drone_id != 0) {
            for(int j=0; j<ASSEMBLY_SIZE; j++) {
                if(swarms[target_id].drone_global_ids[j] == 0) {
                    swarms[target_id].drone_global_ids[j] = drone_id;
                    swarms[target_id].active_count++;
                    break;
                }
            }
            // Enviar comando al truck donante para que reasigne el dron
            msg_t cmd; memset(&cmd,0,sizeof(cmd));
            cmd.type = MSG_COMMAND;
            cmd.swarm_id = donor_id;
            snprintf(cmd.text,sizeof(cmd.text),"REASSIGN_ONE_TO %d", target_id);
            int truck_port = port_for_truck(BASE_PORT, donor_id);
            send_msg(center_sock, truck_port, &cmd);

            printf("[CENTER] Reassigned drone %d from %d to %d\n", drone_id, donor_id, target_id);
        }
    }
    sem_post(&sem_swarms);
    sem_post(&sem_reassign_line);
}

void reconform_from_neighbors(int target_id) {
    int step = 1;
    while(step < NUM_SWARMS) {
        int l = target_id - step;
        int r = target_id + step;
        if(l>=0){
            sem_wait(&sem_swarms);
            int need = (swarms[target_id].active_count < ASSEMBLY_SIZE);
            sem_post(&sem_swarms);
            if(!need) return;
            reassign_one_from(l, target_id);
        }
        if(r<NUM_SWARMS){
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
    for(int i=0;i<NUM_SWARMS;i++){
        // Solo cuenta como terminado si el enjambre está vacío o todos sus drones están detonados
        if(swarms[i].active_count > 0){
            finished = 0;
            break;
        }
    }
    sem_post(&sem_swarms);
    return finished;
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

            if(strstr(m.text,"DETONATED") || strstr(m.text,"FUEL_ZERO_AUTODESTRUCT") || 
               strstr(m.text,"LOST_LINK") || strstr(m.text,"SHOT_DOWN_BY_ARTILLERY") ||
               strstr(m.text,"CAMERA_AUTODESTRUCT")){
                sem_wait(&sem_swarms);
                remove_drone_from_swarm(m.swarm_id, m.drone_id);
                sem_post(&sem_swarms);
                printf("[CENTER] Drone %d del swarm %d terminado. Activos restantes: %d\n", 
                       m.drone_id, m.swarm_id, swarms[m.swarm_id].active_count);
            }
            else if(strstr(m.text,"ARRIVED_DETONATED")){
                sem_wait(&sem_swarms);
                swarms[m.swarm_id].target_destroyed = 1;
                swarms[m.swarm_id].active_count--;
                if(swarms[m.swarm_id].active_count<0) swarms[m.swarm_id].active_count=0;
                sem_post(&sem_swarms);
                printf("[CENTER] *** BLANCO %d DESTRUIDO por drone %d ***\n", 
                       swarms[m.swarm_id].target_id, m.drone_id);
            }
            else if(strstr(m.text,"CAMERA_REPORTED")){
                sem_wait(&sem_swarms);
                int target_id = swarms[m.swarm_id].target_id;
                int destroyed = swarms[m.swarm_id].target_destroyed;
                swarms[m.swarm_id].active_count--;
                if(swarms[m.swarm_id].active_count<0) swarms[m.swarm_id].active_count=0;
                sem_post(&sem_swarms);
                
                printf("[CENTER] *** REPORTE DE CAMARA ***\n");
                printf("[CENTER] *** BLANCO %d: %s ***\n", 
                       target_id, destroyed ? "DESTRUIDO" : "ENTERO");
            }
            else if(strstr(m.text,"IN_ASSEMBLY")){
                int count = 0;
                sem_wait(&sem_swarms);
                for(int j=0;j<ASSEMBLY_SIZE;j++)
                    if(swarms[m.swarm_id].drone_global_ids[j]!=0) count++;
                if(count==ASSEMBLY_SIZE && swarms[m.swarm_id].assembled == 0){
                    swarms[m.swarm_id].assembled = 1;
                }
                int assembled_now = (swarms[m.swarm_id].assembled == 1);
                sem_post(&sem_swarms);

                if(assembled_now){
                    printf("[CENTER] Swarm %d assembled and ready -> TAKEOFF\n", m.swarm_id);
                    
                    sem_wait(&sem_swarms);
                    
                    // Enviar coordenadas del blanco primero (solo una vez)
                    send_target_to_truck(m.swarm_id);
                    
                    // Luego enviar TAKEOFF (solo una vez)
                    msg_t cmd; memset(&cmd,0,sizeof(cmd));
                    cmd.type = MSG_COMMAND;
                    cmd.swarm_id = m.swarm_id;
                    snprintf(cmd.text,sizeof(cmd.text),"TAKEOFF");
                    int truck_port = port_for_truck(BASE_PORT, m.swarm_id);
                    send_msg(center_sock, truck_port, &cmd);
                    
                    // Marcar que ya se envió para evitar duplicados
                    swarms[m.swarm_id].assembled = 2; // 2 = ya despegado
                    sem_post(&sem_swarms);
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
        }
        else if(m.type==MSG_ARTILLERY){
            printf("[CENTER] ARTILLERY MSG: %s\n", m.text);
            if(strstr(m.text,"SHOT_DOWN")){
                int did;
                if(sscanf(m.text,"DRONE %d",&did)==1){
                    // Encontrar swarm al que pertenecía
                    for(int i=0;i<NUM_SWARMS;i++){
                        for(int j=0;j<ASSEMBLY_SIZE;j++){
                            if(swarms[i].drone_global_ids[j]==did){
                                sem_wait(&sem_swarms);
                                swarms[i].active_count--;
                                if(swarms[i].active_count<0) swarms[i].active_count=0;
                                sem_post(&sem_swarms);
                                printf("[CENTER] Drone %d removido del swarm %d por artillería\n", did, i);
                            }
                        }
                    }
                }
            }
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

        if(all_drones_finished()){
            printf("[CENTER] Todos los drones terminaron. Enviando señal de terminación a artillería...\n");
            
            // Enviar señal de terminación a artillería
            msg_t term_msg; memset(&term_msg,0,sizeof(term_msg));
            term_msg.type = MSG_ARTILLERY;
            snprintf(term_msg.text,sizeof(term_msg.text),"TERMINATE");
            int artillery_port = port_for_artillery(BASE_PORT);
            send_msg(center_sock, artillery_port, &term_msg);
            
            sleep(1); // Dar tiempo para que artillería reciba el mensaje
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