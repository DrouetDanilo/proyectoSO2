// control_center.c (corregido con mejor manejo de estados y sincronización)
#include "common.h"
#include <semaphore.h>
#include <math.h>
#include <time.h>

#define MAX_SWARMS 100
#define MAX_DRONES_PER_SWARM 5

typedef struct {
    int swarm_id;
    int truck_pid;
    int truck_id;
    int drone_global_ids[MAX_DRONES_PER_SWARM];
    int drone_terminated[MAX_DRONES_PER_SWARM]; // NUEVO: flag para marcar drones ya terminados
    int active_count;   // drones vivos
    int assembled;      // enjambre listo para TAKEOFF
    int target_id;      // ID del blanco asignado (0, 1, 2, ...)
    double target_x;    // Coordenada X del blanco
    double target_y;    // Coordenada Y del blanco
    int target_destroyed; // Estado del blanco: 0=entero, 1=destruido
    time_t reassembly_start; // Timestamp cuando inició la reconformación
    int in_reassembly;  // Flag: está en proceso de reconformación
    int is_destroyed;   // Flag: enjambre fue autodestruido
} swarm_t;

char *params_path = NULL;
int BASE_PORT = 40000;
int NUM_TARGETS = 2;
int NUM_SWARMS = 2;  // Número de enjambres/trucks
int W = 30, Q = 5, Z = 5;
int ASSEMBLY_SIZE = 5;
int RANDOM_SEED = 0;
double C = 100.0; // Posición X base de los blancos
int MAX_WAIT_REASSEMBLY = 5; // Timeout de 5 segundos para reconformación

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
            if(strcmp(key,"MAX_WAIT_REASSEMBLY")==0) MAX_WAIT_REASSEMBLY=val;
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
            swarms[i].reassembly_start = 0;
            swarms[i].in_reassembly = 0;
            swarms[i].is_destroyed = 0;
            for(int j=0;j<ASSEMBLY_SIZE;j++) {
                swarms[i].drone_global_ids[j]=0;
                swarms[i].drone_terminated[j]=0; // NUEVO: inicializar flags de terminación
            }
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
        if(swarms[i].in_reassembly && !swarms[i].is_destroyed) {
            time_t elapsed = time(NULL) - swarms[i].reassembly_start;
            printf(" [RECONFORMANDO:%lds]", elapsed);
        }
        if(swarms[i].is_destroyed) {
            printf(" [AUTODESTRUIDO]");
        }
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

// CORREGIDO: Marcar drone como terminado antes de buscarlo y eliminarlo
int remove_drone_from_swarm_by_id(int drone_id) {
    int found_swarm = -1;
    
    for(int i = 0; i < NUM_SWARMS; i++) {
        // Saltar enjambres autodestruidos
        if(swarms[i].is_destroyed) continue;
        
        for(int j = 0; j < ASSEMBLY_SIZE; j++) {
            if(swarms[i].drone_global_ids[j] == drone_id) {
                // NUEVO: Marcar como terminado para evitar WARNING posterior
                swarms[i].drone_terminated[j] = 1;
                swarms[i].drone_global_ids[j] = 0;
                swarms[i].active_count--;
                if(swarms[i].active_count < 0) swarms[i].active_count = 0;
                found_swarm = i;
                return found_swarm;
            }
        }
    }
    return -1; // No encontrado
}

// CORREGIDO: También marcar como terminado
void remove_drone_from_swarm(int swarm_id, int drone_id) {
    if(swarm_id < 0 || swarm_id >= NUM_SWARMS) return;
    if(swarms[swarm_id].is_destroyed) return; // No procesar si ya está destruido
    
    for(int j=0; j<ASSEMBLY_SIZE; j++) {
        if(swarms[swarm_id].drone_global_ids[j] == drone_id) {
            swarms[swarm_id].drone_terminated[j] = 1; // NUEVO: marcar terminado
            swarms[swarm_id].drone_global_ids[j] = 0;
            swarms[swarm_id].active_count--;
            if(swarms[swarm_id].active_count < 0) swarms[swarm_id].active_count = 0;
            break;
        }
    }
}

// NUEVO: Verificar si un drone ya fue marcado como terminado
int is_drone_already_terminated(int drone_id) {
    for(int i = 0; i < NUM_SWARMS; i++) {
        for(int j = 0; j < ASSEMBLY_SIZE; j++) {
            if(swarms[i].drone_global_ids[j] == drone_id && swarms[i].drone_terminated[j] == 1) {
                return 1;
            }
        }
    }
    return 0;
}

// Inicia el proceso de reconformación para un enjambre incompleto
void start_reassembly_process(int swarm_id) {
    sem_wait(&sem_swarms);
    if(!swarms[swarm_id].in_reassembly && !swarms[swarm_id].is_destroyed) {
        swarms[swarm_id].in_reassembly = 1;
        swarms[swarm_id].reassembly_start = time(NULL);
        printf("[CENTER] Swarm %d inicia proceso de reconformación (timeout: %ds)\n", 
               swarm_id, MAX_WAIT_REASSEMBLY);
    }
    sem_post(&sem_swarms);
}

// Termina el proceso de reconformación exitosamente
void complete_reassembly_process(int swarm_id) {
    sem_wait(&sem_swarms);
    if(swarms[swarm_id].in_reassembly && !swarms[swarm_id].is_destroyed) {
        swarms[swarm_id].in_reassembly = 0;
        swarms[swarm_id].reassembly_start = 0;
        swarms[swarm_id].assembled = 0; // CORREGIDO: resetear assembled para permitir nueva asignación
        printf("[CENTER] Swarm %d completó reconformación exitosamente\n", swarm_id);
    }
    sem_post(&sem_swarms);
}

// Verifica si un enjambre ha excedido el timeout de reconformación
int has_reassembly_timeout(int swarm_id) {
    int timeout = 0;
    sem_wait(&sem_swarms);
    if(swarms[swarm_id].in_reassembly && !swarms[swarm_id].is_destroyed) {
        time_t elapsed = time(NULL) - swarms[swarm_id].reassembly_start;
        if(elapsed >= MAX_WAIT_REASSEMBLY) {
            timeout = 1;
        }
    }
    sem_post(&sem_swarms);
    return timeout;
}

// CORREGIDO: Enviar comando AUTODESTRUCT_ALL a todos los drones del swarm
void autodestruct_swarm_drones(int swarm_id) {
    // Enviar AUTODESTRUCT_ALL a cada drone individual del swarm
    for(int j = 0; j < ASSEMBLY_SIZE; j++) {
        if(swarms[swarm_id].drone_global_ids[j] != 0) {
            int drone_id = swarms[swarm_id].drone_global_ids[j];
            int drone_port = port_for_drone(BASE_PORT, drone_id);
            
            msg_t cmd; memset(&cmd,0,sizeof(cmd));
            cmd.type = MSG_COMMAND;
            cmd.swarm_id = swarm_id;
            cmd.drone_id = drone_id;
            snprintf(cmd.text,sizeof(cmd.text),"AUTODESTRUCT_ALL");
            send_msg(center_sock, drone_port, &cmd);
            
            printf("[CENTER] Enviando AUTODESTRUCT_ALL a drone %d (puerto %d)\n", drone_id, drone_port);
        }
    }
    
    // También enviar al truck por compatibilidad
    msg_t truck_cmd; memset(&truck_cmd,0,sizeof(truck_cmd));
    truck_cmd.type = MSG_COMMAND;
    truck_cmd.swarm_id = swarm_id;
    snprintf(truck_cmd.text,sizeof(truck_cmd.text),"AUTODESTRUCT_ALL");
    int truck_port = port_for_truck(BASE_PORT, swarm_id);
    send_msg(center_sock, truck_port, &truck_cmd);
}

// Busca un dron activo en el enjambre donante y lo reasigna
void reassign_one_from(int donor_id, int target_id){
    if(donor_id<0 || donor_id>=NUM_SWARMS) return;
    if(target_id<0 || target_id>=NUM_SWARMS) return;
    if(donor_id==target_id) return;

    sem_wait(&sem_reassign_line);
    sem_wait(&sem_swarms);

    // CORREGIDO: No reasignar de/hacia enjambres destruidos
    if(swarms[donor_id].is_destroyed || swarms[target_id].is_destroyed) {
        sem_post(&sem_swarms);
        sem_post(&sem_reassign_line);
        return;
    }

    // Solo reasignar si el destino NO está completo y el donante tiene drones activos
    if(swarms[donor_id].active_count > 0 && swarms[target_id].active_count < ASSEMBLY_SIZE) {
        // Buscar el primer dron activo en el donante
        int drone_id = 0;
        int donor_slot = -1;
        for(int j=0; j<ASSEMBLY_SIZE; j++) {
            if(swarms[donor_id].drone_global_ids[j] != 0) {
                drone_id = swarms[donor_id].drone_global_ids[j];
                donor_slot = j;
                // CORREGIDO: Limpiar completamente la entrada del donante
                swarms[donor_id].drone_global_ids[j] = 0;
                swarms[donor_id].drone_terminated[j] = 0;
                swarms[donor_id].active_count--;
                // CORREGIDO: Si donante queda incompleto, resetear assembled
                if(swarms[donor_id].active_count < ASSEMBLY_SIZE) {
                    swarms[donor_id].assembled = 0;
                }
                break;
            }
        }
        
        // CORREGIDO: Asignar correctamente el dron al destino
        if(drone_id != 0) {
            int target_slot = -1;
            for(int j=0; j<ASSEMBLY_SIZE; j++) {
                if(swarms[target_id].drone_global_ids[j] == 0) {
                    swarms[target_id].drone_global_ids[j] = drone_id;
                    swarms[target_id].drone_terminated[j] = 0; // NUEVO: asegurar que no esté marcado terminado
                    swarms[target_id].active_count++;
                    target_slot = j;
                    // CORREGIDO: Si target se completa, permitir ensamblaje
                    if(swarms[target_id].active_count >= ASSEMBLY_SIZE) {
                        swarms[target_id].assembled = 0; // Permitir nuevo ensamblaje
                    }
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

            printf("[CENTER] Reassigned drone %d from swarm %d (slot %d) to swarm %d (slot %d)\n", 
                   drone_id, donor_id, donor_slot, target_id, target_slot);
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
            int need = (swarms[target_id].active_count < ASSEMBLY_SIZE && !swarms[target_id].is_destroyed);
            sem_post(&sem_swarms);
            if(!need) {
                complete_reassembly_process(target_id);
                return;
            }
            reassign_one_from(l, target_id);
        }
        if(r<NUM_SWARMS){
            sem_wait(&sem_swarms);
            int need = (swarms[target_id].active_count < ASSEMBLY_SIZE && !swarms[target_id].is_destroyed);
            sem_post(&sem_swarms);
            if(!need) {
                complete_reassembly_process(target_id);
                return;
            }
            reassign_one_from(r, target_id);
        }
        step++;
    }
}

// CORREGIDO: Autodestruye todos los drones de un enjambre
void autodestruct_swarm(int swarm_id) {
    sem_wait(&sem_swarms);
    
    // No autodestruir si ya está vacío o ya fue destruido
    if(swarms[swarm_id].active_count <= 0 || swarms[swarm_id].is_destroyed) {
        swarms[swarm_id].in_reassembly = 0;
        swarms[swarm_id].reassembly_start = 0;
        sem_post(&sem_swarms);
        return;
    }
    
    printf("[CENTER] TIMEOUT: Swarm %d no pudo reconformarse en %ds - AUTODESTRUYENDO\n", 
           swarm_id, MAX_WAIT_REASSEMBLY);
    
    // Marcar como destruido ANTES de enviar comando
    swarms[swarm_id].is_destroyed = 1;
    swarms[swarm_id].in_reassembly = 0;
    swarms[swarm_id].reassembly_start = 0;
    swarms[swarm_id].assembled = 0;
    
    sem_post(&sem_swarms);
    
    // CORREGIDO: Enviar comando de autodestrucción a cada drone
    autodestruct_swarm_drones(swarm_id);
    
    // Limpiar estado local después de enviar comandos
    sem_wait(&sem_swarms);
    for(int j=0; j<ASSEMBLY_SIZE; j++) {
        if(swarms[swarm_id].drone_global_ids[j] != 0) {
            int did = swarms[swarm_id].drone_global_ids[j];
            swarms[swarm_id].drone_global_ids[j] = 0;
            swarms[swarm_id].drone_terminated[j] = 1; // NUEVO: marcar como terminado
            printf("[CENTER] Swarm %d autodestruye drone %d por timeout\n", swarm_id, did);
        }
    }
    swarms[swarm_id].active_count = 0;
    sem_post(&sem_swarms);
}

// CORREGIDO: Verificar si hay otros swarms incompletos disponibles para reconformación
int has_incomplete_swarms_available(int exclude_swarm) {
    for(int i = 0; i < NUM_SWARMS; i++) {
        if(i == exclude_swarm) continue;
        if(swarms[i].is_destroyed) continue;
        if(swarms[i].active_count > 0 && swarms[i].active_count < ASSEMBLY_SIZE) {
            return 1; // Hay al menos un swarm incompleto disponible
        }
    }
    return 0; // No hay swarms incompletos disponibles
}

// CORREGIDO: Verificar si todos los demás swarms están completos o destruidos
int all_other_swarms_complete_or_destroyed(int exclude_swarm) {
    for(int i = 0; i < NUM_SWARMS; i++) {
        if(i == exclude_swarm) continue;
        if(swarms[i].is_destroyed) continue;
        if(swarms[i].active_count > 0 && swarms[i].active_count < ASSEMBLY_SIZE) {
            return 0; // Hay un swarm incompleto
        }
    }
    return 1; // Todos los demás están completos o destruidos
}

// Verifica timeouts de reconformación y actúa en consecuencia
void check_reassembly_timeouts() {
    for(int i = 0; i < NUM_SWARMS; i++) {
        sem_wait(&sem_swarms);
        int is_incomplete = (swarms[i].active_count > 0 && swarms[i].active_count < ASSEMBLY_SIZE);
        int in_reassembly = swarms[i].in_reassembly;
        int is_destroyed = swarms[i].is_destroyed;
        sem_post(&sem_swarms);
        
        if(is_incomplete && !is_destroyed) {
            // CORREGIDO: Verificar condiciones de autodestrucción
            if(in_reassembly && has_reassembly_timeout(i)) {
                // Timeout alcanzado
                autodestruct_swarm(i);
            } else if(!in_reassembly && all_other_swarms_complete_or_destroyed(i)) {
                // CORREGIDO: No hay otros swarms para reconformación, autodestruir inmediatamente
                printf("[CENTER] Swarm %d está incompleto y no hay otros swarms disponibles - AUTODESTRUYENDO\n", i);
                sem_wait(&sem_swarms);
                swarms[i].is_destroyed = 1;
                sem_post(&sem_swarms);
                autodestruct_swarm_drones(i);
                
                sem_wait(&sem_swarms);
                for(int j=0; j<ASSEMBLY_SIZE; j++) {
                    if(swarms[i].drone_global_ids[j] != 0) {
                        int did = swarms[i].drone_global_ids[j];
                        swarms[i].drone_global_ids[j] = 0;
                        swarms[i].drone_terminated[j] = 1;
                        printf("[CENTER] Swarm %d autodestruye drone %d (sin opciones de reconformación)\n", i, did);
                    }
                }
                swarms[i].active_count = 0;
                sem_post(&sem_swarms);
            }
        }
    }
}

int all_drones_finished(){
    int finished = 1;
    sem_wait(&sem_swarms);
    for(int i=0;i<NUM_SWARMS;i++){
        // Solo cuenta como terminado si el enjambre está vacío
        if(swarms[i].active_count > 0){
            finished = 0;
            break;
        }
    }
    sem_post(&sem_swarms);
    return finished;
}

// Declaración anticipada
void try_reconform_or_autodestruct(int swarm_id);

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
            // CORREGIDO: No procesar si el enjambre está destruido
            if(!swarms[sid].is_destroyed) {
                int found = 0;
                for(int j=0;j<ASSEMBLY_SIZE;j++){
                    if(swarms[sid].drone_global_ids[j]==gid) { found=1; break; }
                }
                if(!found){
                    for(int j=0;j<ASSEMBLY_SIZE;j++){
                        if(swarms[sid].drone_global_ids[j]==0){
                            swarms[sid].drone_global_ids[j]=gid;
                            swarms[sid].drone_terminated[j]=0; // NUEVO: inicializar flag
                            break;
                        }
                    }
                }
            }
            sem_post(&sem_swarms);

            printf("[CENTER] HELLO drone %d (swarm %d): %s\n", gid, sid, m.text);
        }
        else if(m.type==MSG_STATUS) {
            printf("[CENTER] STATUS swarm:%d drone:%d -> %s\n", m.swarm_id, m.drone_id, m.text);

            if(strstr(m.text,"DETONATED") || strstr(m.text,"FUEL_ZERO_AUTODESTRUCT") || 
               strstr(m.text,"LINK_PERMANENT_LOSS") || strstr(m.text,"SHOT_DOWN_BY_ARTILLERY") ||
               strstr(m.text,"CAMERA_AUTODESTRUCT")){
                
                // CORREGIDO: Verificar si ya fue terminado antes de mostrar WARNING
                sem_wait(&sem_swarms);
                int already_terminated = is_drone_already_terminated(m.drone_id);
                int found_swarm = -1;
                
                if(!already_terminated) {
                    found_swarm = remove_drone_from_swarm_by_id(m.drone_id);
                }
                
                if(found_swarm >= 0) {
                    printf("[CENTER] Drone %d del swarm %d terminado. Activos restantes: %d\n", 
                           m.drone_id, found_swarm, swarms[found_swarm].active_count);
                } else if(!already_terminated) {
                    
                }
                sem_post(&sem_swarms);
            }
            else if(strstr(m.text,"ARRIVED_DETONATED")){
                sem_wait(&sem_swarms);
                // CORREGIDO: Solo procesar si el enjambre no está destruido
                if(!swarms[m.swarm_id].is_destroyed) {
                    swarms[m.swarm_id].target_destroyed = 1;
                    remove_drone_from_swarm(m.swarm_id, m.drone_id);
                    printf("[CENTER] *** BLANCO %d DESTRUIDO por drone %d ***\n", 
                           swarms[m.swarm_id].target_id, m.drone_id);
                }
                sem_post(&sem_swarms);
            }
            else if(strstr(m.text,"CAMERA_REPORTED")){
                sem_wait(&sem_swarms);
                // CORREGIDO: Solo procesar si el enjambre no está destruido
                if(!swarms[m.swarm_id].is_destroyed) {
                    int target_id = swarms[m.swarm_id].target_id;
                    int destroyed = swarms[m.swarm_id].target_destroyed;
                    remove_drone_from_swarm(m.swarm_id, m.drone_id);
                    
                    printf("[CENTER] *** REPORTE DE CAMARA ***\n");
                    printf("[CENTER] *** BLANCO %d: %s ***\n", 
                           target_id, destroyed ? "ENTERO" : "DESTRUIDO");
                }
                sem_post(&sem_swarms);
            }
            else if(strstr(m.text,"IN_ASSEMBLY")){
                sem_wait(&sem_swarms);
                // CORREGIDO: Solo procesar si el enjambre no está destruido
                if(!swarms[m.swarm_id].is_destroyed) {
                    int count = 0;
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
                } else {
                    sem_post(&sem_swarms);
                }
            }
            else if(strstr(m.text,"IN_REASSEMBLY")){
                sem_wait(&sem_swarms);
                int need = (swarms[m.swarm_id].active_count < ASSEMBLY_SIZE && 
                           swarms[m.swarm_id].active_count > 0 && 
                           !swarms[m.swarm_id].is_destroyed);
                int already_in_reassembly = swarms[m.swarm_id].in_reassembly;
                sem_post(&sem_swarms);
                
                if(need && !already_in_reassembly){
                    start_reassembly_process(m.swarm_id);
                    try_reconform_or_autodestruct(m.swarm_id);
                }
            }
        }
        else if(m.type==MSG_ARTILLERY){
            printf("[CENTER] ARTILLERY MSG: %s\n", m.text);
            if(strstr(m.text,"SHOT_DOWN")){
                int did;
                if(sscanf(m.text,"DRONE %d",&did)==1){
                    // CORREGIDO: Buscar en todos los enjambres activos
                    sem_wait(&sem_swarms);
                    int found_swarm = remove_drone_from_swarm_by_id(did);
                    if(found_swarm >= 0) {
                        printf("[CENTER] Drone %d removido del swarm %d por artillería\n", did, found_swarm);
                    }
                    sem_post(&sem_swarms);
                }
            }
        }
    }
    return NULL;
}

void try_reconform_or_autodestruct(int swarm_id) {
    sem_wait(&sem_swarms);
    // No procesar si ya está destruido
    if(swarms[swarm_id].is_destroyed) {
        sem_post(&sem_swarms);
        return;
    }
    sem_post(&sem_swarms);
    
    int can_reconform = 0;
    sem_wait(&sem_swarms);
    for(int i=0; i<NUM_SWARMS; i++) {
        if(i == swarm_id) continue;
        // Solo enjambres incompletos, con drones vivos y no destruidos
        if(swarms[i].active_count > 0 && swarms[i].active_count < ASSEMBLY_SIZE && !swarms[i].is_destroyed) {
            can_reconform = 1;
            break;
        }
    }
    sem_post(&sem_swarms);

    if(can_reconform) {
        printf("[CENTER] Swarm %d intenta reconformarse...\n", swarm_id);
        reconform_from_neighbors(swarm_id);
        
        // Verificar si se completó después del intento
        sem_wait(&sem_swarms);
        int completed = (swarms[swarm_id].active_count >= ASSEMBLY_SIZE && !swarms[swarm_id].is_destroyed);
        sem_post(&sem_swarms);
        
        if(completed) {
            complete_reassembly_process(swarm_id);
        }
    } else {
        // No hay enjambres disponibles, pero aún esperamos el timeout
        printf("[CENTER] No hay enjambres disponibles para reconformar swarm %d - esperando timeout\n", swarm_id);
    }
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
        sleep(1); // Verificar más frecuentemente para detectar timeouts
        
        // Verificar timeouts de reconformación
        check_reassembly_timeouts();
        
        // Imprimir status cada 5 segundos
        static int status_counter = 0;
        if(++status_counter >= 5) {
            print_status();
            status_counter = 0;
        }

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