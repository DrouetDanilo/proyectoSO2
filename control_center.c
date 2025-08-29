// control_center.c (version con fallo de artilleria CORREGIDO)
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
    int drone_terminated[MAX_DRONES_PER_SWARM]; // marca si ese slot ya terminó
    int active_count;   // drones vivos
    int assembled;      // 0: no listo, 1: listo para TAKEOFF, 2: TAKEOFF enviado
    int target_id;      // ID del blanco asignado (0, 1, 2, ...)
    double target_x;    // Coordenada X del blanco
    double target_y;    // Coordenada Y del blanco
    int target_destroyed; // 0=entero, 1=destruido
    time_t reassembly_start; // timestamp inicio de reconformación
    int in_reassembly;  // flag: en proceso de reconformación
    int is_destroyed;   // flag: swarm autodestruido
    int camera_reported; // NEW: para evitar doble reporte de cámara
} swarm_t;

char *params_path = NULL;
int BASE_PORT = 40000;
int NUM_TARGETS = 2;
int NUM_SWARMS = 2;
int W = 30, Q = 5, Z = 5;
int ASSEMBLY_SIZE = 5;
int RANDOM_SEED = 0;
double C = 100.0;
int MAX_WAIT_REASSEMBLY = 5;

swarm_t swarms[MAX_SWARMS];
int center_sock;

// Mapa consistente target_id -> (x,y)
typedef struct { double x,y; } target_pos_t;
static target_pos_t targets_catalog[256]; // soporta hasta 256 blancos

// Semáforos
sem_t sem_swarms;        // protege swarms[]
sem_t sem_reassign_line; // sección crítica de reasignación entre swarms

// ---------- util ----------
static inline void notify_artillery_down(int drone_id){
    msg_t a; memset(&a,0,sizeof(a));
    a.type = MSG_ARTILLERY;
    snprintf(a.text,sizeof(a.text),"DRONE_TERMINATED %d", drone_id);
    int artillery_port = port_for_artillery(BASE_PORT);
    send_msg(center_sock, artillery_port, &a);
}

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

// Genera un catálogo determinista de blancos: MISMO ID → MISMAS COORDS
static void build_targets_catalog(void){
    // X fijo en C, Y espaciado uniforme en [10, 100-10]
    if(NUM_TARGETS > 256) NUM_TARGETS = 256;
    double y0 = 10.0, y1 = 90.0;
    for(int t=0; t<NUM_TARGETS; ++t){
        double frac = (NUM_TARGETS<=1)?0.0: (double)t/(double)(NUM_TARGETS-1);
        targets_catalog[t].x = C;
        targets_catalog[t].y = y0 + (y1 - y0)*frac;
    }
}

void assign_random_targets() {
    printf("[CENTER] Asignando %d enjambres a %d blancos disponibles\n", NUM_SWARMS, NUM_TARGETS);
    for(int i = 0; i < NUM_SWARMS; i++) {
        int tid = i % NUM_TARGETS;   // ✅ round-robin, no random

        swarms[i].target_id = tid;
        swarms[i].target_x = targets_catalog[tid].x;
        swarms[i].target_y = targets_catalog[tid].y;
        swarms[i].target_destroyed = 0;

        printf("[CENTER] Swarm %d asignado a Blanco %d en (%.1f, %.1f)\n",
               i, tid, swarms[i].target_x, swarms[i].target_y);
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
            swarms[i].camera_reported = 0;
            for(int j=0;j<ASSEMBLY_SIZE;j++) {
                swarms[i].drone_global_ids[j]=0;
                swarms[i].drone_terminated[j]=0;
            }
            sem_post(&sem_swarms);
        } else {
            perror("fork truck");
        }
    }
    assign_random_targets();
}

void try_reconform_or_autodestruct(int swarm_id);

void print_status() {
    sem_wait(&sem_swarms);
    printf("=== CENTER STATUS ===\n");
    for(int i=0;i<NUM_SWARMS;i++){
        printf("Swarm %d: active=%d assembled=%d target=%d(%.1f,%.1f)%s drones:",
               i, swarms[i].active_count, swarms[i].assembled,
               swarms[i].target_id, swarms[i].target_x, swarms[i].target_y,
               swarms[i].target_destroyed ? "[DESTRUIDO]" : "[ENTERO]");
        for(int j=0;j<ASSEMBLY_SIZE;j++) {
            if(swarms[i].drone_global_ids[j] != 0) {
                printf(" %d", swarms[i].drone_global_ids[j]);
            }
        }
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

static void send_target_to_truck_coords(int swarm_id, double tx, double ty, int tid) {
    msg_t cmd; memset(&cmd,0,sizeof(cmd));
    cmd.type = MSG_COMMAND;
    cmd.swarm_id = swarm_id;
    snprintf(cmd.text,sizeof(cmd.text),"TARGET %.1f %.1f %d", tx, ty, tid);
    int truck_port = port_for_truck(BASE_PORT, swarm_id);
    send_msg(center_sock, truck_port, &cmd);
}

void send_target_to_truck(int swarm_id) {
    // leer coordenadas bajo protección para coherencia
    double tx, ty; int tid;
    sem_wait(&sem_swarms);
    tx = swarms[swarm_id].target_x;
    ty = swarms[swarm_id].target_y;
    tid = swarms[swarm_id].target_id;
    sem_post(&sem_swarms);
    send_target_to_truck_coords(swarm_id, tx, ty, tid);
}

// Remueve por ID global buscando en todos los swarms (se asume sem_swarms tomado por el caller)
int remove_drone_from_swarm_by_id(int drone_id) {
    int found_swarm = -1;
    for(int i = 0; i < NUM_SWARMS; i++) {
        if(swarms[i].is_destroyed) continue;
        for(int j = 0; j < ASSEMBLY_SIZE; j++) {
            if(swarms[i].drone_global_ids[j] == drone_id) {
                swarms[i].drone_terminated[j] = 1;
                swarms[i].drone_global_ids[j] = 0;
                if(swarms[i].active_count > 0) swarms[i].active_count--;
                found_swarm = i;
                return found_swarm;
            }
        }
    }
    return -1;
}

// Remueve por (swarm, drone) directo (se asume sem_swarms tomado por el caller)
void remove_drone_from_swarm(int swarm_id, int drone_id) {
    if(swarm_id < 0 || swarm_id >= NUM_SWARMS) return;
    if(swarms[swarm_id].is_destroyed) return;
    for(int j=0; j<ASSEMBLY_SIZE; j++) {
        if(swarms[swarm_id].drone_global_ids[j] == drone_id) {
            swarms[swarm_id].drone_terminated[j] = 1;
            swarms[swarm_id].drone_global_ids[j] = 0;
            if(swarms[swarm_id].active_count > 0) swarms[swarm_id].active_count--;
            break;
        }
    }
}

// Marca inicio de reconformación con timeout
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

// Limpia flags tras reconformación exitosa
void complete_reassembly_process(int swarm_id) {
    sem_wait(&sem_swarms);
    if(swarms[swarm_id].in_reassembly && !swarms[swarm_id].is_destroyed) {
        swarms[swarm_id].in_reassembly = 0;
        swarms[swarm_id].reassembly_start = 0;
        swarms[swarm_id].assembled = 0; // permite nuevo ensamblaje/TAKEOFF si se completó
        printf("[CENTER] Swarm %d completó reconformación exitosamente\n", swarm_id);
    }
    sem_post(&sem_swarms);
}

// Envío de AUTODESTRUCT_ALL a todos los drones del swarm (snapshot para evitar carreras)
void autodestruct_swarm_drones(int swarm_id) {
    int snapshot_ids[MAX_DRONES_PER_SWARM] = {0};
    sem_wait(&sem_swarms);
    for(int j = 0; j < ASSEMBLY_SIZE; j++) {
        snapshot_ids[j] = swarms[swarm_id].drone_global_ids[j];
    }
    sem_post(&sem_swarms);

    for(int j = 0; j < ASSEMBLY_SIZE; j++) {
        if(snapshot_ids[j] != 0) {
            int drone_id = snapshot_ids[j];
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

    // Comando también al truck para compatibilidad
    msg_t truck_cmd; memset(&truck_cmd,0,sizeof(truck_cmd));
    truck_cmd.type = MSG_COMMAND;
    truck_cmd.swarm_id = swarm_id;
    snprintf(truck_cmd.text,sizeof(truck_cmd.text),"AUTODESTRUCT_ALL");
    int truck_port = port_for_truck(BASE_PORT, swarm_id);
    send_msg(center_sock, truck_port, &truck_cmd);
}

// Función auxiliar para verificar si un swarm necesita reconformación
int swarm_needs_reassembly(int swarm_id) {
    return (swarms[swarm_id].active_count > 0 && 
            swarms[swarm_id].active_count < ASSEMBLY_SIZE && 
            !swarms[swarm_id].is_destroyed);
}

// Reasigna un drone desde un swarm donante a uno objetivo.
// Regla: solo se puede donar desde swarms incompletos (no tomar de swarms completos).
void reassign_one_from(int donor_id, int target_id) {
    if(donor_id < 0 || donor_id >= NUM_SWARMS) return;
    if(target_id < 0 || target_id >= NUM_SWARMS) return;
    if(donor_id == target_id) return;

    sem_wait(&sem_reassign_line);
    sem_wait(&sem_swarms);

    if(swarms[donor_id].is_destroyed || swarms[target_id].is_destroyed) {
        sem_post(&sem_swarms);
        sem_post(&sem_reassign_line);
        return;
    }

    // target debe estar incompleto; donor debe estar incompleto (>0 y < ASSEMBLY_SIZE)
    int target_needs = (swarms[target_id].active_count < ASSEMBLY_SIZE);
    int donor_can_give = (swarms[donor_id].active_count > 0 &&
                          swarms[donor_id].active_count < ASSEMBLY_SIZE);

    if(!target_needs || !donor_can_give) {
        sem_post(&sem_swarms);
        sem_post(&sem_reassign_line);
        return;
    }

    int drone_id = 0;
    int donor_slot = -1;
    for(int j = 0; j < ASSEMBLY_SIZE; j++) {
        if(swarms[donor_id].drone_global_ids[j] != 0 &&
           swarms[donor_id].drone_terminated[j] == 0) {
            drone_id = swarms[donor_id].drone_global_ids[j];
            donor_slot = j;
            break;
        }
    }

    if(drone_id == 0) {
        sem_post(&sem_swarms);
        sem_post(&sem_reassign_line);
        return;
    }

    int target_slot = -1;
    for(int j = 0; j < ASSEMBLY_SIZE; j++) {
        if(swarms[target_id].drone_global_ids[j] == 0) {
            target_slot = j;
            break;
        }
    }

    if(target_slot == -1) {
        sem_post(&sem_swarms);
        sem_post(&sem_reassign_line);
        return;
    }

    // mover: quitar del donante
    swarms[donor_id].drone_global_ids[donor_slot] = 0;
    swarms[donor_id].drone_terminated[donor_slot] = 0;
    if(swarms[donor_id].active_count > 0) swarms[donor_id].active_count--;
    if(swarms[donor_id].active_count < ASSEMBLY_SIZE) {
        swarms[donor_id].assembled = 0;
    }

    // y agregar al objetivo
    swarms[target_id].drone_global_ids[target_slot] = drone_id;
    swarms[target_id].drone_terminated[target_slot] = 0;
    swarms[target_id].active_count++;
    if(swarms[target_id].active_count >= ASSEMBLY_SIZE) {
        swarms[target_id].assembled = 0; // permitirá un nuevo ensamblaje->TAKEOFF
    }

    // snapshot de coords del target para asegurarnos de RETARGET correcto
    double tx = swarms[target_id].target_x;
    double ty = swarms[target_id].target_y;
    int    tid = swarms[target_id].target_id;

    // ✅ CLAVE: Verificar si el donante ahora necesita reconformación
    int donor_now_needs_reassembly = swarm_needs_reassembly(donor_id);
    int donor_already_in_reassembly = swarms[donor_id].in_reassembly;

    printf("[CENTER] Reassigned drone %d from swarm %d (slot %d) to swarm %d (slot %d)\n",
           drone_id, donor_id, donor_slot, target_id, target_slot);

    sem_post(&sem_swarms);

    // Notificar BOTH trucks para evitar estados fantasmas:
    // 1) El donante sabe que cedió un dron
    msg_t cmd_d; memset(&cmd_d,0,sizeof(cmd_d));
    cmd_d.type = MSG_COMMAND;
    cmd_d.swarm_id = donor_id;
    snprintf(cmd_d.text, sizeof(cmd_d.text), "REASSIGN_ONE_TO %d", target_id);
    int donor_truck_port = port_for_truck(BASE_PORT, donor_id);
    send_msg(center_sock, donor_truck_port, &cmd_d);

    // 2) El receptor recibe/actualiza TARGET y fuerza retargeting del dron reasignado
    send_target_to_truck_coords(target_id, tx, ty, tid);

    // 3) Aviso directo al dron reasignado para que no "ataque" coordenadas antiguas
    int drone_port = port_for_drone(BASE_PORT, drone_id);
    msg_t cmd_dr; memset(&cmd_dr,0,sizeof(cmd_dr));
    cmd_dr.type = MSG_COMMAND;
    cmd_dr.swarm_id = target_id;
    cmd_dr.drone_id = drone_id;
    snprintf(cmd_dr.text,sizeof(cmd_dr.text),"RETARGET %.1f %.1f %d", tx, ty, tid);
    send_msg(center_sock, drone_port, &cmd_dr);

    // ✅ NUEVA LÓGICA: Si el donante ahora necesita reconformación, iniciarla
    if(donor_now_needs_reassembly && !donor_already_in_reassembly) {
        printf("[CENTER] Donor swarm %d now needs reassembly after donation\n", donor_id);
        start_reassembly_process(donor_id);
        // Intentar reconformar inmediatamente en un hilo separado o marcar para proceso posterior
    }

    sem_post(&sem_reassign_line);
}

// Recorre vecinos incrementales L/R para intentar completar el swarm objetivo
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

// Marca y envía autodestrucción de todos los drones del swarm tras timeout
void autodestruct_swarm(int swarm_id) {
    sem_wait(&sem_swarms);

    if(swarms[swarm_id].active_count <= 0 || swarms[swarm_id].is_destroyed) {
        swarms[swarm_id].in_reassembly = 0;
        swarms[swarm_id].reassembly_start = 0;
        sem_post(&sem_swarms);
        return;
    }

    printf("[CENTER] TIMEOUT: Swarm %d no pudo reconformarse en %ds - AUTODESTRUYENDO\n",
           swarm_id, MAX_WAIT_REASSEMBLY);

    swarms[swarm_id].is_destroyed = 1;
    swarms[swarm_id].in_reassembly = 0;
    swarms[swarm_id].reassembly_start = 0;
    swarms[swarm_id].assembled = 0;

    sem_post(&sem_swarms);

    autodestruct_swarm_drones(swarm_id);

    sem_wait(&sem_swarms);
    for(int j=0; j<ASSEMBLY_SIZE; j++) {
        if(swarms[swarm_id].drone_global_ids[j] != 0) {
            int did = swarms[swarm_id].drone_global_ids[j];
            swarms[swarm_id].drone_global_ids[j] = 0;
            swarms[swarm_id].drone_terminated[j] = 1;
            printf("[CENTER] Swarm %d autodestruye drone %d por timeout\n", swarm_id, did);
             // limpieza artillería inmediata
        }
    }
    swarms[swarm_id].active_count = 0;
    sem_post(&sem_swarms);
}

// ✅ NUEVA FUNCIÓN: Verifica si hay swarms donantes que ahora necesitan reconformación
void check_donor_swarms_for_reassembly() {
    for(int i = 0; i < NUM_SWARMS; i++) {
        sem_wait(&sem_swarms);
        int needs_reassembly = swarm_needs_reassembly(i);
        int already_in_reassembly = swarms[i].in_reassembly;
        sem_post(&sem_swarms);

        if(needs_reassembly && !already_in_reassembly) {
            printf("[CENTER] Detected donor swarm %d needs reassembly - starting process\n", i);
            start_reassembly_process(i);
            try_reconform_or_autodestruct(i);
        }
    }
}

// Revisa periódicamente timeouts SOLO si el swarm está efectivamente en reconformación
void check_reassembly_timeouts() {
    for(int i = 0; i < NUM_SWARMS; i++) {
        sem_wait(&sem_swarms);
        int is_incomplete = (swarms[i].active_count > 0 && swarms[i].active_count < ASSEMBLY_SIZE);
        int in_reassembly = swarms[i].in_reassembly;
        int is_destroyed = swarms[i].is_destroyed;
        time_t started = swarms[i].reassembly_start;
        sem_post(&sem_swarms);

        if(is_incomplete && !is_destroyed) {
            if(in_reassembly) {
                time_t elapsed = time(NULL) - started;
                int should_timeout = (elapsed >= (MAX_WAIT_REASSEMBLY + 2)); // margen de gracia
                if(should_timeout) {
                    autodestruct_swarm(i);
                } else {
                    // mientras no haya timeout, intenta reconformar si existen donantes incompletos
                    int can_try = 0;
                    sem_wait(&sem_swarms);
                    for(int k=0;k<NUM_SWARMS;k++){
                        if(k==i) continue;
                        if(!swarms[k].is_destroyed &&
                           swarms[k].active_count > 0 &&
                           swarms[k].active_count < ASSEMBLY_SIZE) { can_try=1; break; }
                    }
                    sem_post(&sem_swarms);
                    if(can_try) try_reconform_or_autodestruct(i);
                }
            } else {
                // ✅ El swarm está incompleto pero aún no ha declarado IN_REASSEMBLY
                // Esto puede pasar con swarms donantes - iniciar su proceso
                printf("[CENTER] Swarm %d is incomplete but not in reassembly - starting process\n", i);
                start_reassembly_process(i);
                try_reconform_or_autodestruct(i);
            }
        }
    }
    
    // ✅ Verificar swarms donantes que puedan necesitar reconformación
    check_donor_swarms_for_reassembly();
}

int all_drones_finished(){
    int finished = 1;
    sem_wait(&sem_swarms);
    for(int i=0;i<NUM_SWARMS;i++){
        if(swarms[i].active_count > 0){
            finished = 0;
            break;
        }
    }
    sem_post(&sem_swarms);
    return finished;
}

void try_reconform_or_autodestruct(int swarm_id) {
    sem_wait(&sem_swarms);
    if(swarms[swarm_id].is_destroyed) {
        sem_post(&sem_swarms);
        return;
    }
    sem_post(&sem_swarms);

    int can_reconform = 0;
    sem_wait(&sem_swarms);
    for(int i=0; i<NUM_SWARMS; i++) {
        if(i == swarm_id) continue;
        if(swarms[i].active_count > 0 && swarms[i].active_count < ASSEMBLY_SIZE && !swarms[i].is_destroyed) {
            can_reconform = 1;
            break;
        }
    }
    sem_post(&sem_swarms);

    if(can_reconform) {
        printf("[CENTER] Swarm %d intenta reconformarse...\n", swarm_id);
        reconform_from_neighbors(swarm_id);

        sem_wait(&sem_swarms);
        int completed = (swarms[swarm_id].active_count >= ASSEMBLY_SIZE && !swarms[swarm_id].is_destroyed);
        sem_post(&sem_swarms);

        if(completed) {
            complete_reassembly_process(swarm_id);
        }
    } else {
        printf("[CENTER] No hay enjambres disponibles para reconformar swarm %d - esperando timeout\n", swarm_id);
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
            if(!swarms[sid].is_destroyed) {
                int found = 0;
                for(int j=0;j<ASSEMBLY_SIZE;j++){
                    if(swarms[sid].drone_global_ids[j]==gid) { found=1; break; }
                }
                if(!found){
                    for(int j=0;j<ASSEMBLY_SIZE;j++){
                        if(swarms[sid].drone_global_ids[j]==0){
                            swarms[sid].drone_global_ids[j]=gid;
                            swarms[sid].drone_terminated[j]=0;
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
                sem_wait(&sem_swarms);
                int found_swarm = remove_drone_from_swarm_by_id(m.drone_id);
                if(found_swarm >= 0) {
                    printf("[CENTER] Drone %d del swarm %d terminado. Activos restantes: %d\n",
                           m.drone_id, found_swarm, swarms[found_swarm].active_count);
                }
                sem_post(&sem_swarms);
                 // limpieza en artillería (Error 6)
            }
            else if(strstr(m.text,"ARRIVED_DETONATED")){
                // Un dron llegó y detonó -> marcar blanco destruido del swarm correspondiente
                sem_wait(&sem_swarms);
                if(!swarms[m.swarm_id].is_destroyed) {
                    swarms[m.swarm_id].target_destroyed = 1;
                    remove_drone_from_swarm(m.swarm_id, m.drone_id);
                    printf("[CENTER] * BLANCO %d DESTRUIDO por drone %d *\n",
                           swarms[m.swarm_id].target_id, m.drone_id);
                }
                sem_post(&sem_swarms);
                
            }
          
else if(strstr(m.text,"CAMERA_REPORTED")){
        
    sem_wait(&sem_swarms);
    if(!swarms[m.swarm_id].is_destroyed && !swarms[m.swarm_id].camera_reported){
        swarms[m.swarm_id].camera_reported = 1;
        
        // Contar cuántos drones del enjambre llegaron efectivamente al blanco
        // (los que no fueron terminados antes de llegar)
       int drones_that_attacked = 5 - swarms[m.swarm_id].active_count;
        
        
        
        
        // Determinar estado del blanco basándose en efectividad del ataque
        const char* target_status_str;
        if(drones_that_attacked >= ASSEMBLY_SIZE-1) {
            target_status_str = "DESTRUIDO";           // Enjambre completo = destrucción total
        } else if(drones_that_attacked >= 2) {
            target_status_str = "PARCIALMENTE_DESTRUIDO"; // 2+ drones = daño parcial
        } else {
            target_status_str = "ENTERO";              // 1 drone = sin daño significativo
        }
        
        remove_drone_from_swarm(m.swarm_id, m.drone_id);
        sem_post(&sem_swarms);

        printf("[CENTER] * REPORTE DE CAMARA *\n");
        printf("[CENTER] * BLANCO %d: %s (%d drones atacaron) *\n",
               swarms[m.swarm_id].target_id, target_status_str, drones_that_attacked);
        
    } else {
        sem_post(&sem_swarms);
    }
}
            else if(strstr(m.text,"IN_ASSEMBLY")){
                sem_wait(&sem_swarms);
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
                        send_target_to_truck(m.swarm_id);

                        msg_t cmd; memset(&cmd,0,sizeof(cmd));
                        cmd.type = MSG_COMMAND;
                        cmd.swarm_id = m.swarm_id;
                        snprintf(cmd.text,sizeof(cmd.text),"TAKEOFF");
                        int truck_port = port_for_truck(BASE_PORT, m.swarm_id);
                        send_msg(center_sock, truck_port, &cmd);

                        sem_wait(&sem_swarms);
                        swarms[m.swarm_id].assembled = 2; // TAKEOFF enviado
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

    // Catálogo consistente de blancos (corrige Error #1)
    build_targets_catalog();

    spawn_trucks_and_drones();

    pthread_t lt;
    pthread_create(&lt,NULL,listener_thread,NULL);

    while(1){
        sleep(1);

        check_reassembly_timeouts();

        static int status_counter = 0;
        if(++status_counter >= 5) {
            print_status();
            status_counter = 0;
        }

        if(all_drones_finished()){
            printf("[CENTER] Todos los drones terminaron. Enviando señal de terminación a artillería...\n");
            msg_t term_msg; memset(&term_msg,0,sizeof(term_msg));
            term_msg.type = MSG_ARTILLERY;
            snprintf(term_msg.text,sizeof(term_msg.text),"TERMINATE");
            int artillery_port = port_for_artillery(BASE_PORT);
            send_msg(center_sock, artillery_port, &term_msg);
            sleep(1);
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
