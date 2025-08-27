// artillery.c - Sistema de defensa anti-drone
#include "common.h"
#include <semaphore.h>
#include <math.h>

#define MAX_DRONES_TRACKED 1000

typedef struct {
    int global_id;
    int swarm_id;
    double x, y;
    int in_defense_zone;
    int active;
    time_t last_update;
} tracked_drone_t;

// Parámetros del sistema
int BASE_PORT = 40000;
int W = 30;  // Probabilidad de derribo (%)
int NUM_TARGETS = 2;
int ARTILLERY_RATE = 2; // Segundos entre disparos

// Zonas de defensa
double B = 20.0;   // Inicio zona de defensa
double A = 50.0;   // Fin zona de defensa

// Estado del sistema
tracked_drone_t drones[MAX_DRONES_TRACKED];
int num_tracked = 0;
int artillery_sock;
int center_port;
sem_t sem_tracking;

void load_params(const char *path) {
    FILE *f = fopen(path, "r");
    if(!f) { 
        printf("[ARTILLERY] No se pudo abrir %s, usando valores por defecto\n", path);
        return; 
    }
    
    char line[200];
    while(fgets(line, sizeof(line), f)) {
        if(line[0] == '#' || strlen(line) < 3) continue;
        
        char key[80];
        int val;
        double dval;
        
        if(sscanf(line, "%[^=]=%d", key, &val) == 2) {
            if(strcmp(key, "BASE_PORT") == 0) BASE_PORT = val;
            else if(strcmp(key, "W") == 0) W = val;
            else if(strcmp(key, "NUM_TARGETS") == 0) NUM_TARGETS = val;
            else if(strcmp(key, "ARTILLERY_RATE") == 0) ARTILLERY_RATE = val;
        }
        else if(sscanf(line, "%[^=]=%lf", key, &dval) == 2) {
            if(strcmp(key, "B") == 0) B = dval;
            else if(strcmp(key, "A") == 0) A = dval;
        }
    }
    fclose(f);
    
    printf("[ARTILLERY] Parámetros cargados: W=%d%%, B=%.1f, A=%.1f\n", W, B, A);
}

void notify_center_hit(int drone_id, int swarm_id) {
    msg_t hit_msg;
    memset(&hit_msg, 0, sizeof(hit_msg));
    hit_msg.type = MSG_ARTILLERY;
    hit_msg.swarm_id = swarm_id;
    hit_msg.drone_id = drone_id;
    snprintf(hit_msg.text, sizeof(hit_msg.text), "DRONE %d SHOT_DOWN", drone_id);
    
    send_msg(artillery_sock, center_port, &hit_msg);
    printf("[ARTILLERY] *** IMPACTO *** Drone %d (swarm %d) derribado!\n", drone_id, swarm_id);
}

void notify_drone_hit(int drone_id) {
    msg_t hit_msg;
    memset(&hit_msg, 0, sizeof(hit_msg));
    hit_msg.type = MSG_ARTILLERY;
    hit_msg.drone_id = drone_id;
    snprintf(hit_msg.text, sizeof(hit_msg.text), "HIT");
    
    int drone_port = port_for_drone(BASE_PORT, drone_id);
    send_msg(artillery_sock, drone_port, &hit_msg);
}

tracked_drone_t* find_drone(int drone_id) {
    for(int i = 0; i < num_tracked; i++) {
        if(drones[i].global_id == drone_id && drones[i].active) {
            return &drones[i];
        }
    }
    return NULL;
}

tracked_drone_t* add_drone(int drone_id, int swarm_id) {
    if(num_tracked >= MAX_DRONES_TRACKED) return NULL;
    
    drones[num_tracked].global_id = drone_id;
    drones[num_tracked].swarm_id = swarm_id;
    drones[num_tracked].x = 0.0;
    drones[num_tracked].y = 0.0;
    drones[num_tracked].in_defense_zone = 0;
    drones[num_tracked].active = 1;
    drones[num_tracked].last_update = time(NULL);
    
    return &drones[num_tracked++];
}

void update_drone_position(int drone_id, int swarm_id, double x, double y) {
    sem_wait(&sem_tracking);
    
    tracked_drone_t* drone = find_drone(drone_id);
    if(!drone) {
        drone = add_drone(drone_id, swarm_id);
        if(!drone) {
            sem_post(&sem_tracking);
            return;
        }
        printf("[ARTILLERY] Rastreando nuevo drone %d (swarm %d)\n", drone_id, swarm_id);
    }
    
    drone->x = x;
    drone->y = y;
    drone->swarm_id = swarm_id; // actualizar swarm en caso de reconformación
    drone->last_update = time(NULL);
    
    // Verificar si entró en zona de defensa
    int was_in_defense = drone->in_defense_zone;
    int now_in_defense = (x >= B && x <= A);
    
    if(!was_in_defense && now_in_defense) {
        drone->in_defense_zone = 1;
        printf("[ARTILLERY] Drone %d entró en zona de defensa (%.1f, %.1f)\n", 
               drone_id, x, y);
    }
    else if(was_in_defense && !now_in_defense) {
        drone->in_defense_zone = 0;
        if(x > A) {
            printf("[ARTILLERY] Drone %d salió de zona de defensa\n", drone_id);
        }
    }
    
    sem_post(&sem_tracking);
}

void artillery_engagement_cycle() {
    sem_wait(&sem_tracking);
    
    time_t now = time(NULL);
    
    for(int i = 0; i < num_tracked; i++) {
        if(!drones[i].active || !drones[i].in_defense_zone) continue;
        
        // Verificar si el drone sigue activo (timeout de 10 segundos)
        if(now - drones[i].last_update > 10) {
            printf("[ARTILLERY] Drone %d timeout, removiendo del tracking\n", 
                   drones[i].global_id);
            drones[i].active = 0;
            continue;
        }
        
        // Intentar disparo con probabilidad W%
        if(rand() % 100 < W) {
            printf("[ARTILLERY] ¡DISPARANDO contra drone %d!\n", drones[i].global_id);
            
            // Notificar al centro de control
            notify_center_hit(drones[i].global_id, drones[i].swarm_id);
            
            // Notificar directamente al drone
            notify_drone_hit(drones[i].global_id);
            
            // Marcar como destruido
            drones[i].active = 0;
        }
    }
    
    sem_post(&sem_tracking);
}

void print_artillery_status() {
    sem_wait(&sem_tracking);
    
    int active_count = 0;
    int in_defense_count = 0;
    
    printf("=== ARTILLERY STATUS ===\n");
    for(int i = 0; i < num_tracked; i++) {
        if(drones[i].active) {
            active_count++;
            if(drones[i].in_defense_zone) in_defense_count++;
            
            printf("Drone %d (S%d): (%.1f,%.1f) %s\n", 
                   drones[i].global_id, 
                   drones[i].swarm_id,
                   drones[i].x, 
                   drones[i].y,
                   drones[i].in_defense_zone ? "[EN ZONA DEFENSA]" : "");
        }
    }
    printf("Total activos: %d, En zona defensa: %d\n", active_count, in_defense_count);
    
    sem_post(&sem_tracking);
}

void mark_drone_dead(int drone_id) {
    sem_wait(&sem_tracking);
    for(int i = 0; i < num_tracked; i++) {
        if(drones[i].global_id == drone_id) {
            drones[i].active = 0;
            printf("[ARTILLERY] Drone %d eliminado del tracking\n", drone_id);
        }
    }
    sem_post(&sem_tracking);
}

void* listener_thread(void* arg) {
    (void)arg;
    
    struct sockaddr_in from;
    msg_t m;
    
    while(1) {
        if(recv_msg(artillery_sock, &m, &from) <= 0) {
            usleep(50000); // 50ms
            continue;
        }
        
        if(m.type == MSG_STATUS) {
            // Procesar mensajes de posición: "POS x y"
            if(strncmp(m.text, "POS ", 4) == 0) {
                double x, y;
                if(sscanf(m.text + 4, "%lf %lf", &x, &y) == 2) {
                    update_drone_position(m.drone_id, m.swarm_id, x, y);
                }
            }
            else if(strstr(m.text, "ARRIVED_DETONATED") ||
                    strstr(m.text, "CAMERA_AUTODESTRUCT")) {
                mark_drone_dead(m.drone_id);
            }
        }
        else if(m.type == MSG_ARTILLERY) {
            if(strstr(m.text, "TERMINATE")) {
                printf("[ARTILLERY] Recibido TERMINATE. Finalizando sistema de artillería...\n");
                exit(0);
            }
            else if(strstr(m.text, "SHOT_DOWN")) {
                mark_drone_dead(m.drone_id);
            }
            else if(strstr(m.text, "ENTERING_DEFENSE")) {
                printf("[ARTILLERY] Drone %d reportó entrada en zona de defensa\n", m.drone_id);
            }
            else if(strstr(m.text, "TRUCK_READY")) {
                printf("[ARTILLERY] %s\n", m.text);
            }
            else if(strncmp(m.text, "REASSIGN", 8) == 0) {
                int drone_id, new_swarm;
                if(sscanf(m.text, "REASSIGN %d %d", &drone_id, &new_swarm) == 2) {
                    sem_wait(&sem_tracking);
                    tracked_drone_t* d = find_drone(drone_id);
                    if(d) {
                        d->swarm_id = new_swarm;
                        printf("[ARTILLERY] Drone %d reasignado a swarm %d\n", drone_id, new_swarm);
                    }
                    sem_post(&sem_tracking);
                }
            }
        }
    }
    return NULL;
}

void* engagement_thread(void* arg) {
    (void)arg;
    
    while(1) {
        sleep(ARTILLERY_RATE);
        artillery_engagement_cycle();
    }
    
    return NULL;
}

int main(int argc, char** argv) {
    if(argc < 2) {
        printf("Uso: artillery params.txt\n");
        exit(1);
    }
    
    // Inicializar semáforos
    sem_init(&sem_tracking, 0, 1);
    
    // Cargar parámetros
    load_params(argv[1]);
    
    // Inicializar red
    artillery_sock = make_udp_socket();
    center_port = port_for_center(BASE_PORT);
    
    int artillery_port = port_for_artillery(BASE_PORT);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(HOST);
    addr.sin_port = htons(artillery_port);
    
    if(bind(artillery_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind artillery");
        exit(1);
    }
    
    printf("[ARTILLERY] Sistema iniciado en puerto %d\n", artillery_port);
    printf("[ARTILLERY] Zona de defensa: %.1f <= X <= %.1f\n", B, A);
    printf("[ARTILLERY] Probabilidad de derribo: %d%%\n", W);
    
    srand(time(NULL));
    
    // Crear hilos
    pthread_t lt, et;
    pthread_create(&lt, NULL, listener_thread, NULL);
    pthread_create(&et, NULL, engagement_thread, NULL);
    
    // Bucle principal con información de estado
    while(1) {
        sleep(10);
        print_artillery_status();
    }
    
    // Cleanup
    pthread_cancel(lt);
    pthread_cancel(et);
    pthread_join(lt, NULL);
    pthread_join(et, NULL);
    
    sem_destroy(&sem_tracking);
    close(artillery_sock);
    
    return 0;
}
