// drone.c (con coordenadas, semaforos y velocidad configurable desde params.txt)
#include "common.h"
#include <math.h>
#include <semaphore.h>

int BASE_PORT = 40000;
int Q = 5;   // prob. pérdida de enlace
int Z = 5;   // ventana de recuperación
int W = 30;  // prob. derribo (artillería)
int ASSEMBLY_SIZE = 5;

// Zonas (eje X)
double B = 20.0;   // fin zona ensamble
double A = 50.0;   // fin zona defensa / inicio re-ensamble
double C = 100.0;  // blanco

// Identidad / red
int global_id;
int swarm_id;
int sock;
int center_port;

// Estado compartido
volatile int have_link = 1;
volatile int fuel_percent = 100;
volatile int detonated = 0;
volatile int reassigned = 0;
volatile int is_camera = 0;

// Semáforos
sem_t sem_state;    // binario -> mutex
sem_t sem_takeoff;  // 0 hasta que el centro ordene despegar

pthread_t fuel_thread, weapon_thread, nav_thread;

// Coordenadas y movimiento
double x=0.0, y=0.0;
double vx=10.0;        // velocidad en X (u/seg)
double theta=0.0;     // ángulo para orbitar
double r=5.0;         // radio órbita
double theta_step=0.3; // paso angular (rad/seg)

static inline void state_lock()   { sem_wait(&sem_state); }
static inline void state_unlock() { sem_post(&sem_state); }

void send_status(const char *txt){
    msg_t m; memset(&m,0,sizeof(m));
    m.type = MSG_STATUS;
    m.swarm_id = swarm_id;
    m.drone_id = global_id;
    strncpy(m.text, txt, sizeof(m.text)-1);
    send_msg(sock, center_port, &m);
}

void send_pos(){
    msg_t m; memset(&m,0,sizeof(m));
    m.type = MSG_STATUS;
    m.swarm_id = swarm_id;
    m.drone_id = global_id;
    snprintf(m.text, sizeof(m.text), "POS %.1f %.1f", x, y);
    send_msg(sock, center_port, &m);
}

int is_detonated(){
    int v;
    state_lock(); v = detonated; state_unlock();
    return v;
}
void set_detonated(){
    state_lock(); detonated = 1; state_unlock();
}

void *fuel_control(void *arg){
    (void)arg;
    while(1){
        sleep(1);
        state_lock();
        if(detonated){ state_unlock(); break; }
        fuel_percent -= 1;
        int fp = fuel_percent;
        state_unlock();

        if(fp <= 0){
            send_status("FUEL_ZERO_AUTODESTRUCT");
            set_detonated();
            exit(0);
        }
    }
    return NULL;
}

void *weapon_or_camera(void *arg){
    (void)arg;
    // marca de cámara (ejemplo: id 5 de cada bloque de 100)
    state_lock();
    if(global_id % 100 == 5) is_camera = 1;
    state_unlock();

    while(1){
        sleep(1);
        if(is_detonated()) break;
    }
    return NULL;
}

void *simulate_flight(void *arg){
    (void)arg;

    // 1) Vuelo hasta zona de ensamble (orbitar)
    // Espera a TAKEOFF con semáforo (centro hace sem_post)
    while(1){
        if(is_detonated()) return NULL;

        // Orbitar en torno a (B,0)
        state_lock();
        theta += theta_step;
        x = B + r*cos(theta);
        y = r*sin(theta);
        state_unlock();

        send_status("IN_ASSEMBLY");
        send_pos(); // <--- AGREGADO
        // Espera TAKEOFF (tiempo corto para no bloquear totalmente)
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000; // 0.1s
        if(ts.tv_nsec >= 1000000000){ ts.tv_sec++; ts.tv_nsec-=1000000000; }
        if(sem_timedwait(&sem_takeoff, &ts) == 0){
            send_status("TAKEOFF_RECEIVED");
            break; // salir de órbita y avanzar
        }
    }

    // 2) Avance recto: B -> A (defensa)
    int entered_defense = 0;
    while(1){
        if(is_detonated()) return NULL;
        sleep(1);

        // Mover recto en X
        state_lock();
        y = 0;
        x += vx;
        double locx = x;
        state_unlock();

        send_pos(); // <--- AGREGADO

        if(!entered_defense && locx >= B){
            entered_defense = 1;
            send_status("ENTERING_DEFENSE");
            // Notificar a artillería
            msg_t art; memset(&art,0,sizeof(art));
            art.type = MSG_ARTILLERY;
            art.swarm_id = swarm_id;
            art.drone_id = global_id;
            snprintf(art.text,sizeof(art.text),"ENTERING_DEFENSE %d", global_id);
            send_msg(sock, port_for_artillery(BASE_PORT), &art);
        }

        // Pérdida de enlace dentro de B->A
        if(locx >= B && locx < A){
            if(rand()%100 < Q){
                state_lock(); have_link = 0; state_unlock();
                send_status("LOST_LINK");
                int recovered = 0;
                for(int w=0;w<Z;w++){
                    sleep(1);
                    if(rand()%100 < 50){ recovered = 1; break; }
                }
                if(!recovered){
                    send_status("LINK_PERMANENT_LOSS");
                    set_detonated();
                    exit(0);
                } else {
                    state_lock(); have_link = 1; state_unlock();
                    send_status("LINK_RESTORED");
                }
            }
        }

        if(locx >= A) break;
    }

    // 3) Re-ensamblaje: A -> C
    int announced_reassembly = 0;
    while(1){
        if(is_detonated()) return NULL;
        sleep(1);

        state_lock();
        y = 0;
        x += vx;
        double locx = x;
        int cam = is_camera;
        state_unlock();

        send_pos(); // <--- AGREGADO

        if(!announced_reassembly && locx >= A){
            announced_reassembly = 1;
            send_status("IN_REASSEMBLY");
        }

        // Llegó al blanco
        if(locx >= C){
            if(cam){
                send_status("CAMERA_REPORTED");
                send_status("CAMERA_AUTODESTRUCT");
            } else {
                send_status("ARRIVED_DETONATED");
            }
            set_detonated();
            exit(0);
        }
    }
    return NULL;
}

void handle_command(msg_t *m){
    if(strcmp(m->text,"TAKEOFF")==0){
        sem_post(&sem_takeoff);
    }
    else if(strncmp(m->text,"GO_TO_SWARM",11)==0){
        int target = -1;
        if(sscanf(m->text+11,"%d",&target)==1){
            state_lock();
            swarm_id = target;
            reassigned = 1;
            state_unlock();
            send_status("REASSIGNED");
        }
    }
}

int main(int argc, char **argv){
    if(argc<4){ fprintf(stderr,"Usage: drone params.txt <global_id> <truck_id>\n"); exit(1); }
    char *params = argv[1];
    global_id = atoi(argv[2]);
    int truck_id = atoi(argv[3]);
    swarm_id = truck_id;

    // Cargar parámetros
    FILE *f = fopen(params,"r");
    if(f){
        char line[200];
        while(fgets(line,sizeof(line),f)){
            if(line[0]=='#') continue;
            char key[80]; double dval; int val;
            if(sscanf(line,"%[^=]=%lf",key,&dval)==2){
                if(strcmp(key,"VX")==0) vx = dval;
                if(strcmp(key,"R")==0) r = dval;
                if(strcmp(key,"THETA_STEP")==0) theta_step = dval;
            } else if(sscanf(line,"%[^=]=%d",key,&val)==2){
                if(strcmp(key,"BASE_PORT")==0) BASE_PORT=val;
                if(strcmp(key,"Q")==0) Q=val;
                if(strcmp(key,"Z")==0) Z=val;
                if(strcmp(key,"W")==0) W=val;
                if(strcmp(key,"ASSEMBLY_SIZE")==0) ASSEMBLY_SIZE=val;
            }
        }
        fclose(f);
    }

    // Inicializar semáforos
    sem_init(&sem_state, 0, 1);
    sem_init(&sem_takeoff, 0, 0);

    center_port = port_for_center(BASE_PORT);
    sock = make_udp_socket();

    // bind a puerto del dron
    int dport = port_for_drone(BASE_PORT, global_id);
    struct sockaddr_in addr; memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(HOST);
    addr.sin_port = htons(dport);
    if(bind(sock,(struct sockaddr*)&addr,sizeof(addr))<0){
        perror("bind drone");
        exit(1);
    }

    // HELLO inicial
    msg_t hello; memset(&hello,0,sizeof(hello));
    hello.type = MSG_HELLO;
    hello.swarm_id = swarm_id;
    hello.drone_id = global_id;
    snprintf(hello.text,sizeof(hello.text),"DRONE_HELLO %d", global_id);
    send_msg(sock, center_port, &hello);

    srand(time(NULL) ^ global_id);

    pthread_create(&fuel_thread,NULL,fuel_control,NULL);
    pthread_create(&weapon_thread,NULL,weapon_or_camera,NULL);
    pthread_create(&nav_thread,NULL,simulate_flight,NULL);

    // Bucle de recepción
    msg_t rcv; struct sockaddr_in from;
    while(1){
        if(recv_msg(sock,&rcv,&from)<=0){ usleep(100000); continue; }
        if(rcv.type==MSG_COMMAND){
            handle_command(&rcv);
        } else if(rcv.type==MSG_ARTILLERY){
            if(strstr(rcv.text,"HIT")){
                send_status("SHOT_DOWN_BY_ARTILLERY");
                set_detonated();
                exit(0);
            }
        }
    }
    return 0;
}

