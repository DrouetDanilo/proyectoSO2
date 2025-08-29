// truck.c - VERSIÓN CORREGIDA
#include "common.h"
#include <sys/wait.h>  // ✅ AGREGADO: Para waitpid()
#include <signal.h>    // ✅ AGREGADO: Para signal handling

// truck <params_path> <truck_id>
int BASE_PORT = 40000;
int ASSEMBLY_SIZE = 5;
char *params_path;

// Coordenadas del blanco asignado
double target_x = 100.0;
double target_y = 0.0;
int target_id = 0;
int target_sent = 0;      // Flag para evitar enviar múltiples veces
int takeoff_sent = 0;     // Flag para evitar enviar múltiples veces

// ✅ NUEVO: Contador de drones vivos para debugging
int drones_alive = 0;

// ✅ NUEVO: Handler para recoger procesos zombie
void sigchld_handler(int sig) {
    (void)sig;
    pid_t pid;
    int status;
    
    // Recoger todos los procesos hijos que hayan terminado
    while((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        drones_alive--;
    }
}

int main(int argc, char **argv){
    if(argc<3){ fprintf(stderr,"Usage: truck params.txt <truck_id>\n"); exit(1); }
    params_path = argv[1];
    int truck_id = atoi(argv[2]);

    // ✅ NUEVO: Configurar handler para SIGCHLD ANTES de hacer fork()
    signal(SIGCHLD, sigchld_handler);
    printf("[TRUCK %d] Handler SIGCHLD configurado\n", truck_id);

    // read base port from params quickly (minimal parsing)
    FILE *f = fopen(params_path,"r");
    if(f){
        char line[200];
        while(fgets(line,sizeof(line),f)){
            if(line[0]=='#') continue;
            char key[80]; int val; double dval;
            if(sscanf(line,"%[^=]=%d",key,&val)==2){
                if(strcmp(key,"BASE_PORT")==0) BASE_PORT=val;
                if(strcmp(key,"ASSEMBLY_SIZE")==0) ASSEMBLY_SIZE=val;
            }
            else if(sscanf(line,"%[^=]=%lf",key,&dval)==2){
                if(strcmp(key,"C")==0) target_x=dval;
            }
        }
        fclose(f);
    }

    int truck_port = port_for_truck(BASE_PORT, truck_id);
    int center_port = port_for_center(BASE_PORT);
    int sock = make_udp_socket();

    // bind antes de lanzar drones
    struct sockaddr_in addr; memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(HOST);
    addr.sin_port = htons(truck_port);
    printf("[TRUCK %d] intentando bind en puerto %d\n", truck_id, truck_port);
    if(bind(sock,(struct sockaddr*)&addr,sizeof(addr))<0){
        perror("bind truck");
        exit(1);
    }
    printf("[TRUCK %d] iniciado puerto %d\n", truck_id, truck_port);

    // announce truck ready
    msg_t m; memset(&m,0,sizeof(m));
    m.type = MSG_ARTILLERY;
    m.truck_id = truck_id;
    snprintf(m.text,sizeof(m.text),"TRUCK_READY %d", truck_id);
    send_msg(sock, center_port, &m);

    // spawn ASSEMBLY_SIZE drones
    printf("[TRUCK %d] Spawning %d drones...\n", truck_id, ASSEMBLY_SIZE);
    for(int i=0;i<ASSEMBLY_SIZE;i++){
        pid_t pid = fork();
        if(pid==0){
            // PROCESO HIJO (DRONE)
            char gid_s[16], ppath[256], tid[16];
            int global_id = truck_id * 100 + i + 1; // global unique (simple)
            snprintf(gid_s,sizeof(gid_s),"%d", global_id);
            snprintf(ppath,sizeof(ppath),"%s",params_path);
            snprintf(tid,sizeof(tid),"%d",truck_id);
            execl("./drone","drone", ppath, gid_s, tid, (char*)NULL);
            perror("execl drone");
            exit(1);
        } else if(pid > 0) {
            // PROCESO PADRE (TRUCK)
            drones_alive++;
            printf("[TRUCK %d] ✅ Drone %d spawned con PID %d (total vivos: %d)\n", 
                   truck_id, truck_id*100 + i + 1, pid, drones_alive);
        } else {
            perror("fork drone");
        }
    }

    printf("[TRUCK %d] Todos los drones spawned. Esperando comandos...\n", truck_id);

    // ✅ MEJORA: Agregar timeout para evitar bloqueo indefinido
    struct timeval timeout;
    timeout.tv_sec = 1;  // 1 segundo de timeout
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // truck listens for commands from center (e.g., REASSIGN_ONE_TO, TARGET)
    msg_t rcv; struct sockaddr_in from;
    int loop_count = 0;
    
    while(1){
        int recv_result = recv_msg(sock, &rcv, &from);
        
        // ✅ NUEVO: Mostrar estado cada cierto tiempo
        if(++loop_count % 1000 == 0) {
            
        }
        
        // ✅ NUEVO: Salir si no quedan drones vivos (opcional)
        if(drones_alive <= 0 && loop_count > 10) {
            
            // Descomentar la siguiente línea si quieres que el truck termine automáticamente:
             break;
        }
        
        if(recv_result <= 0) { 
            usleep(1000); 
            continue; 
        }
        
        if(rcv.type==MSG_COMMAND){
            printf("[TRUCK %d] CMD: %s\n",truck_id, rcv.text);
            
            if(strncmp(rcv.text,"TARGET",6)==0){
                // Recibir coordenadas del blanco: "TARGET x y id"
                if(sscanf(rcv.text,"TARGET %lf %lf %d", &target_x, &target_y, &target_id) == 3 && !target_sent){
                    printf("[TRUCK %d] Blanco asignado: ID=%d, Pos=(%.1f, %.1f)\n", 
                           truck_id, target_id, target_x, target_y);
                    
                    // Enviar coordenadas del blanco a todos los drones (solo una vez)
                    for(int i=0;i<ASSEMBLY_SIZE;i++){
                        int gid = truck_id*100 + i + 1;
                        int dport = port_for_drone(BASE_PORT, gid);
                        msg_t cmd; memset(&cmd,0,sizeof(cmd));
                        cmd.type = MSG_COMMAND;
                        cmd.swarm_id = truck_id;
                        cmd.drone_id = gid;
                        snprintf(cmd.text,sizeof(cmd.text),"TARGET %.1f %.1f %d", target_x, target_y, target_id);
                        send_msg(sock, dport, &cmd);
                        printf("[TRUCK %d] Enviado TARGET a drone %d\n", truck_id, gid);
                    }
                    target_sent = 1; // Marcar como enviado
                }
            }
            else if(strncmp(rcv.text,"REASSIGN_ONE_TO",15)==0){
                printf("[TRUCK %d] Procesando REASSIGN_ONE_TO...\n", truck_id);
                
                // ✅ MEJORA: Solo enviar a drones existentes, no broadcast masivo
                for(int i=0;i<ASSEMBLY_SIZE;i++){
                    int gid = truck_id*100 + i + 1;
                    int dport = port_for_drone(BASE_PORT, gid);
                    msg_t cmd; memset(&cmd,0,sizeof(cmd));
                    cmd.type = MSG_COMMAND;
                    cmd.swarm_id = -1;
                    cmd.drone_id = gid;
                    snprintf(cmd.text,sizeof(cmd.text),"GO_TO_SWARM %s", rcv.text + 16);
                    send_msg(sock, dport, &cmd);
                }
            } 
            else if(strncmp(rcv.text,"TAKEOFF",7)==0){
                // broadcast TAKEOFF to all drones of this truck (solo una vez)
                if(!takeoff_sent){
                    printf("[TRUCK %d] Procesando TAKEOFF...\n", truck_id);
                    for(int i=0;i<ASSEMBLY_SIZE;i++){
                        int gid = truck_id*100 + i + 1;
                        int dport = port_for_drone(BASE_PORT, gid);
                        msg_t cmd; memset(&cmd,0,sizeof(cmd));
                        cmd.type = MSG_COMMAND;
                        cmd.swarm_id = truck_id;
                        cmd.drone_id = gid;
                        snprintf(cmd.text,sizeof(cmd.text),"TAKEOFF");
                        printf("[TRUCK %d] Enviando TAKEOFF a drone %d (puerto %d)\n", truck_id, gid, dport);
                        send_msg(sock, dport, &cmd);
                    }
                    takeoff_sent = 1; // Marcar como enviado
                }
            }
            // ✅ NUEVO: Manejar comando de autodestrucción
            else if(strncmp(rcv.text,"AUTODESTRUCT_ALL",16)==0){
                printf("[TRUCK %d] ⚠️  Procesando AUTODESTRUCT_ALL...\n", truck_id);
                // El center ya envió el comando directamente a los drones
                // El truck solo necesita estar preparado para recoger los procesos
            }
        }
    }
    
    printf("[TRUCK %d] terminado\n", truck_id);
    close(sock);
    return 0;
}
