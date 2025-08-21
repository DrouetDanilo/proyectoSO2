CC=gcc
CFLAGS=-Wall -pthread -lm -lrt
TARGETS=control_center truck drone artillery

all: $(TARGETS)

control_center: control_center.c common.o
	$(CC) -o $@ $^ $(CFLAGS)

truck: truck.c common.o
	$(CC) -o $@ $^ $(CFLAGS)

drone: drone.c common.o
	$(CC) -o $@ $^ $(CFLAGS)

artillery: artillery.c common.o
	$(CC) -o $@ $^ $(CFLAGS)

common.o: common.c common.h
	$(CC) -c common.c $(CFLAGS)

clean:
	rm -f $(TARGETS) *.o

run: all
	@echo "=== Iniciando simulador de drones ==="
	@echo "1. Iniciando sistema de artillería..."
	./artillery params.txt &
	@sleep 2
	@echo "2. Iniciando centro de control..."
	./control_center params.txt
	@echo "=== Simulación terminada ==="

stop:
	@echo "Deteniendo todos los procesos..."
	pkill -f "artillery"
	pkill -f "control_center"
	pkill -f "truck"
	pkill -f "drone"

.PHONY: all clean run stop
