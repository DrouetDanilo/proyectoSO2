CC=gcc
CFLAGS=-Wall -g
LIBS=-lpthread -lm -lrt

OBJS=common.o control_center.o truck.o drone.o artillery.o

all: control_center truck drone artillery

common.o: common.c common.h
	$(CC) $(CFLAGS) -c common.c

control_center: control_center.c common.o
	$(CC) $(CFLAGS) -o control_center control_center.c common.o $(LIBS)

truck: truck.c common.o
	$(CC) $(CFLAGS) -o truck truck.c common.o $(LIBS)

drone: drone.c common.o
	$(CC) $(CFLAGS) -o drone drone.c common.o $(LIBS)

artillery: artillery.c common.o
	$(CC) $(CFLAGS) -o artillery artillery.c common.o $(LIBS)

clean:
	rm -f *.o control_center truck drone artillery

