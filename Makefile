CC = gcc 
CFLAGS = -g -O -Wall -I/usr/local/include/hiredis
LIB = -lhiredis -ldsm -lpthread -lrt -lm

OBSBIN = ./

SOURCES = getRadiometerData.c 

OBJECTS = getRadiometerData.o

all: getRadiometerData

clean:
	rm *.o ./getRadiometerData

install: all
	cp getRadiometerData $(OBSBIN)/

getRadiometerData: $(OBJECTS) ./Makefile
	$(CC) -o getRadiometerData $(OBJECTS) $(LIB)

depend: ./Makefile
	$(CC) -MM $(SOURCES) > dependencies

include dependencies
