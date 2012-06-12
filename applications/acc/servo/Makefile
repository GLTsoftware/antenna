# Makefile for Servo.
#
# CVS Log and RCS Identification of this version:
# $Id: Makefile,v 1.62 2011/03/04 19:22:48 rwilson Exp $
# $Log: Makefile,v $
# Revision 1.62  2011/03/04 19:22:48  rwilson
# Add poundscb
#
# Revision 1.61  2008/08/14 13:53:20  rwilson
# Add createtsshm.c to SOURCES
#
# Revision 1.60  2008/05/09 18:44:06  rwilson
# Add dump of previous state on unexpected drive shutdown
#
# Revision 1.59  2007/01/22 19:29:01  rwilson
# put in rule for s_anti96com.o
#
# Revision 1.58  2006/06/15 19:30:18  rwilson
# Add GLOBALINC for making depend
#
# Revision 1.57  2006/06/13 17:45:59  rwilson
# Got rid of dersFaultNames
#

NEWSERVO=$(ACCAPP)/newServo/src
BINDIR=../bin
CC=gcc
#CFLAGS=-c -g -Wall -Iinclude
CFLAGS=-c -O -Wall -Iinclude
VMESG=$(COMMON)/vme_sg_simple
IPEINC=$(ACCAPP)/iPEncoder/include
NSINC=$(NEWSERVO)/include
MONINC=$(HALAPP)/cursesmonitor
ANTPAGE2=/usr/PowerPC/applications/hal/cursesmonitor/antPage2.o
DSMLD= $(COMMONLIB)/libdsm.a -lrpc -lnetinet
NSINCLUDES=$(NSINC)/s_cmd2.h $(NSINC)/s_anti96com.h $(NSINC)/s_constants.h \
	$(NSINC)/palmpilot.h
#SOURCES=servo.c svdata.c dmytrack.c tsshmdump.c rmtsshm.c tracking_err.c \
#	readtt.c rmvalue.c drivemon.c rmdrivemon.c rmsubs.c checkencoders.c \
#	readencoders.c monitortt.c ttalarmcheck.c recordencoders.c \
#	checktrcmds.c readsc32.c $(NEWSERVO)/s_anti96com.c createtsshm.c
#BINS=$(BINDIR)/servo $(BINDIR)/svdata $(BINDIR)/dmytrack \
#	$(BINDIR)/tsshmdump $(BINDIR)/rmtsshm $(BINDIR)/createtsshm \
#	$(BINDIR)/tracking_err $(BINDIR)/readtt  $(BINDIR)/readsc32 \
#	$(BINDIR)/rmvalue $(BINDIR)/drivemon $(COMMONBIN)/rmdrivemon \
#	$(BINDIR)/monitortt $(BINDIR)/ttalarmcheck \
#	$(BINDIR)/recordencoders $(BINDIR)/ttread $(BINDIR)/sampleEncoders

#	$(BINDIR)/dmyservo $(BINDIR)/servotodd $(BINDIR)/slowservo

SOURCES=createtsshm.c drivemon.c readtt.c openshm.c rmtsshm.c checktrcmds.c \
	dmytrack.c checkencoders.c svdata.c canbus.c heartbeat.c servo.c
BINS=$(BINDIR)/createtsshm $(BINDIR)/drivemon $(BINDIR)/readtt \
	$(BINDIR)/rmtsshm $(BINDIR)/dmytrack $(BINDIR)/svdata

all: $(BINS) $(COMMONINC)/tsshm.h
#	/application/bin/settrackservo

$(BINDIR)/servo: servo.o openshm.o canbus.o heartbeat.o\
	checkencoders.o readencoders.o checktrcmds.o\
	$(COMMONLIB)/commonLib
	$(CC) -o $(BINDIR)/servo servo.o openshm.o\
		$(NEWSERVO)/s_anti96com.o $(COMMONLIB)/rm.o\
		rmsubs.o $(COMMONLIB)/commonLib checkencoders.o \
		readencoders.o checktrcmds.o -lrpc -lnetinet

servo.o:
	$(CC) $(CFLAGS) -DSIMULATING=0 -DSLOW=0 -I$(IPEINC) \
		-I$(NSINC) -I$(COMMONINC) -o servo.o servo.c

poundscb: poundscb.o $(NEWSERVO)/s_anti96com.o
	$(CC) -o poundscb poundscb.o $(NEWSERVO)/s_anti96com.o

poundscb.o: poundscb.c
	$(CC) $(CFLAGS) -I$(COMMONINC) -I$(NSINC) -o poundscb.o poundscb.c

$(BINDIR)/servotodd: servotodd.o $(NEWSERVO)/s_anti96com.o openshm.o rmsubs.o \
	checkencoders.o
	$(CC) -o $(BINDIR)/servotodd servotodd.o openshm.o\
		$(NEWSERVO)/s_anti96com.o $(COMMONLIB)/rm.o\
		rmsubs.o $(COMMONLIB)/commonLib checkencoders.o

servotodd.o:
	$(CC) $(CFLAGS) -DSIMULATING=0 -DSLOW=0 -I$(VMESG) -I$(IPEINC) \
		-I$(NSINC) -I$(COMMONINC) -o servotodd.o servotodd.c

#servo.pp: include/servo.h $(VMESG)/vme_sg_simple.h $(IPEINC)/iPEncoder.h\
#		$(COMMONINC)/rm.h $(COMMONINC)/smadaemon.h\
#		$(NSINCLUDES) include/tsshm.h servo.c $(COMMONLIB)/commonLib
#	$(CC) $(CFLAGS) -E -DSIMULATING=0 -DSLOW=0 -I$(VMESG) -I$(IPEINC) \
#		-I$(NSINC) -I$(COMMONINC) servo.c > servo.pp

$(BINDIR)/slowservo: slowservo.o $(NEWSERVO)/s_anti96com.o \
	openshm.o
	$(CC) -o $(BINDIR)/slowservo slowservo.o openshm.o\
		$(NEWSERVO)/s_anti96com.o $(COMMONLIB)/rm.o\
		rmsubs.o $(COMMONLIB)/commonLib checkencoders.o

slowservo.o: include/servo.h $(VMESG)/vme_sg_simple.h $(IPEINC)/iPEncoder.h\
		$(COMMONINC)/rm.h $(COMMONINC)/smadaemon.h\
		$(NSINCLUDES) include/tsshm.h servo.c $(COMMONLIB)/commonLib
	$(CC) $(CFLAGS) -DSIMULATING=0 -DSLOW=1 -I$(VMESG) -I$(IPEINC) \
		-I$(NSINC) -I$(COMMONINC) -o slowservo.o servo.c

$(BINDIR)/dmyservo: dmyservo.o openshm.o $(COMMONLIB)/commonLib
	$(CC) -o $(BINDIR)/dmyservo dmyservo.o openshm.o $(COMMONLIB)/commonLib

dmyservo.o: include/servo.h $(VMESG)/vme_sg_simple.h $(IPEINC)/iPEncoder.h\
		$(COMMONINC)/rm.h $(COMMONINC)/smadaemon.h\
		$(NSINCLUDES) include/tsshm.h servo.c $(COMMONLIB)/commonLib
	$(CC) $(CFLAGS) -DSIMULATING=1 -DSLOW=0 -I$(VMESG) -I$(IPEINC) \
		-I$(NSINC) -I$(COMMONINC) -o dmyservo.o servo.c

servo.s: include/servo.h $(VMESG)/vme_sg_simple.h $(IPEINC)/iPEncoder.h\
		$(NSINCLUDES) include/tsshm.h servo.c
	$(CC) -O -S -DSIMULATING=0 -Iinclude -I$(VMESG) -I$(IPEINC) -I$(NSINC)\
		-I$(COMMONINC) servo.c

$(BINDIR)/dmytrack: dmytrack.o openshm.o $(COMMONLIB)/commonLib
	$(CC) -o $(BINDIR)/dmytrack dmytrack.o openshm.o -mthreads\
		$(ENV_PREFIX)/usr/lib/libreadline.a $(COMMONLIB)/commonLib

dmytrack.o:
	$(CC) $(CFLAGS) -I$(COMMONINC) dmytrack.c

$(BINDIR)/svdata: svdata.o openshm.o
	$(CC) -o $(BINDIR)/svdata svdata.o openshm.o

svdata.o:
	$(CC) $(CFLAGS) svdata.c

$(BINDIR)/recordencoders: recordencoders.o openshm.o
	$(CC) -o $(BINDIR)/recordencoders recordencoders.o openshm.o \
		$(COMMONLIB)/rm.o

recordencoders.o:
	$(CC) $(CFLAGS) -I$(COMMONINC) recordencoders.c

$(BINDIR)/tsshmdump: tsshmdump.o openshm.o include/tsshm.h
	$(CC) -o $(BINDIR)/tsshmdump tsshmdump.o openshm.o

tsshmdump.o:
	$(CC) $(CFLAGS)  -I$(NSINC) tsshmdump.c

$(BINDIR)/rmtsshm: rmtsshm.o
	$(CC) -o $(BINDIR)/rmtsshm rmtsshm.o

rmtsshm.o:
	$(CC) $(CFLAGS)  rmtsshm.c

$(BINDIR)/createtsshm: createtsshm.o
	$(CC) -o $(BINDIR)/createtsshm createtsshm.o

createtsshm.o:
	$(CC) $(CFLAGS)  createtsshm.c

openshm.o:
	$(CC) $(CFLAGS)  openshm.c

$(BINDIR)/tracking_err: tracking_err.o openshm.o include/tsshm.h
	$(CC) -o $(BINDIR)/tracking_err tracking_err.o openshm.o\
		$(COMMONLIB)/commonLib $(COMMONLIB)/rm.o

tracking_err.o:
	$(CC) $(CFLAGS)  -I$(NSINC) -I$(COMMONINC) tracking_err.c

$(BINDIR)/readsc32: readsc32.o
	$(CC) -o $(BINDIR)/readsc32 readsc32.o

readsc32.o: readsc32.c
	$(CC) $(CFLAGS) -I$(COMMONINC) readsc32.c

$(BINDIR)/readtt: readtt.o
	$(CC) -o $(BINDIR)/readtt readtt.o

readtt.o:
	$(CC) $(CFLAGS) -I$(VMESG) readtt.c

$(BINDIR)/monitortt: monitortt.o
	$(CC) -o $(BINDIR)/monitortt monitortt.o

monitortt.o:
	$(CC) $(CFLAGS) -I$(VMESG) monitortt.c

canbus.o: canbus.c
	$(CC) $(CFLAGS) canbus.c

heartbeat.o: heartbeat.c
	$(CC) $(CFLAGS) heartbeat.c

$(BINDIR)/ttalarmcheck: ttalarmcheck.o
	$(CC) -o $(BINDIR)/ttalarmcheck ttalarmcheck.o

ttalarmcheck.o:
	$(CC) $(CFLAGS) -I$(VMESG) ttalarmcheck.c

$(BINDIR)/rmvalue: rmvalue.o
	$(CC) -o $(BINDIR)/rmvalue rmvalue.o $(COMMONLIB)/rm.o

rmvalue.o:
	$(CC) $(CFLAGS) -I$(COMMONINC) rmvalue.c

rmsubs.o:
	$(CC) $(CFLAGS) -I$(COMMONINC) -o rmsubs.o rmsubs.c

rmsubs.s: rmsubs.c include/servo.h include/tsshm.h include/rmsubs.h \
	include/servo_rm_defs.h $(COMMONINC)/rm.h
	$(CC) -S -Iinclude -I$(COMMONINC) -o rmsubs.s rmsubs.c

$(NEWSERVO)/s_anti96com.o: $(NEWSERVO)/s_anti96com.c
	$(CC) -c -I$(NSINC) -I$(ACCAPP)/iPOctalPlus422/includeFiles $(NEWSERVO)/s_anti96com.c -o \
		$(NEWSERVO)/s_anti96com.o

testsun: checktrcmds.c
	$(CC) -g -O -Wall -Iinclude -o testsun -I$(COMMONINC) -DUSE_MAIN \
		$< $(ENV_PREFIX)/usr/lib/libreadline.a

checktrcmds.o: checktrcmds.c
	gcc $(CFLAGS) -I$(COMMONINC) checktrcmds.c

checktrcmds.s: checktrcmds.c
	gcc -c -g -O -I$(COMMONINC) -Iinclude -Wa,-ahl,-L \
		checktrcmds.c > checktrcmds.s

checktrcmds.i: checktrcmds.c
	gcc -E -I$(COMMONINC) -Iinclude checktrcmds.c > checktrcmds.i

checkencoders.o:
	$(CC) $(CFLAGS) -o checkencoders.o checkencoders.c

readencoders.o:
	$(CC) $(CFLAGS) -I$(COMMONINC) -o readencoders.o readencoders.c

$(BINDIR)/sampleEncoders: readencoders.c openshm.o
	$(CC) -O -g -Wall -Iinclude -I$(COMMONINC) -DUSE_MAIN \
		-o $(BINDIR)/sampleEncoders readencoders.c openshm.o

$(BINDIR)/drivemon: drivemon.o openshm.o include/tsshm.h
	$(CC) -o $(BINDIR)/drivemon drivemon.o openshm.o -lcurses

drivemon.o:
	$(CC) $(CFLAGS)  -I$(NSINC) drivemon.c

$(BINDIR)/ttread: ttread.o
	$(CC) -o $(BINDIR)/ttread  ttread.o

ttread.o: ttread.c
	$(CC) $(CFLAGS) -I$(VMESG) ttread.c

$(COMMONBIN)/rmdrivemon: rmdrivemon.o $(ANTPAGE2)
	$(CC) -o $(COMMONBIN)/rmdrivemon rmdrivemon.o $(ANTPAGE2) \
		$(COMMONLIB)/rm.o -lcurses $(DSMLD)

rmdrivemon.o:
	$(CC) $(CFLAGS)  -I$(NSINC) -I$(MONINC) -I$(COMMONINC) -I$(GLOBALINC) \
		rmdrivemon.c

ttsubs: ttsubs.c
	$(CC) -o ttsubs -g -Wall -I$(COMMONINC) ttsubs.c

$(COMMONINC)/tsshm.h : include/tsshm.h
	ln -s $(ACCAPP)/servo/include/tsshm.h $(COMMONINC)

clean:
	rm -f *.o $(BINS)

depend: $(SOURCES)
	$(CC) $(CFLAGS) -Iinclude -I$(VMESG) -I$(IPEINC) -I$(NSINC) \
		-I$(MONINC) -I$(COMMONINC) -I$(GLOBALINC) -MM $(SOURCES) \
		-I$(ACCAPP)/iPOctalPlus422/includeFiles > dependencies

include dependencies
