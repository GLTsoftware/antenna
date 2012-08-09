/* Svdata is a program to collect servo performance data from the
 * track-servo shared memory and control Servo's putting it there.
 *
 * CVS Log and RCS Identification of this version:
 * $Log: svdata.c,v $
 * Revision 1.9  2008/05/09 18:44:07  rwilson
 * Add dump of previous state on unexpected drive shutdown
 *
 * Revision 1.8  2001/09/27 23:53:00  rwilson
 * change status format to 'd' from 'x'.
 *
 * Revision 1.7  2000/10/18 14:27:37  rwilson
 * use common OpenShm, rename tsshm.h
 */
static char rcsid[] = "$Id: svdata.c,v 1.9 2008/05/09 18:44:07 rwilson Exp $";

#include <resource.h>
#define _POSIX_SOURCE
#include <unistd.h>
/* This is a non-POSIX function, so it is not seen in unistd.h */
extern unsigned int usleep    _AP((time_t));
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include "tsshm.h"

#define MAS (3600000)

static char azMode = 0, elMode = 0;

void *openshm(char *name, int size);

int main(int argc, char *argv[]) {
	TrackServoSHM *tsshm;		/* Local pointer to shared memory */
	int started;
	int nSamp, when;
	int nextSample;
	SAMPLE *samples, *sp;
	FILE *fp;

	/* Deal with command line args */
	when = 's';
	while(argc > 3) {
	    if(argv[1][0] == '-') {
		when = argv[1][1];
		argc--;
		argv++;
	    } else {
		break;
	    }
	}
	if(argc != 3 || (when != 's' && when != 'i')) {
	    fputs("Usage: svdata [-when] sample_len(sec) file_name\n"
		"where when can take on the values:\n"
		"i - Start immediately\n"
		"s - Start on the next state change (default)\n"
		,stderr);
	    exit(1);
	}
	started = (when == 'i')? 1: 0;
	nSamp = atoi(argv[1]) * 100;	/* the number of samples */
/*	printf("when = %c, nSamp = %d, fn = %s\n", when, nSamp, argv[2]); */
	if((samples = (SAMPLE *)malloc(nSamp*sizeof(SAMPLE))) == NULL) {
	    fputs("Could not malloc enough memory for recording test data\n",
		stderr);
	    exit(1);
	}

	setpriority(PRIO_PROCESS, (0), (45));
/*	setprio(0, 75); */
	tsshm = OpenShm(TSSHMNAME, TSSHMSZ);
	usleep(10000);
	tsshm->sampOut = PREV_SAMP(tsshm->sampIn); /* Set samp buffer empty */
	usleep(100000);
	if(SEMPTY) {
	    fputs("Servo doesn't seem to be running\n", stderr);
	    exit(1);
	}
	if(! started) {
	    usleep(10000);
	    INC_OUT;
	    INC_OUT;
	    azMode =  SaO.azMode;
	    elMode =  SaO.elMode;
	}

	if((fp = fopen(argv[2], "w")) == NULL) {
	    fprintf(stderr, "svdata: Couldn't open %s for writing\n", argv[2]);
	    exit(1);
	}

	/* If we are not starting immediately, wait for the state to change */
	if(! started) fprintf(stderr,
	    "Waiting for a state change from (%1d %1d)\n", azMode, elMode);
	while(! started) {
	    if(SEMPTY) {
		fputs("Servo has stopped\n", stderr);
		exit(1);
	    }
	    while(! SEMPTY) {
		INC_OUT;
		if(azMode != SaO.azMode || elMode != SaO.elMode) {
			started = 1;
			break;
		}
	    }
	    usleep(100000);
	}
	fprintf(stderr, "Starting to collect data, state (%1d %1d)\n",
		SaO.azMode, SaO.elMode);
	/* Now collect the data */
	for(nextSample = 0; nextSample < nSamp; nextSample++) {
	    if(SEMPTY) {
		usleep(100000);
		if(SEMPTY) {
		    fputs("Servo has stopped\n", stderr);
		    break;
		}
	    }
	    INC_OUT;
	    samples[nextSample] = SaO;
	}

	fputs("Writing out data\n", stderr);

/* Definition of the data columns:
Column  Content
1       time in seconds
2       Az command from (dmy)track
3       Az from the ACU
4       Az Velocity commanded by servo
5       Az Velocity from the ACU
6       State of the ACU Az drive
7       El command from track
8       El from the ACU
9       El Velocity commanded by servo
10      El Velocity from the ACU
11      State of the El command shaper
*/

	/* Write out the data */
#define PO(x) sp->x*(1.0/MAS)		/* Pos and vel in degrees */
	for(sp = samples; sp < &samples[nextSample]; sp++) {
	    fprintf(fp, "%7.3f "
		"% 11.5f %11.5f %8.4f %8.4f %1d"	/* Az values */
		"% 11.5f %11.5f %8.4f %8.4f %1d"	/* El values */
		"\n", sp->msec*0.001,
		PO(curAz), PO(acuAz),
		PO(cmdAzVel), PO(acuAzVel), sp->azMode, 
		PO(curEl), PO(acuEl),
		PO(cmdElVel), PO(acuElVel), sp->elMode
	    );
	}
	return(0);
}
