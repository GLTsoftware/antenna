/* recordencoders is a program to write the value of the fine and coarse
 * encoders to a file once per second.  It runs on the acc and gets its
 * values from RM and the time from the track-servo shared memory.
 *
 * CVS Log and RCS Identification of this version:
 * $Log: recordencoders.c,v $
 * Revision 1.3  2006/01/04 17:02:52  rwilson
 * Fix description
 *
 * Revision 1.2  2003/07/10 19:00:17  rwilson
 * Catch SIGTERM for smainit
 *
 * Revision 1.1  2003/06/04 00:40:27  rwilson
 * Added
 *
 *
 */
static char rcsid[] = "$Id: recordencoders.c,v";

#include <resource.h>
#define _POSIX_SOURCE
#include <unistd.h>
/* This is a non-POSIX function, so it is not seen in unistd.h */
extern unsigned int usleep    _AP((time_t));
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "tsshm.h"
#include "rm.h"

#define MAS (3600000)

extern void *openshm(char *name, int size);

int gotQUIT = 0, gotINT = 0;
/* Reflective memory variables */
int antenna_num = 0;
float encAz, encEl, limAz, limEl;

struct rm_vars {
	void *a;
	char *name;
} rv[] = {
	{&encAz, "RM_ENCODER_AZ_F"},
	{&encEl, "RM_ENCODER_EL_F"},
	{&limAz, "RM_LIM_ENCODER_AZ_F"},
	{&limEl, "RM_LIM_ENCODER_EL_F"},
};
#define NUMVARS (sizeof(rv) / sizeof(rv[0]))

/* recordencoders.c */
static void SigHndlr(int signo);
void OpenRM(void);
void ReadRM(void);

int main(int argc, char *argv[]) {
	TrackServoSHM *tsshm;		/* Local pointer to shared memory */
	int nSamp;
	int nextSample;
	int usec, startMsec;
	int sampTime;
	FILE *fp;

	/* Deal with command line args */
	if(argc < 2 || argc > 3) {
	    fputs("Usage: recordencoders run_time(sec) file_name\n", stderr);
	    exit(1);
	}
	nSamp = atoi(argv[1]);
	if(nSamp <= 0 || nSamp > 86400) {
	    fprintf(stderr, "Run time must be > 0 and less than a day\n");
	    exit(1);
	}

	if((fp = fopen(argv[2], "w")) == NULL) {
	    fprintf(stderr, "svdata: Couldn't open %s for writing\n", argv[2]);
	    exit(1);
	}

	if(signal(SIGTERM, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting TERM signal disposition\n");
	    exit(1);
	}
	if(signal(SIGQUIT, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting QUIT signal disposition\n");
	    exit(1);
	}
	if(signal(SIGINT, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting INT signal disposition\n");
	    exit(1);
	}

	setpriority(PRIO_PROCESS, (0), (45));
	tsshm = OpenShm(TSSHMNAME, TSSHMSZ);
	OpenRM();
	startMsec = tsshm->msec;
	usleep(100000);
	if(tsshm->msec == startMsec) {
	    fprintf(stderr, "Servo is not running\n");
	    exit(1);
	}
	/* get the values just after the second tic */
	usec = 1000 * (tsshm->msec % 1000);
	usleep(1010000 - usec);
	startMsec = tsshm->msec;
	fprintf(fp, "#time fineAz limAz-fineAz fineEl limEl-fineEl\n");
	/* Now collect the data */
	for(nextSample = 0; nextSample < nSamp; nextSample++) {
	    ReadRM ();
	    sampTime = (tsshm->msec - startMsec)/10;
	    if(sampTime < 0) {
		startMsec -= 86400;
		sampTime = (tsshm->msec - startMsec)/10;
	    }
	    fprintf(fp, "%5d %8.3f %8.3f %8.3f %8.3f\n", sampTime,
		    encAz, limAz - encAz, encEl, limEl - encEl);
	    if(gotINT || gotQUIT) {
		fclose(fp);
		exit(2);
	    }
	    usec = 1000 * (tsshm->msec % 1000);
	    usleep(1010000 - usec);
	}
	return(0);
}

/* Subroutine to handle SIGINT (^C) interrupts and shut down gracefully */
static void SigHndlr(int signo) {
	switch(signo) {
	case SIGINT:
	    gotINT = 1;
	    break;
	case SIGTERM:
	case SIGQUIT:
	    gotQUIT = 1;
	    break;
	}
}

void OpenRM(void) {
	int *ip;
	int rm_open_status;
	int antlist[RM_ARRAY_SIZE];

	rm_open_status=rm_open(antlist);
	if(rm_open_status != RM_SUCCESS) {
	    rm_error_message(rm_open_status,"rm_open()");
	    exit(1);
	}
	for(ip = antlist; *ip != RM_ANT_LIST_END; ip++) {
	    if(*ip == antenna_num) {
		return;
	    }
	}
	fprintf(stderr, "no card for antenna %d\n", antenna_num);
	exit(1);
}

void ReadRM(void) {
	int i;
	int rm_rtn;

	for(i = 0; i < NUMVARS; i++) {
	    rm_rtn=rm_read(antenna_num, rv[i].name, rv[i].a);
	    if(rm_rtn != RM_SUCCESS) {
		fprintf(stderr, "rm_read of ");
		rm_error_message(rm_rtn, rv[i].name);
		exit(1);
	    }
	}
}
