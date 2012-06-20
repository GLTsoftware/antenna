/* Dmytrack is a program to replace Track while testing Servo.  It has
 * a very simple user interface for turning drives on or off and
 * and setting positions and velocities.  It communicates with
 * Servo through the same shared memory and uses the same protocol
 * which the real Track.
 *
 * CVS Log and RCS Identification of this version:
 * $Log: dmytrack.c,v $
 * Revision 1.13  2004/08/03 21:31:06  rwilson
 * Fix startup bug
 *
 * Revision 1.12  2003/06/04 00:34:54  rwilson
 * set cmd pos to encoder values first thing
 *
 * Revision 1.11  2002/09/26 20:56:15  rwilson
 * bounds on vel and pos, more RM vars
 *
 * Revision 1.10  2002/07/30 15:46:01  rwilson
 * added processPresent
 *
 * Revision 1.9  2002/05/08 20:58:26  rwilson
 * Changed names for M3CMD enum
 *
 * Revision 1.8  2002/04/19 21:38:21  rwilson
 * added aa, aas, and a h {on, off} and el
 *
 * Revision 1.7  2000/11/17 21:47:06  rwilson
 * Fix up pthreads for POSIX.1c in LynxOS3.1
 *
 * Revision 1.6  2000/10/18 14:27:37  rwilson
 * use common OpenShm, rename tsshm.h
 *
 * Revision 1.5  2000/09/24 18:38:06  rwilson
 * Faultword to stderr, others
 *
 * Revision 1.4  2000/06/22 01:45:08  rwilson
 * Fixed permissions on tsshm
 *
 * Revision 1.3  2000/06/13 21:15:16  rwilson
 * Added thread to update commands to Servo each second
 *
 */
static char rcsid[] = "$Id: dmytrack.c,v 1.13 2004/08/03 21:31:06 rwilson Exp $";

#include <math.h>
#include <sys/types.h>
#include <resource.h>
#include <unistd.h>
#define _POSIX_SOURCE
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/stat.h>
#include "tsshm.h"
#include "commonLib.h"

#define MSEC_PER_DEG 3600000

/* Definitions which should be in readline/readline.h, but aren't in LynxOS */
void readline_initialize_everything(void);
char *readline(char *);
void add_history(char *);
int read_history(char *);
int write_history(char *);
#define HIST_FILE "./.track_history"

TrackServoSHM *tsshm;		/* Local pointer to shared memory */

/* Communication from the User Interface to driver. */
double newP = 0, newV = 0;	/* New az or el position and velocity */
int newAz = 0, newEl = 0;	/* Set newAz to 1 if new[PV] are new az */

static int lastMsec = -1;	/* Last time reported by Servo */
static int servoKnownDead = 0;

void *driver(void *);
void *openshm(char *name, int size);

int main(int argc, char *argv[]) {
	char *line, word[12], cmdChar, *ip;
	int n;
	pthread_t driverTid;
#if _POSIX_VERSION >= 199506
	struct sched_param param;
#else  /* _POSIX_VERSION >= 199506 */
	pthread_attr_t attr;
#endif /* _POSIX_VERSION >= 199506 */

	if(processPresent("excl_Track")) {
	    printf("Another Track type process is running\n");
	    exit(1);
	}
	setpriority(PRIO_PROCESS, (0), (TRACKUSERPRIO));
	readline_initialize_everything();
	read_history(HIST_FILE);
	tsshm = OpenShm(TSSHMNAME, TSSHMSZ);
	tsshm->azCmd = OFF_CMD;
	tsshm->elCmd = OFF_CMD;
	tsshm->az = tsshm->encAz;
	tsshm->el = tsshm->encEl;
	tsshm->azVel = 0;
	tsshm->elVel = 0;
	tsshm->msecCmd = tsshm->msec;

	/* Start the driver thread */
#if _POSIX_VERSION >= 199506
	if(pthread_create(&driverTid, NULL, driver, (void *)0) < 0) {
	    perror("Failed to create driver");
	    exit(1);
	}
	param.sched_priority = TRACKPRIO;
	pthread_setschedparam(driverTid, SCHED_DEFAULT, &param);
#else  /* _POSIX_VERSION >= 199506 */
	pthread_attr_create(&attr);
	if(pthread_create(&driverTid, attr, driver, (void *)0) < 0) {
	    perror("Failed to create driver");
	    exit(1);
	}
	pthread_setprio(driverTid, TRACKPRIO);
#endif /* _POSIX_VERSION >= 199506 */

	/* This main loop communicates with the user */
	for(;;) {
	    line = readline("trk: ");
	    if(*line) {			/* Readline removes the '\n' */
		add_history(line);

		/* At this point the main part of the program should run.  */
	        cmdChar = *line;
		/* Skip the remainder of the first word */
	        for(ip = 1 + line; ! isspace(*ip); ip++) ;
		switch(cmdChar) {
		case 'p':
		    n = 1;
		    printf("Fault = %d az cmd %d, state %d    "
			    "el cmd %d, state %d\n", tsshm->fault,
			    tsshm->azCmd, tsshm->azState,
			    tsshm->elCmd, tsshm->elState);
		    printf("az %9.4f, azVel %9.4f  curAz %9.4f, encAz %9.4f\n",
			    tsshm->az/MSEC_PER_DEG, tsshm->azVel/MSEC_PER_DEG,
			    (double)tsshm->cmdAz/MSEC_PER_DEG,
			    (double)tsshm->encAz/MSEC_PER_DEG);
		    printf("el %9.4f, elVel %9.4f  curEl %9.4f, encEl %9.4f\n",
			    tsshm->el/MSEC_PER_DEG, tsshm->elVel/MSEC_PER_DEG,
			    (double)tsshm->cmdEl/MSEC_PER_DEG,
			    (double)tsshm->encEl/MSEC_PER_DEG);
		    printf("day %d, msec %d, msecCmd %d   "
			    "msecAccept %d\n", tsshm->day,
			    tsshm->msec, tsshm->msecCmd, tsshm->msecAccept);
		    printf("MSEC_PER_DEG = %9.4f\n", (double)n*MSEC_PER_DEG);
		    printf("m3Cmd = %d = %s  m3State = %d = %s\n", tsshm->m3Cmd,
			(tsshm->m3Cmd == CLOSE_M3_CMD)? "Closed":
			(tsshm->m3Cmd == OPEN_M3_CMD)? "open":"Unknown",
			tsshm->m3State,
			(tsshm->m3State == CLOSED_ST)? "Closed":
			(tsshm->m3State == OPEN_ST)? "open":"Unknown");
		    printf("lastMsec = %d, servoKnownDead = %d\n", lastMsec,
			servoKnownDead);
		    break;
		case 'a':
		    if(line[1] == 'a') {
			n = sscanf(ip, "%lf %lf", &newP, &newV);

			switch(n) {
			case 1:
			    newV = 0;
			case 2:		/* This is supposed to fall through */
			    if(line[2] == 's') {
				newP = tsshm->az/MSEC_PER_DEG + newP/3600;
				newV = tsshm->azVel/MSEC_PER_DEG + newV/3600;
			    } else {
				newP = tsshm->az/MSEC_PER_DEG + newP;
				newV = tsshm->azVel/MSEC_PER_DEG + newV;
			    }
			    newAz = 1;	/* Signal driver new az pos, vel. */
			    break;
			default:
			    printf("??\n");
			    break;
			}
			break;
		    } else if(line[1] == 'h') {
			if(sscanf(ip, "%s", word)) {
			    if(strcmp("on", word) == 0) {
				tsshm->sendEncToPalm = AZ_TO_PALM;
			    } else if(strcmp("off", word) == 0) {
				tsshm->sendEncToPalm = NOTHING_TO_PALM;
			    } else {
				printf("??\n");
			    }
			}
			break;
		    }
		    n = sscanf(ip, "%lf %lf", &newP, &newV);
		    switch(n) {
		    case 1:
			newV = 0;
		    case 2:		/* This is supposed to fall through */
			newAz = 1;	/* Signal driver new az pos, vel. */
			break;
		    case 0:
			if(sscanf(ip, "%s", word)) {
			    if(strcmp("on", word) == 0) {
				tsshm->msecCmd = tsshm->msec;
				usleep(10000);
				tsshm->azCmd = ON_CMD;
			    } else if(strcmp("off", word) == 0) {
				tsshm->azCmd = OFF_CMD;
			    } else {
				printf("??\n");
			    }
			}
			break;
		    default:
			printf("??\n");
			break;
		    }
		    break;
		case 'e':
		    if(line[1] == 'a') {
			n = sscanf(ip, "%lf %lf", &newP, &newV);

			switch(n) {
			case 1:
			    newV = 0;
			case 2:		/* This is supposed to fall through */
			    if(line[2] == 's') {
				newP = tsshm->el/MSEC_PER_DEG + newP/3600;
				newV = tsshm->elVel/MSEC_PER_DEG + newV/3600;
			    } else {
				newP = tsshm->el/MSEC_PER_DEG + newP;
				newV = tsshm->elVel/MSEC_PER_DEG + newV;
			    }
			    newEl = 1;	/* Signal driver new az pos, vel. */
			    break;
			default:
			    printf("??\n");
			    break;
			}
			break;
		    } else if(line[1] == 'h') {
			if(sscanf(ip, "%s", word)) {
			    if(strcmp("on", word) == 0) {
				tsshm->sendEncToPalm = EL_TO_PALM;
			    } else if(strcmp("off", word) == 0) {
				tsshm->sendEncToPalm = NOTHING_TO_PALM;
			    } else {
				printf("??\n");
			    }
			}
			break;
		    }
		    n = sscanf(ip, "%lf %lf", &newP, &newV);
		    switch(n) {
		    case 1:
			newV = 0;
		    case 2:		/* This is supposed to fall through */
			newEl = 1;	/* Signal driver */
			break;
		    case 0:
			if(sscanf(ip, "%s", word)) {
			    if(strcmp("on", word) == 0) {
				tsshm->msecCmd = tsshm->msec;
				usleep(10000);
				tsshm->elCmd = ON_CMD;
			    } else if(strcmp("off", word) == 0) {
				tsshm->msecCmd = tsshm->msec;
				tsshm->elCmd = OFF_CMD;
			    } else {
				printf("??\n");
			    }
			}
			break;
		    default:
			printf("??\n");
			break;
		    }
		    break;
		case 'm':
			if(sscanf(ip, "%s", word)) {
			    if(strcmp("open", word) == 0)
				tsshm->m3Cmd = OPEN_M3_CMD;
			    else if(strcmp("closed", word) == 0)
				tsshm->m3Cmd = CLOSE_M3_CMD;
			    else {
				printf("??\n");
			    }
			}
		    break;
		case 'q':
		    tsshm->azCmd = OFF_CMD;
		    tsshm->elCmd = OFF_CMD;
		    write_history(HIST_FILE);
		    return(0);
		default:
		    printf("??\n");
		    break;
		}

		/* end of the main loop */
	    }
	    free(line);
	}
	fprintf(stderr, "!!! Control should not get here !!!\n");
/*	shm_unlink(TSSHMNAME); */
	return(0);
}

/* Driver communicates with servo through tsshm. */
void *driver(void *dmyarg) {
	double dt;

	/* Drivers working positions to use in commanding servo */
	static double az, el;		/* Pos cmds to servo (mas) */
	static int msecCmd;		/* Ref time (UT) for command (msec) */

	for(;;) {
	    sleep(1);
	    if(lastMsec < 0) {
		lastMsec = tsshm->msec;
	    } else if(tsshm->msec != lastMsec) {/* Servo alive */
		if(servoKnownDead) {
		    printf("Servo is now running\n");
		    servoKnownDead = 0;
		}
		lastMsec = tsshm->msec;
		/* First check whether either axis is running */
		if(tsshm->azCmd == OFF_CMD && tsshm->elCmd == OFF_CMD) {
		    tsshm->az = az = tsshm->encAz;
		    tsshm->el = el = tsshm->encEl;
		    tsshm->azVel = tsshm->elVel = 0;
		    if(newAz || newEl) {
			printf(
		"Warning! Both drives off and position commands ignored\n");
			newAz = newEl = 0;
		    }
		    continue;
		}

		/* Issue new pointing commands as the real track would
		 * By saving lastMsec, the generated positions and velocities
		 * will be consistent even if Servo runs during the
		 * calculations. */
		dt = (lastMsec - msecCmd) * 0.001;
		if(dt < 0) {		/* New day? */
		    dt += 86400.;
		}
		if(newAz) {
		    tsshm->az = az = newP * MSEC_PER_DEG;
		    tsshm->azVel = newV * MSEC_PER_DEG;
		    tsshm->el += tsshm->elVel * dt;/* Correct to new ref time */
		    msecCmd = lastMsec;
		    newAz = 0;
		} else if(newEl) {
		    tsshm->el = el = newP * MSEC_PER_DEG;
		    tsshm->elVel = newV * MSEC_PER_DEG;
		    tsshm->az += tsshm->azVel * dt;/* Correct to new ref time */
		    msecCmd = lastMsec;
		    newEl = 0;
		} else {
		    tsshm->az = az + tsshm->azVel * dt;
		    tsshm->el = el + tsshm->elVel * dt;
		}
		tsshm->msecCmd = lastMsec;	/* Issue command to Servo */
	    } else {				/* Servo not running */
		if(!servoKnownDead) {
		    printf("Servo is not running, commanding drives off\n");
		    tsshm->azCmd = tsshm->elCmd = OFF_CMD;
		    servoKnownDead = 1;
		}
		newAz = newEl = 0;
	    }
	}
}
