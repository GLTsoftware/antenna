#include <stdio.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <resource.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

#include "vme_sg_simple.h"

struct vme_sg_simple_time ttime;
int ttfd, status;

/* ttalarmcheck.c */
static void Add10msec(void);
static void SetTime(void);

int main(int argc, char *argv[]) {
    int ms, oldms = -999;
    
    ttfd = open("/dev/vme_sg_simple",O_RDWR);
    if(ttfd==-1) {
	perror("open()");
	exit(-1);
    }
    setpriority(PRIO_PROCESS, (0), 80);

    if((status = read(ttfd, &ttime, sizeof(ttime))) < 0) {
	fprintf(stderr, "Error %d reading TrueTime\n", status);
    }
    SetTime();
    ttime.timeout_ticks = 3;  	/* Leave a good margin the 1st time */
    for(;;) {
	Add10msec();

#if 0
	if((status = read(ttfd, &ttime2, sizeof(ttime2))) < 0) {
	    fprintf(stderr, "Error %d reading TrueTime\n", status);
	}
	status = ((((ttime.yday - ttime2.yday) * 24 +
		(ttime.hour - ttime2.hour)) * 60 +
		(ttime.min - ttime2.min)) * 60 +
		(ttime.sec - ttime2.sec)) * 1000 +
		(ttime.usec - ttime2.usec) / 1000;
	/* If we missed the next tick, wait for the following one. */
	if(status < 1 || status > 20) {
	    fprintf(stderr, "Add10ms: would have waited from "
		"%d %02d:%02d:%02d.%.03d to %d %02d:%02d:%02d.%.03d\n",
		ttime2.yday, ttime2.hour, ttime2.min, ttime2.sec,
		ttime2.usec / 1000,
		ttime.yday, ttime.hour, ttime.min, ttime.sec,
		ttime.usec / 1000
		);
	    SetTime();
	}
#endif

	if(ioctl(ttfd, VME_SG_SIMPLE_WAIT_UNTIL, &ttime) < 0) {
	    switch(errno) {
	    case EAGAIN:
		fprintf(stderr, 
		    "servo could not wait on TrueTime because it was busy\n");

		/* At this point I will try a simple fix, but there will be
		 * errors.  The stupid LynxOS usleep has 20ms granularity */
		usleep(10000);
		if(read(ttfd, &ttime, sizeof(ttime)) < 0) {
		    fprintf(stderr, "Error reading TrueTime\n");
		}
		/* Now resync to the nearest even 10 msec time */
		ttime.usec += 5000;
		ttime.usec /= 10000;
		ttime.usec *= 10000;
		break;
	    case EINTR:
		/* We received an interrupt and will stop anyway, so don't
		 * worry. */
		fprintf(stderr, "SILENT TrueTime returned an interrupt");
		break;
	    case ETIMEDOUT:
		if((status = read(ttfd, &ttime, sizeof(ttime))) < 0) {
		    fprintf(stderr, "Error %d reading TrueTime  ", status);
		}
		ms = ((ttime.hour * 60 + ttime.min) * 60 + ttime.sec) * 1000 +
			ttime.usec / 1000;
		fprintf(stderr, "SILENT ttalarmcheck: timeout at "
			"%02d:%02d:%02d.%06d  dt = %dms\n",
			ttime.hour, ttime.min, ttime.sec, ttime.usec,
			ms - oldms);
		/* Now resync to the nearest even 10 msec time */
		ttime.usec += 5000;
		ttime.usec /= 10000;
		ttime.usec *= 10000;
		break;
	    default:
		fprintf(stderr, "The TrueTime wait returned error %d  ", errno);
		break;
	    }
	    oldms = -999;
	}
	/* Convert the time now */
	ms = ((ttime.hour * 60 + ttime.min) * 60 + ttime.sec) * 1000 +
		ttime.usec / 1000;
	if(ms > oldms + 10 && oldms > 0) {
	    fprintf(stderr, "return after %d ms\n", ms - oldms);
	}
	oldms = ms;
    }
    return(0);
}

static void Add10msec(void) {
	/* Increment the time by 10 ms */
	ttime.usec += 10000;
	if(ttime.usec >= 1000000) {
	  ttime.usec -= 1000000;
	  if(++ttime.sec >= 60) {
		    ttime.sec = 0;
		    if(++ttime.min >= 60) {
	      ttime.min = 0;
	      if(++ttime.hour >= 24) {
		ttime.hour = 0;
		if(++ttime.yday >= ((ttime.year % 4) == 0)? 366: 365) {
		  ttime.yday = 0;
		  ttime.year++;
		}
	      }
	    }
	  }
	}
}


static void SetTime(void) {
	if(read(ttfd, &ttime, sizeof(ttime)) < 0) {
	    fprintf(stderr, "Error reading TrueTime\n");
	}
	ttime.usec += 9999;		/* round up to next tick */
	ttime.usec /= 10000;
	ttime.usec *= 10000;
}
