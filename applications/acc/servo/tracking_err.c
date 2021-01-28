#include <math.h>
#include <sys/types.h>
#include <resource.h>
#define _POSIX_SOURCE
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include "tsshm.h"
#include "rm.h"
/* #include "s_cmd2.h" */

#define MSEC_PER_DEG (3600000.)

void *openshm(char *name, int size);

int antlist[RM_ARRAY_SIZE],rm_status;

int main(int argc, char *argv[]) {
	TrackServoSHM *tsshm;		/* Local pointer to shared memory */
	int pass;
	float azErr, elErr;
	double rms;

	tsshm = OpenShm(TSSHMNAME, TSSHMSZ);

	/* initializing ref. mem. */
	rm_status=rm_open(antlist);
	if(rm_status != RM_SUCCESS) {
	    rm_error_message(rm_status,"rm_open()");
	    rm_status = RM_NO_SERVICE;
	    return(-1);
	} else {
	    printf("Rm is open\n");
	}

	if(argc > 1) {
	    pass = atoi(argv[1]);
	} else {
	    pass = 1;
	}
	for(;pass > 0; pass--) {

	    /* Instantaneous az error (commanded - actual) */
	    rm_status=rm_read(RM_ANT_0,"RM_AZ_TRACKING_ERROR_F", &azErr);
	    if(rm_status != RM_SUCCESS) {
		rm_error_message(rm_status, "rm_read Az tracking error");
	    }
	    rm_status=rm_read(RM_ANT_0,"RM_EL_TRACKING_ERROR_F", &elErr);
	    if(rm_status != RM_SUCCESS) {
		rm_error_message(rm_status, "rm_read El tracking error");
	    }
	    rm_status=rm_read(RM_ANT_0,"RM_TRACKING_ERROR_ARCSEC_D", &rms);
	    if(rm_status != RM_SUCCESS) {
		rm_error_message(rm_status, "rm_read 1 sec rms tracking error");
	    }
	    printf("az reflmem %9.2f shm %9.2f   el %9.2f %9.2f  "
		"rms %.2f\n", azErr, tsshm->azTrError * 0.001,
		elErr, tsshm->elTrError * 0.001, rms);
	    if(pass) usleep(100000);
	}
	return(0);
}
