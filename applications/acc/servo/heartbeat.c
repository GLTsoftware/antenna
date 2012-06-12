/* Subrouteins for accessing the acromag counter */

#define USE_MAIN 0

#include <math.h>
#include <sys/types.h>
#include <resource.h>
#include <unistd.h>
#define _POSIX_SOURCE
#include <stdio.h>
#include <stdlib.h>
#if 0
#include <sys/mman.h>
#include <ctype.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <smem.h>
#endif
#include <fcntl.h>
#include <sys/ioctl.h>
#include "../include/iP480.h"

/* Things for the couinter device */
char cntrName[] = "/dev/iP480_0";
int cntrfd = 0;
/* Initialize the counter number to 1 in both structures */
iP480ioc_t ioctlArg = {0,1};
iP480_result_t readArg = {0,1};

/* Open the counter device and set up the counter mode and load value
 */
void openCntr(void) {

    if(cntrfd == 0) {
	if((cntrfd = open(cntrName, O_RDONLY, 0)) < 0) {
	    fprintf(stderr, "Can not open %s, I quit\n", cntrName);
	    perror("");
	    exit(1);
	}
    }
    ioctlArg.value = 48;
    if(ioctl(cntrfd, IP480_SET_CONST_REG, &ioctlArg) < 0) {
	    perror("Setting counter constant to 48 failed");
	    exit(2);
    }
    ioctlArg.value = EVENT_COUNTING_MODE | INPUT_ACTIVE_HIGH | INTERRUPT_ENABLE;
    if(ioctl(cntrfd, IP480_SET_CTRL_REG, &ioctlArg) < 0) {
	    perror("Setting counter mode failed");
	    exit(3);
    }
    ioctlArg.value = 0;
    if(ioctl(cntrfd, IP480_TRIGGER_CNTR, &ioctlArg) < 0) {
	    perror("Triggering counter failed");
	    exit(4);
    }
}

void readCntr(void) {
    if(read(cntrfd, &readArg, sizeof(readArg)) != sizeof(readArg)) {
	perror("Reading counter");
	exit(5);
    }
}

#if USE_MAIN
double exTime;
void closeCntr(void) {
    if(cntrfd != 0) {
	close(cntrfd);
	cntrfd = 0;
    }
}

struct timeval tv1, tv2;
struct timezone tz;

void StartTime(void) {
	gettimeofday(&tv1, &tz);
}

void StopTime(void) {
	gettimeofday(&tv2, &tz);
	exTime = (double)(tv2.tv_sec - tv1.tv_sec) +
		(double)(tv2.tv_usec - tv1.tv_usec) / 1e6;
}

/* Test main program */
int main(int argc, char *argv[]) {
    int i;

    openCntr();
    for(i = 0; i < 5; i++) {
	readCntr();
	if(i == 0) {
	    StartTime();
	} else {
	    StopTime();
	    printf("ret %d cntr = %d  elapsed time = %.4f\n", i, readArg.count,
		exTime);
	}
    }
    /* clear the interrupt enable before quitting */
    ioctlArg.value = EVENT_COUNTING_MODE | INPUT_ACTIVE_HIGH;
    if(ioctl(cntrfd, IP480_SET_CTRL_REG, &ioctlArg) < 0) {
	    perror("Setting counter mode failed");
	    exit(7);
    }
    return(0);
}
# endif /* USE_MAIN */
