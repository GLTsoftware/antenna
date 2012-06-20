#include <sys/types.h>
#include <sys/utsname.h>
#include <resource.h>
#include <errno.h>
#define _POSIX_SOURCE 1
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include "vme_sg_simple.h"

int main(int argc, char *argv[]) {
	struct vme_sg_simple_time ttime;
	int ttfd;		/* File descriptor for the TrueTime device */
	int i, j;

	setpriority(PRIO_PROCESS, (0), 100);
	if((ttfd = open("/dev/vme_sg_simple", O_RDWR, 0)) < 0) {
	    perror("Can't open TrueTIme");
	    exit(-1);
	}
    for(i = 0; i < 100; i++) {
	if(read(ttfd, &ttime, sizeof(ttime)) < 0) {
	    perror("Trouble reading the truetime");
	    return(-1);
	}
	printf("day = %d %d:%d:%d.%06d\n", ttime.yday, ttime.hour,
		ttime.min, ttime.sec, ttime.usec);
#if 0
	for(j = 0; j < 230000; j++);
	if(read(ttfd, &ttime, sizeof(ttime)) < 0) {
	    perror("Trouble reading the truetime");
	    return(-1);
	}
	printf("day = %d %d:%d:%d.%06d\n", ttime.yday, ttime.hour,
		ttime.min, ttime.sec, ttime.usec);
#endif
	usleep(1);
    }
	return(0);
}
