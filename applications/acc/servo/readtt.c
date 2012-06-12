#include <stdio.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

#include "vme_sg_simple.h"

int main(int argc, char *argv[]) {

	struct vme_sg_simple_time ts1;
	int fd, s;

	fd = open("/dev/vme_sg_simple",O_RDWR);
	if(fd==-1) {
	    perror("open()");
	    exit(-1);
	}

	s = read(fd, &ts1, 1);
	if(s==-1) {
	    perror("read()");
	    exit(-1);
	}


	printf("Time now: %4d %3d %02d:%02d:%02d.%06d\n", ts1.year, ts1.yday,
	    ts1.hour, ts1.min, ts1.sec, ts1.usec);

	printf("input_reference_error=%d  phase_locked=%d\n",
	    ts1.input_reference_error, ts1.phase_locked);
	close(fd);
	return(0);
}


