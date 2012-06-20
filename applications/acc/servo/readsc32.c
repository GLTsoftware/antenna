#include <sys/file.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <time.h>

#include "syncclock32.h"

int main(int argc, char *argv[]) {

	struct sc32_time ts1;
	struct timeval time;
	struct timezone zone;
	struct tm *tm;
	int fd, s;

	fd = open("/dev/syncclock32",O_RDWR);
	if(fd==-1) {
	    perror("open()");
	    exit(-1);
	}

	if(gettimeofday(&time, &zone) < 0) {
	    perror("Reading system time");
	    exit(1);
	}

	s = read(fd, &ts1, 1);
	if(s==-1) {
	    perror("read()");
	    exit(-1);
	}



	printf("locked to ref = %d\n", ts1.locked_to_ref);
	printf("IRIG-B time: %4d %3d %02d:%02d:%02d.%06d\n", ts1.year,
		ts1.yday, ts1.hour, ts1.min, ts1.sec, ts1.usec);

	tm = gmtime(&time.tv_sec);
	printf("System time: %4d %3d %02d:%02d:%02d.%06d\n",
		1900 + tm->tm_year, tm->tm_yday + 1, tm->tm_hour,
		tm->tm_min, tm->tm_sec, (int)time.tv_usec);
	close(fd);
	return(0);
}


