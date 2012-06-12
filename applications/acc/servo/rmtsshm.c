#include <sys/types.h>
#define _POSIX_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "tsshm.h"

/* This should be in unistd.h, but the compiler can't find it. */
int usleep(unsigned int useconds);


int main(int argc, char *argv[]) {

	if(shm_unlink(TSSHMNAME) < 0) {
	    perror("Failed to unlink track-servo shm");
	    return(1);
	}
	return(0);
}
