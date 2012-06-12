#include <unistd.h>
#include <stdio.h>
#include <sys/mman.h>
#include "tsshm.h"

static void CreateShm(char *name, int size);

int main(int argc, char *argv[]) {
	
	CreateShm(TSSHMNAME, TSSHMSZ);
	return(0);
}

static void CreateShm(char *name, int size) {
	int shmfd;		/* Temporary file pointer for shared memory */

	umask(0111);
	if((shmfd = shm_open(name, O_RDWR | O_CREAT, 0666)) < 0)  {
	    fprintf(stderr, "OpenShm can't open shared memory %s\n", name);
	    perror("");
	    exit(1);
	}
	if(ftruncate(shmfd, size) < 0) {
	    perror("Track: ftruncate:");
	    exit(1);
	}
	close(shmfd);
}
