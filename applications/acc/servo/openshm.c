#include <unistd.h>
#include <stdio.h>
#include <sys/mman.h>

void *OpenShm(char *name, int size) {
	int shmfd;		/* Temporary file pointer for shared memory */
	void *shmp;		/* Temporary pointer to shared memory */

	if((shmfd = shm_open(name, O_RDWR /* | O_CREAT */, 0666)) < 0)  {
	    fprintf(stderr, "OpenShm can't open shared memory %s\n", name);
	    perror("");
	    exit(1);
	}
	if(ftruncate(shmfd, size) < 0) {
	    perror("OpenShm: ftruncate:");
	    exit(1);
	}
	if((shmp = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED,
		shmfd, 0)) == NULL) {
	    perror("OpenShm: mmap:");
	    exit(1);
	}
	close(shmfd);
	return(shmp);
}
