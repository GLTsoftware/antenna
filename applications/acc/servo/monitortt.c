#include <stdio.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <resource.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

#include "vme_sg_simple.h"

int oldRefError = 0, oldPhaseLock = 1;

int main(int argc, char *argv[]) {
    struct vme_sg_simple_time ts1;
    int ms, oldms, oldday, dt;
    int fd, s;

    fd = open("/dev/vme_sg_simple",O_RDWR);
    if(fd==-1) {
        perror("open()");
        exit(-1);
    }

    setpriority(PRIO_PROCESS, (0), 100);
    oldms = 0x7fffffff;
    for(;;) {
        s = read(fd, &ts1, 1);
        if(s==-1) {
            perror("monitorTT: Read of trueTime failed");
            exit(-1);
        }
        ms = ((ts1.hour*60 + ts1.min) * 60 + ts1.sec) * 1000 +
             ts1.usec / 1000;
	dt = ms -oldms;
        if((dt < 19 || dt > 31) && ts1.yday == oldday) {
            fprintf(stderr,
                    " monitortt waited %d ms at %02d:%02d:%02d.%06d\n",
                    dt, ts1.hour, ts1.min, ts1.sec, ts1.usec);
        }
        if(ts1.input_reference_error != oldRefError || ts1.phase_locked
                != oldPhaseLock) {
            fprintf(stderr,
                    "monitortt: %02d:%02d:%02d.%06d ref err %d, phase lock %d\n",
                    ts1.hour, ts1.min, ts1.sec, ts1.usec,
                    ts1.input_reference_error, ts1.phase_locked);
            oldRefError = ts1.input_reference_error;
            oldPhaseLock = ts1.phase_locked;
        }
        oldms = ms;
        oldday = ts1.yday;
        usleep(10000);
    }


    printf("Time now: %4d %3d %02d:%02d:%02d.%06d\n", ts1.year, ts1.yday,
           ts1.hour, ts1.min, ts1.sec, ts1.usec);

    printf("input_reference_error=%d  phase_locked=%d\n",
           ts1.input_reference_error, ts1.phase_locked);
    close(fd);
    return(0);
}


