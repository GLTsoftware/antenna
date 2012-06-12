#include <sys/types.h>
#include <errno.h>
/* If this is put ahead of math.h and sys/types.h, it hides some definitions */
#define _POSIX_SOURCE 1
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
extern unsigned int usleep(unsigned int);
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include "smadaemon.h"
#include "tsshm.h"
#include "encoders.h"
#include "servo.h"

int heid_fd, acc_fd;		/* fd's for ACC and Heidenhain encoders */
enum ENCTYPE elEncType, azEncType;
int azEncoderOffset, elEncoderOffset;
int azEncoderReversed, elEncoderReversed;
TrackServoSHM *tsshm;

/* readencoders.c */
static int GetEncoderTypes(void);

int OpenEncoders(int reset) {
    endatioc_t arg;

    if(GetEncoderTypes() != 0) return(QUIT_RTN);
    if(azEncType == HEIDENHAIN || elEncType == HEIDENHAIN) {
        if((heid_fd = open("/dev/endat0", O_RDONLY, 0)) < 0) {
            fprintf(stderr, "Heidenhain encoder(s) not found, ");
            return(QUIT_RTN);
        }
        arg.enc1 = (azEncType == HEIDENHAIN);
        arg.enc2 = (elEncType == HEIDENHAIN);
	if(reset) {
            /* Now reset the interface cdard and the encoders */
            ioctl(heid_fd, EIOCRESETCARD, &arg);
            ioctl(heid_fd, EIOCRESETENC, &arg);
	}

	if( ioctl(heid_fd, EIOCGETOFFS, &arg) < 0) perror("GETOFFS");
        if(azEncType == HEIDENHAIN) {
            azEncoderOffset = arg.offs.azOffset;
            azEncoderReversed = arg.offs.revAz;
        }
        if(elEncType == HEIDENHAIN) {
            elEncoderOffset = arg.offs.elOffset;
            elEncoderReversed = arg.offs.revEl;
        }
    } else {
        heid_fd = -1;
    }
    if(azEncType == ACC || elEncType == ACC) {
        if((acc_fd = open("/dev/encoder0", O_RDONLY, 0)) < 0) {
            fprintf(stderr, "Acc encoder(s) not found, ");
            return(QUIT_RTN);
        }
        ioctl(acc_fd, EIOCGETOFFS, &arg);
        if(azEncType == ACC) {
            azEncoderOffset = arg.offs.azOffset;
            azEncoderReversed = arg.offs.revAz;
        }
        if(elEncType == ACC) {
            elEncoderOffset = arg.offs.elOffset;
            elEncoderReversed = arg.offs.revEl;
        }
    } else {
        acc_fd = -1;
    }
    return(0);
}

void ReadEncoders(int *encAzp, int *encElp) {
    struct enc_result heid_enc;
    struct enc_result acc_enc;

    if(heid_fd >= 0) {
	static short int oldAzStatus = 1;
	static short int oldElStatus = 1;

        read(heid_fd, &heid_enc, sizeof(heid_enc));
        if(azEncType == HEIDENHAIN) {
            *encAzp = ENC_TO_MAS * heid_enc.az;
            if(heid_enc.statusAz != oldAzStatus) {
                fprintf(stderr, "Az encoder status 0x%02x\n",
                        heid_enc.statusAz);
                oldAzStatus = heid_enc.statusAz;
            }
        }
        if(elEncType == HEIDENHAIN) {
            *encElp = ENC_TO_MAS * heid_enc.el;
            if(heid_enc.statusEl != oldElStatus) {
                fprintf(stderr, "El encoder status 0x%02x\n",
			heid_enc.statusEl);
                oldElStatus = heid_enc.statusEl;
            }
        }
    }
    if(acc_fd >= 0) {
        read(acc_fd, &acc_enc, sizeof(acc_enc));
        if(azEncType == ACC) {
            *encAzp = ENC_TO_MAS * acc_enc.az;
        }
        if(elEncType == ACC) {
            *encElp = ENC_TO_MAS * acc_enc.el;
        }
    }
}

static void mkerror(int code) {
    fprintf(stderr, "Bad or missing \"TYPES\" line in encoders.conf %d, ",
	code);
}

static int GetEncoderTypes(void) {
    FILE *configfp;
    char *cp, line[80];

    if((configfp = fopen("/instance/configFiles/encoders.conf", "r")) < 0) {
        perror("Opening encoders.conf");
        return(QUIT_RTN);
    }
    do {
        if(fgets(line, sizeof(line), configfp) == NULL) {
            mkerror(1);
	    return(QUIT_RTN);
        }
    } while(strncmp(line, "TYPES", 4) != 0);
    cp =strtok(line, " \t");
    if((cp = strtok(NULL, " \t")) != 0) {
        if(strncmp(cp, "HEID", 4) == 0) {
            azEncType = HEIDENHAIN;
        } else if(strncmp(cp, "ACC", 3) == 0) {
            azEncType = ACC;
        } else {
            mkerror(2);
	    return(QUIT_RTN);
        }
    } else {
	mkerror(3);
	return(QUIT_RTN);
    }
    if((cp = strtok(NULL, " \t")) != 0) {
        if(strncmp(cp, "HEID", 4) == 0) {
            elEncType = HEIDENHAIN;
        } else if(strncmp(cp, "ACC", 3) == 0) {
            elEncType = ACC;
        } else {
            mkerror(4);
	    return(QUIT_RTN);
        }
    } else {
	mkerror(5);
	return(QUIT_RTN);
    }
    fclose(configfp);
    return(0);
}
#ifdef USE_MAIN
int usecToWait = 0;
int msecToRun = 0;

int main(int argc, char *argv[]) {
    int az, el;
    int startMSec;

    if(argc == 3) {
	usecToWait = atoi(argv[1])*1000;
	if(usecToWait > 19000) usecToWait -= 18000;
	msecToRun = 1000*atoi(argv[2]);
    } else if(argc != 1) {
	fprintf(stderr,"Usage: sampleEncoders <msec/sample sec to sample>\n");
	exit(1);
    }
    tsshm = OpenShm(TSSHMNAME, TSSHMSZ);
    OpenEncoders(0);
    printf("# AZ: type %d offset %d reversed %d   EL: type %d offset %d "
	"reversed %d\n", azEncType, azEncoderOffset, azEncoderReversed,
	elEncType, elEncoderOffset, elEncoderReversed);
    if(usecToWait == 0) {
	ReadEncoders(&az, &el);
	az += tsshm->padAzOffset;
	printf("Az %.4f  El %.4f\n", (float)az/MAS, (float)el/MAS);
    } else {
	startMSec = tsshm->msec;
	printf("#                         Status  Hex\n");
	printf("# Time    Az       El       a e Fault word\n");
	do {
	    if(tsshm->msec < startMSec) startMSec -= (24*3600000);
	    ReadEncoders(&az, &el);
	    az += tsshm->padAzOffset;
	    printf("%7.2f %9.5f %9.5f %d %d %.08x\n",
		    (float)(tsshm->msec - startMSec)/1000.,
		    (float)az/MAS, (float)el/MAS,
		    tsshm->azState, tsshm->elState, tsshm->scbFaultWord);
	    usleep(usecToWait);
	} while(tsshm->msec - startMSec < msecToRun);

    }
    exit(0);
}
#endif /* USE_MAIN */
