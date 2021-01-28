#include <math.h>
#include <sys/types.h>
#include <resource.h>
#include <unistd.h>
#define _POSIX_SOURCE
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include "tsshm.h"
#include "s_cmd2.h"

#define MSEC_PER_DEG (3600000.)

char hostName[32];

void parseFaults(unsigned long word);
void *openshm(char *name, int size);
void PutHostEnv(void);

int main(int argc, char *argv[]) {
	TrackServoSHM *tsshm;		/* Local pointer to shared memory */
	int pass;

	tsshm = OpenShm(TSSHMNAME, TSSHMSZ);

    if(argc > 1) {
	pass = atoi(argv[1]);
    } else {
	pass = 1;
    }
    if(gethostname(hostName, 31) < 0) {
	fprintf(stderr, "Gethostname failed\n");
    }
    for(;pass > 0; pass--) {
	printf("az cmd %d, state %d    el cmd %d, state %d  tsshmdump on %s\n",
	    tsshm->azCmd, tsshm->azState,
	    tsshm->elCmd, tsshm->elState, hostName);
	printf("azCmd %9.4f azVel %9.4f curAz %9.4f encAz %9.4f limAz %8.3f\n",
	    tsshm->az/MSEC_PER_DEG, tsshm->azVel/MSEC_PER_DEG,
	    (double)tsshm->cmdAz/MSEC_PER_DEG,
	    (double)tsshm->encAz/MSEC_PER_DEG,
	    (double)tsshm->limAz/MSEC_PER_DEG);
	printf("elCmd %9.4f elVel %9.4f curEl %9.4f encEl %9.4f limEl %8.3f\n",
	    tsshm->el/MSEC_PER_DEG, tsshm->elVel/MSEC_PER_DEG,
	    (double)tsshm->cmdEl/MSEC_PER_DEG,
	    (double)tsshm->encEl/MSEC_PER_DEG,
	    (double)tsshm->limEl/MSEC_PER_DEG);
	printf("Tracking errors: az %.6f  el %.6f\n", tsshm->azTrError /
	    MSEC_PER_DEG, tsshm->elTrError / MSEC_PER_DEG);
	printf("day %d, msec %d, msecCmd %d   "
	    "msecAccept %d\n", tsshm->day,
	    tsshm->msec, tsshm->msecCmd, tsshm->msecAccept);
	printf("m3Cmd = %d = %s  m3State = %d = %s\n", tsshm->m3Cmd,
	    (tsshm->m3Cmd == CLOSE_M3_CMD)? "Closed":
	    (tsshm->m3Cmd == OPEN_M3_CMD)? "open":"Unknown", tsshm->m3State,
	    (tsshm->m3State == CLOSED_ST)? "Closed":
	    (tsshm->m3State == OPEN_ST)? "open":"Unknown");
	printf("El motor temp = %.1f  current %.2f\n", tsshm->elMotTemp,
		tsshm->elMotCurrent);
	printf("Az motor #1 temp = %.1f  current %.2f\n", tsshm->azMot1Temp,
		tsshm->azMot1Current);
	printf("Az motor #2 temp = %.1f  current %.2f\n", tsshm->azMot2Temp,
		tsshm->azMot2Current);
	printf("Fault word = 0x%x %s  padID = %d, padAzOffset = %.4f\n",
		tsshm->scbFaultWord,
		(tsshm->fault == LOCKOUT)? "Palm in Control": "Acc in control",
		tsshm->padID, tsshm->padAzOffset / MSEC_PER_DEG);
	parseFaults(tsshm->scbFaultWord);
	printf("limits El (%.3f, %.3f) Az (%.3f, %.3f)\n",
	    tsshm->lowerLimit / MSEC_PER_DEG, tsshm->upperLimit / MSEC_PER_DEG,
	    tsshm->ccwLimit / MSEC_PER_DEG, tsshm->cwLimit / MSEC_PER_DEG);
	sleep(1);
    }
    return(0);
}

void parseFaults(unsigned long word) {
     unsigned long i;

     for (i=0;i<32;i++) {
       if (word & (1<<i)) {
         switch(i) {
            case ELEVATION_UPPER_PRELIM_FAULT: 
              printf("Elevation upper prelimit\n"); break;
            case ELEVATION_LOWER_PRELIM_FAULT:
              printf("Elevation lower prelimit\n"); break;
            case ELEVATION_UPPER_LIMIT_FAULT:
              printf("Elevation upper limit switch\n"); break;
            case ELEVATION_LOWER_LIMIT_FAULT:
              printf("Elevation lower limit switch\n"); break;
            case ELEVATION_ENCODER_FAULT:
              printf("Elevation low-res encoder out-of-bounds\n"); break;
            case ELEVATION_TEMPERATURE_FAULT:
              printf("Elevation motor overtemperature fault\n"); break;
            case ELEVATION_CURRENT_FAULT:
              printf("Elevation motor overcurrent fault\n"); break;
            case ELEVATION_GLENTEK_FAULT:
              printf("Elevation Glentek fault\n"); break;
            case AZIMUTH_CLOCKWISE_PRELIM_FAULT:
              printf("Azimuth clockwise prelimit\n"); break;
            case AZIMUTH_COUNTERCLOCKWISE_PRELIM_FAULT:
              printf("Azimuth counterclockwise prelimit\n"); break;
            case AZIMUTH_CLOCKWISE_LIMIT_FAULT:
              printf("Azimuth clockwise limit switch\n"); break;
            case AZIMUTH_COUNTERCLOCKWISE_LIMIT_FAULT:
              printf("Azimuth counterclockwise limit switch\n"); break;
            case AZIMUTH_ENCODER_FAULT:
              printf("Azimuth low-res encoder out-of-bounds\n"); break;
            case AZIMUTH1_TEMPERATURE_FAULT:
              printf("Azimuth motor #1 overtemperature\n"); break;
            case AZIMUTH2_TEMPERATURE_FAULT:
              printf("Azimuth motor #2 overtemperature\n"); break;
            case AZIMUTH1_CURRENT_FAULT:
              printf("Azimuth motor #1 overcurrent\n"); break;
            case AZIMUTH2_CURRENT_FAULT:
              printf("Azimuth motor #2 overcurrent\n"); break;
            case AZIMUTH1_GLENTEK_FAULT:
              printf("Azimuth motor #1 Glentek fault\n"); break;
            case AZIMUTH2_GLENTEK_FAULT:
              printf("Azimuth motor #2 Glentek fault\n"); break;
            case AZIMUTH_OVERFLOW_FAULT:
              printf("Azimuth overflow fault\n"); break;
            case EMERGENCY_STOP_FAULT:
              printf("Emergency stop pressed\n"); break;
            case QUADADC_RESET_FAULT:
              printf("Quad ADC reset fault\n"); break;
            case COOLANT_FLOW_FAULT:
              printf("Coolant flow fault\n"); break;
            case HANDPADDLE_BYTE_FRAMING_FAULT:
              printf("Hand paddle byte framing fault\n"); break;
            case PALMPILOT_SYNTAX_FAULT:
              printf("Hand paddle syntax error fault\n"); break;
            case PALMPILOT_NO_RESPONSE_FAULT:
              printf("Hand paddle no response fault\n"); break;
            case ANTENNA_COMPUTER_TIMEOUT_FAULT:
              printf("Antenna computer velocity packet timeout fault\n");
              break;
#if 0
            case ANTENNA_TYPE_FAULT:
              printf("Invalid antenna type (use the command 'antenna 0' or 'antenna 1')\n");
#endif
              break;
            case AIR_PRESSURE_SWITCH_FAULT:
              printf("Air pressure failed on azimuth mechanical brake\n");
              break;
            case AZIMUTH_ROCKER_FAULT:
              printf("Azimuth rocker deployment fault\n");
              break;
	 }
       }
     }
}
