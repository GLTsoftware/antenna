#include <curses.h>
/* Compensate for incomplete LynxOS curses.h file */
extern int mvprintw    _AP((int, int, const char *fmt, ...));
#include <math.h>
#include <termio.h>
#include <sys/types.h>
#include <resource.h>
#define _POSIX_SOURCE
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "tsshm.h"

#define SMA 0
#define MSEC_PER_DEG (3600000.)

int quit = 0;
int line;
char hostName[16];
int oldFault = -1;
int oldM3Cmd = -1;
int oldM3State = -1;
unsigned int oldScbFault = -1, lastLine = 0;
int oldScbStatus = -1;
#if SMA
#include "s_cmd2.h"
struct FAULTS {
	int bit;
	char *msg;
} faults[] = {
    {1 << ELEVATION_UPPER_PRELIM_FAULT, "Elevation upper prelimit"},
    {1 << ELEVATION_LOWER_PRELIM_FAULT, "Elevation lower prelimit"},
    {1 << ELEVATION_UPPER_LIMIT_FAULT, "Elevation upper limit switch"},
    {1 << ELEVATION_LOWER_LIMIT_FAULT, "Elevation lower limit switch"},
    {1 << ELEVATION_ENCODER_FAULT, "Elevation low-res encoder out-of-bounds"},
    {1 << ELEVATION_TEMPERATURE_FAULT, "Elevation motor overtemperature fault"},
    {1 << ELEVATION_CURRENT_FAULT, "Elevation motor overcurrent fault"},
    {1 << ELEVATION_GLENTEK_FAULT, "Elevation Glentek fault"},
    {1 << AZIMUTH_CLOCKWISE_PRELIM_FAULT, "Azimuth clockwise prelimit"},
    {1 << AZIMUTH_COUNTERCLOCKWISE_PRELIM_FAULT, "Azimuth counterclockwise prelimit"},
    {1 << AZIMUTH_CLOCKWISE_LIMIT_FAULT, "Azimuth clockwise limit switch"},
    {1 << AZIMUTH_COUNTERCLOCKWISE_LIMIT_FAULT, "Azimuth counterclockwise limit switch"},
    {1 << AZIMUTH_ENCODER_FAULT, "Azimuth low-res encoder out-of-bounds"},
    {1 << AZIMUTH1_TEMPERATURE_FAULT, "Azimuth motor #1 overtemperature"},
    {1 << AZIMUTH2_TEMPERATURE_FAULT, "Azimuth motor #2 overtemperature"},
    {1 << AZIMUTH1_CURRENT_FAULT, "Azimuth motor #1 overcurrent"},
    {1 << AZIMUTH2_CURRENT_FAULT, "Azimuth motor #2 overcurrent"},
    {1 << AZIMUTH1_GLENTEK_FAULT, "Azimuth motor #1 Glentek fault"},
    {1 << AZIMUTH2_GLENTEK_FAULT, "Azimuth motor #2 Glentek fault"},
    {1 << AZIMUTH_OVERFLOW_FAULT, "Azimuth overflow fault"},
    {1 << EMERGENCY_STOP_FAULT, "Emergency stop pressed"},
    {1 << QUADADC_RESET_FAULT, "Quad ADC reset fault"},
    {1 << COOLANT_FLOW_FAULT, "Coolant flow fault"},
    {1 << HANDPADDLE_BYTE_FRAMING_FAULT, "Hand paddle byte framing fault"},
    {1 << PALMPILOT_SYNTAX_FAULT, "Hand paddle syntax error fault"},
    {1 << PALMPILOT_NO_RESPONSE_FAULT, "Hand paddle no response fault"},
    {1 << ANTENNA_COMPUTER_TIMEOUT_FAULT, "Acc velocity packet timeout fault"},
/*    {1 << ANTENNA_TYPE_FAULT, "Invalid antenna type (use the command 'antenna 0' or 'antenna 1')"}, */
    {1 << AIR_PRESSURE_SWITCH_FAULT, "Air pressure failed on azimuth brake"},
    {1 << AZIMUTH_ROCKER_FAULT, "Azimuth rocker deployment fault"}
};
#define NFAULTS (sizeof(faults) / sizeof(faults[0]))
void parseFaults(unsigned long fault);
#endif

void *openshm(char *name, int size);
static void SigHndlr(int signo);

int main(int argc, char *argv[]) {
	TrackServoSHM *tsshm;		/* Local pointer to shared memory */
	char ch;
	struct termio   tio;

	tsshm = OpenShm(TSSHMNAME, TSSHMSZ);
	if(signal(SIGINT, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting INT signal disposition\n");
	    exit(1);
	}
	initscr();
	if(signal(SIGTERM, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting TERM signal disposition\n");
	    exit(1);
	}
	if(signal(SIGQUIT, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting QUIT signal disposition\n");
	    exit(1);
	}
	ioctl(0, TCGETA, &tio);
	tio.c_lflag &= ~ECHO;
	tio.c_lflag &= ~ICANON;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;
	ioctl(0, TCSETA, &tio);

    if(gethostname(hostName, 15) < 0) {
	fprintf(stderr, "Gethostname failed\n");
    }

    mvprintw(0,8,"Track		   |	servo		%s", hostName);
    mvaddstr(1,3,"state     pos     vel   | state commanded  encoder  lim"
	" enc Following Error");
    mvaddstr(2,0,"Az");
    mvaddstr(3,0,"El");
    while(! quit) {
	if(tsshm->fault != oldFault) {
	    oldFault ^= LOCKOUT;
	    mvaddstr(0, 53, (tsshm->fault == LOCKOUT)? ", Palm in Control":
			"in control\n");
	}
	line = 2;
	mvprintw(line++, 4, "%4s %9.4f %7.4f | %4s %9.4f %9.4f %8.3f %10.4f\n",
	    (tsshm->azCmd)? "on ": "off", tsshm->az/MSEC_PER_DEG,
	    tsshm->azVel/MSEC_PER_DEG,
	    (tsshm->azState)? "on ": "off", tsshm->cmdAz/MSEC_PER_DEG,
	    (double)tsshm->encAz/MSEC_PER_DEG,
	    (double)tsshm->limAz/MSEC_PER_DEG,
	    (double)tsshm->azTrError/MSEC_PER_DEG);
	mvprintw(line++, 4, "%4s %9.4f %7.4f | %4s %9.4f %9.4f %8.3f %10.4f\n",
	    (tsshm->elCmd)? "on ": "off", tsshm->el/MSEC_PER_DEG,
	    tsshm->elVel/MSEC_PER_DEG,
	    (tsshm->elState)? "on ": "off", tsshm->cmdEl/MSEC_PER_DEG,
	    (double)tsshm->encEl/MSEC_PER_DEG,
	    (double)tsshm->limEl/MSEC_PER_DEG,
	    (double)tsshm->elTrError/MSEC_PER_DEG);
	mvprintw(line++, 0, "msecCmd %8d	   | msecAccept %8d"
	    "	day %3d msec %8d",
	    tsshm->msecCmd, tsshm->msecAccept, tsshm->day, tsshm->msec);

	if(oldM3Cmd != tsshm->m3Cmd || oldM3State != tsshm->m3State) {
	    mvprintw(line, 0, "m3Cmd = %d = %s  m3State = %d = %s\n",
		tsshm->m3Cmd, (tsshm->m3Cmd == CLOSE_M3_CMD)? "Closed":
		(tsshm->m3Cmd == OPEN_M3_CMD)? "open":"Unknown", tsshm->m3State,
		(tsshm->m3State == CLOSED_ST)? "Closed":
		(tsshm->m3State == OPEN_ST)? "open":"Unknown\n");
	    oldM3Cmd = tsshm->m3Cmd;
	    oldM3State = tsshm->m3State;
	}
	line++;
	mvprintw(line++, 0, "AZ1: temp%6.1f cur %4.1f | "
		"AZ2: temp%6.1f cur %4.1f | EL: temp%6.1f cur %4.1f\n",
		tsshm->azMot1Temp, tsshm->azMot1Current,
		tsshm->azMot2Temp, tsshm->azMot2Current,
		tsshm->elMotTemp, tsshm->elMotCurrent);
	mvprintw(line++, 0, "limits Az (%8.3f, %7.3f) El (%5.3f, %6.3f)\n",
	    tsshm->ccwLimit / MSEC_PER_DEG, tsshm->cwLimit / MSEC_PER_DEG,
	    tsshm->lowerLimit / MSEC_PER_DEG, tsshm->upperLimit / MSEC_PER_DEG);
	mvprintw(line++, 0,
	    "tachAzVel =  %8.5f  tachElVel %8.5f  IRIG lock err %d\n",
	    (double)tsshm->tachAzVel / MSEC_PER_DEG,
	    (double)tsshm->tachElVel / MSEC_PER_DEG, tsshm->irigLock);
mvprintw(line++, 0,
  "In = %d, Out = %d\n", tsshm->sampIn, tsshm->sampOut);
#if SMA
	if(tsshm->scbFaultWord != oldScbFault ||
		tsshm->scbStatus != oldScbStatus) {
	    mvprintw(line++, 0, "Fault word = 0x%08x  SCB Status 0x%02x  "
		"padID = %d, padAzOffset = %.4f\n",
		tsshm->scbFaultWord, tsshm->scbStatus,
		tsshm->padID, tsshm->padAzOffset / MSEC_PER_DEG);
	    if(tsshm->scbFaultWord != oldScbFault) {
		parseFaults(tsshm->scbFaultWord);
	    }
	    oldScbFault = tsshm->scbFaultWord;
	    oldScbStatus = tsshm->scbStatus;
	}
/*	parseFaults(rand()); */
#endif
	refresh();
	sleep(1);
	while(read(0, &ch, 1)) {
	    if(ch == 'q')
		goto DONE;
	}
    }
DONE:
    endwin();
    return(0);
}

#if SMA
void parseFaults(unsigned long fault) {
	int i, col;

	oldScbFault = fault;
	col = 0;
	for(i = 0; i < NFAULTS; i++) {
	    if(fault & faults[i].bit) {
		mvprintw(line, col, "%-40s", faults[i].msg);
		if(col == 0) {
		    col = 40;
		} else {
		    line++;
		    col = 0;
		}
	    }
	}
	if(col != 0) {
	    mvprintw(line, col, "\n");
	    line++;
	}
	i = lastLine;
	lastLine = line;
	for(; line < i; line++) {
	    mvprintw(line, 0, "\n");
	}
}
#endif

/* Subroutine to handle SIGINT (^C) interrupts and shut down gracefully */
static void SigHndlr(int signo) {
	quit = 1;
}
