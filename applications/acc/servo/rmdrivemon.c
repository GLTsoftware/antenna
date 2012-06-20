#include <curses.h>
/* Compensate for incomplete LynxOS curses.h file */
extern int mvprintw    _AP((int, int, const char *fmt, ...));
#include <math.h>
#include <termio.h>
#include <sys/types.h>
#include <resource.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include "s_cmd2.h"
#include "tsshm.h"
#include "rm.h"
#include "monitor.h"

#define MSEC_PER_DEG (3600000.)
#define OLD 0

int quit = 0;
int antlist[RM_ARRAY_SIZE];
int count = 0;
char hostName[16];

#if OLD
int line;
int oldFault = -1;
int oldM3Cmd = -1;
int oldM3State = -1;
unsigned int oldScbFault = -1, lastLine = 0;
int oldScbStatus = -1;
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
#endif /* OLD */

/* Reflective memory variables */
int rm_open_status;
int antenna_num = 0;
int true_antenna_num;

#if OLD
unsigned char padID, irigLock, palmCodeDate[10];
float padAzOffset, lowerLimit, upperLimit, cwLimit, ccwLimit;
float azRockerCWLimit, azRockerCCWLimit;
float elMotCurrent, azMot1Current, azMot2Current;
float elMotTemp, azMot1Temp,Ji, azMot2Temp;
unsigned char irigLocck, scbEPROMVersion, scbSRAMVersion;
int scbFaultWord;
unsigned char scbStatus, fault, driveState, m3Cmd, m3State;
enum DRV_CMD azCmd, elCmd;
unsigned char azDrvState, elDrvState;
int msecCmd, msecAccept, msec;
float az, el, azVel, elVel, cmdAz, cmdEl;
float tachAzVel, tachElVel;
float encAz, encEl, limAz, limEl;
float azTrErrorArcSec, elTrErrorArcSec, totTrErrorArcSec, trErrorBoxtime;

struct rm_vars {
	void *a;
	char *name;
} rv[] = {
	{&padID, "RM_PAD_ID_B"},
	{&padAzOffset, "RM_PAD_AZ_OFFSET_DEG_F"},
	{&lowerLimit, "RM_SCB_LOW_LIMIT_F"},
	{&upperLimit, "RM_SCB_UP_LIMIT_F"},
	{&cwLimit, "RM_SCB_CW_LIMIT_F"},
	{&ccwLimit, "RM_SCB_CCW_LIMIT_F"},
	{&azRockerCWLimit, "RM_AZ_ROCKER_CW_LIMIT_F"},
	{&azRockerCCWLimit, "RM_AZ_ROCKER_CCW_LIMIT_F"},
	{&scbEPROMVersion, "RM_SCB_EPROM_VERSION_B"},
	{&scbSRAMVersion, "RM_SCB_SRAM_VERSION_B"},
	{&palmCodeDate, "RM_SCB_PALM_CODE_DATE_C10"},
	{&trErrorBoxtime, "RM_TRACKING_ERROR_BOXTIME_SEC_F"},
	{&elMotCurrent, "RM_EL_MOT_CUR_AMP_F"},
	{&azMot1Current, "RM_AZ1_MOT_CUR_AMP_F"},
	{&azMot2Current, "RM_AZ2_MOT_CUR_AMP_F"},
	{&elMotTemp, "RM_EL_MOT_TEMP_C_F"},
	{&azMot1Temp, "RM_AZ1_MOT_TEMP_C_F"},
	{&azMot2Temp, "RM_AZ2_MOT_TEMP_C_F"},
	{&irigLock, "RM_IRIG_LOCK_ERROR_B"},
	{&scbFaultWord, "RM_SCB_FAULTWORD_L"},
	{&scbStatus, "RM_SCB_STATUS_B"},
	{&fault, "RM_SERVO_FAULT_STATE_B"},
	{&driveState, "RM_ANTENNA_DRIVE_STATUS_B"},
	{&azCmd, "RM_AZ_DRV_CMD_B"},
	{&elCmd, "RM_EL_DRV_CMD_B"},
	{&azDrvState, "RM_AZ_DRV_STATE_B"},
	{&elDrvState, "RM_EL_DRV_STATE_B"},
	{&m3Cmd, "RM_M3CMD_B"},
	{&m3State, "RM_M3STATE_B"},
	{&msecCmd, "RM_MSEC_TRACK_CMD_L"},
	{&msecAccept, "RM_MSEC_SERVO_ACCEPT_L"},
	{&msec, "RM_MSEC_NOW_L"},
	{&az, "RM_TRACK_AZ_F"},
	{&el, "RM_TRACK_EL_F"},
	{&azVel, "RM_TRACK_AZ_VEL_F"},
	{&elVel, "RM_TRACK_EL_VEL_F"},
	{&tachAzVel, "RM_TACH_AZ_VEL_F"},
	{&tachElVel, "RM_TACH_EL_VEL_F"},
	{&cmdAz, "RM_SHAPED_CMD_AZ_F"},
	{&cmdEl, "RM_SHAPED_CMD_EL_F"},
	{&encAz, "RM_ENCODER_AZ_F"},
	{&encEl, "RM_ENCODER_EL_F"},
	{&limAz, "RM_LIM_ENCODER_AZ_F"},
	{&limEl, "RM_LIM_ENCODER_EL_F"},
	{&azTrErrorArcSec, "RM_AZ_TRACKING_ERROR_F"},
	{&elTrErrorArcSec, "RM_EL_TRACKING_ERROR_F"},
	{&totTrErrorArcSec, "RM_TRACKING_ERROR_ARCSEC_F"}
};
#define NUMVARS (sizeof(rv) / sizeof(rv[0]))
char drvStateStrings[][6] = DRVSTATE_STRINGS ;
char m3StateStrings[][8] = M3STATE_STRINGS ;

/* rmdrivemon.c */
void parseFaults(unsigned long fault);
void ReadRM(void);
char *StateStrings(int state);
#else /* OLD */

int deadAntennas[MAX_NUMBER_ANTENNAS+1] = {0,0,0,0,0,0,0,0,0,0,0};
double radian=0.01745329;
void antPage2(int count, int *antlist, int antNo, int antName);
char *toddtime(time_t *t, char *str);
char *hsttime(time_t *t, char *str);
#endif /* OLD */
static void SigHndlr(int signo);
void OpenRM(void);

int main(int argc, char *argv[]) {
	struct termio   tio, tin;

	if(argc > 1) {
	    if(strncmp("-h", argv[1], 2) == 0) {
		printf("Usage: rmdrivemon [antenna_number] \n"
			"Antenna number is only needed on hal9000\n");
		exit(1);
	    }
	    antenna_num = atoi(argv[1]);
	}
	OpenRM();
	initscr();
	if(signal(SIGTERM, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting TERM signal disposition\n");
	    exit(1);
	}
	if(signal(SIGQUIT, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting QUIT signal disposition\n");
	    exit(1);
	}
	if(signal(SIGINT, SigHndlr) == SIG_ERR) {
	    fprintf(stderr, "Error setting INT (^C) signal disposition\n");
	    exit(1);
	}
	ioctl(0, TCGETA, &tio);
	tin = tio;
	tin.c_lflag &= ~ECHO;
	tin.c_lflag &= ~ICANON;
	tin.c_cc[VMIN] = 0;
	tin.c_cc[VTIME] = 0;
	ioctl(0, TCSETA, &tin);

    if(antenna_num == 0) {
	if(gethostname(hostName, 15) < 0) {
	    fprintf(stderr, "Gethostname failed\n");
	}
	if(sscanf(hostName, "acc%d", &true_antenna_num) != 1) {
	    fprintf(stderr, "Failed to identify this antenna\n");
	}
    } else {
	true_antenna_num = antenna_num;
    }

#if OLD
    mvprintw(0,8,"Track		   |	servo		%s", hostName);
    mvaddstr(1,3,"state     pos     vel   | state  commanded  encoder  lim"
	" enc Following Error");
    mvaddstr(2,0,"Az");
    mvaddstr(3,0,"El");
    while(! quit) {
	ReadRM();
	if(fault != oldFault) {
	    oldFault ^= LOCKOUT;
	    mvaddstr(0, 53, (fault == LOCKOUT)? ", Palm in Control":
			"in control\n");
	}
	line = 2;
	mvprintw(line++, 4, "%4s %9.4f %7.4f | %5s %9.4f %9.4f %8.3f %10.4f\n",
	    (azCmd)? "on ": "off", az, azVel, StateStrings(azDrvState),
	    cmdAz, encAz, limAz, azTrErrorArcSec / 3600.);
	mvprintw(line++, 4, "%4s %9.4f %7.4f | %5s %9.4f %9.4f %8.3f %10.4f\n",
	    (elCmd)? "on ": "off", el, elVel, StateStrings(elDrvState), cmdEl,
	    encEl, limEl, elTrErrorArcSec / 3600.);

	mvprintw(line++, 0, "msecCmd %8d	   | msecAccept %8d"
	    "	msec %8d", msecCmd, msecAccept, msec);
	if(oldM3Cmd != m3Cmd || oldM3State != m3State) {
	    mvprintw(line, 0, "m3Cmd = %d = %s  m3State = %d = %s\n",
		m3Cmd, (m3Cmd == CLOSE_M3_CMD)? "Closed":
		(m3Cmd == OPEN_M3_CMD)? "open":"Unknown", m3State,
		m3StateStrings[m3State]);
	    oldM3Cmd = m3Cmd;
	    oldM3State = m3State;
	}
	line++;
	mvprintw(line++, 0, "AZ1: temp%6.1f cur %4.1f | "
		"AZ2: temp%6.1f cur %4.1f | EL: temp%6.1f cur %4.1f\n",
		azMot1Temp, azMot1Current, azMot2Temp, azMot2Current,
		elMotTemp, elMotCurrent);
	mvprintw(line++, 0, "limits Az (%8.3f, %7.3f) El (%5.3f, %6.3f)\n",
	    ccwLimit, cwLimit, lowerLimit, upperLimit);
	mvprintw(line++, 0, "EPROM 0x%2x  SRAM 0x%2x Palm Code Date %10s\n",
	    scbEPROMVersion, scbSRAMVersion, palmCodeDate);

	if(scbFaultWord != oldScbFault || scbStatus != oldScbStatus) {
	    mvprintw(line++, 0, "Fault word = 0x%08x  SCB Status 0x%02x  "
		"padID = %d, padAzOffset = %.4f\n",
		scbFaultWord, scbStatus, padID, padAzOffset);
	    if(scbFaultWord != oldScbFault) {
		parseFaults(scbFaultWord);
	    }
	    oldScbFault = scbFaultWord;
	    oldScbStatus = scbStatus;
	}
/*	parseFaults(rand()); */
	refresh();
#else /* OLD */
    while(! quit) {
	antPage2(++count, antlist, antenna_num, true_antenna_num);
#endif /* OLD */
	ioctl(0, TCSETA, &tin);
	sleep(1);
	if(getchar() == 'q')
	    break;
    }
    endwin();
    ioctl(0, TCSETA, &tio);
    return(0);
}

char *hsttime(time_t *t, char *str) {
  /* according to man ctime, the string length is always 26, with \n\0
   * as the final characters.  we want to chop out the \n
   */
  time_t modified_time;
  modified_time = *t-36000;
  return(toddtime(&modified_time,str));
}

char *toddtime(time_t *t, char *str) {
  /* according to man ctime, the string length is always 26, with \n\0
   * as the final characters.  we want to chop out the \n
   */
  strcpy(str,ctime(t));
  str[24] = str[25]; /* move the '0' back one char */
  return(str);
}

double sunDistance(double az1,double el1,double az2,double el2) {
	extern double radian;
	double cosd,sind,d;
	cosd=sin(el1)*sin(el2)+cos(el1)*cos(el2)*cos(az1-az2);
	sind=pow((1.0-cosd*cosd),0.5);
	d=atan2(sind,cosd);
	d=d/radian;
	return d;
}


#if OLD
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
#endif /* OLD */

/* Subroutine to handle SIGINT (^C) interrupts and shut down gracefully */
static void SigHndlr(int signo) {
	quit = 1;
}

void OpenRM(void) {
	int *ip;

	rm_open_status=rm_open(antlist);
	if(rm_open_status != RM_SUCCESS) {
	    rm_error_message(rm_open_status,"rm_open()");
	    exit(1);
	}
	for(ip = antlist; *ip != RM_ANT_LIST_END; ip++) {
	    if(*ip == antenna_num) {
		return;
	    }
	}
	fprintf(stderr, "no card for antenna %d\n", antenna_num);
	exit(1);
}

#if OLD
void ReadRM(void) {
	int i;
	int rm_rtn;

	for(i = 0; i < NUMVARS; i++) {
	    rm_rtn=rm_read(antenna_num, rv[i].name, rv[i].a);
	    if(rm_rtn != RM_SUCCESS) {
		fprintf(stderr, "rm_read of ");
		rm_error_message(rm_rtn, rv[i].name);
	    }
	}
}

char *StateStrings(int state) {
	if(state >= 4) {
	    return(drvStateStrings[4]);
	} else if(state <= 0) {
	    return(drvStateStrings[0]);
	} else {
	    return(drvStateStrings[state]);
	};
}
#endif /* OLD */
