/* Servo  communicates with the ACU for the GLT.  Unlike the SMA, the ACU
 * contains a position loop and servo updates the commanded position every
 * 48 ms in addition to controlling communications to the ACU and PTC.
 */

#define GLT 1

#include <math.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <resource.h>
#include <errno.h>
/* If this is put ahead of math.h and sys/types.h, it hides some definitions */
#define _POSIX_SOURCE 1
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
/* This is a non-POSIX function, so it is not seen in unistd.h */
extern unsigned int usleep    _AP((time_t));
#include <string.h>
void bzero(void *s, int n);	/* This should be in string.h, but isn't */
#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <signal.h>
#include <setjmp.h>
#include <sys/stat.h>
#include "tsshm.h"
#include "servo.h"
#include "vme_sg_simple.h"
#define VME_SG_SIMPLE_RATE_1K        ((short)0x02)
#include "dsmsubs.h"
#include "canbus.h"
#include "heartbeat.h"
#include "smadaemon.h"
#include "commonLib.h"
#include "checkencoders.h"
#include "stderrUtilities.h"
#include "dsm.h"

#define USE_SYSTEM_TIME 1
#define CATCH_SIGNALS 0
#define USE_SIGTIMEDWAIT 0
#define DO_TIMESTAMP 0
#define ERR_VERBOSE 0
#define AZ_VERBOSE 0
#define EL_VERBOSE 0
#define dprintf if(verbose) ErrPrintf
#define ddprintf if(verbose > 1) ErrPrintf
#define TO_SHUTDOWN_CNT 100  /* 5 sec */
#define TO_STANDBY_CNT 100  /* 5 sec */
#define TO_ACTIVE_CNT 100
#define TO_STOP_CNT 200

int auxCycle;	/* Sub cycle number for doing infrequent tasks */

static int shutdownSignal = 0;		/* signal main program to shutdown */
static int gotQuit = 0;			/* Return with QUITRTN */
int verbose = 0;

enum DRVSTATE azState, nextAzState, elState, nextElState;
#if AZ_VERBOSE || EL_VERBOSE
static enum DRVSTATE oldAzState, oldElState;
#endif /* AZ_VERBOSE || EL_VERBOSE */

double az_amax, el_amax;

double trAzVmax, trElVmax;		/* Max allowable command velocities */
double avgDt = 0;
struct vme_sg_simple_time ttime;

#if USE_SIGTIMEDWAIT
sigset_t sigs = {{0,0}};
struct timespec timeout= {
                           0, 10000000
                         };
#endif /* USE_SIGTIMEDWAIT */

int lastAz, lastEl;			/* Positions from the ACU */
enum ACUAXISMODE azDriveMode = SHUTDOWN;	/* Mode reported by the ACU */
enum ACUAXISMODE elDriveMode = SHUTDOWN;	/* Mode reported by the ACU */
enum ACUACCESS  ACUAccessMode;
int ACUError;
int LastACUErrorAddress;
int azModeCmd, elModeCmd;
/* The following are used by AzCycle and ElCycle for commands to the ACU */
int nxtAz[2], nxtEl[2];

#if USE_SYSTEM_TIME
struct timeval tv;
struct timezone tz;
struct tm *tmp;
#endif /* USE_SYSTEM_TIME */
int ttfd;			/* File descriptor for the TrueTime device */
int irig_error_count = 0, irig_error_seen = 0;
double lastTimeStep;		/* Time (sec) from the prev clock rtn to now */
int curSec, prevSec;;

int posInSunAvoid;		/* Current posn from Track in avoid */
short requierdSunSafeMinutes;	/* # min ahead of sun zone to avoid */
int presentSunSafeMinutes;	/* # min ahead of sun zone now */
char antmsg[] = "GLT is";

TrackServoSHM *tsshm;			/* Local pointer to shared memory */
/* Values received in shared memory from Track and copied here for use */
enum DRVCMD trAzCmd, trElCmd;		/* Requested drive state */
/*
 * Pos and Vel in mas and mas/sec.  The Raw values are directly from
 * Track, the others have been changed to avoid limits and the sun.
 */
int trAzRaw, trAzVelRaw, trElRaw, trElVelRaw;
int trAz, trAzVel, trEl, trElVel;
int trMsecCmd;			/* Time of last command in msec */
double dt;			/* (tsshm->msec - trMsecCmd)/1000. (sec) */

int azCnt = 0, elCnt = 0;	/* counts 10ms intervals as drives start */
int azTry, elTry;		/* counts startup tries */
float azTrErrorArcSec, elTrErrorArcSec;
int trAzVelBad, trElVelBad;

/* servo.c */
void WaitTickGetTime(void);
void GetACUPosAndStatus(void);
void SetACUMode(int azMode, int elMode);
static void AzCycle(void);
static void ElCycle(void);
#if CATCH_SIGNALS
#if !USE_SIGTIMEDWAIT
static void SigIntHndlr(int);
static void SigQuitHndlr(int signo);
#endif /* USE_SIGTIMEDWAIT */
#endif /* CATCH_SIGNALS */

void ErrPrintf(char *s, ...);

#if EL_VERBOSE > 2 || AZ_VERBOSE > 2
static void DumpMove(MOVE *mp, char *axis, int time);
#endif /* VERBOSE */

void servoUsage(char *programName) {
  fprintf(stderr,
          "Usage: %s [-h] [-v]\n", programName);
  exit(QUIT_RTN);
}

int main(int argc, char *argv[]) {
  int i;
  enum DRVSTATE oldAzState, oldElState;
  char msg[80];

  DAEMONSET
#if USE_SIGTIMEDWAIT
  sigemptyset(&sigs);
  sigaddset(&sigs, SIGHUP);	/* 1 */
  sigaddset(&sigs, SIGINT);	/* 2 */
  sigaddset(&sigs, SIGQUIT);	/* 3 */
  sigaddset(&sigs, SIGTERM);	/* 15 */
#else /* USE_SIGTIMEDWAIT */

#if CATCH_SIGNALS
  if(signal(SIGINT, SigIntHndlr) == SIG_ERR) {
    ErrPrintf("Error setting INT signal disposition\n");
    exit(SYSERR_RTN);
  }
  if(signal(SIGQUIT, SigQuitHndlr) == SIG_ERR) {
    ErrPrintf("Error setting QUIT signal disposition\n");
    exit(SYSERR_RTN);
  }
  if(signal(SIGTERM, SigQuitHndlr) == SIG_ERR) {
    ErrPrintf("Error setting TERM signal disposition\n");
    exit(SYSERR_RTN);
  }
  if(signal(SIGHUP, SigQuitHndlr) == SIG_ERR) {
    ErrPrintf("Error setting HUP signal disposition\n");
    exit(SYSERR_RTN);
  }
#endif /* CATCH_SIGNALS */
#endif /* USE_SIGTIMEDWAIT */

  az_amax = AZ_AMAX;
  el_amax = EL_AMAX;

  for (i=1; i<argc; i++) {
    if (strstr(argv[i],"-h") != NULL) {
      servoUsage(argv[0]);
    }
    if(strstr(argv[i],"-v") != NULL) {
      verbose++;
    }
  }

  /* Set some constants */
  trAzVmax = 3*MAS;
  trElVmax = 3*MAS;

  setpriority(PRIO_PROCESS, (0), (SERVOPRIO));
  umask(0111);
  tsshm = OpenShm(TSSHMNAME, TSSHMSZ);
  if(verbose) {
    fprintf(stderr, "tsshm = %d. ", (int)tsshm);
    fprintf(stderr, "Starting check of tsshm..");
    for(i = 0; i < TSSHMSZ; i++) {
      msg[i%20] = ((char *)tsshm)[i];
    }
    fprintf(stderr, "Done\n");
  }
  /* Set samp buffer full so by default servo will not collect data */
  tsshm->sampIn = tsshm->sampOut = 0;
  if(tsshm->azCmd != OFF_CMD || tsshm->elCmd != OFF_CMD) {
    ErrPrintf("Warning: Track was not commanding drives off\n");
    tsshm->azCmd = tsshm->elCmd = OFF_CMD;
  }
  tsshm->sendEncToPalm = 0;

#if SIMULATING
  scbStatus = 0;
  encAz = 0;
  encEl = 45*MAS;
  elState = azState = 0;
#else /* SIMULATING */
  ttfd = open("/dev/vme_sg_simple", O_RDWR, 0);
  if(ttfd <= 0) {
    ErrPrintf("Error opening TrueTime - /dev/vme_sg_simple\n");
    exit(SYSERR_RTN);
  }
  i = VME_SG_SIMPLE_RATE_1K;
  ioctl(ttfd, VME_SG_SIMPLE_SET_PULSE_RATE, &i);
  tsshm->fault = NO_FAULT;
  

  OpenCntr();			/* Open the heartbeat counter */
  dprintf("Counter, ");
  SetupCanBus();
  dprintf("Canbus, ");
  SafeOpenDsm();
  dprintf("Dsm, ");
  WriteDsmMonitorPoints();
  dprintf("Monitor points, ");
  /* Set a few things in a safe state. */
  azState = SERVOOFF;
  elState = SERVOOFF;
  oldAzState = oldElState = SERVOOFF;

#endif /* SIMULATING */

  /* Set a few things in a safe state. */
  azState = SERVOOFF;
  elState = SERVOOFF;
  oldAzState = oldElState = SERVOOFF;

#if	AZ_VERBOSE > 1 || EL_VERBOSE > 1

  azPrintNext = 0;
  elPrintNext = 0;
#endif /* VERBOSE */

  WaitTickGetTime();            /* Wait for the 48 ms heartbeat */
  dprintf("Tick and time, ");
  GetACUPosAndStatus();
  dprintf("ACU pos, status, ");
  nxtAz[1] = nxtAz[0] = lastAz / ACU_TURNS_TO_MAS;      /* set zero velocity */
  nxtEl[1] = nxtEl[0] = lastAz / ACU_TURNS_TO_MAS;      /* set zero velocity */
  /* if the drives are on, set to standby, otherwise to shutdown */
  if(ACUAccessMode == REMOTE) {
    if(azDriveMode >= STANDBY) {
      azModeCmd = STANDBY;
    }
    if(elDriveMode >= STANDBY) {
      elModeCmd = STANDBY;
    }
    SetACUMode(azDriveMode, elDriveMode);
  }

  if((tsshm->el < 10*MAS) || (tsshm->el > 90*MAS) ||
      (tsshm->az > 270*MAS) || (tsshm->az < -270*MAS)) {
    tsshm->az = lastAz;
    tsshm->azVel = 0;
    tsshm->el = lastEl;
    tsshm->elVel = 0;
  }
  trAz = trAzRaw = tsshm->az;
  trEl = trElRaw = tsshm->el;
  dprintf("completed\n");

  /* Here the infinite loop begins */
  dprintf("Entering servo's main loop\n");
  while(shutdownSignal == 0) {

    tsshm->azState = azState;
    tsshm->elState = elState;
    /* Get time and read encoder */
    WaitTickGetTime();
    GetACUPosAndStatus();
    tsshm->encAz = lastAz;
    tsshm->encEl = lastEl;

#if 1
    CheckTrCmds();	/* Is there a new command from Track? */
#endif
  trAz = trAzRaw = tsshm->az;
  trEl = trElRaw = tsshm->el;
  trAzVel = trAzVelRaw = tsshm->azVel;
  trElVel = trElVelRaw = tsshm->elVel;
    dt = (tsshm->msec - trMsecCmd)/1000.;
    if(dt < -3600)
      dt += 24*3600;
    tsshm->azTrError = trAzRaw + trAzVelRaw * dt - lastAz;
    tsshm->elTrError = trElRaw + trElVelRaw * dt - lastEl;

#if AZ_VERBOSE || EL_VERBOSE

    if(azState != oldAzState || elState != oldElState) {
      printf("at %.3f   azState = %d, elState = %d\n",
             tsshm->msec/1000., azState, elState);
      oldAzState = azState;
      oldElState = elState;
    }
#endif /* AZ_VERBOSE || EL_VERBOSE */
    i = 0;
    if(tsshm->azCmd != trAzCmd) {
      if(tsshm->azCmd == OFF_CMD && azState != SERVOOFF) {
        azState = STOPPING1;
        i = 1;
      } else if(tsshm->azCmd == ON_CMD && azState == SERVOOFF) {
          azTry = 0;
          azCnt = 0;
          azState = STARTING1;
      }
      trAzCmd = tsshm->azCmd;
    }
    if(tsshm->elCmd != trElCmd) {
      if(tsshm->elCmd == OFF_CMD && elState != SERVOOFF) {
        elState = STOPPING1;
        i = 1;
      }
      if(tsshm->elCmd == ON_CMD && elState == SERVOOFF) {
          elTry = 0;
          elCnt = 0;
          elState = STARTING1;
      }
      trElCmd = tsshm->elCmd;
    }
    if(i) {
      if(i == 1) {
        ErrPrintf("servo received OFF_CMD from Track\n");
      }
    }

    if(fabs(dt) > 3.0 && (
       (azState != SERVOOFF && azState != STOPPING1 && azState != STOPPING2) ||
       (elState != SERVOOFF && elState != STOPPING1 && elState != STOPPING2))) {
      ErrPrintf("Track timeout or clock jump, dt %.2f sec.,"
                "avg. dt %.2f sec.\n", dt, avgDt);
#if !SIMULATING

      vSendOpMessage(OPMSG_SEVERE, 19, 60, "Track timeout");
#endif /* !SIMULATING */

      if(azState != SERVOOFF && azState != STOPPING1 && azState != STOPPING2) {
        azState = STOPPING1;
      }
      if(elState != SERVOOFF && elState != STOPPING1 && elState != STOPPING2) {
        elState = STOPPING1;
      }
    }
    avgDt += (dt - avgDt)/10;

    /* If svdata is not running, keep the sample buffer moving */
    if(SFULL) {
      INC_OUT;
    }
    AzCycle();
    ElCycle();

    if(azState != oldAzState || elState != oldElState) {
      if(elState >= TRACKING && azState >= TRACKING) {
        /* Both drives are up, so report the old FaultWord
         * and clear it. */
      }
      if(azState != oldAzState && azState == SERVOOFF) {
        dprintf("Az drive is off\n");
      }
      if(elState != oldElState && elState == SERVOOFF) {
        dprintf("El drive is off\n");
      }
      oldElState = elState;
      oldAzState = azState;
    }
    /* read the parameters from the ACU and send them to dsm at the
     * change of the second */
    curSec = tsshm->msec/1000;
    if(curSec != prevSec ) {
      WriteDsmMonitorPoints();
      prevSec = curSec;
    }

#if !SIMULATING
    /* Make checks of the scb less frequently than once/cycle */
    auxCycle = (tsshm->msec / 10) % 100;
    /* Compute average pointing errors */
    if((auxCycle % 10) == 9) {
      static double ssq = 0;

      /* Instantaneous az error (commanded - actual) */
      azTrErrorArcSec = (double)tsshm->azTrError *
                        cos(RAD_PER_MAS * (trEl + trElVel * dt)) * 0.001;
      ssq += azTrErrorArcSec * azTrErrorArcSec;
      elTrErrorArcSec = (double)tsshm->elTrError * 0.001;
      ssq += elTrErrorArcSec * elTrErrorArcSec;
      dsm_write(dsm_host, "DSM_AZ_TRACKING_ERROR_F", &azTrErrorArcSec);
      dsm_write(dsm_host, "DSM_EL_TRACKING_ERROR_F", &elTrErrorArcSec);
      if(auxCycle == 99) {
        static int trErrCnt = 0;
        static char m[] = "Tracking error is excessive";

        tsshm->avgTrErrorArcSec = sqrt(ssq / 10.);
        ssq = 0.;
        if(tsshm->azCmd > 0 && tsshm->elCmd > 0 &&
            tsshm->avgTrErrorArcSec > 10 &&
            (fabs(tsshm->tachAzVel) < 1.0) &&
            (fabs(tsshm->tachElVel) < 1.0)) {
          if(trErrCnt == 10) {
            vSendOpMessage(OPMSG_SEVERE, 18, 0, m);
          }
          if(trErrCnt < 15)
            trErrCnt++;
        } else {
          if(trErrCnt > 0) {
            if(trErrCnt-- == 10) {
              vSendOpMessage(OPMSG_SEVERE, 18, 0, "");
              trErrCnt = 0;
            }
          }
        }
      }
    }
#endif /* !SIMULATING */

  }
  return((gotQuit)? QUIT_RTN: NORMAL_RTN);
}

static void AzCycle(void) {
  switch(azState) {
  case SERVOOFF:
    break;
  /* The normal entry to SHUTDOWN1 will be when the drive is either in
   * SHUTDOWN or STANDBY mode.  In that case set azCnt to 0.  If the
   * drive is in SHUTDOWN mode, STANDBY will be requested before
   * passing to STANDBY2, otherwise that will happen immediately.
   * When entering STANDBY1 from a failed start attempt, set azCnt to a
   * positive number to wait for the drive to reach SHUTDOWN mode before
   * commanding STANDBY again. */
  case STARTING1:
    if(azDriveMode == SHUTDOWN) {
      dprintf("Commanding az to STANDBY\n");
      azModeCmd = STANDBY;
      SetACUMode(azModeCmd, elModeCmd);
    } else if(azCnt > 0) {
      azCnt--;
      break;
    }
    azCnt = TO_STANDBY_CNT;
    azState = STARTING2;
    break;
  case STARTING2:	/* Request ENCODER Mode once STANDBY is reached */
    if(azDriveMode == STANDBY) {
       dprintf("az in standby, commanding ENCODER\n");
       azModeCmd = ENCODER;
       SetACUMode(azModeCmd, elModeCmd);
       azTry = 0;
       azCnt = TO_ACTIVE_CNT;
       azState = STARTING3;
    } else if(--azCnt <= 0) {	/* Getting to STANDBY failed */
      if(azTry == 0) {		/* If the first try failed, try again */
      dprintf("Az failed to reach STANDBY, returning to SHUTDOWN\n");
        azTry = 1;
        azModeCmd = SHUTDOWN;
        SetACUMode(azModeCmd, elModeCmd);
        azState = STARTING1;
	azCnt = TO_SHUTDOWN_CNT;
      } else {			/* IF the 2nd try fails, give up */
	dprintf("2nd failure to reach STANDBY, quitting\n");
        azState = SERVOOFF;
        azModeCmd = SHUTDOWN;
        SetACUMode(azModeCmd, elModeCmd);
        vSendOpMessage(OPMSG_SEVERE, 15, 10,
		"Az drive failed to enter STANDBY mode");
      }
    }
    break;
  case STARTING3:		/* Wait for ENCODER Mode */
    if(azDriveMode == ENCODER) {
      dprintf("az is tracking in ENCODER mode\n");
      azState = TRACKING;
    } else if(--azCnt <= 0) {	/* Getting to ENCODER failed */
      if(azTry == 0) {
      dprintf("az failed to reach ENCODER mode, retry\n");
        azTry = 1;
        azModeCmd = STANDBY;
        SetACUMode(azModeCmd, elModeCmd);
        azState = STARTING2;
	azCnt = 2*TO_STANDBY_CNT;
      } else {
	dprintf("az failed to reach ENCODER the 2nd time, quitting\n");
        azState = SERVOOFF;
        azModeCmd = SHUTDOWN;
        SetACUMode(azModeCmd, elModeCmd);
        vSendOpMessage(OPMSG_SEVERE, 15, 10,
		"Az drive failed to enter ENCODER mode");
      }
    }
    break;
  case STOPPING1:
    dprintf("Az drive stopping\n");
    nxtAz[1] = nxtAz[0];		/* set zero velocity */
    azCnt = TO_STOP_CNT;
    SetCANValue(AZ_TRAJ_CMD, nxtAz, 8);
    azState = STOPPING2;
    break;
  case STOPPING2:
    if(--azCnt <= 0) {
      dprintf("az set to STANDBY\n");
      azModeCmd = STANDBY;
      SetACUMode(azModeCmd, elModeCmd);
      azState = SERVOOFF;
    }
    break;
  case TRACKING:
    if(azDriveMode != ENCODER) {
      azState = STOPPING1;
      vSendOpMessage(OPMSG_SEVERE, 15, 10, "Az drive stopped");
      break;
    }
    /* Issue the next tracking command */
    nxtAz[0] = (trAz + trAzVel*(dt + HEARTBEAT_PERIOD)) / ACU_TURNS_TO_MAS;
    nxtAz[1] = (trAz + trAzVel*(dt + 2*HEARTBEAT_PERIOD)) / ACU_TURNS_TO_MAS;
    SetCANValue(AZ_TRAJ_CMD, nxtAz, 8);
  }
}

static void ElCycle(void) {
  switch(elState) {
  case SERVOOFF:
    break;
  /* The normal entry to SHUTDOWN1 will be when the drive is either in
   * SHUTDOWN or STANDBY mode.  In that case set elCnt to 0.  If the
   * drive is in SHUTDOWN mode, STANDBY will be requested before
   * passing to STANDBY2, otherwise that will happen immediately.
   * When entering STANDBY1 from a failed start attempt, set elCnt to a
   * positive number to wait for the drive to reach SHUTDOWN mode before
   * commanding STANDBY again. */
  case STARTING1:
    if(elDriveMode == SHUTDOWN) {
      dprintf("Commanding el to STANDBY\n");
      elModeCmd = STANDBY;
      SetACUMode(elModeCmd, elModeCmd);
    } else if(elCnt > 0) {
      elCnt--;
      break;
    }
    elCnt = TO_STANDBY_CNT;
    elState = STARTING2;
    break;
  case STARTING2:	/* Request ENCODER Mode once STANDBY is reached */
    if(elDriveMode == STANDBY) {
       elModeCmd = ENCODER;
       SetACUMode(elModeCmd, elModeCmd);
       elTry = 0;
       elCnt = TO_ACTIVE_CNT;
       elState = STARTING3;
    } else if(--elCnt <= 0) {	/* Getting to STANDBY failed */
      if(elTry == 0) {		/* If the first try failed, try again */
        elTry = 1;
        elModeCmd = SHUTDOWN;
        SetACUMode(elModeCmd, elModeCmd);
        elState = STARTING1;
	elCnt = TO_SHUTDOWN_CNT;
      } else {			/* IF the 2nd try fails, give up */
        elState = SERVOOFF;
        elModeCmd = SHUTDOWN;
        SetACUMode(elModeCmd, elModeCmd);
        vSendOpMessage(OPMSG_SEVERE, 15, 10,
		"El drive failed to enter STANDBY mode");
      }
    }
    break;
  case STARTING3:		/* Wait for ENCODER Mode */
    if(elDriveMode == ENCODER) {
      elState = TRACKING;
    } else if(--elCnt <= 0) {	/* Getting to ENCODER failed */
      if(elTry == 0) {
        elTry = 1;
        elModeCmd = STANDBY;
        SetACUMode(elModeCmd, elModeCmd);
        elState = STARTING2;
	elCnt = 2*TO_STANDBY_CNT;
      } else {
        elState = SERVOOFF;
        elModeCmd = SHUTDOWN;
        SetACUMode(elModeCmd, elModeCmd);
        vSendOpMessage(OPMSG_SEVERE, 15, 10,
		"El drive failed to enter ENCODER mode");
      }
    }
    break;
  case STOPPING1:
    nxtEl[1] = nxtEl[0];		/* set zero velocity */
    elCnt = TO_STOP_CNT;
    SetCANValue(EL_TRAJ_CMD, nxtEl, 8);
    elState = STOPPING2;
    break;
  case STOPPING2:
    if(--elCnt <= 0) {
      elModeCmd = STANDBY;
      SetACUMode(elModeCmd, elModeCmd);
      elState = SERVOOFF;
    }
    break;
  case TRACKING:
    if(elDriveMode != ENCODER) {
      elState = STOPPING1;
      vSendOpMessage(OPMSG_SEVERE, 15, 10, "El drive stopped");
      break;
    }
    /* Issue the next tracking command */
    nxtEl[0] = (trEl + trElVel*(dt + HEARTBEAT_PERIOD)) / ACU_TURNS_TO_MAS;
    nxtEl[1] = (trEl + trElVel*(dt + 2*HEARTBEAT_PERIOD)) / ACU_TURNS_TO_MAS;
    SetCANValue(EL_TRAJ_CMD, nxtEl, 8);
  }
}

#if CATCH_SIGNALS
#if !USE_SIGTIMEDWAIT
/* Subroutine to handle SIGINT (^C) interrupts and shut down gracefully */
static void SigIntHndlr(int signo) {
  shutdownSignal = 1;
  exit(NORMAL_RTN);
}

/* Subroutine to handle SIGQUIT, SIGTERM & SIGHUP interrupts and shut
 * down gracefully */
static void SigQuitHndlr(int signo) {
  shutdownSignal = 1;
  gotQuit = 1;
  exit(QUIT_RTN);
}
#endif /* !USE_SIGTIMEDWAIT */
#endif /* CATCH_SIGNALS */

void WaitTickGetTime(void) {
    static int oldusec = -1;
    int stat;

    ReadCntr();                   /* Wait for the 48 ms heartbeat */
#if USE_SYSTEM_TIME
    gettimeofday(&tv, &tz);
    tmp = gmtime(&tv.tv_sec);
    tsshm->msec = ((tmp->tm_hour * 60 + tmp->tm_min) * 60 + tmp->tm_sec) *
   	 1000 + (tv.tv_usec + 500) / 1000;
    tsshm->day = tmp->tm_yday;
#else /* USE_SYSTEM_TIME */
    /* get the current time in any case */
    if((stat = read(ttfd, &ttime, sizeof(ttime))) < 0) {
      ErrPrintf("Error %d reading TrueTime\n", stat);
    }
    if(ttime.input_reference_error || (ttime.phase_locked ^ 1)) {
      irig_error_seen = 1;
    }
    lastTimeStep = (ttime.usec - oldusec) / 1e6;	/* Time step (sec) */
    if(lastTimeStep < 0) {
      lastTimeStep += 1;
    }
    oldusec = ttime.usec;

    /* Convert the time now */
    tsshm->msec = ((ttime.hour * 60 + ttime.min) * 60 + ttime.sec) * 1000 +
                  (ttime.usec + 500) / 1000;
    tsshm->day = ttime.yday;
#endif /* USE_SYSTEM_TIME */
    ddprintf("day %d msec %d\n", tsshm->day, tsshm->msec);
}

void GetACUPosAndStatus(void) {
  unsigned char buf[8];
  float f;

  if(ReadCANValue(GET_ACU_MODE, buf, 2)) {
      ACUAccessMode = buf[2] = buf[1];
      elDriveMode = buf[1] = buf[0] >> 4;
      azDriveMode = buf[0] = 0xf & buf[0];
      dsm_write(dsm_host, "DSM_ACU_MODE_V3_B", buf);
  }
  if(ReadCANValue(GET_AZ_POSN, buf, 8)) {
    lastAz = (int)(ACU_TURNS_TO_MAS * *(int *)buf);
    f = ACU_TURNS_TO_DEG * *(int *)buf;
    dsm_write(dsm_host, "DSM_AZ_POSN_DEG_F", &f);
  }
  if(ReadCANValue(GET_EL_POSN, buf, 8)) {
    lastEl = (int)(ACU_TURNS_TO_MAS * *(int *)buf);
    f = ACU_TURNS_TO_DEG * *(int *)buf;
    dsm_write(dsm_host, "DSM_EL_POSN_DEG_F", &f);
  }
  if(ReadCANValue(GET_ACU_ERROR, buf, 5)) {
    if((ACUError = buf[0]) != 0) {
      memcpy(&LastACUErrorAddress, buf + 1, 4);
    }
    dsm_write(dsm_host, "DSM_ACU_ERROR_B", buf);
  }
}

void SetACUMode(int azMode, int elMode) {
  static unsigned char modes[2] = {0, 1};  /* Alwaus in remote mode */

  modes[0] = (azMode & 0xf) | (elMode << 4);
  SetCANValue(ACU_MODE_CMD, modes, 2);
}

/*VARARGS*/
void ErrPrintf(char *s, ...) {
  char buf[256];
  va_list ap;

  va_start(ap, s);
  vsprintf(buf, s, ap);
  va_end(ap);
  fputs(buf, stderr);
  fflush(stderr);
}
