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
#include "dsmsubs.h"
#include "canbus.h"
#if ! GLT
#include "syncclock32.h"
#include "s_constants.h"
#if !SIMULATING
#include "rmsubs.h"
#endif /* !SIMULATING */
#include "palmpilot.h"
#include "rm.h"
#endif /* !GLT */
#include "smadaemon.h"
#include "commonLib.h"
#include "checkencoders.h"
#include "stderrUtilities.h"

#define CATCH_SIGNALS 0
#define USE_SIGTIMEDWAIT 0
#define DO_TIMESTAMP 0
#define ERR_VERBOSE 0
#define AZ_VERBOSE 0
#define EL_VERBOSE 0
#define dprintf if(verbose) ErrPrintf
#define ddprintf if(verbose > 1) ErrPrintf
#define IS_COLLISION_PAD(id) (id < 8 || id == 22)

#define SERVO_DOES_TACH_DIVISORS 0
#define AZ_CLOSED_LOOP 1
#define EL_CLOSED_LOOP 1
#define RECORD 0

#define REFLMEMCYCLE 0
#define M3_CHECK_CYCLE 10
#define FAULTWORDCYCLE 20
#define CHECKSUNCYCLE 30
#define PACKETERRORCYCLE 40
#define INFREQUENTCYCLE 60
#define ENCODERTOPALMCYCLE 70
#define READMOTORCYCLE 80
#define ROCKER_CHECK_CYCLE 90
#define ENCODER_CHECK_CYCLE 95
int auxCycle;	/* Sub cycle number for doing infrequent tasks */

/* #define ENCODER_DIFF_TOLERANCE 5*MAS */
#define ADCFACTOR (1.013*5.0/1024.)  /* 10-bit 80C196 ADC */

static int shutdownSignal = 0;		/* signal main program to shutdown */
static int gotQuit = 0;			/* Return with QUITRTN */
static int cnt = 0;
int beepCnt = 0;
int verbose = 0;
#if ! GLT
static int bypassPreLimits = 0;
static int useSyncClock32 = 0;
static int collisionPossible;
static int prevComNumCalls, prevComTimeouts, prevComBadPkts;
#endif /* !GLT */

enum DRVSTATE azState, nextAzState, elState, nextElState;
#if AZ_VERBOSE || EL_VERBOSE
static enum DRVSTATE oldAzState, oldElState;
#endif /* AZ_VERBOSE || EL_VERBOSE */

#if ! GLT
#define LINUX 0
#define LYNXOS 1
#define ANTENNA 0
#if LINUX
  #define PORT "/dev/cua0"
#elif LYNXOS
  #if ANTENNA == 1
    #define PORT "/dev/IPOP422-1"
  #else
    #define PORT "/dev/IPOP422-0"
  #endif
#endif
#endif /* !GLT */


/* Define a structure to hold all of the information associated with a phase
 * of a move.  When given this structure, movePos can calculate curPos and
 * curVel for this move.
 */
typedef struct {
  int curPos, curVel; /* Current vals from move. (MAS and MAS/sec) */
  int nextVel;	    /* Velocity at end of this cycle */
  int cTime;	    /* Center time of the move (ms) */
  int hTime;	    /* Half of the move time */
  int p0; 	    /* Pos & vel at cntr time extrapolated from */
  int v0; 	    /* previous track (MAS and MAS/sec) */
  double t0;	    /* Scaling for erf and gaussian time args */
  int delPos, delVel; /* Coeff of pos and vel components of move */
}
MOVE;
MOVE azm, elm;

/* Replacement for defined constants */
double az_vmax, el_vmax;
double az_amax, el_amax;
float az_amax_f;			/* for writing into rm */
double trAzVmax, trElVmax;		/* Max allowable command velocities */

double avgDt = 0;
struct vme_sg_simple_time ttime;

#if ! GLT
struct sc32_time sctime, sctime2;
#endif /* !GLT */
#if USE_SIGTIMEDWAIT
sigset_t sigs = {{0,0}};
struct timespec timeout= {
                           0, 10000000
                         };
#endif /* USE_SIGTIMEDWAIT */

int azTurn = 0;
int encAz, encEl;		/* Values read from the encoders (mas) */
int oldEncAz;			/* Prev az reading.  Used to update turns */
double elGearRatio;
double lastTimeStep;		/* Time (sec) from the prev clock rtn to now */

#if ! GLT
int elLimEncZero, azLimEncZero;

/* Things for use with the scb */
char warning[TTYLEN];
char serialPort[40];
int pktErrCnt = 0;
int ttfd;			/* File descriptor for the TrueTime device */
int scfd;			/* File descriptor for the syncclock32 device */
int reportTrueTimeTimeout = 0;
int irig_error_count = 0, irig_error_seen = 0;
int scbStatus;
int azLimitEncoder, elLimitEncoder;
int azLimEncDiff, elLimEncDiff;	/* Limit encoder - fine encoder at time read */
int driveState;
int prelimBypassCycles, softLimBypassCycles;
int azTachDivisor, elTachDivisor;
float azIntegralGain, azProportionalGain, azDerivativeGain, azTorqueBias;
float elIntegralGain, elProportionalGain, elDerivativeGain;
int lowerLimit, collisionLimit;	/* From the SCB */
int rawTachs;
double azVel2Scb, elVel2Scb;	/* Convert mas/sec to scb tach, taking into
				 * account the tach divisors. */
char m3Cmd;
enum M3STATE m3State;
int padAzOffset = 0;		/* Az offset (MAS) of the pad */
int padID = 0;
int azMotorDisable = 0;
int useFakePadID = 0;
int fakePadID = 0;
#else /* !GLT */
int ttfd;			/* File descriptor for the TrueTime device */
int reportTrueTimeTimeout = 0;
int irig_error_count = 0, irig_error_seen = 0;
int nonLockoutSetupDone = 0;
int checkCollisionLimitSwitch = 0;
int suppressAirPressureMessage  = 0;
int prevNumResets;
#if USE_SIGTIMEDWAIT
int ttTimeoutCount = -1;
#else /* USE_SIGTIMEDWAIT */
int ttTimeoutCount = -2;
#endif /* USE_SIGTIMEDWAIT */
#define HORN_MOVE (5*MAS)	/* minimum move distance to sound horn */
#define HORN_DEAD_TIME 300	/* Min time (.01 sec) between moves for horn */
int needsHorn, hornCnt;		/* Sound horn before large moves */
int azRockerBits;
int airPressureSwitch = 0;	/* assume it is not released */
int prevAirPressureSwitch;
#endif /* !GLT */

short requierdSunSafeMinutes;	/* # min ahead of sun zone to avoid */
int presentSunSafeMinutes;	/* # min ahead of sun zone now */
short antInArray;
int mailCount = 0;
unsigned int scbShutdownRequest;
unsigned int scbStowRequest;
int posInSunAvoid;		/* Current posn from Track in avoid */
int myAntennaNumber;
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
int trAzVelBad, trElVelBad;

int shpAz, shpEl;		/* Position derived from the command shaper **/
int azCnt = 0, elCnt = 0;	/* counts 10ms intervals as drives start */
int azTry, elTry;		/* counts startup tries */
#if	AZ_VERBOSE > 1 || EL_VERBOSE > 1
int azPrintNext = 0, elPrintNext = 0;
#endif	/* VERBOSE */

/* The following are returned by AzCycle and ElCycle */
int cmdAzVel,cmdElVel;		/* Vel cmds for vel loop (mas/sec) */
int cmdAzAccel, cmdElAccel;	/* Accel cmds for vel loop (mas/sec^2) */
/* Vel cmds scaled for the scb (+-~32k).  Use int to avoid overflow */
int scbAzVel,scbElVel;

#if ! GLT
/* Reflective memory stuff */
float azTrErrorArcSec, elTrErrorArcSec;
float trErrorBoxtime = 1.0;
int azRockerCWLimit, azRockerCCWLimit;
int scbEPROMVersion, scbSRAMVersion;
char palmDateCode[10];
char elCollisionLimits[9] = {0,0,0,0,0,0,0,0,0};
#endif /* !GLT */

/* servo.c */
static void AzCycle(void);
static void ElCycle(void);
static void movePos(MOVE *mp, int time);
#if ! GLT
static double GearRatio(int el);
#endif /* !GLT */
#if !SIMULATING
static void OpenScb(void);
void checkCollision(void);
#if ! GLT
static void SetM3Cmd(void);
#endif /* !GLT */
void PrintSamples(void);
void ReadScbParameters(void);
void NonLockoutSetup(void);
static void CloseScb(char *s);
static void InitEncoders(void);
static void ReadLimEncoders(int *azEnc, int *elEnc);
static void SetScbState(void);
#endif /* !SIMULATING */
#if CATCH_SIGNALS
#if !USE_SIGTIMEDWAIT
static void SigIntHndlr(int);
static void SigQuitHndlr(int signo);
#endif /* USE_SIGTIMEDWAIT */
#endif /* CATCH_SIGNALS */
static void GetPosTime();
static void GetNextTickTime(void);
#if 0
static void PrintTime(struct vme_sg_simple_time *tp);
#endif
#if !SIMULATING
static void PrintEncoders(void);
static void printStatusFault(void);
static void ReadFault(int *faultword, int *storedfaultword);
static void parseFaults(unsigned long word);
static void parseStatusByte(unsigned int statusByte);
float Thermistor(unsigned short counts);
void CheckResets(void);
void SetPalmCodeDate(void);
#endif /* !SIMULATING */

void ErrPrintf(char *s, ...);

#if EL_VERBOSE > 2 || AZ_VERBOSE > 2
static void DumpMove(MOVE *mp, char *axis, int time);
#endif /* VERBOSE */

void servoUsage(char *);

void servoUsage(char *programName) {
  fprintf(stderr,
          "Usage: %s [-h]\\\n"
          "\t[-az_vmax<vmax: default=4.0>] [-el_vmax<vmax: default=2.0>]\\\n"
          "\t[-az_amax<amax: default=5.0>] [-el_amax<amax: default=8.7>]\\\n"
          "\t[-e] [-t] [-p] [-v [-v]] [-s <serialPort: default=%s>]\n"
          "\t\tAz vmax should be in [0.1, 4] and El in [0.1, 2]\n"
          "\t\tAz and El amax should be in [2.0, 12.0]\n"
          "\t\t-e turns on encoder checking\n"
          "\t\t-tt use the truetime time card for time and pacing.\n"
          "\t\t-t report to stderr when pacing differs from 10 msec intervals\n"
          "\t\t-p turns on prelimit bypassing\n"
          "\t\t-v is for more verbose diagnostics\n",
          programName,PORT);
  exit(QUIT_RTN);
}


int main(int argc, char *argv[]) {
  int i;
  enum DRVSTATE oldAzState, oldElState;
#if !SIMULATING

  enum M3CMD oldM3Cmd = -1;
#endif /* !SIMULATING */

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

  az_vmax = AZ_VMAX_DEFAULT;
  el_vmax = EL_VMAX_DEFAULT;
  az_amax = AZ_AMAX;
  el_amax = EL_AMAX;
  for (i=1; i<argc; i++) {
    if (strstr(argv[i],"-h") != NULL) {
      servoUsage(argv[0]);
    }
    if(strstr(argv[i],"-tt") != NULL) {
      useSyncClock32 = 0;
    } else if(strstr(argv[i],"-s") != NULL) {
      if (++i < argc) {
        sscanf(argv[i],"%s",serialPort);
        ErrPrintf("SILENT Will use RS422 serial port = %s\n",serialPort);
      } else {
        servoUsage(argv[0]);
      }
    } else if (strstr(argv[i],"-el_vmax") != NULL) {
      if(argv[i][sizeof("-el_vmax") - 1] != NULL) {
        el_vmax = atof(argv[i] + sizeof("-el_vmax") - 1);
      } else if(++i < argc) {
        el_vmax = atof(argv[i]);
      } else {
        el_vmax = 0;
      }
      if(el_vmax < 0.1 || el_vmax > 2.0) {
        ErrPrintf("Bad el_vmax (%.3f) specified.\n", el_vmax);
        servoUsage(argv[0]);
      }
      ErrPrintf("el_vmax set to %.3f\n", el_vmax);
      el_vmax *= MAS;
    } else if (strstr(argv[i],"-az_vmax") != NULL) {
      if(argv[i][sizeof("-az_vmax") - 1] != NULL) {
        az_vmax = atof(argv[i] + sizeof("-az_vmax") - 1);
      } else if(++i < argc) {
        az_vmax = atof(argv[i]);
      } else {
        az_vmax = 0;
      }
      if(az_vmax < 2 || az_vmax > 12) {
        ErrPrintf("Bad az_vmax (%.3f) specified.\n", az_vmax);
        servoUsage(argv[0]);
      }
      ErrPrintf("az_vmax set to %.3f\n", az_vmax);
      az_vmax *= MAS;
    } else if (strstr(argv[i],"-el_amax") != NULL) {
      if(argv[i][sizeof("-el_amax") - 1] != NULL) {
        el_amax = atof(argv[i] + sizeof("-el_amax") - 1);
      } else if(++i < argc) {
        el_amax = atof(argv[i]);
      } else {
        el_amax = 0;
      }
      if(el_amax < 2 || el_amax > 12.0) {
        ErrPrintf("Bad el_amax (%.3f) specified.\n", el_amax);
        servoUsage(argv[0]);
      }
      ErrPrintf("el_amax set to %.3f\n", el_amax);
      el_amax *= MAS;
    } else if (strstr(argv[i],"-az_amax") != NULL) {
      if(argv[i][sizeof("-az_amax") - 1] != NULL) {
        az_amax = atof(argv[i] + sizeof("-az_amax") - 1);
      } else if(++i < argc) {
        az_amax = atof(argv[i]);
      } else {
        az_amax = 0;
      }
      if(az_amax < 2.0 || az_amax > 12.0) {
        ErrPrintf("Bad az_amax (%.3f) specified.\n", az_amax);
        servoUsage(argv[0]);
      }
      ErrPrintf("az_amax set to %.3f\n", az_amax);
      az_amax *= MAS;
    } else if (strstr(argv[i],"-p") != NULL) {
      bypassPreLimits = 1;
    } else if (strstr(argv[i],"-t") != NULL) {
      reportTrueTimeTimeout = 1;
    } else if (strstr(argv[i],"-v") != NULL) {
      verbose++;
#if !SIMULATING

    } else if (strncmp(argv[i],"-e", 2) == NULL) {
      InitCheckEncoders(-100, -100);
#endif /* !SIMULATING */

    } else {
      servoUsage(argv[0]);
    }
  }
  az_amax_f = az_amax / MAS;

  /* Set some constants */
  trAzVmax = az_vmax * 0.76;
  trElVmax = el_vmax * 0.76;


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

#if ! GLT
  scbStatus = 0;
#endif /* !GLT */
  encAz = 0;
  encEl = 45*MAS;
  elState = azState = 0;
  ttfd = open("/dev/vme_sg0", O_RDWR, 0);
  if(ttfd <= 0) {
    ErrPrintf("Error opening TrueTime - /dev/vme_sg_simple\n");
    exit(SYSERR_RTN);
  }
  i = VME_SG_SIMPLE_RATE_1K;
  ioctl(ttfd, VME_SG_SIMPLE_SET_PULSE_RATE, &i);
  OpenCntr();			/* Open the heartbeat counter */
  tsshm->fault = NO_FAULT;
#else /* SIMULATING */

#if GLT
  SetupCanBus();
  SafeOpenDsm();
#else /* GLT */
  /* initializing ref. mem. */

  OpenRM();
  dprintf("completed\nOpening RM ... ");
  dprintf("SetScbState ... ");
  OpenScb();
  InitEncoders();
  dprintf("completed\nInitEncoders... ");
  if((i = rm_write(RM_ANT_0, "RM_EL_COLLISION_LIMITS_V9_B",
	elCollisionLimits)) != RM_SUCCESS) {
    rm_error_message(i, "Initial check of RM_EL_COLLISION_LIMITS_V9_B");
  }
  RM_WRITE_ALL;			/* Init all regular RM variables */
  usleep(1000000);
  SetPalmCodeDate();
#endif /* !GLT */
  dprintf("completed\n");
#endif /* SIMULATING */

  /* Set a few things in a safe state. */
  azState = SERVOOFF;
  elState = SERVOOFF;
  oldAzState = oldElState = SERVOOFF;
  cmdAzVel = cmdElVel = 0;
  SetM3Cmd();

#if	AZ_VERBOSE > 1 || EL_VERBOSE > 1

  azPrintNext = 0;
  elPrintNext = 0;
#endif /* VERBOSE */

#if GLT
  ReadCntr();			/* Wait for the 48 ms heartbeat */
  if(!useSyncClock32)
    GetNextTickTime();
  GetPosTime();
  if((tsshm->el < 10*MAS) || (tsshm->el > 90*MAS) ||
      (tsshm->az > 360*MAS) || (tsshm->az < -180*MAS)) {
    tsshm->az = encAz;
    tsshm->azVel = 0;
    tsshm->el = encEl;
    tsshm->elVel = 0;
  }
  trAz = trAzRaw = tsshm->az;
  trEl = trElRaw = tsshm->el;

  /* Here the infinite loop begins */
  dprintf("Entering servo's main loop\n");
  while(shutdownSignal == 0) {

    /* Get time and read encoder */
    tsshm->azState = azState;
    tsshm->elState = elState;
#if ! GLT
    WaitTickGetTime();
#else  /* GLT */
    GetPosTime();
#endif /* !GLT */
    tsshm->encAz = encAz;
    tsshm->encEl = encEl;

    CheckTrCmds();	/* Is there a new command from Track? */
    dt = (tsshm->msec - trMsecCmd)/1000.;
    if(dt < -3600)
      dt += 24*3600;
    tsshm->azTrError = trAzRaw + trAzVelRaw * dt - encAz;
    tsshm->elTrError = trElRaw + trElVelRaw * dt - encEl;

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
        azState = STOPPING;
        i = 1;
      } else if(tsshm->azCmd == ON_CMD && azState == SERVOOFF) {
        if(tsshm->padID > 26 || tsshm->padID < 1) {
          i = 2;
        } else if(tsshm->fault == LOCKOUT) {
          i = 3;
        } else if(airPressureSwitch) {
          i = 4;
        } else if(collisionPossible) {
	  i = 5;
	} else if(azIntegralGain < 10) {
	  i = 6;
        } else {
          if(azCnt > 0)
            azCnt = 0;
          azTry = 0;
          azState = STARTING;
#if !SIMULATING

          if(! nonLockoutSetupDone)
            NonLockoutSetup();
#endif /* !SIMULATING */

        }
      }
      trAzCmd = tsshm->azCmd;
    }
    if(tsshm->elCmd != trElCmd) {
      if(tsshm->elCmd == OFF_CMD && elState != SERVOOFF) {
        elState = STOPPING;
        i = 1;
      }
      if(tsshm->elCmd == ON_CMD && elState == SERVOOFF) {
        if(tsshm->padID > 26) {
          i = 2;
        } else if(tsshm->fault == LOCKOUT) {
          i = 3;
        } else if(collisionPossible) {
	  i = 5;
	} else if(elIntegralGain < 10) {
	  i = 6;
        } else {
          if(elCnt > 0)
            elCnt = 0;
          elTry = 0;
          elState = STARTING;
#if !SIMULATING

          if(! nonLockoutSetupDone)
            NonLockoutSetup();
#endif /* !SIMULATING */

        }
      }
      trElCmd = tsshm->elCmd;
    }
    if(i) {
      static char *reason[] = {"No pad ID", "Palm in control",
	       "Az Brake Manually Released", "Collision Possible",
	       "Gains Too Low (Transporter gains?)"};

      beepCnt = 2;
      if(i == 1) {
        ErrPrintf("servo received OFF_CMD from Track\n");
      } else {
        sprintf(msg, "Tracking not allowed: %s", reason[i-2]);
        vSendOpMessage(OPMSG_SEVERE, 19, 60, msg);
      }
    }

    if(fabs(dt) > 3.0 && (
          (azState != SERVOOFF && azState != STOPPING) ||
          (elState != SERVOOFF && elState != STOPPING))) {
      ErrPrintf("Track timeout or clock jump, dt %.2f sec.,"
                "avg. dt %.2f sec.\n", dt, avgDt);
#if !SIMULATING

      vSendOpMessage(OPMSG_SEVERE, 19, 60, "Track timeout");
#endif /* !SIMULATING */

      if(azState != SERVOOFF && azState != STOPPING) {
        azState = STOPPING;
        beepCnt = 2;
      }
      if(elState != SERVOOFF && elState != STOPPING) {
        elState = STOPPING;
        beepCnt = 2;
      }
    }
    avgDt += (dt - avgDt)/10;

#if !SIMULATING

    if(azState < STOPPING && elState < STOPPING) {
      SetScbState();
    }
#endif /* !SIMULATING */
    /* If svdata is not running, keep the sample buffer moving */
    if(SFULL) {
      INC_OUT;
    }
    AzCycle();
    tsshm->cmdAz = shpAz;
    ElCycle();
    tsshm->cmdEl = shpEl;
    cnt++;

    if(azState >= STOPPING || elState >= STOPPING) {
#if SIMULATING
#else /* SIMULATING */
      ComStartPkt(AZELVELOCITY);
      ComPutS(scbElVel);
      /* Scale the acceleration feed forward to the same units as
      * the velocity command, but divide by 3 to prevent
      * overflow in a single byte.
      ( */
      ComPutC((int)(cmdElAccel * elGearRatio * elVel2Scb *
                    (0.333333333 * 100./558.)));
      ComPutS((short)scbAzVel);
      ComPutC((int)(cmdAzAccel * azVel2Scb *
                    (0.333333333 * 100./558.)));
      if(SendPktGetRtn(AZELVELOCITY)) {
	if(++pktErrCnt < 4) goto VEL_PKT_ERR;
        azState = SERVOOFF;
        elState = SERVOOFF;
        tsshm->azState = tsshm->elState = SERVOOFF;
        SetScbState();
        if(tsshm->fault == LOCKOUT) {
          ErrPrintf("Shutting down: Palm took control of scb\n");
        }
        vSendOpMessage(OPMSG_SEVERE, 19, 60,
                       "Drives shut down: Errors sending Vel Cmd to SCB");
	PrintSamples();
        beepCnt = 2;
      } else {
        scbStatus = ComGetC();
	pktErrCnt = 0;
      }
read(scfd, &sctime2, sizeof(sctime2));
#endif /* SIMULATING */
      tsshm->scbStatus = scbStatus;
      /* Loop performance data */
        SaI.msec = tsshm->msec;
        SaI.scbStatus = scbStatus;
        SaI.encAz = encAz;
        SaI.encEl = encEl;
#if SIMULATING

        SaI.tachElVel = 0;
        SaI.elTorq = 0;
        SaI.tachAzVel = 0;
        SaI.azTorq = 0;
#else /* SIMULATING */

        SaI.tachElVel = tsshm->tachElVel =  (((int)ComGetS()) << 1) /
                                            (elGearRatio * elVel2Scb);
        SaI.elTorq = ComGetS();
        SaI.tachAzVel = tsshm->tachAzVel =  (((int)ComGetS()) << 1) /
                                            azVel2Scb;
        SaI.azTorq = ComGetS();
#endif /* SIMULATING */

        tsshm->sampIn = NEXT_SAMP(tsshm->sampIn);
#if !SIMULATING
      if(azState >= TRACKING && !(scbStatus & STATUS_AZIMUTH_SERVO)) {
        azState = STOPPING;
        ErrPrintf( "[[SCB az drive stopped, trAz = %.4f trAzVel = %.3f,"
                   " cmdAzVel = %.3f\n", trAz / (double)MAS,
                   trAzVel / (double)MAS, cmdAzVel / (double)MAS);
        PrintEncoders();
        printStatusFault();
        vSendOpMessage(OPMSG_SEVERE, 19, 60, "SCB az drive stopped");
	PrintSamples();
        beepCnt = 2;
      }
      if(elState >= TRACKING && !(scbStatus & STATUS_ELEVATION_SERVO)) {
        elState = STOPPING;
        ErrPrintf( "[[SCB el drive stopped, trEl = %.4f trElVel = %.3f,"
                   " cmdElVel = %.3f\n", trEl / (double)MAS,
                   trElVel / (double)MAS, cmdElVel / (double)MAS);
        if(azState != STOPPING) {
          PrintEncoders();
        }
        printStatusFault();
        vSendOpMessage(OPMSG_SEVERE, 19, 60, "SCB el drive stopped");
	PrintSamples();
        beepCnt = 2;
      }
#endif /* !SIMULATING */

    }

    if(azState != oldAzState || elState != oldElState) {
      if(elState >= TRACKING && azState >= TRACKING) {
        /* Both drives are up, so report the old FaultWord
         * and clear it. */
#if !SIMULATING
        ComStartPkt(CLRFAULTWORD);
        SendPktGetRtn(CLRFAULTWORD);
        tsshm->scbFaultWord = ComGetL();
        dprintf("Both drives came up: fw = 0x%x\n", tsshm->scbFaultWord);
        CheckResets();
#endif /* !SIMULATING */

      }
      if(azState != oldAzState && azState == SERVOOFF) {
        dprintf("Az drive is off\n");
        beepCnt = 2;
      }
      if(elState != oldElState && elState == SERVOOFF) {
        dprintf("El drive is off\n");
        beepCnt = 2;
      }
      oldElState = elState;
      oldAzState = azState;
    }
VEL_PKT_ERR:
#if !SIMULATING
    CheckEncoders();
#endif /* !SIMULATING */

#if !SIMULATING
    /* Make checks of the scb less frequently than once/cycle */
    auxCycle = (tsshm->msec / 10) % 100;
    switch(auxCycle) {
    case ROCKER_CHECK_CYCLE :

      ComStartPkt(READDIGIN);
      SendPktGetRtn(READDIGIN);
      azRockerBits = ((unsigned char)ComGetC()) >> AZIMUTH_ROCKER1;
      (void)ComGetC();
      prevAirPressureSwitch = airPressureSwitch;
      i = ComGetC();
      airPressureSwitch = (0x04 & i) == 0;
      if(prevAirPressureSwitch != airPressureSwitch) {
        UPDATE_SOME_INIT;
      }
#if 1
      scbShutdownRequest = (i >> 7) & 1;
      scbStowRequest = (ComGetC() & 1) ^ 1;
#else
      if((i >> 7) & 1) scbShutdownRequest++;
      if(ComGetC() & 1) scbStowRequest++;
#endif
      if(bypassPreLimits) {
        ComStartPkt(BYPASSPRELIMS);
        ComPutS(27800); /* prelimit bypass: 100seconds*278 */
        ComPutS(0);     /* softlimit bypass */
        if(SendPktGetRtn(BYPASSPRELIMS)) break;
        prelimBypassCycles = (unsigned short)ComGetS();
        softLimBypassCycles = (unsigned short)ComGetS();
      } else {
        ComStartPkt(READBYPASSPRELIMS);
        if(SendPktGetRtn(READBYPASSPRELIMS)) break;
        prelimBypassCycles =  (unsigned short)ComGetS();
        softLimBypassCycles = (unsigned short)ComGetS();
      }
      break;
    case ENCODER_CHECK_CYCLE:
      ReadLimEncoders(&azLimitEncoder, &elLimitEncoder);

      if(abs(azLimEncDiff) > (0.9*AZ_ENCODER_DIFF_TOLERANCE)
          && azState != SERVOOFF) {
        ErrPrintf( "Az encoders differ fine = %.4f, Lim = %.2f\n",
                   encAz*(1.0/MAS), azLimitEncoder*(1.0/MAS));
        azState = STOPPING;
        vSendOpMessage(OPMSG_SEVERE, 19, 60,
                       "Turning off Az drive: encoders differ");
        beepCnt = 2;
      }
      if(abs(elLimEncDiff) > (0.9*EL_ENCODER_DIFF_TOLERANCE)
          && elState != SERVOOFF) {
        ErrPrintf( "El encoders differ fine = %.4f, Lim = %.2f\n",
                   encEl*(1.0/MAS), elLimitEncoder*(1.0/MAS));
        elState = STOPPING;
        vSendOpMessage(OPMSG_SEVERE, 19, 60,
                       "Turning off El drive: encoders differ");
        beepCnt = 2;
      }
      break;
    case M3_CHECK_CYCLE:
      (void)rm_read(RM_ANT_0, "RM_M3CMD_B", &m3Cmd);
      if((m3Cmd != oldM3Cmd) && (tsshm->fault != LOCKOUT)) {
        ComStartPkt(SETM3);
        ComPutC((m3Cmd == CLOSE_M3_CMD)? CLOSE_M3: OPEN_M3);
        SendPktGetRtn(ACK);
        oldM3Cmd = m3Cmd;
        m3State = MOVING_ST;

      } else {
        ComStartPkt(READM3);
        SendPktGetRtn(READM3);
        i = (ComGetC() << 1);
        i |= ComGetC();
        (void)ComGetC();		/* pass over m3control */
        switch(i) {
        case 0:
          if(ComGetS() == 0) {	/* Timeout is zero */
            m3State = STUCK_ST;
          }
          break;
        case 3:
          if(ComGetS() == 0) {	/* Timeout is zero */
            m3State = (m3Cmd == CLOSE_M3_CMD)?
                      CLOSED_QUESTION: OPEN_QUESTION;
          }
          break;
        default:
          m3State = i;
        }
      }
      break;
    case FAULTWORDCYCLE:
      if(tsshm->fault != LOCKOUT) {
        ComStartPkt(CLRFAULTWORD);
        if(SendPktGetRtn(CLRFAULTWORD)) break;
      } else {
        ComStartPkt(FAULTWORD);
        if(SendPktGetRtn(FAULTWORD)) break;
      }
      tsshm->scbFaultWord = ComGetL();
#if 1
      if(suppressAirPressureMessage  > 0) {
	suppressAirPressureMessage --;
	tsshm->scbFaultWord &= ~(1 << AIR_PRESSURE_SWITCH_FAULT);
      }
#endif
      if(tsshm->scbFaultWord) dprintf("fw = 0x%x\n", tsshm->scbFaultWord);
      break;
    case FAULTWORDCYCLE + 2:
      UPDATE_MONITOR_AND_STATE;
      break;
    case READMOTORCYCLE:
      ComStartPkt(READMOTORS);
      SendPktGetRtn(READMOTORS);
      tsshm->elMotTemp = Thermistor(ComGetS() & 0x03ff);
      tsshm->elMotCurrent = (ADCFACTOR*20)*(0x03ff&ComGetS());
      tsshm->azMot1Temp = Thermistor(ComGetS() & 0x03ff);
      tsshm->azMot1Current = (ADCFACTOR*20)*(0x03ff&ComGetS());
      tsshm->azMot2Temp = Thermistor(ComGetS() & 0x03ff);
      tsshm->azMot2Current = (ADCFACTOR*20)*(0x03ff&ComGetS());
      break;
    case CHECKSUNCYCLE:
      /* Now check for sun avoidance */
      (void)rm_read(RM_ANT_0, "RM_PROJECT_STATUS_S", &antInArray);
      presentSunSafeMinutes = SunSafeMinutes(tsshm->encAz, tsshm->encEl);
      if(presentSunSafeMinutes >= 0 && padID < 27) {
        if(presentSunSafeMinutes == 0) {
          if(antInArray < IN_ARRAY) {
            sprintf(msg, "%s in the Sun avoidance zone %.1f deg from Sun",
		    (antInArray == LOW_IN_ARRAY)? "Array antennas are": antmsg,
                    (double)(i = SunDistance(tsshm->encAz, tsshm->encEl))/MAS);
            if(azState < SLEWING && elState < SLEWING) {
	      strcat(msg, "Email will be sent after 2 minutes");
              vSendOpMessage(OPMSG_EMERGENCY, 20, 5, msg);
	      if(!verbose && ++mailCount == 120) {
		ErrPrintf("MAIL=*antennaInSun %s\n", msg);
	      }
            } else if(i < RELAXEDLIMIT) {
              ErrPrintf("%s\n", msg);
            }
          }
        } else if(presentSunSafeMinutes < abs(requiredSunSafeMinutes)) {
          if(antInArray < IN_ARRAY && azState < SLEWING && elState < SLEWING) {
            sprintf(msg, "%s within %d minutes of the Sun avoidance zone",
		    (antInArray == LOW_IN_ARRAY)? "Array antennas are": antmsg,
                    presentSunSafeMinutes);
	    if(azState < TRACKING && elState < TRACKING) {
		strcat(msg, "And the drives are off.  "
		    "Email will be sent after 5 minutes");
	        if(!verbose && ++mailCount == 300) {
		    ErrPrintf("MAIL=*antennaInSun %s\n", msg);
		}
	    }
            vSendOpMessage((presentSunSafeMinutes > 10)? OPMSG_WARNING:
                           OPMSG_SEVERE, 20, 5, msg);
          }

	} else {
	  mailCount = 0;
        }
      } else {
	mailCount = 0;
      }
      break;
#if ! GLT
    /* When the SCB is timing out, ~half of the cycles are skipped */
    case PACKETERRORCYCLE:
    case PACKETERRORCYCLE+1:
      i = comNumCalls - prevComNumCalls;
      if(i < 3) break;
#if 0
printf("auxCycle %d num %d numnew %d Timeout %d newTimeout %d bad %d newBad %d\n",
	auxCycle, comNumCalls, i, comTimeouts, comTimeouts - prevComTimeouts,
	comBadPkts, comBadPkts - prevComBadPkts);
#endif
      if(comTimeouts - prevComTimeouts == i) {
        vSendOpMessage(OPMSG_SEVERE, 28, 10, "No response from the SCB");
      } else if(comBadPkts-prevComBadPkts+comTimeouts-prevComTimeouts > 2) {
        vSendOpMessage(OPMSG_SEVERE, 29, 10, "SCB Communication problems");
      }
      prevComTimeouts = comTimeouts;
      prevComBadPkts = comBadPkts;
      prevComNumCalls = comNumCalls;
      break;
#endif /* !GLT */
    case REFLMEMCYCLE:
      /* Adjust Limit Encoder values for motion since reading */
      tsshm->limAz = encAz + azLimEncDiff;
      tsshm->limEl = encEl + elLimEncDiff;
      UPDATE_TIME_POSITION;
      RMTimestamp();
      break;
    case REFLMEMCYCLE + 4:
      driveState = azState && elState;
      if(useSyncClock32) {
        tsshm->irigLock = !sctime.locked_to_ref;
      } else {
        tsshm->irigLock = (ttime.input_reference_error << 1) |
                          (ttime.phase_locked ^ 1);
      }
      if(irig_error_seen) {
        irig_error_seen = 0;
        irig_error_count++;
      } else {
        static int initialCountNotCleared = 10;

        if(initialCountNotCleared > 0) {
          if(--initialCountNotCleared <= 0)
            irig_error_count = 0;
        }
      }
      UPDATE_MONITOR_AND_STATE;
      break;
    case INFREQUENTCYCLE:
#if ! GLT
      if(tsshm->msec < 61000 && tsshm->msec > 60000) {
        struct timespec tp;
        ComStartPkt(SET_PALM_TIME);
        clock_gettime(CLOCK_REALTIME,&tp);
        ComPutL(tp.tv_sec);
        SendPktGetRtn(ACK);
      }
#endif /* !GLT */
      if(beepCnt > 0) {
        /* Only one shutdown message to un-clutter stderr file */
        if(--beepCnt == 1)
          ErrPrintf("Warning: drive shutdown\a\n");
      }
      /* Things to do every 10 sec. */
      if(((tsshm->msec/1000) % 10) == 1) {
        /* Get the pad azimuth offset */
        ComStartPkt(READPADID);
        if(SendPktGetRtn(READPADID) == 0) {
          padID = ComGetC();
          useFakePadID = ComGetC();
          fakePadID = ComGetC();
          padAzOffset = ComGetL() * 1000;	/* change to MAS */
          checkCollisionLimitSwitch = ComGetC();
	  tsshm->lowerLimit = ((checkCollisionLimitSwitch)? collisionLimit:
		lowerLimit);
          tsshm->padID = (useFakePadID)? fakePadID: padID;
          tsshm->padAzOffset = padAzOffset;
	  checkCollision();
          UPDATE_SOME_INIT;
        }
        CheckResets();
      }
      break;
    case ENCODERTOPALMCYCLE:
      if(tsshm->sendEncToPalm != NOTHING_TO_PALM) {
        ComStartPkt(TOPALM);
        if(tsshm->sendEncToPalm == AZ_TO_PALM) {
          ComPutC(AZ_HIGHRES_ENCODER);
          ComPutL(encAz / ENC_TO_MAS);
        } else {
          ComPutC(EL_HIGHRES_ENCODER);
          ComPutL(encEl / ENC_TO_MAS);
        }
        SendPktGetRtn(ACK);
      }
      break;
    default:
      if(needsHorn) {
        ComStartPkt(SOUNDHORN);
        ComPutS(2*ONE_SEC);
        ComPutC(BEEPING_HORN);
        SendPktGetRtn(ACK);
        needsHorn = 0;
        hornCnt = HORN_DEAD_TIME;
      } else if(hornCnt > 0) {
        if(azState > TRACKING || elState > TRACKING) {
          hornCnt = HORN_DEAD_TIME;
        } else {
          hornCnt--;
        }
      }
    }
#if 1
    /* Compute average pointing errors */
    if((auxCycle % 10) == 9) {
      static double ssq = 0;

      /* Instantaneous az error (commanded - actual) */
      azTrErrorArcSec = (double)tsshm->azTrError *
                        cos(RAD_PER_MAS * (trEl + trElVel * dt)) * 0.001;
      ssq += azTrErrorArcSec * azTrErrorArcSec;
      elTrErrorArcSec = (double)tsshm->elTrError * 0.001;
      ssq += elTrErrorArcSec * elTrErrorArcSec;
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
#endif
#endif /* !SIMULATING */

  }

#if !SIMULATING
  CloseScb("Received a signal");
#endif /* SIMULATING */

#if RECORD

  WriteData();
#endif /* RECORD */

  return((gotQuit)? QUIT_RTN: NORMAL_RTN);
}

static void AzCycle(void) {
  /* At each cycle set to anticipated next position and velocity.
   * Used as the the current position & vel in setting up moves. */
  static int nxtAz, nxtAzVel;
  int curAz, curAzVel;

START_AZ:
  curAz = trAz + trAzVel*dt;
  curAzVel = trAzVel;
#if 0

  if(abs(curAzVel) < AZ_VCRIT) {
    azStopPosn =  curAz + curAzVel * (AZ_MIN_HTIME/1000.);
  } else {
    azStopPosn =  curAz + curAzVel * ((double)abs(curAzVel)) *
                  ((AZ_MIN_HTIME/1000.) / AZ_VCRIT);
  }
  if(azStopPosn > tsshm->cwLimit) {
    curAz = tsshm->cwLimit;
    curAzVel = 0;
  } else if(azStopPosn < tsshm->ccwLimit) {
    curAz = tsshm->ccwLimit;
    curAzVel = 0;
  }
#endif

#if 0
  p.taz = tsshm->az;
  p.tazVel = tsshm->azVel;
  p.tmsecCmd = tsshm->msecCmd;
  p.tmsecAccept = tsshm->msecAccept;
  p.tday = tsshm->day;
  p.tmsec = tsshm->msec;
  p.curAz = curAz;
  p.trAz = trAz;
  p.trAzVel = trAzVel;
#endif

  cmdAzAccel = 0; 	/* true unless in a move */
RESTART_AZ:
  switch(azState) {
  case SERVOOFF:
#if	    AZ_VERBOSE > 1

    azPrintNext = 0;
#endif	    /* AZ_VERBOSE */
    /* There should be code here to turn the Glentecks off
     * and then if rm.posType changes, respond by turning
     * the Glenteks on and changing to TRACKING mode.  For now
     * just write out data and die.
     */
    cmdAzVel = 0;
    if(azCnt < 0)
      azCnt++;
    break;
  case STARTING:
    /* at this point, there should be a failure in the air pressure
     * if not, then it may be overriden in the cabin, so we should
     * not start up */
    if(azCnt == 0) {
#if SIMULATING
      scbStatus |= STATUS_AZIMUTH_SERVO;
#else /* SIMULATING */

      ComStartPkt(AZSERVOON);
      ComPutC(ENABLE);
      SendPktGetRtn(ACK);
#endif /* SIMULATING */

      tsshm->az = encAz;
      tsshm->azVel = 0;
      suppressAirPressureMessage = 15;
      azCnt++;
      break;
    }
    if(azCnt++ < 10) {
      break;			/* wait a bit to retry */
    }
    if(scbStatus & STATUS_AZIMUTH_SERVO) {
      nxtAz = encAz;
      nxtAzVel = 0;
      azState = TRACKING;
      goto START_AZ;
    }
    if((scbStatus & (STATUS_AZIMUTH_HARDFAULT |
                     STATUS_AZIMUTH_SOFTFAULT |
                     STATUS_DATA_READY_HANDLER)) ||
			azCnt > 800) {
      if(azCnt > 800) {
        if(azTry == 0) {
          azTry = 1;
          azCnt = 0;
          ErrPrintf(
            "[[Az not started after 8 sec: trying again\n");
	  printStatusFault();
          azState = STARTING;
          goto START_AZ;
        } else {
          ErrPrintf("Az not started after two 8 sec. tries\n");
	  suppressAirPressureMessage = 0;
        }
      }
      if(azTry == 0) {
        azTry = 1;
        azCnt = -10;
        ErrPrintf( "[[Az fault on startup.  Will retry.\n");
#if !SIMULATING

        printStatusFault();
#endif /* !SIMULATING */

        azState = STARTING;
        goto START_AZ;
      }

      ErrPrintf("[[Shutting down Az\n");
#if !SIMULATING

      printStatusFault();
#endif /* !SIMULATING */

      azState = STOPPING;
      beepCnt = 2;
      goto START_AZ;
    }
    cmdAzVel = 0;
    break;
  case STOPPING:
    if(abs(cmdAzVel) > 1000)
      azCnt = -2000;
    if(cmdAzVel > 0) {
      cmdAzVel -= (AZ_VMAX_DEFAULT/100);
      if(cmdAzVel < 0)
        cmdAzVel = 0;
    } else {
      cmdAzVel += (AZ_VMAX_DEFAULT/100);
      if(cmdAzVel > 0)
        cmdAzVel = 0;
    }
    if(cmdAzVel == 0) {
#if SIMULATING
      scbStatus &= ~STATUS_AZIMUTH_SERVO;
#else /* SIMULATING */

      ComStartPkt(AZSERVOON);
      ComPutC(DISABLE);
      SendPktGetRtn(ACK);
#endif /* SIMULATING */

      azState = SERVOOFF;
    }
    break;
  case TRACKING:
#if	    AZ_VERBOSE > 1

    if(azPrintNext) {
      printf("#nxtAz = %10.5f, azm.curVel = %9.5f\n"
             "#curAz = %10.5f,	 curAzVel = %9.5f\n",
             nxtAz/(double)MAS, azm.curVel/(double)MAS,
             curAz/(double)MAS, curAzVel/(double)MAS);
      azPrintNext = 0;
    }
#endif	    /* AZ_VERBOSE */
    /* Check to see if source position has changed enough that
     * a move sequence should be initiated.  The tolerance will
     * need to be determined experimentally and should probably
     * be a function of elevation angle.  For now use 5"Arc which
     * will allow moves to transition to tracking up to 89 deg
     * elevation in spite of nonlinear motion in Az at high el.
     */
    if(abs(nxtAz - curAz) > POS_TOLERANCE ||
        abs(curAzVel - nxtAzVel) > VEL_TOLERANCE) {
      double eqn;
      int delP;

      azm.delVel = curAzVel - nxtAzVel;
      delP = curAz - nxtAz;
      if(abs(delP) > HORN_MOVE)
        needsHorn = 1;

      if(az_amax < az_vmax * M_SQRT2/EXP_HALF*M*500/AZ_MIN_HTIME) {

        /* If we get here, single step moves in Az are limited
         * by the max acceleration available and are optimim up to
         * MIN_HTIME*4 msec of move time.  The following code
         * calculates the minimum t0 (and thus move time) for
         * this move given AMAX.
         */
        if((delP ^ azm.delVel) > 0) { /*delP*azm.delVel overflows*/
          eqn = ((0.120985*M + 0.170883)/az_amax);
        } else {
          eqn = ((-0.120985*M + 0.170883)/az_amax);
        }
        eqn *= abs(azm.delVel);
        azm.t0 = eqn + sqrt((0.48393/az_amax)*abs(delP) + eqn*eqn);

      } else {
        azm.t0 = 0;
      }

      if(az_amax*2 >= az_vmax * M_SQRT2/EXP_HALF*M*500/AZ_MIN_HTIME) {
        double tmpt0;
        /* If we get here, the max velocity may be the limiting
         * factor in the move. */
        if(delP < 0) {
          tmpt0 = -delP/((2./M_2_SQRTPI)*(az_vmax + (curAzVel + nxtAzVel)/2) +
                         M*azm.delVel/2.);
        } else {
          tmpt0 = delP/((2./M_2_SQRTPI)*(az_vmax - (curAzVel + nxtAzVel)/2) -
                        M*azm.delVel/2.);
        }
        if(tmpt0 > azm.t0)
          azm.t0 = tmpt0;
      }
      azm.t0 *= 1000.;

      /* Now see if a short move is appropriate. */
      if(azm.t0 <= (4.0*AZ_MIN_HTIME/M)) {
        if(azm.t0 < (2.0*AZ_MIN_HTIME/M)) {
          azm.t0 = 2.0*AZ_MIN_HTIME/M;
        }
        /* Round hTime to nearest 10 ms */
        azm.hTime = 0.5 + azm.t0*(M/20.0);
        azm.hTime *= 10;

        azm.cTime = tsshm->msec + azm.hTime;
        azm.v0 = nxtAzVel;
        /* Current position projected to cTime at curVel */
        azm.p0 = nxtAz + ((double)azm.v0 * azm.hTime) / 1000;
        azm.delPos = delP + ((double)azm.delVel*azm.hTime) *
                     0.001;
        nextAzState = TRACKING;
        azState = SLEWING;
#if		AZ_VERBOSE > 2

        printf("#Az Start short move\n");
        DumpMove( &azm, "Az", tsshm->msec);
#endif		/* AZ_VERBOSE */

      } else {
        int vmax, vsum, vsel, dP;

        /* If we got here, the move is too long for a short move and we
        * must now set up a Long move with acceleration to Vmax,
        * a possible Const Vel section, and deceleration to the new track's
        * velocity.  EL_AMAX, el_vmax, and MIN_HTIME are potential limits.
        * The move parameters will be chosen with equal accel and decel time.
        * Since the initial and final velocities may be different,
        * vsel is selected as the one requiring the larger velocity change
        * and used in setting up move parameters so that EL_AMAX will
         * not be exceeded in either case.  The time lost to this
        * simplification is negligable except above el = 88 deg.
         */
        if(az_amax < az_vmax * M_2_SQRTPI*M*250/AZ_MIN_HTIME) {
          vsum = nxtAzVel - 3*curAzVel;
          /* In solving the quadratic eqn we need the absolute
                  * value of the motion and velocities */
          if(delP < 0) {
            vsum = - vsum;
            vsel = (curAzVel>nxtAzVel)? -curAzVel: -nxtAzVel;
            dP = - delP;
          } else {
            vsel = (curAzVel < nxtAzVel)? curAzVel: nxtAzVel;
            dP = delP;
          }
          eqn = (2*vsel + vsum)/4;
          vmax = (2*vsel - vsum)/4 + (int)sqrt(AZ_ACC_CONST*dP +
                                               eqn*eqn);

          if(vmax > az_vmax) {
            vmax = az_vmax;
            nextAzState = LONGMOVECV_SETUP;
          } else {
            nextAzState = LONGMOVEDECEL;
          }
          /* Round hTime to the nearest 10 ms */
          azm.hTime = 0.5 + (500/(AZ_ACC_CONST*10))*
                      (vmax - vsel);
          azm.hTime *= 10;
          if(azm.hTime < AZ_MIN_HTIME) {
#if		    AZ_MIN_HTIME == 500
            vmax = dP - vsum/2;
#else

            vmax =(int)((500./AZ_MIN_HTIME)*dP) - vsum/2;
#endif

            azm.hTime = AZ_MIN_HTIME;
          }
        } else {

          /* With this combination of parameters we do the
                  * much simpler velocity limited move. */
          vmax = abs((int)(delP*(1000./(AZ_MIN_HTIME*2)) +
                           azm.delVel));
          azm.hTime = AZ_MIN_HTIME;
          if(vmax > az_vmax) {
            vmax = az_vmax;
            nextAzState = LONGMOVECV_SETUP;
          } else {
            nextAzState = LONGMOVEDECEL;
          }
        }
        azm.cTime = tsshm->msec + azm.hTime;
        azm.v0 = nxtAzVel;
        /* Current position projected to cTime at curVel */
        azm.p0 = nxtAz + ((double)azm.v0 * azm.hTime) / 1000;
        azm.t0 = azm.hTime * (2.0/M);
        if(delP < 0)
          vmax = -vmax;
        azm.delVel = vmax - nxtAzVel;
        azm.delPos = 0;
        azState = SLEWING;
#if		AZ_VERBOSE > 2

        printf("#Az Start long move accel\n");
        DumpMove( &azm, "Az", tsshm->msec);
#endif		/* AZ_VERBOSE */

      }
      goto RESTART_AZ;
    }
    /* This is the core of the linear az position loop.  Velocity
     * feed forward is not really needed, but used for consistency.
     * Accel feed forward is not needed or used.
     */
    nxtAz = curAz + curAzVel / 100;
    nxtAzVel = curAzVel;
    cmdAzVel =
#if AZ_CLOSED_LOOP
      (curAz - encAz) * AZ_GAIN +
#endif

      curAzVel;
    break;
  case SLEWING:
    /* The task here is to follow the planned move and change to
     * the nextAzState at its end.
     */
    movePos(&azm, tsshm->msec);
    shpAz = azm.curPos;
    nxtAzVel = (azm.curVel + azm.nextVel)/2;
    /* Velocity feed forward is needed here since the position
     * loop may not have enough bandwidth to follow the accel
     * path, but the vel loop will.
     */
    cmdAzVel =
#if	    AZ_CLOSED_LOOP
      (shpAz - encAz) * AZ_GAIN +
#endif
      nxtAzVel;
    cmdAzAccel = azm.nextVel - azm.curVel;

    if(tsshm->msec >= azm.cTime + azm.hTime) {
      nxtAz = shpAz;
      azState = nextAzState;
#if		AZ_VERBOSE > 1

      azPrintNext = 1;
#endif		/* AZ_VERBOSE */

      goto START_AZ;
    }
    break;
  case LONGMOVEDECEL:
    azm.delVel = curAzVel - nxtAzVel;
    azm.cTime = tsshm->msec + azm.hTime;
    azm.v0 = nxtAzVel;
    /* Current position projected to cTime at curVel */
    azm.p0 = nxtAz + ((double)azm.v0 * azm.hTime) / 1000;
    azm.t0 = azm.hTime * (2.0/M);
    azm.delPos = curAz - nxtAz + ((double)azm.delVel*azm.hTime) *
                 0.001;
    /* At this point we would expect that if we simply decelerated
     * to the final velocity we would be very close to the target
     * position and velocity.  Thus unless a new track has been
     * given, delPos should be small.  We prefer to use the hTime
     * calculated at acceleration time, so if delPos isn't small,
     * the safest thing to do is just stop and then let TRACKING
     * start a new move.
     */
    if(abs(azm.delPos) > 0.1 * MAS) {
#if		AZ_VERBOSE
      printf("#Az Long Move Decel: Position error %10.5f\n",
             azm.delPos/(double)MAS);
#endif		/* AZ_VERBOSE */

      azm.delPos = 0;
    }
    azState = SLEWING;
    nextAzState = TRACKING;
#if	    AZ_VERBOSE > 2

    printf("#Az Start long move decel\n");
    DumpMove( &azm, "Az", tsshm->msec);
#endif	    /* AZ_VERBOSE */

    goto START_AZ;
    break;
  case LONGMOVECV_SETUP:
    azState = LONGMOVECV;
    nextAzState = LONGMOVEDECEL;
#if	    AZ_VERBOSE > 2

    printf("#Az Start CV\n");
    DumpMove( &azm, "Az", tsshm->msec);
#endif	    /* AZ_VERBOSE */

    goto AZCVENTRY;	/* Skip position update the first time */
  case LONGMOVECV:
    azm.curPos += azm.curVel * lastTimeStep;
AZCVENTRY:

    /* Picking the time to start deceleration is the trick. */
    if((nxtAzVel > curAzVel &&
        azm.curPos + 0.001*(nxtAzVel-curAzVel)*azm.hTime >= curAz) ||
        (nxtAzVel <= curAzVel &&
         azm.curPos + 0.001*(nxtAzVel-curAzVel)*azm.hTime <= curAz)) {

      nxtAz = azm.curPos;
      azState = nextAzState;
      nextAzState = TRACKING;
      goto START_AZ;
    }
    shpAz = azm.curPos;
    nxtAzVel = (azm.curVel + azm.nextVel)/2;
    /* velocity feed forward is used here because the commanded
     * velocity is high and there would be a position lag
     * otherwise causing a transient when switching to decel.
     */
    cmdAzVel =
#if AZ_CLOSED_LOOP
      (shpAz - encAz) * AZ_GAIN +
#endif
      nxtAzVel;
    break;
  }
  /* shpAz should always have a value.  When just tracking it should be
   * the same as the current request */
  if(azState <= TRACKING) {
    shpAz = curAz;
  }
  /* Loop performance data */
#if 0
  if(! SFULL) {
#endif
    SaI.curAz = curAz;
    SaI.shpAz = shpAz;
    SaI.cmdAzVel = cmdAzVel;
    SaI.azState = azState;
#if 0
  }
#endif
  /* Do this conversion in floating point for now. */
  /* Need to deal with az accel */
  scbAzVel = cmdAzVel * azVel2Scb;
  if(scbAzVel > MAX_AZ_SCB_VEL) {
    scbAzVel = MAX_AZ_SCB_VEL;
    cmdAzAccel = 0;
  }
  if(scbAzVel < -MAX_AZ_SCB_VEL) {
    scbAzVel = -MAX_AZ_SCB_VEL;
    cmdAzAccel = 0;
  }
}

static void ElCycle(void) {
  /* At each cycle set to anticipated next position and velocity.
   * Used as the the current position & vel in setting up moves. */
  static int nxtEl, nxtElVel;
  int curEl, curElVel;

START_EL:
  curEl = trEl + trElVel*dt;
  curElVel = trElVel;
#if 0

  if(abs(curElVel) < EL_VCRIT) {
    elStopPosn =  curEl + curElVel * (EL_MIN_HTIME/1000.);
  } else {
    elStopPosn =  curEl + curElVel * ((double)abs(curElVel)) *
                  ((EL_MIN_HTIME/1000.) / EL_VCRIT);
  }
  if(elStopPosn > tsshm->upperLimit) {
    curEl = tsshm->upperLimit;
    curElVel = 0;
  } else if(elStopPosn < tsshm->lowerLimit) {
    curEl = tsshm->lowerLimit;
    curElVel = 0;
  }
#endif

#if 0
  p.tel = tsshm->el;
  p.telVel = tsshm->elVel;
  p.curEl = curEl;
  p.trEl = trEl;
  p.trElVel = trElVel;
#endif

  cmdElAccel = 0; 	/* true unless in a move */
  switch(elState) {
  case SERVOOFF:
#if	    EL_VERBOSE > 1

    elPrintNext = 0;
#endif	    /* EL_VERBOSE */

    cmdElVel = 0;
    if(elCnt < 0)
      elCnt++;
    break;
  case STARTING:
    if(elCnt == 0) {
#if SIMULATING
      scbStatus |= STATUS_ELEVATION_SERVO;
#else /* SIMULATING */

      ComStartPkt(ELSERVOON);
      ComPutC(ENABLE);
      SendPktGetRtn(ACK);
#endif /* SIMULATING */

      tsshm->el = encEl;
      tsshm->elVel = 0;
      elCnt++;
      break;
    }
    if(elCnt++ < 10) {
      break;			/* wait a bit to retry */
    }
    if(scbStatus & STATUS_ELEVATION_SERVO) {
      nxtEl = encEl;
      nxtElVel = 0;
      tsshm->elState = elState = TRACKING;
      goto START_EL;
    }
    if((scbStatus & (STATUS_ELEVATION_HARDFAULT |
                     STATUS_ELEVATION_SOFTFAULT |
                     STATUS_DATA_READY_HANDLER)) ||
        elCnt > 800) {
      if(elCnt > 800) {
        if(elTry == 0) {
          elTry = 1;
          elCnt = 0;
          ErrPrintf(
            "[[El not started after 8 sec: trying again\n");
	  printStatusFault();
          elState = STARTING;
          goto START_EL;
        } else {
          ErrPrintf("El not started after two 8 sec. tries\n");
        }
      }
      if(elTry == 0) {
        elTry = 1;
        elCnt = -10;
        ErrPrintf( "[[El fault on startup.  Will retry.\n");
#if !SIMULATING

        printStatusFault();
#endif /* !SIMULATING */

        elState = STARTING;
        goto START_EL;
      }
      ErrPrintf("[[Shutting down El\n");
#if !SIMULATING

      printStatusFault();
#endif /* !SIMULATING */

      elState = STOPPING;
      goto START_EL;
    }
    cmdElVel = 0;
    break;
  case STOPPING:
    /* If there is a substantial velocity, delay 20 sec before
     * allowing a restart */
    if(abs(cmdElVel) > 1000)
      elCnt = -2000;
    if(cmdElVel > 0) {
      cmdElVel -= (EL_VMAX_DEFAULT/100);
      if(cmdElVel < 0)
        cmdElVel = 0;
    } else {
      cmdElVel += (EL_VMAX_DEFAULT/100);
      if(cmdElVel > 0)
        cmdElVel = 0;
    }
    if(cmdElVel == 0) {
#if SIMULATING
      scbStatus &= ~STATUS_ELEVATION_SERVO;
#else /* SIMULATING */

      ComStartPkt(ELSERVOON);
      ComPutC(DISABLE);
      SendPktGetRtn(ACK);
#endif /* SIMULATING */

      elState = SERVOOFF;
      tsshm->elState = SERVOOFF;
    }
    break;
  case TRACKING:
#if	    EL_VERBOSE > 1

    if(elPrintNext) {
      printf("#elm.curEl = %10.5f, elm.curVel = %9.5f\n"
             "#	  curEl = %10.5f,   trElVel = %9.5f\n",
             nxtEl/(double)MAS, elm.curVel/(double)MAS,
             curEl/(double)MAS, trElVel/(double)MAS);
      elPrintNext = 0;
    }
#endif	    /* EL_VERBOSE */
    /* Check to see if source position has changed enough that
     * a move sequence should be initiated.  The tolerance will
     * need to be determined experimentally.
     */
    if(abs(nxtEl - curEl) > POS_TOLERANCE ||
        abs(curElVel - nxtElVel) > VEL_TOLERANCE) {
      double eqn;
      int delP;

      elm.delVel = curElVel - nxtElVel;
      delP = curEl - nxtEl;
      if(abs(delP) > HORN_MOVE)
        needsHorn = 1;

      if(el_amax < el_vmax * M_SQRT2/EXP_HALF*M*500/EL_MIN_HTIME) {

        /* The Elevation drive is capable of very high
         * accelerations.  If we get here, single step moves in
         * El are limited by the max acceleration limit (EL_AMAX)
         * and are optimim up to MIN_HTIME*4 msec of move time.
         * The following code calculates the minimum t0 (and
         * thus move time) for this move given AMAX.
         */
        if((delP ^ elm.delVel) > 0) { /*delP*elm.delVel overflows*/
          eqn = ((0.120985*M + 0.170883)/el_amax);
        } else {
          eqn = ((-0.120985*M + 0.170883)/el_amax);
        }
        eqn *= abs(elm.delVel);
        elm.t0 = eqn + sqrt((0.48393/el_amax)*abs(delP) + eqn*eqn);
      } else {
        elm.t0 = 0;
      }
      if(el_amax*2 > el_vmax * M_SQRT2/EXP_HALF*M*500/EL_MIN_HTIME) {
        double tmpt0;
        /* If we get here, the max velocity may be the limiting
             * factor in the move. */
        if(delP < 0) {
          tmpt0 = -delP/((2./M_2_SQRTPI)*(el_vmax + (curElVel + nxtElVel)/2) +
                         M*elm.delVel/2.);
        } else {
          tmpt0 = delP/((2./M_2_SQRTPI)*(el_vmax - (curElVel + nxtElVel)/2) -
                        M*elm.delVel/2.);
        }
        if(tmpt0 > elm.t0)
          elm.t0 = tmpt0;
      }
      elm.t0 *= 1000.;
      if(elm.t0 <= (4.0*EL_MIN_HTIME/M)) {
        if(elm.t0 < (2.0*EL_MIN_HTIME/M)) {
          elm.t0 = 2.0*EL_MIN_HTIME/M;
        }
        /* Round hTime to nearest 10 ms */
        elm.hTime = 0.5 + elm.t0*(M/20.0);
        elm.hTime *= 10;

        elm.cTime = tsshm->msec + elm.hTime;
        elm.v0 = nxtElVel;
        /* Current position projected to cTime at curVel */
        elm.p0 = nxtEl + ((double)elm.v0 * elm.hTime) / 1000;
        elm.delPos = delP + ((double)elm.delVel*elm.hTime) *
                     0.001;
        nextElState = TRACKING;
        elState = SLEWING;
#if		EL_VERBOSE > 2

        printf("#El Start short move\n");
        DumpMove( &elm, "El", tsshm->msec);
#endif		/* EL_VERBOSE */

        goto START_EL;
      } else {
        int vmax, vsum, vsel, dP;

        /* If we got here, the move is too long for a short move and we
        * must now set up a Long move with acceleration to Vmax,
        * a possible Const Vel section, and deceleration to the new track's
        * velocity.  EL_AMAX, el_vmax, and MIN_HTIME are potential limits.
        * The move parameters will be chosen with equal accel and decel time.
        * Since the initial and final velocities may be different,
        * vsel is selected as the one requiring the larger velocity change
        * and used in setting up move parameters so that EL_AMAX will
         * not be exceeded in either case.  The time lost to this
        * simplification is negligable except above el = 88 deg.
         */
        if(el_amax < el_vmax * M_2_SQRTPI*M*250/EL_MIN_HTIME) {
          vsum = nxtElVel - 3*curElVel;
          /* In solving the quadratic eqn we need the absolute
           * value of the motion and velocities */
          if(delP < 0) {
            vsum = - vsum;
            vsel = (curElVel>nxtElVel)? -curElVel: -nxtElVel;
            dP = - delP;
          } else {
            vsel = (curElVel < nxtElVel)? curElVel: nxtElVel;
            dP = delP;
          }
          eqn = (2*vsel + vsum)/4;
          vmax = (2*vsel - vsum)/4 + (int)sqrt(EL_ACC_CONST*dP +
                                               eqn*eqn);

          if(vmax > el_vmax) {
            vmax = el_vmax;
            nextElState = LONGMOVECV_SETUP;
          } else {
            nextElState = LONGMOVEDECEL;
          }
          /* Round hTime to the nearest 10 ms */
          elm.hTime = 0.5 + (500/(EL_ACC_CONST*10))*
                      (vmax - vsel);
          elm.hTime *= 10;
          if(elm.hTime < EL_MIN_HTIME) {
#if		    EL_MIN_HTIME == 500
            vmax = dP - vsum/2;
#else

            vmax =(int)((500./EL_MIN_HTIME)*dP) - vsum/2;
#endif

            elm.hTime = EL_MIN_HTIME;
          }
        } else {

          /* With this combination of parameters we do the
           * much simpler velocity limited move.
           */
          vmax = abs((int)(delP*(1000./(EL_MIN_HTIME*2)) +
                           elm.delVel));
          elm.hTime = EL_MIN_HTIME;
          if(vmax > el_vmax) {
            vmax = el_vmax;
            nextElState = LONGMOVECV_SETUP;
          } else {
            nextElState = LONGMOVEDECEL;
          }
        }
        elm.cTime = tsshm->msec + elm.hTime;
        elm.v0 = nxtElVel;
        /* Current position projected to cTime at curVel */
        elm.p0 = nxtEl + ((double)elm.v0 * elm.hTime) / 1000;
        elm.t0 = elm.hTime * (2.0/M);
        if(delP < 0)
          vmax = -vmax;
        elm.delVel = vmax - nxtElVel;
        elm.delPos = 0;
        elState = SLEWING;
#if		EL_VERBOSE > 2

        printf("#El Start long move accel\n");
        DumpMove( &elm, "El", tsshm->msec);
#endif		/* EL_VERBOSE */

        goto START_EL;
      }
    }
    nxtEl = curEl + curElVel / 100;
    /* this is the core of the linear el position loop */
    nxtElVel = curElVel;
    cmdElVel =
#if		EL_CLOSED_LOOP
      (curEl - encEl) * EL_GAIN +
#endif
      curElVel;

    break;
  case SLEWING:
    movePos(&elm, tsshm->msec);
    shpEl = elm.curPos;
    nxtElVel = elm.curVel;
    cmdElVel =
#if	    EL_CLOSED_LOOP
      (shpEl - encEl) * EL_GAIN +
#endif
      nxtElVel;
    cmdElAccel = elm.nextVel - elm.curVel;

    if(tsshm->msec >= elm.cTime + elm.hTime) {
      nxtEl = shpEl;
      elState = nextElState;
#if		EL_VERBOSE > 1

      elPrintNext = 1;
#endif		/* EL_VERBOSE */

      goto START_EL;
    }
    break;
  case LONGMOVEDECEL:
    elm.delVel = curElVel - nxtElVel;
    elm.cTime = tsshm->msec + elm.hTime;
    elm.v0 = nxtElVel;
    /* Current position projected to cTime at curVel */
    elm.p0 = nxtEl + ((double)elm.v0 * elm.hTime) / 1000;
    elm.t0 = elm.hTime * (2.0/M);
    elm.delPos = curEl - nxtEl + ((double)elm.delVel*elm.hTime) *
                 0.001;
    /* At this point we would expect that if we simply decelerated
     * to the final velocity we would be very close to the target
     * position and velocity.  Thus unless a new track has been
     * given, delPos should be small.  We prefer to use the hTime
     * calculated at acceleration time, so if delPos isn't small,
     * the safest thing to do is just stop and then let TRACKING
     * start a new move.
     */
    if(abs(elm.delPos) > 0.1 * MAS) {
#if		EL_VERBOSE
      printf("#El Long Move Decel: Position error %10.5f\n",
             elm.delPos/(double)MAS);
#endif		/* EL_VERBOSE */

      elm.delPos = 0;
    }
    elState = SLEWING;
    nextElState = TRACKING;
#if	    EL_VERBOSE > 2

    printf("#El Start long move decel\n");
    DumpMove( &elm, "El", tsshm->msec);
#endif	    /* EL_VERBOSE */

    goto START_EL;
    break;
  case LONGMOVECV_SETUP:
    elState = LONGMOVECV;
    nextElState = LONGMOVEDECEL;
#if	    EL_VERBOSE > 2

    printf("#El Start CV\n");
    DumpMove( &elm, "El", tsshm->msec);
#endif	    /* EL_VERBOSE */

    goto ELCVENTRY;	/* Skip position update the first time */
  case LONGMOVECV:
    elm.curPos += elm.curVel * lastTimeStep;
ELCVENTRY:

    if((nxtElVel > curElVel &&
        elm.curPos + 0.001*(nxtElVel-curElVel)*elm.hTime >=
        curEl) ||
        (nxtElVel <= curElVel &&
         elm.curPos + 0.001*(nxtElVel-curElVel)*elm.hTime <=
         curEl)) {

      nxtEl = elm.curPos;
      elState = nextElState;
      nextElState = TRACKING;
      goto START_EL;
    }
    shpEl = elm.curPos;
    nxtElVel = elm.curVel;
    cmdElVel =
#if EL_CLOSED_LOOP
      (shpEl - encEl) * EL_GAIN +
#endif
      nxtElVel;
    break;
  }
  /* shmEl should always have a value */
  if(elState <= TRACKING) {
    shpEl = curEl;
  }
  /* Loop performance data */
#if 0
  if(! SFULL) {
#endif
    SaI.curEl = curEl;
    SaI.shpEl = shpEl;
    SaI.cmdElVel = cmdElVel;
    SaI.elState = elState;
#if 0
  }
#endif

  /* Gear ratio need not be calculated often, except during moves */
  elGearRatio = GearRatio(encEl);
  scbElVel = cmdElVel * elGearRatio * elVel2Scb;
  if(scbElVel > MAX_EL_SCB_VEL) {
    scbElVel = MAX_EL_SCB_VEL;
    cmdElAccel = 0;
  } else if(scbElVel < -MAX_EL_SCB_VEL) {
    scbElVel = -MAX_EL_SCB_VEL;
    cmdElAccel = 0;
  }
}	/* End of ElCycle */


/* Calculate curPos and curVel in the move structure for the present time */
static void movePos(MOVE *mp, int time) {
  double t, tot0;		/* Time, normalized time wrt cntr of move */
  double e;		/* storage for 0.5 + 0.5 * erf(t/t0) */
  double g;		/* Storage for exp(-(t/t0)^2) */

  t = time - mp->cTime;
  tot0 = t/mp->t0;
  e = 0.5 + 0.5*erf(tot0);
  g = exp(-tot0*tot0);
  mp->curPos = mp->p0 + mp->v0*t/1000. + mp->delPos*e +
               0.001*mp->delVel*(t*e + (0.25*M_2_SQRTPI)*mp->t0*g);
  mp->curVel = mp->v0 + (500*M_2_SQRTPI)*mp->delPos*g/mp->t0 +
               mp->delVel*e;

  /* Compute velocity for the next period (10ms later).  This
   * is used for more accurate velocity feed forward and for
   * deriving the acceleration to smooth the velocity loop
   * and reduce motor noise. */
  tot0 += 10/mp->t0;
  e = 0.5 + 0.5*erf(tot0);
  g = exp(-tot0*tot0);
  mp->nextVel = mp->v0 + (500*M_2_SQRTPI)*mp->delPos*g/mp->t0 +
                mp->delVel*e;
}

/* This normalized gear ratio calc returns 1.000 at its peak */
static double GearRatio(int el) {
  /* d(motor-axis)/d(el.axis) = */
  /* Paul says the 632.386 should be 629.9, but Nimesh's Gear Ratio
   * function fits with 632.386 */
  return((1366.2*2/632.386) * sin(el*RAD_PER_MAS + 1.7/R) /
         sqrt(29.585-28.551*cos(el*RAD_PER_MAS + 1.7/R)));
}

#if AZ_VERBOSE > 2 || EL_VERBOSE > 2
static void DumpMove(MOVE *mp, char *axis, int time) {
  printf("#%s move struct at utc = %d:\n"
         "#\thTime = %4d cTime = %4d t0 = %8.3f\n",
         axis, time, mp->hTime, mp->cTime, mp->t0);
  printf("#\tp0 = %10.5f v0 = %10.5f "
         "delPos = %10.5f delVel = %10.5f\n",
         mp->p0/(double)MAS, mp->v0/(double)MAS,
         mp->delPos/(double)MAS, mp->delVel/(double)MAS);
}
#endif /* VERBOSE */

#if !SIMULATING
#if ! GLT
/* Subroutine to read the state M3 was left in and to reset it to closed if
 * servo has been inactive long or the value is something other than 0 or 1
 */
static void SetM3Cmd(void) {
  int rm_rtn, unixtime, prevTimestamp, dt;

  rm_rtn = rm_read(RM_ANT_0,"RM_SERVO_TIMESTAMP_L", &prevTimestamp);
  if(rm_rtn != RM_SUCCESS) {
    rm_error_message(rm_rtn, "Reading Previous servo timestamp");
  }
  rm_rtn=rm_read(RM_ANT_0,"RM_UNIX_TIME_L", &unixtime);
  if(rm_rtn != RM_SUCCESS) {
    rm_error_message(rm_rtn,"rm_read() unix time");
  }
  dt = unixtime - prevTimestamp;
  rm_rtn = rm_read(RM_ANT_0, "RM_M3CMD_B", &m3Cmd);
  if(rm_rtn != RM_SUCCESS) {
    rm_error_message(rm_rtn, "Reading m3 cmd");
  }
  if(dt > 3600 || dt < 0 || m3Cmd > 1 || m3Cmd < 0) {
    fprintf(stderr, "clearing m3Cmd of %d\n", m3Cmd);
    m3Cmd = 0;
    rm_write(RM_ANT_0, "RM_M3CMD_B", &m3Cmd);
  }
}

static void OpenScb(void) {
  int errors, pass;
  short projectStatus = 2;

  /* Set up for communication to the scb and initialize it */
  dprintf("Opening the serial line for the scb ... ");
  while(ComSetup(serialPort) < 0)  {
    if(projectStatus > 0) {
      vSendOpMessage(OPMSG_SEVERE, 15, 10, "can't open the line to the SCB");
    }
    rm_read(RM_ANT_0, "RM_PROJECT_STATUS_S", &projectStatus);
    sleep(1);
  }
  dprintf("Setting up the scb ... ");
  for(pass = 0;;pass++) {
    errors = comTimeouts + comBadPkts;
    ComStartPkt(FAULTWORD);
    if(SendPktGetRtn(FAULTWORD) == 0) tsshm->scbFaultWord = ComGetL();
    SetScbState();
    ReadScbParameters();
    /* Turn both servos off to start in a known state */
    if(tsshm->fault != LOCKOUT) {
      ComStartPkt(ELSERVOON);
      ComPutC(DISABLE);
      SendPktGetRtn(ACK);
      ComStartPkt(AZSERVOON);
      ComPutC(DISABLE);
      SendPktGetRtn(ACK);
      NonLockoutSetup();
      ComStartPkt(CLRFAULTWORD);
      SendPktGetRtn(CLRFAULTWORD);
      tsshm->scbFaultWord = ComGetL();
    }
    if(errors == comTimeouts + comBadPkts) break;
    if(pass > 1) {
      if(comTimeouts - prevComTimeouts == comNumCalls - prevComNumCalls) {
        vSendOpMessage(OPMSG_SEVERE, 28, 10, "No response from the SCB");
      } else if(comBadPkts-prevComBadPkts+comTimeouts-prevComTimeouts > 2) {
        vSendOpMessage(OPMSG_SEVERE, 29, 10, "SCB Communication problems");
      }
      prevComTimeouts = comTimeouts;
      prevComBadPkts = comBadPkts;
      prevComNumCalls = comNumCalls;
      sleep(1);
    }
  }
  tsshm->elState = SERVOOFF;
  tsshm->azState = SERVOOFF;
}
#endif /* !GLT */

void checkCollision(void) {
  /* For each collision pad, padList has one or two padIDs of pads which
   * are close enough for collisions. padList[0] is the entry for pad 22
   * The pad in the second position of padList is too close for operation
   * even if the extended hard stop is in place.
   * The first entry for pad 22 is also too close for operation even with
   * the extended hard stop in place.
   * The pointer to the element for our pad (plp) will be zero if this is
   * not a collision pad.
   */
  struct PADLIST {
    char goodPad, badPad;
  };
  static struct PADLIST *plp, 
    padList[] = {{5,4},{2,0},{1,0},{4,7},{3,22},{6,22},{5,0},{0,3}};
  static int oldPadID = -1;
  char configuration_pads[9];
  int i;

  if(padID != oldPadID) {
    oldPadID = padID;
    if(padID == 22) {
	plp = padList;
    } else if(padID > 0 && padID < 8) {
	plp = &padList[padID];
    } else {
	plp = (struct PADLIST *)0;
    }
  }
  if(plp) {
    i = rm_read(RM_ANT_0, "RM_CONFIGURATION_PADS_V9_B",
	configuration_pads);
    if(i != RM_SUCCESS) {
      rm_error_message(i, "Checking adjacent pad occupancy");
      collisionPossible = 1;
    } else {
      /* Initialize collisionPossible for the tests below */
      collisionPossible = 0;
    }
    for(i = 1; i < 9; i++) {
      if(configuration_pads[i] == plp->goodPad) {
	if(padID == 22) { /* For pad 22, the "goodPad" is also bad. */
	  collisionPossible = 1;
	} else if(!checkCollisionLimitSwitch) {
	  /* Check to see that at least one of the two antennas in the pair
	   * has the extended hard stop in place. */
	  if(rm_read(RM_ANT_0, "RM_EL_COLLISION_LIMITS_V9_B",
		elCollisionLimits) != RM_SUCCESS) {
            rm_error_message(i, "Checking adjacent pad extended hard stop");
            collisionPossible = 1;
          }
	  if(elCollisionLimits[i] != 1) {
            collisionPossible = 1;
	  }
	}
      } else if(configuration_pads[i] == plp->badPad) {
	collisionPossible = 1;
      }
    }
    if(collisionPossible) {
      if(azState != SERVOOFF) azState = STOPPING;
      if(elState != SERVOOFF) elState = STOPPING;
      vSendOpMessage(OPMSG_SEVERE, 15, 60, "Shutting down the drives because "
		"a collision may be possible");
    }
  }
}

/* Definition of the data columns:
Column  Content
1       time in seconds
2       scb Status (bit mapped)
3       Az command from (dmy)track
4       Az command out of the command shaper
5       Az from the fine encoder
6       Az Velocity commanded by servo
7       Az Velocity from the tachometer
8       Az Torque in Amperes (average of the two motors)
9       State of the Az command shaper
10      El command from track
11      El command out of the command shaper
12      El from the fine encoder12
13      El Velocity commanded by servo
14      El Velocity from the tach
15      El Torque (Amps)
16      State of the El command shaper
*/

#define PO(x) SaO.x*(1.0/MAS)		/* Pos and vel in degrees */
#define TO(x) SaO.x*(70./32768.0)	/* Torque in Amps */

void PrintSamples(void) {
  static int lastPrintTime = -1;
  int unixtime;
  FILE *fp;
  char fn[sizeof("/instance/logs/servo_stopped.XXXXXXXXXX") + 1];

  (void)rm_read(RM_ANT_0, "RM_UNIX_TIME_L", &unixtime);
  if(unixtime - lastPrintTime < 10) return;
  lastPrintTime = unixtime;
  sprintf(fn, "/instance/logs/servo_stopped.%d", unixtime);
  if((fp = fopen(fn, "w")) == NULL) {
    ErrPrintf("Open of sample print file %s failed\n", fn);
    return;
  }
  while(!SEMPTY) {

    fprintf(fp, "%7.3f %2d"
	"% 11.5f %11.5f %11.5f %8.4f %8.4f %5.1f %1d"	/* Az values */
	"% 11.5f %11.5f %11.5f %8.4f %8.4f %5.1f %1d"	/* El values */
	"\n", SaO.msec*0.001, SaO.scbStatus,
	PO(curAz), PO(shpAz), PO(encAz),
	PO(cmdAzVel), PO(tachAzVel), TO(azTorq), SaO.azState, 
	PO(curEl), PO(shpEl), PO(encEl),
	PO(cmdElVel), PO(tachElVel), TO(elTorq), SaO.elState
    );
    INC_OUT;
  }
  ErrPrintf("Saved last 0.6 sec of state in %s\n", fn);
}

void ReadScbParameters(void) {
  /* Ask the SCB to get the Palm date code */
  ComStartPkt(QUERYPALMVERSION);
  ComPutC(0);
  SendPktGetRtn(ACK);
  ComStartPkt(QUERYVERSION);
  SendPktGetRtn(QUERYVERSION);
  scbEPROMVersion = (int)ComGetS();
  ComStartPkt(E1);
  ComPutC(0);
  SendPktGetRtn(E1);
  scbSRAMVersion = (int)ComGetS();
  ComStartPkt(GETLIMITS);
  if(SendPktGetRtn(GETLIMITS) == 0) {
    lowerLimit = ComGetL() * EL_LIM_ENC_TO_MAS + (2*EL_ENCODER_DIFF_TOLERANCE);
    collisionLimit = ComGetL() * EL_LIM_ENC_TO_MAS +
		  (2*EL_ENCODER_DIFF_TOLERANCE);
    tsshm->upperLimit = ComGetL() * EL_LIM_ENC_TO_MAS -
                      EL_ENCODER_DIFF_TOLERANCE;
    tsshm->ccwLimit = ComGetL() * AZ_LIM_ENC_TO_MAS +
                    AZ_ENCODER_DIFF_TOLERANCE;
    tsshm->cwLimit = ComGetL() * AZ_LIM_ENC_TO_MAS -
                   AZ_ENCODER_DIFF_TOLERANCE;
  }
  ComStartPkt(READTACHDIVISORS);
  SendPktGetRtn(READTACHDIVISORS);
  azTachDivisor = ComGetS();
  elTachDivisor = ComGetS();
  rawTachs = ComGetC();

  /* Get the pad azimuth offset */
  ComStartPkt(READPADID);
  if(SendPktGetRtn(READPADID) == 0) {
    padID = ComGetC();
    useFakePadID = ComGetC();
    fakePadID = ComGetC();
    padAzOffset = ComGetL() * 1000;	/* change to MAS */
    checkCollisionLimitSwitch = ComGetC();
    tsshm->lowerLimit = ((checkCollisionLimitSwitch)? collisionLimit:
	lowerLimit);
    dprintf("Pad az offset %.4f Deg.  Collision check %s ...",
            padAzOffset/(double)MAS, (checkCollisionLimitSwitch)? "on":
            "off");
#if 0
  } else {
    ErrPrintf("Pad az offset unknown, set to zero\n");
#endif
  }
  tsshm->padID = (useFakePadID)? fakePadID: padID;
  tsshm->padAzOffset = padAzOffset;
  ComStartPkt(READROCKERS);
  SendPktGetRtn(READROCKERS);
  azRockerCCWLimit = ComGetL() * AZ_LIM_ENC_TO_MAS;
  azRockerCWLimit = ComGetL() * AZ_LIM_ENC_TO_MAS;

  ComStartPkt(GETELGAINS);
  ComPutS(PRESENT_GAINS);
  if(SendPktGetRtn(GETELGAINS) == 0) {
    elIntegralGain = (64. * ONE_SEC)/ComGetS();
    elProportionalGain = (double)ComGetS()/16.;
    elDerivativeGain = (double)ComGetS()/16.;
#if 0
  } else {
    ErrPrintf("SILENT El Gains unknown, set to zero\n");
#endif
  }
  ComStartPkt(GETAZGAINS);
  ComPutS(PRESENT_GAINS);
  if(SendPktGetRtn(GETAZGAINS) == 0) {
    azIntegralGain = (64. * ONE_SEC)/ComGetS();
    azProportionalGain = (double)ComGetS()/16.;
    azDerivativeGain = (double)ComGetS()/16.;
    azTorqueBias = (double)ComGetS()/16.;
    (void) ComGetS();	/* Type of Gains */
    azMotorDisable = ComGetC();
#if 0
  } else {
    ErrPrintf("SILENT Az Gains & motor disable unknown, set to zero\n");
#endif
  }
  ComStartPkt(GETENCODERZEROS);
  SendPktGetRtn(GETENCODERZEROS);
  elLimEncZero = ComGetL();
  azLimEncZero = ComGetL();
  dprintf("done\n");
}

void NonLockoutSetup(void) {
  int errors;

  errors = comTimeouts + comBadPkts;
#if SERVO_DOES_TACH_DIVISORS
  ComStartPkt(TACHDIVISORS);
  ComPutC(ENABLE_TACH_DIVISORS);
  ComPutC(USE_RAW_TACHS);
  SendPktGetRtn(TACHDIVISORS);
  azVel2Scb = (AZ_SCB_VEL_PER_VEL/1024)*ComGetS(); /* azTachDivisor */
  elVel2Scb = (EL_SCB_VEL_PER_VEL/1024)*ComGetS(); /* elTachDivisor */
#else /* SERVO_DOES_TACH_DIVISORS */

  ComStartPkt(TACHDIVISORS);
  ComPutC(ENABLE_TACH_DIVISORS);
  ComPutC(USE_TACH_DIVISORS);
  SendPktGetRtn(TACHDIVISORS);
  azVel2Scb = AZ_SCB_VEL_PER_VEL;
  elVel2Scb = EL_SCB_VEL_PER_VEL;
#endif /* SERVO_DOES_TACH_DIVISORS */

  if(errors == comTimeouts + comBadPkts)
    nonLockoutSetupDone = 1;
  ComStartPkt(SOFTWARE);
  SendPktGetRtn(SOFTWARE);
  prevNumResets = ComGetS();
}

static void CloseScb(char *s) {
  if(tsshm->fault != LOCKOUT) {
    ComStartPkt(ELSERVOON);
    ComPutC(DISABLE);
    SendPktGetRtn(ACK);
    ComStartPkt(AZSERVOON);
    ComPutC(DISABLE);
    SendPktGetRtn(ACK);
  }

  ComClose();
  ErrPrintf("%s: servo quitting\n", s);
}
#endif /* !SIMULATING */

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

#if GLT
void WaitTickGetTime(void) {
    int stat;

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
    if(ttTimeout) {
      ErrPrintf("TrueTime timeout: Time step is %.6f\n", lastTimeStep);
    }
    oldusec = ttime.usec;

    /* Convert the time now */
    tsshm->msec = ((ttime.hour * 60 + ttime.min) * 60 + ttime.sec) * 1000 +
                  (ttime.usec + 500) / 1000;
    tsshm->day = ttime.yday;
}

#else /* GLT */

#if !SIMULATING
static void InitEncoders(void) {
  int i;
  static unsigned short hBDivisor = 30000;

  if(useSyncClock32) {
    scfd = open("/dev/syncclock32", O_RDWR, 0);
    if(scfd <= 0) {
      ErrPrintf("Error opening SyncClock32 - /dev/syncclock32\n");
      exit(SYSERR_RTN);
    }
    if(ioctl(scfd, SC32_SET_HB_DIV, &hBDivisor)!= 0) {
      perror("Setting Heart Beat divisor");
      exit(-1);
    }
    if(ioctl(scfd, SC32_ENABLE_HB_INT, NULL) != 0) {
      perror("enabling Heart beat interrupt");
      exit(-1);
    }
  } else {
    ttfd = open("/dev/vme_sg_simple", O_RDWR, 0);
    if(ttfd <= 0) {
      ErrPrintf("Error opening TrueTime - /dev/vme_sg_simple\n");
      exit(SYSERR_RTN);
    }
    /* Make sure that the TrueTime is putting out pulses to strobe
     * the ACC encoders */
    i = VME_SG_SIMPLE_RATE_1K;
    ioctl(ttfd, VME_SG_SIMPLE_SET_PULSE_RATE, &i);
  }


  if((i = OpenEncoders(1))) {
    ErrPrintf("servo quitting\n");
    exit(i);
  }
#if 0

  if((encfd = open("/dev/endat0", O_RDONLY, 0)) > 0) {
    azEncType = elEncType = HEIDENHAIN;
    ddprintf("Opened Heidenhain encoders\n");
    /* Now reset the interface cdard and the encoders */
    arg.enc1 = 1;
    arg.enc2 = 1;
    ioctl(encfd, EIOCRESETCARD, &arg);
    ioctl(encfd, EIOCRESETENC, &arg);
  } else if((encfd = open("/dev/encoder0", O_RDONLY, 0)) > 0) {
    azEncType = elEncType = ACC;
  } else {
    ErrPrintf("servo quitting: no encoders found\n");
    exit(SYSERR_RTN);
  }
  if(ioctl(encfd, EIOCGETOFFS, &arg) == 0) {
    azEncoderOffset = arg.offs.azOffset;
    azEncoderReversed = arg.offs.revAz;
    elEncoderOffset = arg.offs.elOffset;
    elEncoderReversed = arg.offs.revEl;
  } else {}
#endif

  /* Start by reading the limit and fine encoders' raw values.
   * The limit encoders have 4096 counts/turn.  The Az Gear Ratio
   * is 422/25.  The old El gear ratio is 16.75 and the new ratio
   * will be 188/50.  Az should be in the range (-180, 360) in degrees.
   * El should be in (5, 93).
   */
  ReadLimEncoders(&azLimitEncoder, &elLimitEncoder);
  if(azLimitEncoder > MAS*360 || azLimitEncoder < -MAS*180) {
    ErrPrintf("Az limit encoder out of range - reads %.2f\n",
              azLimitEncoder*(1.0/MAS));
    /*	    exit(SYSERR_RTN); */
  }
  if(elLimitEncoder > MAS*93 || elLimitEncoder < MAS*4) {
    ErrPrintf("El limit encoder out of range - reads %.2f\n",
              elLimitEncoder*(1.0/MAS));
    /*	    exit(SYSERR_RTN); */
  }

  /* Guess the proper value of azTurn from the limit encoder.
   * If we are within 1/2 turn, GetPosTime will put it right.
   */
  azTurn = (azLimitEncoder < 0)? -MAS*360: 0;
  oldEncAz = azLimitEncoder;
  ttime.timeout_ticks = 2;  	/* This margin should be safe */
  GetPosTime();			/* This will set the time */

  /* Check for encoder errors.  Check that both encoders are
   * within the expected range and that both are within 5 deg of the
   * corresponding limit encoders.
   */
  if(encAz > 370*MAS || encAz < -190*MAS)
    ErrPrintf("servo: Az encoder out of range: %10.4f\n",
              encAz/(double)MAS);
  if(encEl < 5*MAS || encEl > 93*MAS)
    ErrPrintf("servo: El encoder out of range: %10.4f\n",
              encEl/(double)MAS);

#if 0

  if(abs(encEl - elLimitEncoder) > ENCODER_DIFF_TOLERANCE) {
    ErrPrintf("servo: El fine and limit encoders differ:\n"
              "Fine = %10.4f, Limit = %8.2f\n", encEl/(double)MAS,
              elLimitEncoder/(double)MAS);
  }
#endif


#if AZ_VERBOSE || EL_VERBOSE
  ErrPrintf("Limit encoders read Az %.3f El %.3f deg.\n",
            azLimitEncoder/(double)MAS , elLimitEncoder / (double)MAS);
  ErrPrintf("Fine encoders read Az %.4f El %.4f deg.\n",
            encAz/(double)MAS , encEl / (double)MAS);
#endif /* AZ_VERBOSE || EL_VERBOSE */
}

/* Read the limit encoders and return the az and el in MAS */
static void ReadLimEncoders(int *azEnc, int *elEnc) {
  ComStartPkt(READENCODERS);
  if(SendPktGetRtn(READENCODERS) && SendPktGetRtn(READENCODERS)) {
    return;
#if 0
    CloseScb("Error reading Limit Encoders");
    exit(SYSERR_RTN);
#endif
  }
  *elEnc = ComGetL() * EL_LIM_ENC_TO_MAS;
  *azEnc = ComGetL() * AZ_LIM_ENC_TO_MAS;
  azLimEncDiff = *azEnc - encAz;
  elLimEncDiff = *elEnc - encEl;
}

#endif /* !SIMULATING */

#if !SIMULATING
static void SetScbState(void) {

  ComStartPkt(STATUSBYTE);
  if(SendPktGetRtn(STATUSBYTE)) {
/*    ErrPrintf("Problem reading status from the scb\n"); */
    return;
  }
  tsshm->scbStatus = scbStatus = ComGetC();
  if(ComGetC()) {
    tsshm->fault = LOCKOUT;
    return;
  } else if(tsshm->fault == LOCKOUT) {
    ReadScbParameters();
    UPDATE_INIT;
  }
  if((tsshm->scbFaultWord & 0xb873ff7f) != 0 ||
      (scbStatus & STATUS_DATA_READY_HANDLER) != 0) {
    tsshm->fault = FAULT;
  } else if(azMotorDisable > 0) {
    if(azMotorDisable > 2) {
      tsshm->fault = ONE_TACH_FAULT;
    } else {
      tsshm->fault = ONE_MOTOR_FAULT;
    }
  } else {
    tsshm->fault = NO_FAULT;
  }
}
#endif /* !SIMULATING */

/* Get time, read encoder and convert values */
static void GetPosTime() {
#if 0
  struct enc_result enc;
#endif

  int stat;
  static int oldusec = -1, oldSecDay;
  int secDay;
  int usecDiff, secDiff;
#if USE_SIGTIMEDWAIT

  siginfo_t info;
#else /* USE_SIGTIMEDWAIT */

  int ttTimeout;
#endif /* USE_SIGTIMEDWAIT */

  if(useSyncClock32) {
    /* get the current time */
    if((stat = read(scfd, &sctime, sizeof(sctime))) < 0) {
      if(sctime.year == -1) {
        vSendOpMessage(OPMSG_SEVERE, 17, 60, "The IRIG-B board has an "
                       " incorrect year.  Please reboot the acc");
      } else {
        perror("Reading the SyncClock32");
      }
    }
    if(sctime.locked_to_ref == SC32_FALSE) {
      irig_error_seen = 1;
    }
    usecDiff = sctime.usec - oldusec;
    secDay = (sctime.hour * 60 + sctime.min) * 60 + sctime.sec;
    if((secDiff = secDay - oldSecDay) < -86000) secDiff += 86400;
    usecDiff += secDiff * 1000000;		/* Time Change (usec) */
    if(usecDiff > 100000) {
      int secDay2;
      int usecDiff2, secDiff2;
      usecDiff2 = sctime2.usec - oldusec;
      secDay2 = (sctime2.hour * 60 + sctime2.min) * 60 + sctime2.sec;
      if((secDiff2 = secDay2 - oldSecDay) < -86000) secDiff2 += 86400;
      usecDiff2 += secDiff2 * 1000000;		/* Time Change (usec) */
      ErrPrintf("Prev GetPosTime %.3f sec ago vel pkt %.3f ago, usec %d, auxCycle %d\n",
		(double)usecDiff * .000001,
		(double)usecDiff2 * .000001, sctime.usec, auxCycle);
    }
    if(usecDiff < 9500) {
      stat = ioctl(scfd, SC32_HB_WAIT, NULL);
      if(stat != 0) {
        perror("Waiting for sc32 heartbeat");
#if CATCH_SIGNALS
        exit((gotQuit)? QUIT_RTN: NORMAL_RTN);
#endif /* CATCH_SIGNALS */
      }
      if((stat = read(scfd, &sctime, sizeof(sctime))) < 0) {
        if(sctime.year == -1) {
          vSendOpMessage(OPMSG_SEVERE, 17, 60, "The IRIG-B board has an "
                         " incorrect year.  Please reboot the acc");
        } else {
          perror("Reading the SyncClock32");
        }
      }
    }
    usecDiff = sctime.usec - oldusec;
    secDay = (sctime.hour * 60 + sctime.min) * 60 + sctime.sec;
    if((secDiff = secDay - oldSecDay) < -86000) secDiff += 86400;
    lastTimeStep = secDiff + usecDiff / 1e6;		/* Time step (sec) */
    if(lastTimeStep > .021 || lastTimeStep < .001) {
      ttTimeoutCount++;
      if(reportTrueTimeTimeout && ttTimeoutCount > 0) {
        ErrPrintf("Time step %.3f sec %d old sec %d usec %d old usec %d\n",
                  lastTimeStep, secDay, oldSecDay, sctime.usec, oldusec);
      }
    }
    /* Convert the time now */
    tsshm->msec = secDay * 1000 + (sctime.usec + 500) / 1000;
    tsshm->day = sctime.yday;
    oldusec = sctime.usec;
    oldSecDay = secDay;
  } else {

    GetNextTickTime();

#if USE_SIGTIMEDWAIT

    if((stat = read(ttfd, &ttime, sizeof(ttime))) < 0) {
      ErrPrintf("Error %d reading TrueTime\n", stat);
    }
    if(ttime.input_reference_error || (ttime.phase_locked ^ 1)) {
      irig_error_seen = 1;
    }
#endif /* USE_SIGTIMEDWAIT */
    usecDiff = ttime.usec - oldusec;
    if(usecDiff < 0)
      usecDiff += 1000000;

#if USE_SIGTIMEDWAIT
    /* if we would skip a tick, don't wait */
    if(usecDiff < 9500) {
      if(sigtimedwait(&sigs, &info, &timeout) < 0) { /* Normal timeout */
        switch(errno) {
        case EAGAIN:
          break;
        case EINTR:
          ErrPrintf("Received signal %d caught elsewhere\n",
                    info.si_signo);
          break;
        case EINVAL:
          ErrPrintf("invalid tv_nsec = %d\n", (int)timeout.tv_nsec);
          exit(1);
        }
      } else {
        shutdownSignal = 1;
        if(info.si_signo != SIGINT) {
          gotQuit = 1;
        }
      }
    }
#else /* USE_SIGTIMEDWAIT */
    ttTimeout = 0;
    /* if we would skip a tick, don't wait for the TrueTime */
    if(usecDiff < 15000) {
      if(ioctl(ttfd, VME_SG_SIMPLE_WAIT_UNTIL, &ttime) < 0) {
        switch(errno) {
        case EAGAIN:
          ErrPrintf(
            "servo could not wait on TrueTime because it was busy\n");
          /* Recover by reading the time below and skip the wait */
          break;
        case EINTR:
          /* We received an interrupt and will stop anyway, so don't
          * worry. */
          ErrPrintf(
            "SILENT TrueTime returned an interrupt,"
            " shutdown %d, quit %d\n",
            shutdownSignal, gotQuit);
          break;
        case ETIMEDOUT:
          ttTimeoutCount++;
          if(reportTrueTimeTimeout) {
            ttTimeout = 1;
          }
          break;
        default:
          ErrPrintf( "The TrueTime wait returned error %d  \n", errno);
          break;
        }
      }
    }
#endif /* USE_SIGTIMEDWAIT */
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
#if USE_SIGTIMEDWAIT
    if(lastTimeStep > 0.011 || lastTimeStep < 0.009) {
      ttTimeoutCount++;
      if(reportTrueTimeTimeout) {
        ErrPrintf("Time step %.3f usecDiff %d oldusec %d usec %d\n",
                  lastTimeStep, usecDiff, oldusec, ttime.usec);
      }
    }
#else /* USE_SIGTIMEDWAIT */
    if(ttTimeout) {
      ErrPrintf("TrueTime timeout: Time step is %.6f\n", lastTimeStep);
    }
#endif /* USE_SIGTIMEDWAIT */
    oldusec = ttime.usec;

    /* Convert the time now */
    tsshm->msec = ((ttime.hour * 60 + ttime.min) * 60 + ttime.sec) * 1000 +
                  (ttime.usec + 500) / 1000;
    tsshm->day = ttime.yday;
  }

  /* Time is done, so read the encoders, etc.. */
#if SIMULATING
  encEl += cmdElVel *0.01;
  encAz += cmdAzVel *0.01;
#else /* SIMULATING */

  ReadEncoders(&encAz, &encEl);
#if 0

  read(encfd, &enc, sizeof(enc));
  if(azEncType == HEIDENHAIN) {
    static short int oldAzStatus = 1;
    static short int oldElStatus = 1;

    if(enc.statusAz != oldAzStatus) {
      ErrPrintf("Az encoder status 0x%02x\n", enc.statusAz);
      oldAzStatus = enc.statusAz;
    }
    if(enc.statusEl != oldElStatus) {
      ErrPrintf("El encoder status 0x%02x\n", enc.statusEl);
      oldElStatus = enc.statusEl;
    }
  }

#endif
  encAz += padAzOffset + azTurn;
  if(encAz - oldEncAz > MAS * 180) {
    encAz -= MAS*360;
    azTurn -= MAS*360;
  } else if(encAz - oldEncAz < -MAS * 180) {
    encAz += MAS*360;
    azTurn += MAS*360;
  }
  oldEncAz = encAz;
#endif /* SIMULATING */
}

/* Set the ttime structure to the next even 10 msec time.  GetNextTickTime
 * first reads the current time and then rounds up
 */
static void GetNextTickTime(void) {
  int stat;

  if(( stat = read(ttfd, &ttime, sizeof(ttime))) < 0) {
    ErrPrintf("Error %d reading TrueTime\n", stat);
  }
  if(ttime.input_reference_error || (ttime.phase_locked ^ 1)) {
    irig_error_seen = 1;
  }
  /* Round the usec up to the next even 10msec.  Require 50usec spare */
  ttime.usec += 10050;
  ttime.usec /= 10000;
  ttime.usec *= 10000;
  if(ttime.usec >= 1000000) {
    ttime.usec -= 1000000;
    if(++ttime.sec >= 60) {
      ttime.sec = 0;
      if(++ttime.min >= 60) {
        ttime.min = 0;
        if(++ttime.hour >= 24) {
          ttime.hour = 0;
          if(++ttime.yday >= ((ttime.year % 4) == 0)? 366: 365) {
            ttime.yday = 0;
            ttime.year++;
          }
        }
      }
    }
  }
}
#endif /* !GLT */

#if 0
static void PrintTime(struct vme_sg_simple_time *tp) {
  printf("%.4d day %3d %2d:%02d:%02d.%06d\n", tp->year, tp->yday,
         tp->hour, tp->min, tp->sec, tp->usec);
}
#endif

#if !SIMULATING
static void PrintEncoders(void) {
  ErrPrintf("Fine encoder (az, el) = (%.4f, %.4f) "
            "limit = (%.2f, %.2f)\n", encAz / (double) MAS,
            encEl / (double)MAS, azLimitEncoder / (double)MAS,
            elLimitEncoder / (double)MAS);
}

/* This subroutine must be preceeded by an ErrPrintf with "[[" which is
 * not closed. */
static void printStatusFault(void) {
  static int lastPrintTime = -1;
  int faultWord, storedFaultWord;

  if(lastPrintTime == tsshm->msec) {
    ErrPrintf("]]\n");
    return;
  } else {
    lastPrintTime = tsshm->msec;
  }

  ReadFault(&faultWord, &storedFaultWord);
  /*	ErrPrintf("SILENT[["); */
  parseStatusByte(scbStatus);
  ErrPrintf("  storedFaultWord = 0x%08x\n", storedFaultWord);
  parseFaults(storedFaultWord);
  ErrPrintf("---------\n");
  ErrPrintf(" currentFaultWord = 0x%08x\n",faultWord);
  parseFaults(faultWord);
  ErrPrintf("]]\n");

  if(tsshm->fault != LOCKOUT) {
    ComStartPkt(CLRFAULTWORD);
    SendPktGetRtn(CLRFAULTWORD);
  }
}

static void ReadFault(int *faultWord, int *storedFaultWord) {
  ComStartPkt(FAULTWORD);
  if (SendPktGetRtn(FAULTWORD) != 0)
    return;
  *faultWord = ComGetL();
  *storedFaultWord = ComGetL();
  return;
}

static void parseFaults(unsigned long word) {
  unsigned long i;

  for (i=0;i<32;i++) {
    if (word & (1<<i)) {
      switch(i) {
      case ELEVATION_UPPER_PRELIM_FAULT:
        ErrPrintf("Elevation upper prelimit\n");
        break;
      case ELEVATION_LOWER_PRELIM_FAULT:
        ErrPrintf("Elevation lower prelimit\n");
        break;
      case ELEVATION_UPPER_LIMIT_FAULT:
        ErrPrintf("Elevation upper limit switch\n");
        break;
      case ELEVATION_LOWER_LIMIT_FAULT:
        ErrPrintf("Elevation lower limit switch\n");
        break;
      case ELEVATION_ENCODER_FAULT:
        ErrPrintf("Elevation low-res encoder beyond %s limit\n",
                  (elLimitEncoder > 45*MAS)? "upper": "lower");
        break;
      case ELEVATION_TEMPERATURE_FAULT:
        ErrPrintf("Elevation motor overtemperature fault\n");
        break;
      case ELEVATION_CURRENT_FAULT:
        ErrPrintf("Elevation motor overcurrent fault\n");
        break;
      case ELEVATION_GLENTEK_FAULT:
        ErrPrintf("Elevation Glentek fault\n");
        break;
      case ELEVATION_OVERFLOW_FAULT:
        ErrPrintf("Elevation overflow fault\n");
        break;
      case AZIMUTH_CLOCKWISE_PRELIM_FAULT:
        ErrPrintf("Azimuth clockwise prelimit\n");
        break;
      case AZIMUTH_COUNTERCLOCKWISE_PRELIM_FAULT:
        ErrPrintf("Azimuth counterclockwise prelimit\n");
        break;
      case AZIMUTH_CLOCKWISE_LIMIT_FAULT:
        ErrPrintf("Azimuth clockwise limit switch\n");
        break;
      case AZIMUTH_COUNTERCLOCKWISE_LIMIT_FAULT:
        ErrPrintf("Azimuth counterclockwise limit switch\n");
        break;
      case AZIMUTH_ENCODER_FAULT:
        ErrPrintf("Azimuth low-res encoder beyond %s limit\n",
                  (azLimitEncoder > 0)? "CW": "CCW");
        break;
      case AZIMUTH1_TEMPERATURE_FAULT:
        ErrPrintf("Azimuth motor #1 overtemperature\n");
        break;
      case AZIMUTH2_TEMPERATURE_FAULT:
        ErrPrintf("Azimuth motor #2 overtemperature\n");
        break;
      case AZIMUTH1_CURRENT_FAULT:
        ErrPrintf("Azimuth motor #1 overcurrent\n");
        break;
      case AZIMUTH2_CURRENT_FAULT:
        ErrPrintf("Azimuth motor #2 overcurrent\n");
        break;
      case AZIMUTH1_GLENTEK_FAULT:
        ErrPrintf("Azimuth motor #1 Glentek fault\n");
        break;
      case AZIMUTH2_GLENTEK_FAULT:
        ErrPrintf("Azimuth motor #2 Glentek fault\n");
        break;
      case AZIMUTH_OVERFLOW_FAULT:
        ErrPrintf("Azimuth overflow fault\n");
        break;
      case EMERGENCY_STOP_FAULT:
        ErrPrintf("Emergency stop pressed\n");
        break;
      case QUADADC_RESET_FAULT:
        ErrPrintf("Quad ADC reset fault\n");
        break;
      case COOLANT_FLOW_FAULT:
        ErrPrintf("Coolant flow fault\n");
        break;
      case HANDPADDLE_BYTE_FRAMING_FAULT:
        ErrPrintf("Hand paddle byte framing fault\n");
        break;
      case PALMPILOT_SYNTAX_FAULT:
        ErrPrintf("Hand paddle syntax error fault\n");
        break;
      case PALMPILOT_NO_RESPONSE_FAULT:
        ErrPrintf("Hand paddle no response fault\n");
        break;
      case ANTENNA_COMPUTER_TIMEOUT_FAULT:
        ErrPrintf("Antenna computer velocity packet timeout fault\n");
        break;
      case ELEVATION_COLLISION_LIMIT_FAULT:
        ErrPrintf("Elevation below Collision limit and limit enabled\n");
        break;
      case AIR_PRESSURE_SWITCH_FAULT:
        ErrPrintf("Air pressure failed on azimuth mechanical brake\n");
        break;
      case AZIMUTH_ROCKER_FAULT:
        ErrPrintf("Azimuth rocker deployment fault\n");
        break;
      case ENC_TACH_DIFF_FAULT:
        ErrPrintf("Integral of tach not tracking lim encoder\n");
        break;
      }
    }
  }
}

static void parseStatusByte(unsigned int statusByte) {

#define PARSEBIT(mask,message) if(statusByte&mask)ErrPrintf("%s 1\n",message);else ErrPrintf("%s 0\n",message);

  ErrPrintf("status byte = 0x%0x\n",statusByte);
  PARSEBIT(STATUS_DATA_READY_HANDLER, "    Data ready idle");
  PARSEBIT(STATUS_ELEVATION_SOFTFAULT,"Elevation softfault");
  PARSEBIT(STATUS_AZIMUTH_SOFTFAULT,  "  Azimuth softfault");
  PARSEBIT(STATUS_ELEVATION_HARDFAULT,"Elevation hardfault");
  PARSEBIT(STATUS_AZIMUTH_HARDFAULT,  "  Azimuth hardfault");
  PARSEBIT(STATUS_ELEVATION_SERVO,    "    Elevation servo");
  PARSEBIT(STATUS_AZIMUTH_SERVO,      "      Azimuth servo");
  if (statusByte & STATUS_ALL_NOMINAL) {
    ErrPrintf(" ---->  All nominal, ready for servo operation  <----\n");
  } else {
    ErrPrintf(" ---->  not ready for servo operation  <----\n");
  }
}

float Thermistor(unsigned short counts) {
  double Voltage, Resistance;
  Voltage = ADCFACTOR*counts;
  Resistance = 10000*(5.-Voltage)/Voltage;
  return(pow(10,-0.085968*log10(Resistance)+2.819627)-273.14);
}

/* Check the number of resets on the SCB and if not equal to the previous value
 * write an error message. */
void CheckResets(void) {
  int numResets;

  ComStartPkt(SOFTWARE);
  SendPktGetRtn(SOFTWARE);
  numResets = ComGetS();
  if(numResets != prevNumResets) {
    ErrPrintf("The SCB was reset %d times\n", numResets -
              prevNumResets);
    prevNumResets = numResets;
  }
}

/* This assumes that the request for the scb to get the date was done in
 * OpenScb() */
void SetPalmCodeDate(void) {
  int i;
  unsigned char date[10];

  ComStartPkt(QUERYPALMVERSION);
  ComPutC(1);
  if(SendPktGetRtn(QUERYPALMVERSION) == 0) {
    for(i = 0; i < 9; i++) {
      date[i] = ComGetC();
      if((i == 5 && date[i - 1] > 'A') || (i == 4 && date[i] < 'A')) {
        date[i + 1] = date[i];
        date[i] = '2';
        i++;
      }
    }
    date[9] = '\0';
    RMSafeWrite("RM_SCB_PALM_CODE_DATE_C10", date);
  }
}

#if 0
enum IN_ARRAY_STATE StatusInArray(void) {
  int members[11], lowest;

  members[0] = 0;
  lowest = getAntennaList(members);
  if(myAntennaNumber == lowest) {
    return(LOW_IN_ARRAY);
  } else if(members[myAntennaNumber]) {
    return(IN_ARRAY);
  } else {
    return(NOT_IN_ARRAY);
  }
}
#endif

#endif /* !SIMULATING */

/*VARARGS*/
void ErrPrintf(char *s, ...) {
  char buf[256];
  va_list ap;

  va_start(ap, s);
  vsprintf(buf, s, ap);
  va_end(ap);
  fputs(buf, stderr);
}
