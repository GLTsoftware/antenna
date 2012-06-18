/*
 * Version for the GLT
 */

#define SERVOPRIO 75
#define TRACKPRIO 50
#define TRACKUSERPRIO 25

/* Define the shared memory for track to send servo position commands */
/* In LynxOS, BLOCKSIZE for memory allocation is 8K */
enum DRVCMD	{OFF_CMD=0, ON_CMD};
enum ACUACCESS { LOCAL = 1, REMOTE };
enum ACUAXISMODE { SHUTDOWN = 0, STANDBY, ENCODER, AUTONOMOUS,
	SURVIVAL_STOW, MAINTENANCE_STOW, VELOCITY};
enum DRVSTATE {
	SERVOOFF = 0, STARTING1, STARTING2, STARTING3, STOPPING1, STOPPING2, \
	TRACKING
};
enum FLTSTATE	{NO_FAULT = 0, FAULT, LOCKOUT, ONE_MOTOR_FAULT, EL_LMT_FAULT,
			AZ_LMT_FAULT, ONE_TACH_FAULT};
enum M3CMD	{CLOSE_M3_CMD = 0, OPEN_M3_CMD};
enum M3STATE	{UNKNOWN_ST = 0, CLOSED_ST, OPEN_ST, MOVING_ST, STUCK_ST,
	CLOSED_QUESTION, OPEN_QUESTION};
#define M3STATE_STRINGS {"unknown", "closed", "open", "moving", "stuck", \
	"closed?", "open?"}
#define TSSHMNAME "/trackservo"

enum ENCTYPE {HEIDENHAIN, ACC};

typedef struct {
	int msec;		/* msec of the day of this sample */
	int curAz;		/* The command to servo from Track (mas) */
	int shpAz;		/* Servo's shaped command to the pos loop */
	int encAz;		/* Current encoder value (mas)*/
	int cmdAzVel;		/* Vel cmd to scb (mas/s) */
	int tachAzVel;		/* Tach voltage cvt to Az axis vel (mas/s) */
	int curEl;		/* The command to servo from Track (mas) */
	int shpEl;		/* Servo's shaped command to the pos loop */
	int encEl;		/* Current encoder value (mas)*/
	int cmdElVel;		/* Vel cmd to scb (mas/s) */
	int tachElVel;		/* Tach voltage cvt to el axis vel (mas/s) */
	short azTorq;		/* Az torque cmd 32767 cnts -> 70 A/motor */
	short elTorq;		/* El torque cmd 32767 cnts -> 70 A */
	char azState, elState;
	char scbStatus;
} SAMPLE;

#ifndef SAMP_BUF_SIZE
#	define SAMP_BUF_SIZE 64		/* Must be a power of 2 */
#endif

#define NEXT_SAMP(i) ((i + 1) & (SAMP_BUF_SIZE - 1))
#define PREV_SAMP(i) ((i - 1) & (SAMP_BUF_SIZE - 1))
#define SFULL (tsshm->sampIn == tsshm->sampOut)
#define SaI tsshm->sampBuf[tsshm->sampIn]
#define SEMPTY (tsshm->sampIn == NEXT_SAMP(tsshm->sampOut))
#define SaO tsshm->sampBuf[tsshm->sampOut]
#define INC_OUT tsshm->sampOut = NEXT_SAMP(tsshm->sampOut)

enum topalm { NOTHING_TO_PALM = 0, EL_TO_PALM, AZ_TO_PALM };

typedef struct{
	/* Track to servo.  Servo only looks at these after msecCmd changes. */
	enum DRVCMD azCmd, elCmd;	/* Track's requested drive state */ 
	enum M3CMD m3Cmd;		/* Track's requested M3 position */
	double az, el;			/* Pos cmds to servo (mas) */
	double azVel, elVel;		/* Vel cmds to servo (mas/sec) */
	int msecCmd;			/* Ref time (UT) for command (msec) */
	enum topalm  sendEncToPalm;	/* If true, send HiRes enc to scb */

	/* Servo to Track */
	enum DRVSTATE azState, elState;	/* Servo's reported drive state */ 
	enum M3STATE m3State;		/* Servo's reported M3 state */
	double azTrError, elTrError;	/* az and el tracking errors in mas */
	double limAz, limEl;		/* Limit encoder readings (mas) */
	int msecAccept;			/* msecCmd of last cmd accepted */
	int day, msec;			/* UT Time which servo keeps current */
	int encAz, encEl, cmdAz, cmdEl;
	enum FLTSTATE fault;
	int scbStatus;
	unsigned int scbFaultWord;
	float elMotTemp, elMotCurrent;
	float azMot1Temp, azMot1Current;
	float azMot2Temp, azMot2Current;
	double lowerLimit, upperLimit;	/* Limit encoder el limits (mas) */
	double ccwLimit, cwLimit;	/* Limit encoder az limits (mas) */
	int padAzOffset ;
	int padID;
	int irigLock;
	int tachAzVel, tachElVel;
	float avgTrErrorArcSec;


	/* Data Recording
	 * The convention is that the buffer is full if Out == In and Servo
	 * will stop storing.  smapOut is the index to the buffer position
	 * which was last removed and should be incremented before use.
	 * sampIn is the position to store the next sample and should be
	 * incremented after use.  At startup Servo will set sampIn  =
	 * sampOut = 0, but otherwise only change sampIn.  The data storing
	 * program will only change sampOut as it removes samples or requests
	 * data taking to begin.  The macros NEXT_SAMP and PREV_SAMP should
	 * be used for manipulating sampIn and sampOut.
	 */
	SAMPLE sampBuf[SAMP_BUF_SIZE];	/* Servo writes samples here */
	int sampIn, sampOut;		/* Indices for putting or getting */
} TrackServoSHM;
#define TSSHMSZ sizeof(TrackServoSHM)

void *OpenShm(char *name, int size);

/*
The handshaking between Track and Servo needs to allow:

	Starting up in either order.
	Either or both to restart.
	Either to know whether the other is running with only a short delay.

The handshake conventions will be:

	Track will signal new position commands by updating msecCmd.
	It will do so at about once/sec.  After 2 sec with no update,
	Servo will presume that track has died or not started.  AzCmd
	and elCmd will be used immediately by servo without looking at
	msecCmd.  Servo will, however, check for new positions first.

	On starting, Track will set azCmd and elCmd to OFF_CMD.  If
	Servo is updating the time, Track may request ON_CMD

	Servo will keep the time up to date, so if msec stops changing,
	Track will presume that Servo has died or not started yet.

	On Starting, Servo will report OFF_ST, LOCKOUT_ST, or FAULT_ST
	and start reporting the time.  It will set msecAccept and the
	cmd time to the current time and wait for Track to change the
	cmd time before trying to do more.  Servo will run at a higher
	priority than Track and cannot be interrupted by track.  Track,
	however could be interrupted by Servo.

	After Track has put new positions, velocities, and commands in
	shared memory it will update msecCmd with the reference time
	for the new positions.  When Servo sees that msecCmd has
	changed, it will copy the new values and time into local memory
	and use them until msecCmd is changed again.

*/
