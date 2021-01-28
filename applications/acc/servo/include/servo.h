/*
 * Servo for the GLT with the original ACU
 */

#define MAS (3600000)

/* The following constants are used by checktrcmds.  They are set for
 * the SMA antennas, but should be save for the GLT */
#define AZ_AMAX (5*MAS)		/* For single motor operation. */
#define EL_AMAX (8.71095*MAS)
#define AZ_MIN_HTIME 800
#define EL_MIN_HTIME 500
/*
 * Constants for use in avoiding hitting limits.
 * VCRIT is the velocity from which stopping in MIN_HTIME requires AMAX.
 * At higher velocities the stopping time will be longer than MIN_HTIME
 * by Vel / VCRIT.
 */
#define AZ_VCRIT (AZ_MIN_HTIME * AZ_ACC_CONST / 500.)
#define EL_VCRIT (EL_MIN_HTIME * EL_ACC_CONST / 500.)
#define M 6
#define AZ_ACC_CONST (az_amax*(2.0/M_2_SQRTPI)/M)
#define EL_ACC_CONST (el_amax*(2.0/M_2_SQRTPI)/M)

#define EXP_HALF (1.64872127070012819416)
#define R (180./M_PI)
#define RAD_PER_MAS (1.0/(R*MAS))
#define MAS_PER_RAD (R*MAS)
#define vSendOpMessage(a,b,c,s) if(verbose) {fprintf(stderr, "%s\n", s); \
	} else {sendOpMessage(a,b,c,s);}

/* servo.c */
extern void ErrPrintf(char *s, ...);

/* checktrcmds.c */
#define SUNLIMIT (25*MAS)
#define RELAX (1*MAS)
#define RELAXEDLIMIT (SUNLIMIT-RELAX)
#define SAFELIMIT (SUNLIMIT+RELAX)
extern void CheckTrCmds(void);
extern int SunSafeMinutes(int az, int el);
extern int SunDistance(int az, int el);
enum {NOT_AVOIDING_SUN = 0, GOING_AROUND_SUN, ESCAPING_FROM_SUN};
int posInSunAvoid, avoidingSun;

/* servo.c */
enum IN_ARRAY_STATE {NOT_IN_ARRAY = 0, LOW_IN_ARRAY, IN_ARRAY};
enum IN_ARRAY_STATE StatusInArray(void);

extern TrackServoSHM *tsshm;	/* Local pointer to shared memory */
extern int lastAz, lastEl;	/* Values read from the ACU (mas) */
extern enum DRVSTATE azState, elState;
extern int trAzRaw, trAzVelRaw, trElRaw, trElVelRaw;
extern int trAz, trAzVel, trEl, trElVel;
extern int trMsecCmd;			/* Time of last command in msec */
extern double trAzVmax, trElVmax;	/* Max allowable command velocities */
extern int trAzVelBad, trElVelBad;
extern int azRockerBits;
extern double az_amax, el_amax;
extern short requiredSunSafeMinutes;	/* # min ahead of sun zone to avoid */
extern int presentSunSafeMinutes;	/* # min ahead of sun zone now */
extern int posInSunAvoid;		/* Current posn from Track in avoid */
extern int verbose;
