/*
 * Servo for the GLT with the original ACU
 */

#if !GLT
#define MAX_CMD_AZ 358*MAS
#define MIN_CMD_AZ -175*MAS
#define MAX_CMD_EL 88.75*MAS
#define MIN_CMD_EL 9.5*MAS
#if SLOW
#define AZ_GAIN 5
#define AZ_VMAX_DEFAULT (2*MAS)
#else /* SLOW */
#define AZ_GAIN 5
#define AZ_VMAX_DEFAULT (4*MAS)
#endif /* SLOW */
#define EL_GAIN 5
#define EL_VMAX_DEFAULT (2.0*MAS)
#if 1
#define AZ_VMAX_ANTENNA7 (4*MAS)
#define EL_VMAX_ANTENNA7 (2*MAS)
#else
#define AZ_VMAX_ANTENNA7 (2*MAS)
#define EL_VMAX_ANTENNA7 (1.5*MAS)
#endif
#if SLOW
#define AZ_MIN_HTIME 800
#else /* SLOW */
#define AZ_MIN_HTIME 800
#endif /* SLOW */
#define EL_MIN_HTIME 500

/* MAX_XX_SCB_VEL should allow some headroom for overshoot in the vel loop
 * az a fault will occur if the ADC overloads. Commands are limited to
 * MAX_XX_SCB_VEL.  The nominal max velocities should result from commanding
 * FULL_XX_SCB_VEL.
 */
#define MAX_AZ_SCB_VEL 32000
#define MAX_EL_SCB_VEL 32000
#define FULL_AZ_SCB_VEL 30000
#define FULL_EL_SCB_VEL 30000

#if SLOW
#define AZ_AMAX (5*MAS)		/* For single motor operation. */
#else /* SLOW */
#define AZ_AMAX (5*MAS)		/* For single motor operation. */
#endif /* SLOW */
/* #define AZ_AMAX (8.71095*MAS) */
/* #define AZ_AMAX (4*MAS) */
#define EL_AMAX (8.71095*MAS)
#define M 6
#define AZ_ACC_CONST (az_amax*(2.0/M_2_SQRTPI)/M)
#define EL_ACC_CONST (el_amax*(2.0/M_2_SQRTPI)/M)
/*
 * Constants for use in avoiding hitting limits.
 * VCRIT is the velocity from which stopping in MIN_HTIME requires AMAX.
 * At higher velocities the stopping time will be longer than MIN_HTIME
 * by Vel / VCRIT.
 */
#define AZ_VCRIT (AZ_MIN_HTIME * AZ_ACC_CONST / 500.)
#define EL_VCRIT (EL_MIN_HTIME * EL_ACC_CONST / 500.)
#endif /* ! GLT */

#define AZ_AMAX (5*MAS)		/* For single motor operation. */
#define EL_AMAX (8.71095*MAS)
#define MAS (3600000)

#if ! GLT
#define POS_TOLERANCE 5000	/* Position change to force shaping (mas) */
#define VEL_TOLERANCE 5000	/* Velocity change to force shaping (mas/sec) */

/* The constants AZ_VEL_SCALE and EL_VEL_SCALE in the next two lines are
 * the velocities which should result from FULL_XX_SCB_VEL outputs. */
#define AZ_VEL_SCALE (4.000)
#define EL_VEL_SCALE (2.000)
#define AZ_SCB_VEL_PER_VEL (FULL_AZ_SCB_VEL / ((double)MAS * AZ_VEL_SCALE))
#define EL_SCB_VEL_PER_VEL (FULL_EL_SCB_VEL / ((double)MAS * EL_VEL_SCALE))

/* The following encoder scaling values are all given as doubles.  If
 * Integer values are needed, they must be (rounded and) cast as int.
 * Fine Axis encoder scaling.  Both acc and endat encoders are returned
 * as 23 bit integers. */
#define ENC_TURN (8388608)
#define ENC_TO_MAS (MAS*360./ENC_TURN)

/* Limit encoder scaling */
#define AZ_LIM_ENC_GR (422./25.)
#define AZ_LIM_ENC_TURN (4096.*AZ_LIM_ENC_GR)
#define AZ_LIM_ENC_TO_MAS (MAS*360./AZ_LIM_ENC_TURN)
#define EL_LIM_ENC_GR (188./50.)
#define EL_LIM_ENC_TURN (4096.*EL_LIM_ENC_GR)
#define EL_LIM_ENC_TO_MAS (MAS*360./EL_LIM_ENC_TURN)
#define AZ_ENCODER_DIFF_TOLERANCE (MAS/4)
#define EL_ENCODER_DIFF_TOLERANCE (MAS/2)
#if 0
#define OLD_EL_LIM_ENC_GR (422./25.)
#define OLD_EL_LIM_ENC_TURN (4096.*OLD_EL_LIM_ENC_GR)
#define OLD_EL_LIM_ENC_TO_MAS (MAS*360./OLD_EL_LIM_ENC_TURN)
#endif
#endif /* ! GLT */

#define EXP_HALF (1.64872127070012819416)
#define R (180./M_PI)
#define RAD_PER_MAS (1.0/(R*MAS))
#define MAS_PER_RAD (R*MAS)
#define vSendOpMessage(a,b,c,s) if(verbose) {fprintf(stderr, "%s\n", s); \
	} else {sendOpMessage(a,b,c,s);}

/* servo.c */
extern short antInArray;
extern void ErrPrintf(char *s, ...);

/* readencoders.c */
int OpenEncoders(int reset);
void ReadEncoders(int *encAzp, int *encElp);

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
#if ! GLT
extern int beepCnt;
extern int myAntennaNumber;
#endif /* ! GLT */
extern short requiredSunSafeMinutes;	/* # min ahead of sun zone to avoid */
extern int presentSunSafeMinutes;	/* # min ahead of sun zone now */
extern int posInSunAvoid;		/* Current posn from Track in avoid */
extern int verbose;
