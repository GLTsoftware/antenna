/*
 * CVS Log and RCS Identification of this version:
 * $Id: servo.h,v 1.35 2007/02/06 00:00:25 rwilson Exp $
 * $Log: servo.h,v $
 * Revision 1.35  2007/02/06 00:00:25  rwilson
 * collision pad changes
 *
 * Revision 1.34  2006/06/29 21:48:51  rwilson
 * Timing diagnostica added, getAntennaList() removed
 *
 * Revision 1.33  2006/05/18 14:55:20  rwilson
 * move sun avoidance defs to servo.h
 *
 * Revision 1.32  2006/05/18 14:33:46  rwilson
 * Escape from avoidance zone, Op messages if unsafe
 *
 * Revision 1.31  2006/01/05 21:08:18  rwilson
 * OpenEncoders(reset) makes resetting Heidenhain encoders and EnDat card optional
 *
 * Revision 1.30  2005/10/25 15:24:31  rwilson
 * Report sun distance if in avoid zone
 *
 * Revision 1.29  2005/09/28 12:38:25  rwilson
 * put vSendOpMessage so when verbose, no op msgs
 *
 * Revision 1.28  2005/09/13 21:23:14  rwilson
 * change SunSafeTime to return minutes and remane SunSafeMinutes
 *
 * Revision 1.27  2005/09/01 18:13:26  rwilson
 * Rename sunSafeTime to SunSafeTime
 *
 * Revision 1.26  2005/08/09 13:24:25  rwilson
 * put in warning about sun safe minutes, IN_ARRAY stuff
 *
 * Revision 1.25  2005/07/29 16:20:18  rwilson
 * Fix bug, prepare to call SunSafeTime when tracking off
 *
 * Revision 1.24  2005/07/29 15:22:14  rwilson
 * moved CheckTrCmds to a separate src file, starting to check the sun
 *
 * Revision 1.23  2005/07/21 22:45:30  rwilson
 * Add sun safe vars
 *
 * Revision 1.22  2005/07/14 00:42:07  rwilson
 * add external declarations for some variables and subroutines for CheckTrCmds, define MAS_PER_RAD
 *
 * Revision 1.21  2005/05/20 17:25:23  rwilson
 * Add declaration for readencoders package
 *
 * Revision 1.20  2005/01/12 23:18:00  rwilson
 * Reduce elevation encoder diff tolerance to reach higher elevation, more op mesages
 *
 * Revision 1.19  2003/12/19 16:53:03  rwilson
 * Double EL_ENCODER_DIFF_TOLERANCE
 *
 * Revision 1.18  2003/08/05 13:03:56  rwilson
 * add amax to command line options
 *
 * Revision 1.17  2003/06/04 00:39:12  rwilson
 * larger, more rationsl ENCODER_DIFF_TOLLERANCE
 *
 * Revision 1.16  2002/12/04 19:01:50  rwilson
 * Rocker report
 *
 * Revision 1.15  2002/11/12 20:08:41  rwilson
 * extern variables for checkencoders
 *
 * Revision 1.14  2002/09/26 20:56:16  rwilson
 * bounds on vel and pos, more RM vars
 *
 * Revision 1.13  2002/01/16 22:08:01  rwilson
 * Ant 7 more normal
 *
 * Revision 1.12  2001/10/02 17:52:14  rwilson
 * fix compiled in az & el limits
 *
 * Revision 1.11  2001/09/01 15:13:27  rwilson
 * put in slowservo
 *
 * Revision 1.10  2001/06/29 19:02:29  rwilson
 * Todd's VMAX stuff for antenna 7
 *
 * Revision 1.9  2001/06/29 18:59:04  rwilson
 * Todd's VMAX stuff for antenna 7
 *
 * Revision 1.8  2001/05/31 19:51:08  rwilson
 * change ENC_TURN to 2^23
 *
 * Revision 1.7  2000/10/18 14:27:37  rwilson
 * use common OpenShm, rename tsshm.h
 *
 * Revision 1.6  2000/09/24 18:38:06  rwilson
 * Faultword to stderr, others
 *
 * Revision 1.5  2000/09/01 00:50:06  rwilson
 * more error messages
 *
 * Revision 1.4  2000/06/13 21:19:00  rwilson
 * Added cvs $ and $
 *
 */

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

#define MAS (3600000)

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
extern int encAz, encEl;	/* Values read from the encoders (mas) */
extern enum DRVSTATE azState, elState;
extern int azRockerBits;
extern int trAzRaw, trAzVelRaw, trElRaw, trElVelRaw;
extern int trAz, trAzVel, trEl, trElVel;
extern int trMsecCmd;			/* Time of last command in msec */
extern double trAzVmax, trElVmax;	/* Max allowable command velocities */
extern double az_amax, el_amax;
extern int trAzVelBad, trElVelBad;
extern int beepCnt;
extern short requiredSunSafeMinutes;	/* # min ahead of sun zone to avoid */
extern int presentSunSafeMinutes;	/* # min ahead of sun zone now */
extern int posInSunAvoid;		/* Current posn from Track in avoid */
extern int myAntennaNumber;
extern int verbose;
