/* Version for the GLT */
#define SMA 0

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
#if SMA
#include "rm.h"
#endif
#include "tsshm.h"
#include "servo.h"
#include "stderrUtilities.h"

/* Definitions which should be in readline/readline.h, but aren't in LynxOS */
void readline_initialize_everything(void);
char *readline(char *);
void add_history(char *);
int read_history(char *);
int write_history(char *);
#define HIST_FILE "./.testsun_history"
#define p(s) if(verbose) {fprintf(stderr, "%s\n", s);}
#define dprintf if(verbose) ErrPrintf

#define DO_DEL_HA 0

/* Sun Avoidance stuff */
#define ENFORCE_REQUIRED_SUN_SAFE_MINUTES 0
#define LATITUDE_DEGREES (19.824205263889)
static double clat = 0.94073758125, slat = 0.33913537595;
#define COSSUNLIMIT 0.90630778704
#define SINSUNLIMIT 0.42261826174
#define SINRELAXEDSUNLIMIT 0.39073112849
#define SUNLIMITMAS (25*MAS)
#if 1
#define HISUN (tsshm->upperLimit - SUNLIMITMAS)
#define LOSUN (tsshm->lowerLimit + SUNLIMITMAS)
#else
#define HISUN (60*MAS)
#define LOSUN (40*MAS)
#endif
int posInSunAvoid, avoidingSun;
static int sunAz = 64*MAS, sunEl = 0*MAS;
/* static int sunAz = 270*MAS, sunEl = 50*MAS; */

short requiredSunSafeMinutes;	/* # min ahead of sun zone to avoid */
static int azHalfWidth;	/* Half width of sun avoidance zone (mas) */
static int holding = 0;	/* If non-zero, don't accept new positions */
enum{NOT_ESCAPING = 0, HOLDING_SAFE_POS, MOVING_TO_SAFETY};
static int escaping = 0; /* Non-zero when moving out of the avoidance zone */
static int escAz, escEl; /* poistion escaping to (dec = sunDec +- SAFELIMIT) */

/* Structure to hold the classification of the start and destination positions.
 *
 * The av structure holds the classification with respect to the avoidance zone.
 * For elevation:
 *   LT is below the av zone
 *   EQ within the zone
 *   GT above it.
 * For azimuth:
 *   LT_NEG is more negative than the sun in the negative wrap (az < 0) zone
 *   EQ_NEG is within the neg wrap av zone
 *   LT between the neg wrap and pos wrap avoidance zones
 *   EQ within the pos wrap av zone
 *   GT az greater than the pos wrap zone.
 *
 * The sun struct holds the classification with respect to the sun position.
 * The classifications are the same, except that equality is not allowed.
 */
enum CLASSIFICATION {LT_NEG = 1, EQ_NEG, LT, EQ, GT};
#ifdef USE_MAIN
char clasStrings[][7] = {"  ", "LT_NEG", "EQ_NEG", "LT", "EQ", "GT"};
#endif

#define SAFE(p) (p & 1)
struct POSCLASS {
    /* Classification of position wrt the avoidance zone */
    struct {
        enum CLASSIFICATION el;
        enum CLASSIFICATION az;
    }
    av;
    /* classification of the position wrt the center of the sun */
    struct {
        enum CLASSIFICATION el;
        enum CLASSIFICATION az;
    }
    sun;
}
start, dest;

/* checktrcmds.c */
void AEtoHD(int az, int el, int *ha, int *dec);
void HDtoAE(int ha, int dec, int *az, int *el);
void SetSafeAz(struct POSCLASS *pcp);
void SetSafeEl(struct POSCLASS *pcp);
int  SunDistance(int az, int el);
void ClassifyPosition(int az, int el, struct POSCLASS *pcp);
void Run(void);
void PrintAll(void);

void CheckTrCmds(void) {
    double azStopPosn, elStopPosn;
    int tmpEl, tmpElVel, tmpAz, tmpAzVel;
    int ssm;
    char msg[80];

    if(tsshm->msecCmd != tsshm->msecAccept) {
      if(escaping == MOVING_TO_SAFETY) {
        if(abs(encAz - escAz) < (MAS/2048) && abs(encEl - escEl) < (MAS/2048)) {
            escaping = HOLDING_SAFE_POS;
	    goto CheckTrCmdsEnd;
        }
	tmpAz = escAz;
	tmpAzVel = 0;
	tmpEl = escEl;
	tmpElVel = 0;
      } else {
        trAzRaw = tsshm->az;
        trElRaw = tsshm->el;
        if(tsshm->az != tsshm->az || tsshm->azVel != tsshm->azVel ||
                tsshm->el != tsshm->el || tsshm->elVel != tsshm->elVel) {
            if(azState > STOPPING) {
                azState = STOPPING;
                beepCnt = 2;
            }
            if(elState > STOPPING) {
                elState = STOPPING;
                beepCnt = 2;
            }
            if(beepCnt == 2) {
                vSendOpMessage(OPMSG_SEVERE, 19, 60, "Received NaN from Track");
                ErrPrintf("Track sent NaN az %.4f azVel %.4f el %.4f elVel "
		    "%.4f\n", tsshm->az/(double)MAS, tsshm->azVel/(double)MAS,
		    tsshm->el/(double)MAS, tsshm->elVel/(double)MAS);
            }
            trAzVelRaw = tsshm->azVel;
            trElVelRaw = tsshm->elVel;
	    goto CheckTrCmdsEnd;
        }
        tmpAz = tsshm->az;
        trAzVelRaw = tsshm->azVel;
        if(fabs(tsshm->azVel) > trAzVmax) {
            if(trAzVelBad < 2 ) {
                trAzVelBad++;
                tmpAzVel = trAzVel;	/* Maintian the old value */
            } else {
                tmpAzVel = 0;
            }
        } else {
            tmpAzVel = tsshm->azVel;
            trAzVelBad = 0;
        }
        if(abs(tmpAzVel) < AZ_VCRIT) {
            azStopPosn =  tmpAz + tmpAzVel * (AZ_MIN_HTIME/1000.);
        } else {
            azStopPosn =  tmpAz + tmpAzVel * ((double)abs(tmpAzVel)) *
                          ((AZ_MIN_HTIME/1000.) / AZ_VCRIT);
        }
        if(azStopPosn > tsshm->cwLimit || tmpAz > tsshm->cwLimit) {
            tmpAz = tsshm->cwLimit;
            tmpAzVel = 0;
        } else if(azStopPosn < tsshm->ccwLimit || tmpAz < tsshm->ccwLimit) {
            tmpAz = tsshm->ccwLimit;
            tmpAzVel = 0;
        }

        tmpEl = tsshm->el;
        trElVelRaw = tsshm->elVel;
        if(fabs(tsshm->elVel) > trElVmax) {
            if(trElVelBad < 2 ) {
                trElVelBad++;
                tmpElVel = trElVel;	/* Maintian the old value */
            } else {
                tmpElVel = 0;
            }
        } else {
            tmpElVel = tsshm->elVel;
            trElVelBad = 0;
        }

        /* Make sure that we can stop before hitting an upper or lower limit */
        if(abs(tmpElVel) < EL_VCRIT) {
            elStopPosn =  tmpEl + tmpElVel * (EL_MIN_HTIME/1000.);
        } else {
            elStopPosn =  tmpEl + tmpElVel * ((double)abs(tmpElVel)) *
                          ((EL_MIN_HTIME/1000.) / EL_VCRIT);
        }
        if(elStopPosn > tsshm->upperLimit || tmpEl > tsshm->upperLimit) {
            tmpEl = tsshm->upperLimit;
            tmpElVel = 0;
        } else if(elStopPosn < tsshm->lowerLimit || tmpEl < tsshm->lowerLimit) {
            tmpEl = tsshm->lowerLimit;
            tmpElVel = 0;
        }
      }
        /* Now check for sun avoidance */
        ssm = SunSafeMinutes(tmpAz, tmpEl);
        if(ssm == 0) {
#if ENFORCE_REQUIRED_SUN_SAFE_MINUTES
	  if((ssm = SunSafeMinutes(encAz, encEl)) >= 0 &&
		  ssm < requiredSunSafeMinutes) {
#else /* ENFORCE_REQUIRED_SUN_SAFE_MINUTES */
	  if(SunSafeMinutes(encAz, encEl) != 0) {
#endif /* ENFORCE_REQUIRED_SUN_SAFE_MINUTES */
	    posInSunAvoid = 1;
            if(antInArray < IN_ARRAY) {
                sprintf(msg,
                    "Ant %d cmd pos too close to Sun - Ignoring it",
                    myAntennaNumber);
                vSendOpMessage(OPMSG_SEVERE, 21, 5, msg);
            }
	    goto CheckTrCmdsEnd;
	  } else {
	    int sunHa, sunDec, curHa, curDec;

	    dprintf("Starting from unsafe position\n");
	    AEtoHD(sunAz, sunEl, &sunHa, &sunDec);
	    AEtoHD(encAz, encEl, &curHa, &curDec);
	    curDec = sunDec + ((curDec < sunDec)? -SAFELIMIT: SAFELIMIT);
	    HDtoAE(curHa, curDec, &tmpAz, &tmpEl);
	    if(encAz < 0 && tmpAz > 180*MAS) tmpAz -= 360*MAS;
	    dprintf("Trying  %10.4fA %10.4fE %10.4fH %10.4fD)\n",
		(double)tmpAz/MAS, (double)tmpEl/MAS,
		(double)curHa/MAS, (double)curDec/MAS);
	    if(tmpAz < tsshm->ccwLimit || tmpAz > tsshm->cwLimit ||
		tmpEl < tsshm->lowerLimit || tmpEl > tsshm->upperLimit ||
		abs(tmpAz - encAz) > 110*MAS){
	      curDec = sunDec + ((curDec < sunDec)? SAFELIMIT: -SAFELIMIT);
	      HDtoAE(curHa, curDec, &tmpAz, &tmpEl);
	      if(encAz < 0 && tmpAz > 180*MAS) tmpAz -= 360*MAS;
	      dprintf("Trying  %10.4fA %10.4fE %10.4fH %10.4fD)\n",
		(double)tmpAz/MAS, (double)tmpEl/MAS,
		(double)curHa/MAS, (double)curDec/MAS);
	      if(tmpAz < tsshm->ccwLimit || tmpAz > tsshm->cwLimit ||
		  tmpEl < tsshm->lowerLimit || tmpEl > tsshm->upperLimit ||
		    abs(tmpAz - encAz) > 110*MAS){
		tmpAz = 10*MAS;
		tmpEl = 45*MAS;
	      }
	    }
	    escAz = tmpAz;
	    escEl = tmpEl;
	    escaping = MOVING_TO_SAFETY;
	    ErrPrintf("Escaping from sun at %8.3f %8.3f to %8.3f %8.3f "
		"from %8.3f %8.3f\n", (double)sunAz/MAS, (double)sunEl/MAS,
		 (double)escAz/MAS, (double)escEl/MAS,
		 (double)encAz/MAS, (double)encEl/MAS);
	  }
        } else {
	    posInSunAvoid = 0;
	    escaping =  NOT_ESCAPING;
#ifdef USE_MAIN
            printf("New position tmpAz = %.2f tmpEl = %.2f ssm = %d\n",
		(double)tmpAz / MAS, (double)tmpEl / MAS, ssm);
#endif
        }
        if(holding) {
            if(abs(encAz - trAz) < 2*MAS && abs(encEl - trEl) < 1*MAS) {
                holding = 0;
            } else {
		goto CheckTrCmdsEnd;
            }
        }
	/* If the position change is not large, no need to plan it as both
	 * ends are guaranteed to be safe. */
        if((abs(trAz-tmpAz) < 5*MAS) && (abs(trEl - tmpEl) < 5*MAS)) {
/*	    p("Small change, so doit"); */
            goto doit;
        }
        ClassifyPosition(tmpAz, tmpEl, &dest);
        ClassifyPosition(encAz, encEl, &start);
#ifdef USE_MAIN
        printf("Moving from av sun (%s, %s)(%s, %s) to (%s, %s)(%s, %s)"
		" az hwidth %.2f\n",
               clasStrings[start.av.az], clasStrings[start.av.el],
	       clasStrings[start.sun.az], clasStrings[start.sun.el],
               clasStrings[dest.av.az], clasStrings[dest.av.el],
	       clasStrings[dest.sun.az], clasStrings[dest.sun.el],
               (double)azHalfWidth/MAS);
#endif
        /* if either axis starts and ends in the same safe zone, do it */
        if(((start.av.az == dest.av.az) && SAFE(start.av.az)) ||
                ((start.av.el == dest.av.el) && SAFE(start.av.el))) {
	    p("One axis start and dest in same safe zone, do it.");
            goto doit;
	}

        /* If the sun is high and azimuth will pass the sun, start by
         * moving elevation safely below the sun.  If az does not pass
         * the Sun, doit. */
        if(sunEl > HISUN && start.av.el > LT) {
            if(start.sun.az == dest.sun.az) {
		p("HISUN ok");
                goto doit;
            } else {
		p("HISUN move down");
                trEl = sunEl - SUNLIMIT;
                trElVel = 0;
            }

        /* If az has both end points in safe zones, but not the same one */
        } else if(SAFE(start.av.az) & SAFE(dest.av.az)) {
            /* If el starts in an unsafe zone */
            if(!SAFE(start.av.el)) {
                /* Move it to a safe zone */
		p("Az both ok, moving el to safe");
		SetSafeEl(&start);
            } else {
                /* Move az to destination */
		p("Az both ok, start el ok, move az");
                trAz = tmpAz;
                trAzVel = tmpAzVel;
            }

        /* If el has both end points in safe zones, but not the same one */
        } else if(SAFE(start.av.el) & SAFE(dest.av.el)) {
            /* If az starts in an unsafe zone */
            if(!SAFE(start.av.az)) {
                /* Move it to a safe zone */
		p("El both ok, moving az to safety");
		SetSafeAz(&start);
            } else {
                /* Move el to destination */
		p("El both ok, start az ok, move el");
                trEl = tmpEl;
                trElVel = tmpElVel;
            }

        /* If az starts safe and el ends safe, move el first */
        } else if(SAFE(start.av.az) && SAFE(dest.av.el)) {
	    p("Az starts safe and el ends safe, move el first");
            trEl = tmpEl;
            trElVel = tmpElVel;

        /* If el starts safe and az ends safe, move az first */
        } else if(SAFE(start.av.el) && SAFE(dest.av.az)) {
	    p("El starts safe and az ends safe, move az first");
            trAz = tmpAz;
            trAzVel = tmpAzVel;

	/*
	 *If we get here, either start or dest must have both az and el
	 * unsafe.
	 */

	/* If start is doubly unsafe, move to the corner unless start and
	 * dest are in same quadrant from the Sun. */
	} else if(!SAFE(start.av.az) && !SAFE(start.av.el)) {
	    if((start.sun.az == dest.sun.az) && (start.sun.el == dest.sun.el)) {
		p("Start doubly unsafe dest in same quadrant, doit");
		goto doit;
	    }  else {
		p("start doubly unsafe, move to corner");
		SetSafeAz(&start);
		SetSafeEl(&start);
		dprintf("Move to Az, El %9.4f  %9.4f\n",
			(double)trAz/MAS, (double)trEl/MAS);
	    }

	/* Dest must be doubly unsafe.  If el must cross the sun */
	} else if(start.sun.el != dest.sun.el) {
	    /* If az is safe, move to a safe elevation on the same side of
	     * the sun as the dest. */
	    if(SAFE(start.av.az)) {
		p("Dest doubly unsafe, az safe, move el to dest side");
		SetSafeEl(&dest);
	    /* Otherwise go to a safe azimuth on the same side as start */
	    } else {
		p("Dest doubly unsafe move az to safety on start side");
		SetSafeAz(&start);
	    }

	/* If az must cross the sun */
	} else if(start.sun.az != dest.sun.az) {
	    if(SAFE(start.av.el)) {
		p("Dest doubly unsafe, el safe, so move az to dest side");
		SetSafeAz(&dest);
	    } else {
		p("Dest doubly unsafe move el to safety on start side");
		SetSafeEl(&start);
	    }

	/* Start and dest are in the same quadrant from the sun, so we can
	 * move directly to dest and at worst nick the avoidance zone. */
	} else {
	    p("Dest doubly unsafe in same quadrant, doit");
	    goto doit;
	}

        /* If we fall through to here, we will make a move to an
         * intermediate position and then recalculate what to do next. */
        holding = 1;
	goto CheckTrCmdsEnd;

doit:
	/* If we get here, we will carry out the requested move directly. */
        trAz = tmpAz;
        trAzVel = tmpAzVel;
        trEl = tmpEl;
        trElVel = tmpElVel;
	goto CheckTrCmdsEnd;
    }
CheckTrCmdsEnd:
    if(escaping) {
        if(antInArray < IN_ARRAY) {
            sprintf(msg, "Ant %d cmd and actual positions too close to sun - "
		"Moving to %9.4f %9.4f\n", myAntennaNumber, (double)escAz/MAS,
		(double)escEl/MAS);
            vSendOpMessage(OPMSG_SEVERE, 15, 5, msg);
        }
        avoidingSun = ESCAPING_FROM_SUN;
    } else {
	avoidingSun = holding;
    }
    trMsecCmd = tsshm->msecCmd;
    tsshm->msecAccept = tsshm->msecCmd;
}

/* Assume that the position whose classification has been passed is unsafe */
void SetSafeAz(struct POSCLASS *pcp) {
    if(pcp->av.az == EQ) {
	trAz = sunAz + ((pcp->sun.az == GT)? azHalfWidth: -azHalfWidth);
    } else {
	trAz = sunAz -(360*MAS) +
	    ((pcp->sun.az == LT)? azHalfWidth: -azHalfWidth);
    }
    trAzVel = 0;
}

void SetSafeEl(struct POSCLASS *pcp) {
    int d;

    if(sunEl < LOSUN) d = SUNLIMIT;
    else if(sunEl > HISUN) d = -SUNLIMIT;
    else if(pcp->sun.el == GT) d = SUNLIMIT;
    else d = -SUNLIMIT;
    trEl = sunEl + d;
    trElVel = 0;
}

void AEtoHD(int az, int el, int *ha, int *dec) {
    double sel, cel, caz, x, y, z;

    cel = cos((double)el * RAD_PER_MAS);
    sel = sin((double)el * RAD_PER_MAS);
    caz = cos((double)az * RAD_PER_MAS);
    y = -cel * sin((double)az * RAD_PER_MAS);
    x = sel * clat - cel * caz * slat;
    z = sel * slat + cel * caz * clat;
    *dec = (int)(atan2(z, sqrt(x*x + y*y)) * MAS_PER_RAD);
    *ha = (int)(atan2(y, x) * MAS_PER_RAD);
    /*  if(*ha < 0) *ha += (360 * MAS); */
}

void HDtoAE(int ha, int dec, int *az, int *el) {
    AEtoHD(ha, dec, az, el);
    if(*az < 0) *az += 360*MAS;
}

int SunDistance(int az, int el) {
    double cosd, srcElR, sunElR;

    srcElR = (double)el * RAD_PER_MAS;
    sunElR = (double)sunEl * RAD_PER_MAS;

    /* convert to delta az */
    az -= sunAz;
    cosd = sin(srcElR) * sin(sunElR) + cos(srcElR) * cos(sunElR) *
	cos((double)(az) * RAD_PER_MAS);

    return(acos(cosd) * MAS_PER_RAD);
}

void ClassifyPosition(int az, int el, struct POSCLASS *pcp) {
    static int oldSunEl = -200*MAS;
    static int relaxedAzHalfWidth;	/* in mas */

    if(sunEl < 0) {
        pcp->av.az = pcp->av.el = pcp->sun.az = pcp->sun.el = GT;
	return;
    }
    if(sunEl != oldSunEl) {
        oldSunEl = sunEl;
        if(sunEl >= (90*MAS) - SUNLIMIT) {
            relaxedAzHalfWidth = azHalfWidth = 0;
        } else {
            azHalfWidth = MAS_PER_RAD * asin(SINSUNLIMIT /
                                             cos(sunEl * RAD_PER_MAS));
            relaxedAzHalfWidth = MAS_PER_RAD * asin(SINRELAXEDSUNLIMIT /
                                                    cos(sunEl * RAD_PER_MAS));
        }
    }
    if(az > sunAz + relaxedAzHalfWidth)
        pcp->av.az = GT;
    else if(az > sunAz - relaxedAzHalfWidth)
        pcp->av.az = EQ;
    else if(az > sunAz - (360*MAS) + relaxedAzHalfWidth)
        pcp->av.az = LT;
    else if(az > sunAz - (360*MAS) - relaxedAzHalfWidth)
        pcp->av.az = EQ_NEG;
    else
        pcp->av.az = LT_NEG;
    if(sunEl < HISUN && el > sunEl + RELAXEDLIMIT)
        pcp->av.el = GT;
    else if(el > (sunEl - RELAXEDLIMIT))
        pcp->av.el = EQ;
    else
        pcp->av.el = LT;
    if(az > sunAz)
        pcp->sun.az = GT;
    else if(az > sunAz - (360*MAS))
        pcp->sun.az = LT;
    else
        pcp->sun.az = LT_NEG;
    if(el > sunEl)
        pcp->sun.el = GT;
    else
        pcp->sun.el = LT;

#if 0
    printf("pos (%.2f, %.2f) class (%d, %d) Sun (%.2f, %.2f)\n",
           (double)az / MAS, (double)el / MAS, pcp->av.az, pcp->av.el,
           (double)sunAz / MAS, (double)sunEl / MAS);
#endif
}

#if DO_DEL_HA
int minRelHa(int sunDec, int trDec) {
    return(acos((COSSUNLIMIT -
                 cos((90*MAS - trDec)*RAD_PER_MAS) *
                 cos((90*MAS - sunDec)*RAD_PER_MAS)) /
                (sin((90*MAS - trDec)*RAD_PER_MAS) *
                 sin((90*MAS - sunDec)*RAD_PER_MAS)))
           / RAD_PER_MAS);
}
#endif

/* Get the position of the sun from RM and calculate the time (min)  before the
 * sun will be within SUNLIMIT degrees of the given az and el.  If the position
 * is already unsafe, return 0.  If the position is always safe (declination
 * of the position more than SUNLIMIT degrees from sunDec) return -1.
 *
 * I use the cosine formula for a spherical triangle with vertices ABC and
 * opposite sides abc: cos(a) = cos(b)*cos(c) + sin(b)*sin(c)*cos(A) where
 * A is the North Celestial Pole, B is the sun and C is the point on the
 * circle of radius SUNLIMIT around the sun at the given declination and larger
 * ha than the sun.  The length of a is SUNLIMIT, b is the co-dec of the
 * given position and c is the co-dec of the sun.  The angle A gives the unsafe
 * time ahead of the ha of the sun.
 */
int SunSafeMinutes(int az, int el) {
    static int sunHa, sunDec;
#ifndef USE_MAIN
    float f;
#endif
    int ha, dec, unsafeDHa, safeDHa;

#define USE_FAKE_SUN 0
#if USE_FAKE_SUN
    AEtoHD(sunAz, sunEl, &sunHa, &sunDec);
#else /* USE_FAKE_SUN */

#ifndef USE_MAIN

    static int oldMsec = -100000;

    if(abs(tsshm->msec - oldMsec) > 30000) {
#if SMA
        rm_read(RM_ANT_0,"RM_SUN_AZ_DEG_F", &f);
        sunAz = f * MAS;
        rm_read(RM_ANT_0,"RM_SUN_EL_DEG_F", &f);
        sunEl = f * MAS;
        rm_read(RM_ANT_0,"RM_REQUIERD_SUN_SAFE_MINUTES_S",
                &requiredSunSafeMinutes);
#else
	sunAz = 180.0 * MAS;
	sunEl = 45.0 * MAS;
#endif
        AEtoHD(sunAz, sunEl, &sunHa, &sunDec);
        oldMsec = tsshm->msec;
    }
#else
    AEtoHD(sunAz, sunEl, &sunHa, &sunDec);
#endif

#endif /* USE_FAKE_SUN */


    if(sunEl > HISUN && el > 80*MAS && abs(az -sunAz) < 60*MAS)
        return(0);

    AEtoHD(az, el, &ha, &dec);
#if 0
    printf("Sun az-el %.4f %.4f\n", (double)sunAz / MAS, (double)sunEl / MAS);
    printf("Sun %.4f %.4f Src %.4f %.4f\n", (double)sunHa/MAS,
           (double)sunDec/MAS, (double)ha/MAS, (double)dec/MAS);
#endif

    if(abs(sunDec - dec) > SUNLIMIT)
        return(-1);
    unsafeDHa = acos((COSSUNLIMIT -
                      cos((90*MAS - dec)*RAD_PER_MAS) *
                      cos((90*MAS - sunDec)*RAD_PER_MAS)) /
                     (sin((90*MAS - dec)*RAD_PER_MAS) *
                      sin((90*MAS - sunDec)*RAD_PER_MAS))) * MAS_PER_RAD;
    if(ha < sunHa) {
        if(sunHa - ha < unsafeDHa + 450000) {
	  if(requiredSunSafeMinutes == -5 && sunEl < 5*MAS)
            return(1);
	  else
            return(0);
	}
        safeDHa = (MAS * 360) + ha - (sunHa - unsafeDHa);
        /* printf("Sun W of src, safeDHa %d\n", safeDHa/15000); */
    } else {
        if(ha - sunHa < unsafeDHa + 450000) {
	  if(requiredSunSafeMinutes == -5 && sunEl < 5*MAS)
            return(1);
	  else
            return(0);
	}
        safeDHa = ha - (sunHa + unsafeDHa);
        /* printf("Sun E of src, safeDHa %d\n", safeDHa/15000); */
    }
    return((safeDHa + 451000) / 900000);	/* 900000 mas per min of time */
}

#ifdef USE_MAIN
TrackServoSHM sharedMem;		/* Local pointer to shared memory */
TrackServoSHM *tsshm = &sharedMem;	/* Local pointer to shared memory */
int encAz = 100*MAS, encEl = 30*MAS;	/* Values from the encoders (mas) */
int trAz = 100*MAS, trAzVel = 0, trEl = 30*MAS, trElVel = 0;
enum DRVSTATE azState, elState;
int azRockerBits;
int trAzRaw, trAzVelRaw, trElRaw, trElVelRaw;
int trMsecCmd;			/* Time of last command in msec */
double trAzVmax = 3*MAS, trElVmax = 1.5*MAS;	/* Max command velocities */
double az_amax = 5*MAS, el_amax = 8.7*MAS;
int trAzVelBad, trElVelBad;
int beepCnt;
short requiredSunSafeMinutes = 120;	/* # min ahead of sun zone to avoid */
int presentSunSafeMinutes;		/* # min ahead of sun zone now */
int posInSunAvoid;			/* Current posn from Track in avoid */
int myAntennaNumber = 1;
int verbose = 1;
short antInArray = LOW_IN_ARRAY;

void usage(void) {
    printf("The commands and arguments are:\n"
           "r run\np print\n"
           "sa sunAz sunEl\nsh sunHa sunDec\nss sunSafeMinutes\n"
           "ta srcAz srcEl\nth srcHa srcDec\n"
           "aa encAz encEl\nah encHa encDec\n");
}


int main(int argc, char *argv[]) {
    double a1, a2;
    int c1, c2;
    int wrap = 0;
#if 0
    int sunHa, sunDec, trHa, trDec, srcAz, srcEl;
#endif

    int nFields;
    char cmd[8], *line;

    readline_initialize_everything();
    read_history(HIST_FILE);
    tsshm->az = trAz = encAz;
    tsshm->azVel = 0;
    tsshm->el = trEl = encEl;
    tsshm->elVel = 0;
    tsshm->lowerLimit = 11*MAS;
    tsshm->upperLimit = 87.5*MAS;
    tsshm->cwLimit = 350*MAS;
    tsshm->ccwLimit = -170*MAS;
    for(;;) {
        line = readline("testsun: ");
        if(line == 0 || line[0] == 'q') {
            write_history(HIST_FILE);
            exit(0);
        }
        if(!*line)
            continue;			/* Readline removes the '\n' */
        add_history(line);
        nFields = sscanf(line, "%6s %lf %lf", cmd, &a1, &a2);
        free(line);
        if(strcmp(cmd, "p") == 0 && nFields == 1) {
            PrintAll();
        } else if(strcmp(cmd, "w") == 0) {
	    if(nFields <= 1) {
		printf("wrap = %d\n", wrap);
	    } else {
	        wrap = a1;
	    }
        } else if(strcmp(cmd, "ss") == 0 && nFields == 2) {
            requiredSunSafeMinutes = a1;
        } else if(strcmp(cmd, "r") == 0) {
	    Run();
        } else if(nFields != 3) {
            usage();
            continue;
        } else if(strcmp(cmd, "sa") == 0) {
            sunAz = a1 * MAS;
            sunEl = a2 * MAS;
        } else if(strcmp(cmd, "sh") == 0) {
            HDtoAE((int)(a1*MAS), (int)(a2*MAS), &sunAz, &sunEl);
        } else if(strcmp(cmd, "ta") == 0) {
            tsshm->az = a1*MAS;
            tsshm->el = a2*MAS;
	    Run();
        } else if(strcmp(cmd, "th") == 0) {
            HDtoAE((int)(a1*MAS), (int)(a2*MAS), &c1, &c2);
	    if(wrap && c1 > tsshm->ccwLimit + 360*MAS) c1 -= 360*MAS;
            tsshm->az = c1;
            tsshm->el = c2;
	    Run();
        } else if(strcmp(cmd, "aa") == 0) {
            tsshm->az = trAz = encAz = a1*MAS;
            tsshm->el = trEl = encEl = a2*MAS;
        } else if(strcmp(cmd, "ah") == 0) {
            HDtoAE((int)(a1*MAS), (int)(a2*MAS), &c1, &c2);
	    if(wrap && c1 > tsshm->ccwLimit + 360*MAS) c1 -= 360*MAS;
            encAz = c1;
            encEl = c2;
	} else {
	    printf("No command %s\n", cmd);
        }
    }

#if 0
    if(argc < 5) {
        fprintf(stderr,
                "Usage: testsun targetHa, targetDec, sunHa, sunDec (All in Deg)\n");
        exit(1);
    }
    trHa = MAS * atof(argv[1]);
    trDec = MAS * atof(argv[2]);
    sunHa = MAS * atof(argv[3]);
    sunDec = MAS * atof(argv[4]);
    msecEnd = (argc > 5)? atol(argv[5]) * 1000: 3000;
    for(tsshm->msecCmd = 0; tsshm->msecCmd < msecEnd; tsshm->msecCmd += 1000) {
        HDtoAE(sunHa, sunDec, &sunAz &sunEl);
        HDtoAE(trHa, trDec, &srcAz, &srcEl);
        tsshm->az = srcAz;
        tsshm->el = srcEl;
        printf("Sun: ha %.4f dec %.4f az %.4f el %.4f\n", (double)sunHa/MAS,
               (double)sunDec/MAS, (double)sunAz / MAS, (double)sunEl / MAS);
        printf("Tgt: ha %.4f dec %.4f az %.4f el %.4f\n", (double)trHa/MAS,
               (double)trDec/MAS, (double)tsshm->az/MAS, (double)tsshm->el/MAS);
#if DO_DEL_HA

        if(abs(sunDec - trDec) < SUNLIMIT) {
            printf("DelHa %.4f\n",  minRelHa(sunDec, trDec)/(double)MAS);
        }
#endif
	printf("Safe minutes %d\n", SunSafeMinutes(srcAz, srcEl));
	trHa += 15000;
	sunHa += 15000;
    }
#endif
    return(0);
}

void Run(void) {
    int ssm, hcount;

    hcount = 0;
    do {
	ssm = SunSafeMinutes(encAz, encEl);
	if(ssm == 0) {
	    printf("Present antenna position is unsafe, continuing\n");
	}
	printf("Ssm = %d\n", ssm);
	tsshm-> msecCmd += 10;
	CheckTrCmds();
	encAz = trAz;
	encEl = trEl;
	PrintAll();
	if(hcount++ > 3) {
	    printf("Loop encountered, quitting\n");
	    break;
	}
    } while(holding || escaping == MOVING_TO_SAFETY);
}

void sendOpMessage(int severity, int priority, int duration, char *text) {
    printf("OpMsg: %d %d %d %s", severity, priority, duration, text);
}

/*VARARGS*/
void ErrPrintf(char *s, ...) {
    char buf[256];
    va_list ap;

    va_start(ap, s);
    vsprintf(buf, s, ap);
    va_end(ap);
    fputs(buf, stderr);
}
#endif

#if 0
/* the following statements give the sun distance using the original
 * form of Transform in which the 3rd arg is Latitude */
int ssDist, ssAz;

Transform(trHa - sunHa, trDec, sunDec, &ssAz, &ssDist);
ssDist = 90*MAS - ssDist;
printf("Az to src %.4f, dist %.4f\n", ssAz/(double)MAS, ssDist/(double)MAS);
#endif

void PrintAll(void) {
    int c1, c2;
    char fmt[] = "%-8s%10.4f  %10.4f  %10.4f  %10.4f\n";

    printf("	    Az		El	    Ha		Dec\n");
    AEtoHD(sunAz, sunEl, &c1, &c2);
    printf(fmt, "Sun", (double)sunAz / MAS, (double)sunEl / MAS,
           (double)c1/MAS, (double)c2/MAS);
    AEtoHD(tsshm->az, tsshm->el, &c1, &c2);
    printf(fmt, "Track", (double)tsshm->az/MAS, (double)tsshm->el/MAS,
           (double)c1/MAS, (double)c2/MAS);
    AEtoHD(encAz, encEl, &c1, &c2);
    printf(fmt, "Enc", (double)encAz/MAS, (double)encEl/MAS,
           (double)c1/MAS, (double)c2/MAS);
    printf("Sun dist %.2f  holding %d  escaping %d\n",
	(float)SunDistance(encAz, encEl)/MAS, holding, escaping);
}
