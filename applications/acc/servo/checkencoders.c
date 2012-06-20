/* Subroutines for checking the hi-res encoders for glitches and reporting
 * glitch events in stderr messages.
 */
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
#include "tsshm.h"
#include "checkencoders.h"
#include "servo.h"

/* Threshold for recording a glitch (mas) */
#define THRESHOLD 15000

/* Set up a circular buffer of encoder readings.  The implementation
 * requires that the length be a power of 2.
 */
#define CBSIZE 16
#define PRESIZE 5
#define PRINTSIZE 16
struct data {
	char name[4];
	int index;
	int buf[CBSIZE];
	int state[CBSIZE];
} azData = {"Az", 0}, elData = {"El", 0};
#define AZ_REL_INDEX(i) ((azData.index + i) & (CBSIZE - 1))
#define EL_REL_INDEX(i) ((elData.index + i) & (CBSIZE - 1))

/* CheckEncoder can be in three states for each axis:
 *   0 - accumulating readings and checking each against the previous three.
 *  <0 - accumulating data, but not checking (startup).  Increment the state
 *	 on each call until it reaches 0.
 *  >0 - accumulating data , but not checking.  Decrement the count until
 *	 it reaches 0 and then print the previous PRINTSIZE values.
 *  NO_ENCODER_CHECK - Do nothing.
 */
static int azChkState = NO_ENCODER_CHECK, elChkState = NO_ENCODER_CHECK;

/* checkencoders.c */
static void PrintBuffer(struct data *dp);

void InitCheckEncoders(int newAzChkState, int newElChkState) {
	if((azChkState = newAzChkState) <= -PRINTSIZE)
	    azData.index = 0;
	if((elChkState = newElChkState) <= -PRINTSIZE)
	    elData.index = 0;
}

void CheckEncoders(void) {
	int t;

	if(azChkState != NO_ENCODER_CHECK) {
	    azData.index = AZ_REL_INDEX(1);
	    azData.buf[azData.index] = encAz;
	    azData.state[azData.index] = azState;
	    if(azChkState == 0) {
		t = azData.buf[AZ_REL_INDEX(-1)] - azData.buf[AZ_REL_INDEX(-2)];
		t += t + t + azData.buf[AZ_REL_INDEX(-3)];
		if(abs(azData.buf[azData.index] - t) > THRESHOLD) {
		    azChkState = (PRINTSIZE - PRESIZE - 1);
		}
	    } else if(azChkState > 0) {
		if(--azChkState == 0) {
		    PrintBuffer(&azData);
		    azChkState = -PRESIZE;
		    azData.index = 0;
		}
	    } else {
		azChkState++;
	    }
	}

	if(elChkState != NO_ENCODER_CHECK) {
	    elData.index = EL_REL_INDEX(1);
	    elData.buf[elData.index] = encEl;
	    elData.state[elData.index] = elState;
	    if(elChkState == 0) {
		t = elData.buf[EL_REL_INDEX(-1)] - elData.buf[EL_REL_INDEX(-2)];
		t += t + t + elData.buf[EL_REL_INDEX(-3)];
		if(abs(elData.buf[elData.index] - t) > THRESHOLD) {
		    elChkState = (PRINTSIZE - PRESIZE + 1);
		}
	    } else if(elChkState > 0) {
		if(--elChkState == 0) {
		    PrintBuffer(&elData);
		    elChkState = -PRESIZE;
		    elData.index = 0;
		}
	    } else {
		elChkState++;
	    }
	}
}

static void PrintBuffer(struct data *dp) {
	int i, ind;
	char c, str[PRINTSIZE+1];

	fprintf(stderr, "%s Encoder Jump ", dp->name);
	for(i = 1 - PRINTSIZE; i <= 0; i++) {
	    ind = (dp->index + i) & (CBSIZE - 1);
	    if(dp->state[ind] < 3) {
		c = 'O';
	    } else if(dp->state[ind] == 3) {
		c = 'T';
	    } else {
		c = 'S';
	    }
	    str[i + PRINTSIZE - 1] = c;
	    fprintf(stderr, "%.4f ", (double)dp->buf[ind] / (double)MAS);
	}
	str[PRINTSIZE] = 0;
	fprintf(stderr, "%s\n", str);
}
