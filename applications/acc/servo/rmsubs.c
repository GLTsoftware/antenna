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
#include "smadaemon.h"
#include "rmsubs.h"
#include "rm.h"
#include "tsshm.h"
#include "servo.h"

int rm_open_status;

/* Since we want to be able to have variables from tsshm as well as externs
 * from servo, the address field will be initialized with either the offset
 * in tsshm or the address of the extern variable.  When the reflective
 * memory is opened, the offset will be fixed up.  The previous value
 * union will be initialized with either OF if an offset is used or AD
 * if the address is supplied.  The TSSHM_OFFSET macro is used to initialize
 * the address with an offset in tsshm.  The ADDRESS macro will do the
 * same for variables in smainit.
 */
#define OFFSET (0x80000000)
#define TSSHM_OFFSET(X) {OFFSET}, (void *)((void *)&tsshm->X - \
	(void *)&tsshm->azCmd)
#define ADDRESS(v) {0}, &v

/* The first char is the RM type char and the second is the type in tsshm
 * or the servo variable.  'M' signifies a conversion from mas to degrees */
enum conv_type {FDM, FF, FIM, LI, BI, SI};
struct rmval {
	enum conv_type ct;	/* conversion type */
	union {
	    int l;
	    float f;
	    short s;
	    unsigned char b;
	} pv;			/* the previous value written to rm */
	void *a;		/* Offset of value in tsshm */
	char *vname;		/* rm variable name */
} rv[] = {
	INIT_VALUES,
	MONITOR_AND_STATE,
	TIME_POSITION 
};
#define RVSIZE (sizeof(rv) / sizeof(rv[0]))

void OpenRM(void) {
	int antlist[RM_ARRAY_SIZE];
	int i;

	rm_open_status=rm_open(antlist);
	if(rm_open_status != RM_SUCCESS) {
	    rm_error_message(rm_open_status,"rm_open()");
	}
	for(i = 0; i < RVSIZE; i++) {
	    if(rv[i].pv.l == OFFSET) {
		rv[i].a = (void *)&tsshm->azCmd + (int)rv[i].a;
		rv[i].pv.l = 0;
	    }
	}
	if(RVSIZE != LAST_T_P + 1) {
	    fprintf(stderr, "Size Error in table of rm values to write\n");
	}
}

/* Update the RM values on servo's list.  If first is zero, write all of
 * the variables, otherwise only if the ones which have changed.
 */
void UpdateRM(int first, int last) {
	static double iMAS = (1.0 / MAS);
	int rm_rtn;
	int i, l;
	float f;
	unsigned char b;
	short s;
	char errmsg[48];

	if(rm_open_status != RM_SUCCESS) {
	    return;
	}
	for(i = first; i <= last; i++) {
	    if(rv[i].vname == (char *)0) {
		continue;
	    }
	    switch(rv[i].ct) {
	    case FDM:
		f = *(double *)rv[i].a;
		if(f != rv[i].pv.f || first == 0) {
		    rv[i].pv.f = f * iMAS;
		    rm_rtn = rm_write(RM_ANT_0, rv[i].vname, &rv[i].pv.f);
		    rv[i].pv.f = f;
		} else {
		    continue;
		}
	        break;
	    case FIM:
		l = *(int *)rv[i].a;
		if(l != rv[i].pv.l || first == 0) {
		    f = l * iMAS;
		    rm_rtn = rm_write(RM_ANT_0, rv[i].vname, &f);
		    rv[i].pv.l = l;
		} else {
		    continue;
		}
	        break;
	    case FF:
		f = *(float *)rv[i].a;
		if(f != rv[i].pv.f || first == 0) {
		    rm_rtn = rm_write(RM_ANT_0, rv[i].vname, &f);
		    rv[i].pv.f = f;
		} else {
		    continue;
		}
	        break;
	    case LI:
		l = *(int *)rv[i].a;
		if(l != rv[i].pv.l || first == 0) {
		    rm_rtn = rm_write(RM_ANT_0, rv[i].vname, &l);
		    rv[i].pv.l = l;
		} else {
		    continue;
		}
	        break;
	    case BI:
		b = *(int *)rv[i].a;
		if(b != rv[i].pv.b || first == 0) {
		    rm_rtn = rm_write(RM_ANT_0, rv[i].vname, &b);
		    rv[i].pv.b = b;
		} else {
		    continue;
		}
	        break;
	    case SI:
		s = *(int *)rv[i].a;
		if(s != rv[i].pv.s || first == 0) {
		    rm_rtn = rm_write(RM_ANT_0, rv[i].vname, &s);
		    rv[i].pv.s = s;
		} else {
		    continue;
		}
	        break;
	    default:
		ErrPrintf("Bad rm variable type %d\n", rv[i].ct);
		exit(QUIT_RTN);
	    }
	    if(rm_rtn != RM_SUCCESS) {
		sprintf(errmsg, "rm_write: %s", rv[i].vname);
		rm_error_message(rm_rtn, errmsg);
		if(rm_rtn == RM_NAME_INVALID) {
		    rv[i].vname = (char *)0;
		} else {
		   rm_close();
		   rm_open_status = RM_NO_SERVICE;
		   fprintf(stderr, "RM shut down\n");
		   return;
		}
	    }
	}
}

/* WARNING: This routine will set the first char of name to NULL if it
 * gets a RM_NAME_INVALID return.
 */
void RMSafeWrite(char *name, void *value) {
	int rm_rtn;
	char errmsg[64];

	if(rm_open_status != RM_SUCCESS)
	    return;
	if(name[0] == 0) {
	    return;
	}
	rm_rtn = rm_write(RM_ANT_0, name, value);
	if(rm_rtn != RM_SUCCESS) {
	    sprintf(errmsg, "rm_write: %s", name);
	    rm_error_message(rm_rtn, errmsg);
	    if(rm_rtn == RM_NAME_INVALID) {
		name[0] = 0;
	    } else {
		rm_close();
		rm_open_status = RM_NO_SERVICE;
		return;
	    }
	}
}

void RMTimestamp(void) {
	int rm_rtn;
	int unixtime;
	static int oldAzState = -1, oldElState = -1;

	if(rm_open_status != RM_SUCCESS)
	    return;
	rm_rtn=rm_read(RM_ANT_0,"RM_UNIX_TIME_L", &unixtime);
	if(rm_rtn != RM_SUCCESS) {
	    rm_error_message(rm_rtn,"rm_read() unix time");
	}
	rm_rtn=rm_write(RM_ANT_0,"RM_SERVO_TIMESTAMP_L", &unixtime);
	if(rm_rtn != RM_SUCCESS) {
	    rm_error_message(rm_rtn,"rm_write() Unix time");
	    rm_rtn = RM_SUCCESS;
	}
	if(oldAzState != azState) {
	    rm_rtn=rm_write(RM_ANT_0,"RM_AZ_DRV_STATE_TIME_L", &unixtime);
	    oldAzState = azState;
	}
	if(oldElState != elState) {
	    rm_rtn |= rm_write(RM_ANT_0,"RM_EL_DRV_STATE_TIME_L", &unixtime);
	    oldElState = elState;
	}
	if(rm_rtn != RM_SUCCESS) {
	    rm_error_message(rm_rtn,"rm_write() drive state time");
	}
}
