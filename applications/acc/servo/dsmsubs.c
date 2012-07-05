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
#include "dsmsubs.h"
#include "dsm.h"
#include "tsshm.h"
#include "servo.h"
#include "canbus.h"

int dsm_open_status;
char dsm_host[] = "gltobscon";

#if 0
    DSM_ACU_MODE_V3_B           # Modified, the bytes are Az mode, El mode, 
                                # and Access mode. 0x00040022 # 2 5
    DSM_AZ_POSN_DEG_F           # Most recent pos in deg. 0x00040012 # 8 0.048
    DSM_EL_POSN_DEG_F           # Most recent pos in deg. 0x00040002 # 8 0.048
#endif

struct monitor_point {
  char *dsm_name;
  int id;
  int len;
} mptab[] = {
    {"DSM_ACU_ERROR_B",			0x0004002F, 1},
    {"DSM_AZ_BRAKE_B",			0x00040014, 1},
    {"DSM_AZ_ENC_L",			0x00040017, 4},
    {"DSM_AZ_ENC_STATUS_B",		0x00040018, 1},
    {"DSM_AZ_MOTOR_CURRENTS_V2_B",	0x00040019, 4},
    {"DSM_AZ_MOTOR_TEMPS_V2_B",		0x0004001A, 4},
    {"DSM_AZ_MOTOR_TORQUE_V2_B",	0x00040015, 2},
    {"DSM_AZ_STATUS_V8_B",		0x0004001B, 8},
    {"DSM_EL_BRAKE_B",			0x00040004, 1},
    {"DSM_EL_ENC_V2_L",			0x00040007, 8},
    {"DSM_EL_ENC_STATUS_V3_B",		0x00040008, 3},
    {"DSM_EL_MOTOR_CURRENTS_V4_B",	0x00040009, 4},
    {"DSM_EL_MOTOR_TEMPS_V4_B",		0x0004000A, 4},
    {"DSM_EL_MOTOR_TORQUE_V4_B",	0x00040005, 4},
    {"DSM_EL_STATUS_V8_B",		0x0004000B, 8},
    {"DSM_SHUTTER_B",			0x0004002E, 1},
    {"DSM_STOW_PIN_V2_B",		0x00040024, 2},
    {"DSM_SYSTEM_STATUS_V3_B",		0x00040023, 3}
};
#define NPOINTS (sizeof(mptab) / sizeof(struct monitor_point))

void SafeOpenDsm(void) {
  static int warned = 0;

  if(dsm_open_status != DSM_SUCCESS) {
    dsm_open_status = dsm_open();
  }
  if(dsm_open_status != DSM_SUCCESS) {
    if(warned > 0) {
      warned--;
    } else {
      dsm_error_message(dsm_open_status, "Dsm open failed");
      warned = 60;
    }
  }
}

void WriteDsmMonitorPoints(void) {
  int i,dsm_status;
  char buf[8];
 
  extern float azTrErrorArcSec,elTrErrorArcSec;

  if(dsm_open_status != DSM_SUCCESS) {
    return;
  }
  for(i= 0; i< NPOINTS; i++) {
    if(ReadCANValue(mptab[i].id, buf, mptab[i].len)) {
      dsm_write(dsm_host, mptab[i].dsm_name, buf);
    }
  }
  dsm_status=dsm_write("gltobscon", "DSM_AZ_TRACKING_ERROR_F", &azTrErrorArcSec);
  dsm_status=dsm_write("gltobscon", "DSM_EL_TRACKING_ERROR_F", &elTrErrorArcSec);  
}
