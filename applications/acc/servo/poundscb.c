#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <resource.h>
#include <time.h>
#include "s_cmd2.h"	/* list of commands resident in the EPROM and EEPROM */
#include "s_constants.h"
#include "s_anti96com.h" /* serial line communication protocol */

#define SERVOPRIO 75

short scbElVel, scbAzVel;
char  cmdElAccel, cmdAzAccel, scbStatus;
char serialPort[] = "/dev/IPOP422-0" ;
char warning[TTYLEN];
struct timeval tv;

int main(int argc, char *argv[]) {
  int cnt, starttime;

  printf("Opening the serial line for the scb ... ");
  while(ComSetup(serialPort) < 0)  {
    printf("Setup of serial line to SCB failed\n");
    sleep(1);
  setpriority(PRIO_PROCESS, (0), (SERVOPRIO));
  }
  printf("Successfully opened the SCB\n");
  gettimeofday(&tv, 0);
  starttime = tv.tv_sec;
  for(cnt = 1;; ++cnt) {
      ComStartPkt(AZELVELOCITY);
      ComPutS(scbElVel);
      ComPutC(cmdElAccel);
      ComPutS((short)scbAzVel);
      ComPutC(cmdAzAccel);
      if(SendPktGetRtn(AZELVELOCITY)) {
	printf("AZELVEL packet send failed\n");
      } else {
        scbStatus = ComGetC();
#if 0
        printf("Received status %d cnt = %d\n", scbStatus, cnt);
#endif
      }
      if((cnt % 10000) == 0) {
        gettimeofday(&tv, 0);
	printf("completed %d passes after %ld sec.\n", cnt, tv.tv_sec - starttime);
      }
#if 0
      usleep(1);
#endif
  }
}
