#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "tpmc816.h"
#include "tsshm.h"
#include "servo.h"
#include "canbus.h"

#define USE_MAIN 0

#define TRUE 1
#define FALSE 0
#define CAN_DEVICE "/dev/tp816a1"
#define READ_BUFFER_NUMBER 15
#define MSG_OBJ_NUM 15
#define RX_QUEUE 1
#define EXTENDED 1
#define TIMEOUT_JIFFIES 200

TP816_MSG_BUF  MsgBuf = {0, TIMEOUT_JIFFIES, RX_QUEUE, EXTENDED, 0, 8, ""};
TP816_BUF_DESC  BufDesc = {0, MSG_OBJ_NUM, RX_QUEUE, EXTENDED, 8, ""};
TP816_TIMING BitTimingParam = {TP816_1MBIT, 0};
TP816_ACCEPT_MASKS AcceptMasksParam = {0,0,0};

unsigned char azMode, elMode, accessMode;
double azPos, prevAzPos, azEncoder, elPos, prevElPos, elEncoder1, elEncoder2;
unsigned char acuError, azAuxMode, azBrake, azEncStatus;

#if 0
struct MONPOINTS {
  int canID;		/* Can Bus ID of value(s) */
  void (*fp)(int );	/* pointer to subroutine to  get and convert value */
  void * rp1;		/* pointer to 1st result */
  void * rp2;		/* pointer to 2nd result if needed */
  char * description;
} monPoints = {
  {0x40022, &Get2Bytes,	&axisModes, &accessMode,	"Acu Mode"},
  {0x40012, &Get2Ints,	&azPos,		&prevAzPos,	"Az Positiona"{;
}

typedef struct {
  int l;
  int p;
} POSITION_RETURN;
POSITION_RETURN *prp = (POSITION_RETURN *)MsgBuf.Data;
#endif

double posFactor = 180. / 0x40000000;
int canfd;

void SetupCanBus(void) {

  if((canfd = open(CAN_DEVICE, O_RDWR)) < 0) {
    fprintf(stderr, "Opening %s failed", CAN_DEVICE);
    exit(1);
  }
  if(ioctl(canfd, TP816_BITTIMING, (char*)&BitTimingParam) < 0) {
    perror("Setting bit timing failed");
    exit(2);
  }
  if(ioctl(canfd, TP816_SETFILTER, (char*)&AcceptMasksParam) < 0) {
    perror("Setting filter masks failed");
    exit(3);
  }
  ioctl(canfd, TP816_RELEASEBUF, (char*)&BufDesc);
  if(ioctl(canfd, TP816_DEFRXBUF, (char*)&BufDesc) < 0) {
    perror("Assigning a read buffer failed");
    exit(4);
  }
  if(ioctl(canfd, TP816_BUSON, NULL) < 0) {
    perror("Turning on the bus failed");
    exit(5);
  }
  /* remove any queued messages */
  MsgBuf.Timeout = 0;
  for(;;) {
    if(read(canfd, &MsgBuf, sizeof(MsgBuf)) <= 0) break;
  }
  MsgBuf.Timeout = TIMEOUT_JIFFIES;
}

int ReadCANValue(int msgID, void *value, int len) {
  char msg[24];

  MsgBuf.Identifier = msgID;
  MsgBuf.Data[0] = 0;
  MsgBuf.MsgLen = 1;
  if(write(canfd, &MsgBuf, sizeof(MsgBuf)) < 0) {
    sprintf(msg, "Write of ID 0x%x failed", msgID);
    perror(msg);
    return(0);
  }
  memset(MsgBuf.Data, 0, 8);
  if(read(canfd, &MsgBuf, sizeof(MsgBuf)) > 0 && MsgBuf.Identifier == msgID) {
    if(len > 8) len = 8;
    if(len > MsgBuf.MsgLen) len = MsgBuf.MsgLen;
    memcpy(value, MsgBuf.Data, len);
#if 0
    int i;
    printf("\nReturn msg ID %lx, MsgLen %d bytes - ",
	 MsgBuf.Identifier, MsgBuf.MsgLen);
    for(i = 0; i < len; i++) {
      printf("%2x ", MsgBuf.Data[i]);
    }
    printf("\n");
#endif
    return(1);
  } else {
    sprintf(msg, "read of ID 0x%x failed", msgID);
    perror(msg);
    return(0);
  }
}

int SetCANValue(int msgID, void *data, int len) {
  char msg[24];

  MsgBuf.Identifier = msgID;
  if(len > 8) len = 8;
  memcpy(MsgBuf.Data, data, len);
  MsgBuf.MsgLen = len;
  if(write(canfd, &MsgBuf, sizeof(MsgBuf)) < 0) {
    sprintf(msg, "Write of ID 0x%x failed", msgID);
    perror(msg);
    return(0);
  }
  return(1);
}

#if USE_MAIN
void getUByte(int ID, unsigned char *vp, char *name) {
  if(ReadCANValue(ID, vp, 1)) {
    printf("%s = %x ", name, *vp); 
  }
}

double posFactor = 180. / 0x40000000;
int main(int argc, char *argv[]) {
  unsigned char ca[8];
  int ia[2];

  SetupCanBus();
  if(ReadCANValue(0x40022,  ca, 2)) {
    azMode = 0xf & ca[0];
    elMode = ca[0] >> 4;
    accessMode = ca[1];
    printf("Az Mode %1x  El Mode %1x Access %s\n", azMode, elMode,
	(accessMode ==1)? "Local": ((accessMode == 2)? "Remote": "Unknown"));
  }
  if(ReadCANValue(0x40012, ia, 8)) {
    azPos = posFactor * ia[0];
    prevAzPos = posFactor * ia[2];
    printf("Az positions are %.5f %.5f\n", azPos, prevAzPos);
  }
  if(ReadCANValue(0x40017, ia, 4)) {
    azEncoder = *ia[0];
    printf("Az Encoder %.0f\n", azEncoder);
  }
  if(ReadCANValue(0x40002, ia, 8)) {
    elPos = posFactor * ia[0];
    prevElPos = posFactor * ia[2];
    printf("El`positions are %.5f %.5f\n", elPos, prevElPos);
  }
  if(ReadCANValue(0x40007, ia, 8)) {
    elEncoder1 = ia[0];
    elEncoder2 = ia[1];
    printf("El Encoders %.0f %.0f\n", elEncoder1, elEncoder2);
  }
  getUByte(0x4002f, &acuError, "ACU Error");
  getUByte(0x40016, &azAuxMode, "Az Aux Mode");
  getUByte(0x40014, &azBrake, "Az Brake");
  getUByte(0x40018, &azEncStatus, "Az EncStatus");
  printf("\n");
  return(0);
}
#endif /* USE_MAIN */
