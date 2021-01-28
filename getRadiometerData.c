/* program to read the radiometer data. and log to redis and DSM.
 NAP 27 Nov 2017 - 9 Dec 2017*/
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <hiredis.h>
#include <arpa/inet.h>
#include "dsm.h"

#include "radiometerData.h"
#include "radiometerCommands.h"

#define DSM_HOST "gltobscon"
#define EPOCH_1JAN2001 978307200

#define DEBUG 1

int sockfd=0;
struct sockaddr_in serv_addr;
  char recvBuff[1024];
  char sendBuff[1024];
int openSocket();
 
int main(void)
{
  int n = 0,dsm_status;
  char rain,rainFlag;
  float tau,elevation;
  char radstatus,alarm;
  short rainflagQuality;
  int radstatusTS=0,hkdTS=0,tauTS=0;
  int radstatusTSprev=0,hkdTSprev=0,tauTSprev=0;
  int firstTime=1,radiometerAlarm;
  time_t timeStamp;
  char redisData[1024];
  redisContext *c;
  redisReply *reply;
  struct timeval timeout = {2,500000 }; /* 1.5 seconds  for redis */
  
  radiometerData radiometerResp;
  radiometerCmd radiometerCommand;
  radiometerStatus radiometerRespStatus;
  radiometerHouseKeeping radiometerHKDstatus;


/*The following four bytes are for password (not set, so zeroes */
/* otherwise see section 4.26 of the manual */
  radiometerCommand.pwd[0] = 0x0;
  radiometerCommand.pwd[1] = 0x0;
  radiometerCommand.pwd[2] = 0x0;
  radiometerCommand.pwd[3] = 0x0;

 
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(7000);
  serv_addr.sin_addr.s_addr = inet_addr("88.83.29.84");
/*IP address and port number of radiometer after the firewall setup Aug. 2020*/
printf("-----------------------------------------------------------------------\n");
while(1){
/* time stamp */
  timeStamp = time(NULL);
/* get the Radiometer status RAD_Stat */
  openSocket();
  radiometerCommand.commandID = 0xad;
  memcpy(sendBuff,(char*)&radiometerCommand,sizeof(radiometerCommand));
  n = send(sockfd,sendBuff,sizeof(radiometerCommand),0);
  if (n<0) printf("ERROR writing to Radiometer.");
  n = recv(sockfd, (char *)&radiometerRespStatus, sizeof(radiometerRespStatus),0);
  if( n < 0)  printf("\n Read Error \n"); 

  radstatusTS=radiometerRespStatus.timeRAD+EPOCH_1JAN2001;
  radstatus=radiometerRespStatus.RAD_Stat;
#if DEBUG
  printf ("radiometer status time stamp: %d \n",radstatusTS);
  printf ("Radiometer status: 0x%x\n",radstatus);
#endif
  close(sockfd);

/* get the Housekeeping alarm */
  openSocket();
  radiometerCommand.commandID = 0xac;
  memcpy(sendBuff,(char*)&radiometerCommand,sizeof(radiometerCommand));
  n = send(sockfd,sendBuff,sizeof(radiometerCommand),0);
  if (n<0) printf("ERROR writing to Radiometer.");
  n = recv(sockfd, (char *)&radiometerHKDstatus, sizeof(radiometerHKDstatus),0);
  if( n < 0)  printf("\n Read Error \n"); 

  hkdTS=radiometerHKDstatus.timeHKD+EPOCH_1JAN2001;
  alarm=radiometerHKDstatus.HKD_Alarm;
#if DEBUG
  printf ("housekeeping time stamp: %d \n",hkdTS);
  printf ("Radiometer Housekeeping Alarm: 0x%x\n",alarm);
#endif
 close(sockfd); 

if ((radstatus==1) && (alarm==0)) {
  openSocket();

  radiometerCommand.commandID = 0xab;
  memcpy(sendBuff,(char*)&radiometerCommand,sizeof(radiometerCommand));
  n = send(sockfd,sendBuff,sizeof(radiometerCommand),0);
  if (n<0) printf("ERROR writing to Radiometer.");
  n = recv(sockfd, (char *)&radiometerResp, sizeof(radiometerResp),0);
  if( n < 0)  printf("\n Read Error \n"); 
  tauTS=radiometerResp.timeATN+EPOCH_1JAN2001;
  rainFlag=radiometerResp.ATN_RF & 0x1f;
  rain=rainFlag&0x1;
  rainFlag=((rainFlag&0x6)>>1);
  rainflagQuality=(short)(rainFlag & 0x6);
  tau=radiometerResp.ATN[0]/4.34294;
  elevation=radiometerResp.ATN_El[0];
#if DEBUG
  if (rain) printf("Raining!\n"); else printf("Not raining.\n");
  if (rainflagQuality==0) printf("Rain flag quality not evaluated.\n");
  if (rainflagQuality==1) printf("Rain flag quality high.\n");
  if (rainflagQuality==2) printf("Rain flag quality medium.\n");
  if (rainflagQuality==3) printf("Rain flag quality low.\n");
  printf ("tau time stamp: %d \n",tauTS);
  printf ("ATN_NO: %d (number of channels)\n",radiometerResp.noATN&0xffff);
  printf("ATN: %f dB \n",radiometerResp.ATN[0]);
  printf("\n");
  printf("tau: %f \n",tau);
  printf("\n");
  printf("ATN_Frq: %f GHz\n",radiometerResp.ATN_Frq[0]);
  printf("ATN_Az: %f deg\n",radiometerResp.ATN_Az[0]);
  printf("ATN_El: %f deg\n",elevation);
#endif
  close(sockfd);


/* write data to DSM only if radiometer  is looking  at zenith */
if((elevation>=88.0)&&(elevation<=92.0)) {
printf("\n Writing data to DSM and Redis...\n");
/* write the data to DSM */
        /* initializing DSM */
        dsm_status=dsm_open();
        if(dsm_status != DSM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_open()");
                fprintf(stderr,"Could not open distributed shared  memory.");
                fprintf(stderr,"Check if dsm is running.");
                exit(-1);
        }
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_TAU_F",&tau);
if(dsm_status!=DSM_SUCCESS) {
dsm_error_message(dsm_status,"dsm_write()");
} else {printf("wrote %f to RADIOMETER_TAU variable\n",tau);}
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_ELEVATION_F",&elevation);
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_RAINFLAG_B",&rain);
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_RAINFLAG_QUALITY_S",&rainflagQuality);
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_HKDALARM_B",&alarm);
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_STATUS_B",&radstatus);
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_STATUS_TIMESTAMP_L",&radstatusTS);
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_HKDALARM_TIMESTAMP_L",&hkdTS);
        dsm_status=dsm_write(DSM_HOST,"DSM_RADIOMETER_TAU_TIMESTAMP_L",&tauTS);


        dsm_close();
   /* send data to redis server */
  c = redisConnectWithTimeout("192.168.1.11",6379,timeout);
    if (c == NULL || c->err) {
        if (c) {
            printf("Connection error: %s\n", c->errstr);
            redisFree(c);
        } else {
            printf("Connection error: can't allocate redis context\n");
        }
    }

   /* alarm set if time stamps do not increment */

if(firstTime==0){
    if ((tauTS==tauTSprev)||(hkdTS==hkdTSprev)||(radstatusTS==radstatusTSprev)) {radiometerAlarm=-1;}
    else radiometerAlarm=0;
}


/*
   sprintf(redisData,"ZADD radiometerData %d '{\"timestamp\":%d,\"tau\":%f,\"elevation\":%f,\"rainflag\":%d,\"rainflagQuality\":%d,\"HKDalarm\":%d,\"status\":%d,\"statusTimestamp\":%d,\"hkdTimestamp\":%d,\"tauTimestamp\":%d}'",
   (int)timeStamp,(int)timeStamp,tau,elevation,rain,rainflagQuality,alarm,radstatus,radstatusTS,hkdTS,tauTS);
*/
   sprintf(redisData,"ZADD radiometerData %d \"{'timestamp':%d,'tau':%f,'elevation':%f,'rainflag':%d,'rainflagQuality':%d,'HKDalarm':%d,'status':%d,'statusTimestamp':%d,'hkdTimestamp':%d,'tauTimestamp':%d,'radiometerAlarm':%d}\"",
   (int)timeStamp,(int)timeStamp,tau,elevation,rain,rainflagQuality,alarm,radstatus,radstatusTS,hkdTS,tauTS,radiometerAlarm);

printf("redis data string: %s\n",redisData);

    reply = redisCommand(c,redisData);
    printf("ZADD: %s\n",reply->str);
    freeReplyObject(reply);
    
    redisFree(c);
    
    }/*  elevation check */



   if(firstTime==1) {
   tauTSprev=tauTS;
   hkdTSprev=hkdTS;
   radstatusTSprev=radstatusTS;
   firstTime=0;
  }

} /* if condition for status and housekeeping alarm. Read ATN only if OK */



sleep(600);
} /* while loop */
 
  return 0;
}

int openSocket() {
  if((sockfd = socket(AF_INET, SOCK_STREAM, 0))< 0) {
      printf("\n Error : Could not create socket \n");
      return 1;
    }
  if(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))<0) {
      printf("\n Error : Connect Failed \n");
      return 1;
    }
  memset(recvBuff, '0' ,sizeof(recvBuff));
  memset(sendBuff, '0' ,sizeof(sendBuff));
  return 0;
}
