/* program to read the radiometer data. NAP 27 Nov 2017 */
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

#include "radiometerData.h"
#include "radiometerCommands.h"

int sockfd=0;
struct sockaddr_in serv_addr;
  char recvBuff[1024];
  char sendBuff[1024];
int openSocket();
 
int main(void)
{
  int i,n = 0;
  char rainFlag;
  radiometerData radiometerResp;
  radiometerCmd radiometerCommand;
  radiometerStatus radiometerRespStatus;
  radiometerHouseKeeping radiometerHKDstatus;

  radiometerCommand.commandID = 0xab;
/*The following four bytes are for password (not set, so zeroes */
/* otherwise see section 4.26 of the manual */
  radiometerCommand.pwd[0] = 0x0;
  radiometerCommand.pwd[1] = 0x0;
  radiometerCommand.pwd[2] = 0x0;
  radiometerCommand.pwd[3] = 0x0;


 
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(7000);
  serv_addr.sin_addr.s_addr = inet_addr("88.83.29.82");
/*IP address and port number of radiometer from Pierre's email of 25 Nov 2017*/
 
  openSocket();

  memcpy(sendBuff,(char*)&radiometerCommand,sizeof(radiometerCommand));
  n = send(sockfd,sendBuff,sizeof(radiometerCommand),0);
  if (n<0) printf("ERROR writing to Radiometer.");
  n = recv(sockfd, (char *)&radiometerResp, sizeof(radiometerResp),0);
  if( n < 0)  printf("\n Read Error \n"); 
  printf ("time stamp: %d (seconds since 00:00:00 1/1/2001)\n",radiometerResp.timeATN);
  rainFlag=radiometerResp.ATN_RF & 0x1f;
  if (rainFlag &0x1) printf("Raining!\n"); else printf("Not raining.\n");
  rainFlag=((rainFlag&0x6)>>1);
  if ((rainFlag & 0x6) ==0) printf("Rain flag quality not evaluated.\n");
  if ((rainFlag & 0x6) ==1) printf("Rain flag quality high.\n");
  if ((rainFlag & 0x6) ==2) printf("Rain flag quality medium.\n");
  if ((rainFlag & 0x6) ==3) printf("Rain flag quality low.\n");
  printf ("ATN_NO: %d (number of channels)\n",radiometerResp.noATN&0xffff);
  printf("ATN: %f dB \n",radiometerResp.ATN[0]);
  printf("tau: %f \n",radiometerResp.ATN[0]/4.34294);
  printf("ATN_Frq: %f GHz\n",radiometerResp.ATN_Frq[0]);
  printf("ATN_Az: %f deg\n",radiometerResp.ATN_Az[0]);
  printf("ATN_El: %f deg\n",radiometerResp.ATN_El[0]);

  radiometerCommand.commandID = 0xad;
/*
  radiometerCommand.pwd[0] = 0x0;
  radiometerCommand.pwd[1] = 0x0;
  radiometerCommand.pwd[2] = 0x0;
  radiometerCommand.pwd[3] = 0x0;
*/

  close(sockfd);
  openSocket();
  memcpy(sendBuff,(char*)&radiometerCommand,sizeof(radiometerCommand));
  n = send(sockfd,sendBuff,sizeof(radiometerCommand),0);
  if (n<0) printf("ERROR writing to Radiometer.");
  n = recv(sockfd, (char *)&radiometerRespStatus, sizeof(radiometerRespStatus),0);
  if( n < 0)  printf("\n Read Error \n"); 

  printf ("time stamp: %d (seconds since 00:00:00 1/1/2001)\n",radiometerRespStatus.timeRAD);
  printf ("Radiometer status: 0x%x\n",radiometerRespStatus.RAD_Stat);
  close(sockfd);

  openSocket();
  radiometerCommand.commandID = 0xac;
  memcpy(sendBuff,(char*)&radiometerCommand,sizeof(radiometerCommand));
  n = send(sockfd,sendBuff,sizeof(radiometerCommand),0);
  if (n<0) printf("ERROR writing to Radiometer.");
  n = recv(sockfd, (char *)&radiometerHKDstatus, sizeof(radiometerHKDstatus),0);
  if( n < 0)  printf("\n Read Error \n"); 

  printf ("time stamp: %d (seconds since 00:00:00 1/1/2001)\n",radiometerHKDstatus.timeHKD);
  printf ("Radiometer Housekeeping Alarm: 0x%x\n",radiometerHKDstatus.HKD_Alarm);
 close(sockfd); 
 
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
