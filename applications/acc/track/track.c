/****************************************************************

track.c
for GLT

Nimesh Patel

version 1.0
8 June 2012
	
This program runs as a daemon on gltacc.
It calculates az, el and rates and communicates these 
to servo via shared memory.
Higher level commands issued from gltobscon are handled via DSM.

All RM(rm) replaced by DSM(dsm).

*****************************************************************/
#include <stdio.h>
#include <sys/utsname.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <smem.h>
#include <math.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <termio.h>
#include <time.h>
#include <resource.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <pthread.h>
#include <rpc/rpc.h>
#include <smapopt.h>
#include "novas.h"
#include "track.h"
#include "vme_sg_simple.h"
#include "syncclock32.h"
#include "dsm.h"
#include "acu.h"
#include "ccd.h"
#include "tsshm.h"
#include "optics.h"
#include "smadaemon.h"
#include "commonLib.h"
#include "stderrUtilities.h"

#define DEBUG 0
#define COORDINATES_SVC 1
#define T1950   2433281.0
#define T2000   2451544.0

#define MSEC_PER_DEG 3600000.
#define MSEC_PER_HR 3600000.      
#define AUPERDAY_KMPERSEC 1731.45717592 

#define	HIGH_EL_TO_AVOID_SUN_MAS 288000000. 
#define	LOW_EL_TO_AVOID_SUN_MAS 54000000.

#define IDLE_DRIVES_TIMEOUT 300 

#define DSM_HOST "gltobscon"

/************************************************************************/
struct source
{
    char            sourcename[20], veltype[20], comment[100], decsign;
    int             rah, ram, decd, decm;
    float           ras, decs, epoch, vel, pmr, pmd;
};

/************************ Function declarations *************************/
void handlerForSIGINT(int signum);
void *CommandHandler();
void SendMessageToDSM(char *messg);
void SendLastCommandToDSM(char *lastCommand);       
void            local(double *lst, double *ra, double *dec, double *az, double *el, double *tjd, double *azoff, double *eloff, float *pressure, float *temperature, float *humidity, int *radio_flag, float *refraction, float *pmdaz, float *pmdel,  short *target_flag, double *commanded_az,double *commanded_el);

void            split(unsigned long * lw, unsigned short * sw1, unsigned short * sw2);

void            print_upper(char *name);
void            pad(char *s, int length);
void            is_planet(char *s, int *flag, int *id);
void            starcat(char *s, int *star_flag, struct source * observe_source);
void sidtim(double *tjdh,double *tjdl,int *k,double *gst);
double sunDistance(double az1,double el1,double az2,double el2);
double tjd2et(double tjd);


/** Global variables *********************************************** */


struct sigaction action, old_action;
int sigactionInt;

pthread_t	CommandHandlerTID, CCDClientTID ;

	struct sched_param param;
	pthread_attr_t *attr,*attr2,*attr3,*attr4,*attr5;
	int policy = SCHED_FIFO;

unsigned long   window;

	int antlist[RM_ARRAY_SIZE],dsm_status;

int	first_time_spk=1; /* this variable is used for opening the 
		ephemeris file only once, on  first pass */

int             user;  /* input command */ 

char            messg[100];
char            lastCommand[100];
short command_flag=0;
char            sptype[10]="----------";
float           magnitude=0.;
int             read_mount_model_flag = 1;
double		azmodelrms,elmodelrms,azdc,azcol,eltilt,aztilt_sin,aztilt_cos,aztilt_sin2,aztilt_cos2,
		azenc_sin,azenc_cos,azenc_sin2,azenc_cos2,
		azenc_sin3,azenc_cos3,
                eldc,elsag,eaztilt_sin,eaztilt_cos,eaztilt_sin2,eaztilt_cos2;
double		razmodelrms,relmodelrms,razdc,razcol,reltilt,raztilt_sin,raztilt_cos,raztilt_sin2,
		raztilt_cos2, reldc,relsag,reaztilt_sin,reaztilt_cos,
		razenc_sin,razenc_cos,razenc_sin2,razenc_cos2,
		razenc_sin3,razenc_cos3, reaztilt_sin2,reaztilt_cos2;
double		pmaztiltAzoff,pmaztiltEloff;
short 		interrupt_command_flag=0;
char pointingParameter[20];
int numParameters=0;
int defaultTiltFlag,rdefaultTiltFlag;
char opticalValue[20],radioValue[20];

double 	radian;

int 	antennaNumber=1;

TrackServoSHM *tsshm;    /* Local pointer to shared memory for
                                    communicating az,el and rates to
                                    the dmytrack thread */

double subx_counts=0.,suby_counts=0.,focus_counts=0.,subtilt_counts=0.;
char 		command[159];

int	cal_flag=0;

    int             radio_flag=1;

	double sourceVelocity=0.;

	int receivedSignal=0;

	double polar_dut;

	double setFeedOffsetA1=0.,setFeedOffsetA2=0.;


	/* default position read from antennaPosition.h */
	double longitude_hours = LONGITUDE_HOURS;
	double latitude_degrees = LATITUDE_DEGREES;
	double longitude_degrees = LONGITUDE_DEGREES;
	double sinlat = SINLAT;
	double coslat = COSLAT;
	double longrad = LONGRAD;
	double height_m = HEIGHT_M;

	char operatorErrorMessage[MAX_OPMSG_SIZE+1];

/*end of global variables***************************************************/

void
main(int argc, char *argv[])
{

    FILE            *fp_mount_model,*fp_tilt,*fp_tiltdc,*fp_polar;

    double          ra, dec, lst, lst_prev, lst_radian, lst_radian_prev;

    double          ra_cat, dec_cat, pra, pdec, rvel;
    double 	    cmdpmra=0.0,cmdpmdec=0.0;

    double          azoff = 0.0;

    double          eloff = 0.0;

    float           pressure, humidity, temperature;
    float           windspeed, winddirection;

    float           epoch=2000.;

    double          ra0, dec0,radialVelocity0,radialVelocity;

    double          radot, decdot,radialVelocitydot;
	double		radot_prev=0.0;
	double		decdot_prev=0.0;	
	int 	radec_offset_flag=0;

    double          tjd_prev, et_prev, ra_prev, dec_prev,radialVelocity_prev;


    double          tjd, tjd0, tjdn;

    double          az, el, azrate=0., elrate=0., az1, el1 ;

    double          pi, secunit, milliarcsec;

    double          hr;

    int             i;

    int             azint, elint, tjdint, azrateint, elrateint;

    int             icount = 0, app_pos_flag = 0;

    short           ret;

    double          hour_angle, hangle1, hangle2;

    short int       hangleint;

    int             nplanet;

    /* The following variables are defined for time calculations */
 int              hours, minutes;	
	double seconds,et,delta=0.,utcsec;

    /* Variables for LST calculation */
    double          d1, dtupi, gst;

    /* variables for actual az */
    double          az_actual, el_actual;
    double          az_actual_disp, el_actual_disp;

    /* end of time variables definitions */

    /* variables used for display part */
    double          lst_disp, ra_disp, dec_disp, ra_cat_disp, dec_cat_disp;

    double          az_disp, el_disp, utc_disp;
    float          az_disp_rm, el_disp_rm;

    double             tjd_disp;

    int             initflag = 0;

    /* source counter */
    int             source = 0;

    int             source_skip_flag = 1;

        int     padid=0;
	char line[BUFSIZ];

    /* the following variables are from name.c */
    char            sname[34];
    char            sname2[34];

    int             sol_sys_flag, id, star_flag;

    struct source   observe_source;

    /* scan flags */
    int             azscan_flag = 0, elscan_flag = 0;
    int             position_switching_flag = 0;
    int             on_source = 0, off_source = 0;
    int             on_source_timer = 0, off_source_timer = 0;

    double          scan_unit = 1.;



    double          Az_cmd, El_cmd, Az_cmd_rate, El_cmd_rate;

    int             az_cmd, el_cmd;

    double          az_actual_msec, el_actual_msec;

    unsigned long          az_actual_msec_int, el_actual_msec_int;

    double          posn_error, az_error, el_error;

    /* for weather parameters */

    short           azoff_int, eloff_int;

    /* integration time for ncam */
    int             integration = DEFAULT_INTEGRATION_TIME;
    short           int integration_short;

    time_t          time1,timeStamp;
    struct tm      *tval;


    float           pmdaz, pmdel, refraction;
	double		drefraction;

    short            source_on_off_flag = INVALID;
    char            previous_source_on_off_flag = INVALID;
    char            spectrometer, send_spectrometer_interrupt;

    /* tracking error smoothing */
    double          tracking_error, tracking_error_accum = 0.0, tracking_error_buffer[SMOOTH_LENGTH],
                   *dp, smoothed_tracking_error = 0.0;
    int             off_position_counter = 0, off_position_sign;

    short           target_flag = 0;
    double          commanded_az=0., commanded_el=15.;
    float          smarts_az, smarts_el;


	short scan_unit_int;

	short errorflag=OK;
	short waitflag=0;
	unsigned short slength;

	double az_actual_corrected;

double et_prev_big_time_step=0.,et_time_interval;

	
/* for sun avoidance: */
	
	double sunaz,sunel;
	int suneloff=0;
	short sun_avoid_flag=0;

/* for IRIG device driver*/
	/*
	int device_fd, irig_status;
	short irig_status_s;
	POS_TIME pt_struct;
	SMA_PT sma_struct;
	DAY_TYPE julianDay;
	*/
	
	int device_fd,irig_status;
	struct vme_sg_simple_time ts;
	struct sc32_time sctime;
	int sc32fd;


/* for earthtilt and sidereal_time */
	double equinoxes,tjd_upper,tjd_lower,dpsi,deps,tobl,mobl;
	
 	float sunazf,sunelf;
	float az_tracking_error,el_tracking_error;
	short dummyshortint;
	float dummyFloat;
	double dummyDouble;
	char dummyByte;
	double hourangle;
	body Planet;
	body earth = {0, 399, "earth"};

	cat_entry star = {"cat","star",0,0.,0.,0.,0.,0.,0.};
	double distance=0.;
	/* planet radii from explanatory supplement physical ephemeredis*/
	double planetradius[11]={0.0,2439.7,6501.9,3200.0,3397.0,71492.0,
			60268.0, 25559.0,24764.,1151.0,1738.0};

	double planetradius301=1738.; /* Moon radius in km*/
	double planetradius501=1820.; /* io radius in km*/
	double planetradius502=1565.; /* europa radius in km*/
	double planetradius503=2634.; /* ganymede radius in km*/
	double planetradius504=2403.; /* callisto radius in km*/
	double planetradius601=200.; /* mimas radius in km*/
	double planetradius602=250.; /* enceladus radius in km*/
	double planetradius603=530.; /* tethys radius in km*/
	double planetradius604=560.; /* dione radius in km*/
	double planetradius605=764.; /* rhea radius in km*/
	double planetradius606=2575.; /* titan radius in km*/
	double planetradius607=145.; /* hyperion radius in km*/
	double planetradius608=718.; /* iapetus radius in km*/

	double planetradius801=1352.6; /* triton radius in km*/
	double planetradius802=170.; /* nereid radius in km */
	double planetradius375=474.; /* ceres radius in km */
	double planetradius376=266.; /* pallas radius in km */
	double planetradius377=203.6; /* hygiea radius in km */
	double planetradius378=265.; /*vesta radius in km */
	/* the above two comet radii are arbitrary- just to avoid junk
	 values for angular diameter*/

	double planetdistance=0.;
	double planetdiameter=0.;
	int beep_flag=0;

	float az_actual_corrected_rm,el_actual_disp_rm;

	struct utsname unamebuf;

 	/* for getting the system time to find the year */
	/* defined in vme_sg.h */
	/*
	YEAR_TYPE year;	
	*/

	/* for ccd image header */
	char snamefits[100];
	short int bias,gain;
	
	float lst_disp_float, utc_disp_float;

	int scbComm=0;
        int az_enc,el_enc;

        int milliseconds;
	int servomilliseconds;
	int checkmsec;
	double dmilliseconds;

        int servoOnFlag=0;

        int az_offset_flag=0;
        int el_offset_flag=0;

        double prev_azrate = 0.;
        double prev_elrate = 0.;
                                      
	char antdir[10];
	
	int azelCommandFlag=0;

	double pos1950[3],pos2000[3];

	 /* timestamp for ders */
        int timestamp;

	int subcorflag=0; /* whether to correct subref Y,Z vs el */

	double museconds;
	
	int polar_mjd;
	double polar_dx,polar_dy;

	
	short padid_disp=0;

	double raOffset=0.0;
	double decOffset=0.0;
	double cosdec=1.0;

	char modeldate[10];
	char rmodeldate[10];


	char newCmdSourceName[34];
	int newSourceFlag=0;


        site_info location = {latitude_degrees,longitude_degrees,height_m,0.0,0.0};       

	int corruptedMountModelFile=0;
	int end_of_file=0;

	char junkstring[256];
	
	short disableDrivesFlag=0;

	double sundistance=0.;
	
	int logTiltsTS;
	int logTiltsRunning=0;
	int logTiltsErrorMsgSent=0;

	short rmTiltFlagBits=0;
	
    /* END OF VARIABLE DECLARATIONS */

    /********Initializations**************************/


	strcpy(sname,"test");
	strcpy(sname2,"test");

/* for stderr buffering- see smainit docs */

DAEMONSET

	 /* First of all, find out if some other instance of
         Track is running */
        if(processPresent("excl_Track"))
        {
        fprintf(stderr,"Track is already running - goodbye.\n");
        exit(1);
        }

/*
	This was hanging up the cpu on giving the resume command.
	Perhaps, all threads should not be at 50.
*/
	setpriority(PRIO_PROCESS,0,50);

    pi = 4.0 * atan(1.0);
    dtupi = pi * 2.0;
    radian = pi / 180.;
    secunit = 1.15740741e-5;	/* day in one second */
    milliarcsec = 180. / pi * 3600000.;


	 /* signal handler for control C */
        action.sa_flags=0;
        sigemptyset(&action.sa_mask);
        action.sa_handler = handlerForSIGINT;
        sigactionInt = sigaction(SIGINT,&action, &old_action);
        sigactionInt = sigaction(SIGTERM,&action, &old_action);




	/* get the antenna number by identifying the host computer*/
	uname(&unamebuf);
	if(!strcmp(unamebuf.nodename,"acc1")) antennaNumber=1;
	if(!strcmp(unamebuf.nodename,"acc2")) antennaNumber=2;
	if(!strcmp(unamebuf.nodename,"acc3")) antennaNumber=3;
	if(!strcmp(unamebuf.nodename,"acc4")) antennaNumber=4;
	if(!strcmp(unamebuf.nodename,"acc5")) antennaNumber=5;
	if(!strcmp(unamebuf.nodename,"acc6")) antennaNumber=6;
	if(!strcmp(unamebuf.nodename,"acc7")) antennaNumber=7;
	if(!strcmp(unamebuf.nodename,"acc8")) antennaNumber=8;


	/* initializing the IRIG board */
	  /* get the year from the system time and set it on the
                IRIG device driver */

	sc32fd = open("/dev/syncclock32",O_RDWR,0);
	if(sc32fd<=0) {
	device_fd = open("/dev/vme_sg_simple", O_RDWR,0);     
	   if(device_fd==-1) { 
	   	perror("open()");
		fprintf(stderr,"Could not open vme_sg_simple device");
		exit(SYSERR_RTN);
	   }
        }



	/*
	irig_status = ioctl(device_fd, SET_YEAR, &year);
	*/

        /* initializing ref. mem. */
        dsm_status=dsm_open(antlist);
        if(dsm_status != DSM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_open()");
		fprintf(stderr,"Could not open reflective memory.");
                exit(QUIT_RTN);
        }

	/* start listening to ref. mem. interrupts for higher-level commands*/
	dsm_status=rm_monitor(DSM_HOST,"DSM_COMMAND_FLAG_S");
                if(dsm_status != DSM_SUCCESS) {
                dsm_error_message(dsm_status,"rm_monitor()");
                exit(1);
                }
 
	pthread_attr_init(attr);
	if (pthread_create(&CommandHandlerTID, attr, CommandHandler,
			 (void *) 0) == -1) { 
	perror("main: pthread_create CommandHandler");
	exit(-1);
	}
	param.sched_priority=18;
	pthread_attr_setschedparam(&attr,&param);
	pthread_setschedparam(CommandHandlerTID,policy,&param);


    /* tracking smoothing error */

    for (i = 0; i < SMOOTH_LENGTH; i++)
	tracking_error_buffer[i] = 0.0;
    dp = tracking_error_buffer;

    /*
     * setting these proper-motion and radial vel terms to zero for now
     */
    pra = 0.0;
    pdec = 0.0;
    rvel = 0.0;


	radio_flag=1;


    /*
     * This is to get the user input as a single unbuffered char and
     * zero-wait
     */

	 tsshm = OpenShm(TSSHMNAME, TSSHMSZ);    


	/* Now check if servo is running */
	 checkmsec=tsshm->msec;
	usleep(100000);
	if((tsshm->msec)==checkmsec) {
	fprintf(stderr,"Warning: Servo is not running.\n");
	}


    /********************end of initializations*******************/

	/* on first pass, make cmd posn = actual posn. */

	if(icount==0)
	{

	az_enc = tsshm->encAz;
        el_enc = tsshm->encEl;     
	az_actual=(double)az_enc/MSEC_PER_DEG;
        el_actual=(double)el_enc/MSEC_PER_DEG;       
	

	/* call local to get mount model corrections for the first
		commanded position */
	ra = 0.;
        dec = 0.;
        ra_disp = 0.;
        dec_disp = 0.;
        ra_cat_disp = 0.;
        dec_cat_disp = 0.;
        ra0 = 0.;
        dec0 = 0.;
        radot = 0.;
        decdot = 0.;
        hangle1 = 0.;
        hangle2 = 0.;
	lst_radian_prev=0;
	tjd_prev=0.;
	commanded_az=az_actual;
	commanded_el=el_actual;


	target_flag=1;
        local(&lst_radian_prev, &ra, &dec, &az, &el, &tjd_prev, &azoff,
		&eloff, &pressure, &temperature, &humidity, &radio_flag, 
		&refraction, &pmdaz, &pmdel,
		&target_flag,&commanded_az,&commanded_el);
        commanded_el = el_actual-pmdel/3600.;
	commanded_az = az_actual-pmdaz/3600./cos(commanded_el*radian);
	strcpy(sname,"target");
	} /* if icount==0 */

new_source:                    

	  /* read polar wobble parameters 
	     from the file /global/polar/polar.dat */

        fp_polar = fopen("/global/polar/polar.dat","r");
        if(fp_polar==NULL) {
        fprintf(stderr,"Failed to open /global/polar/polar.dat file\n");
        exit(QUIT_RTN);
        }
	fscanf(fp_polar,"%d %lf %lf %lf",
		&polar_mjd,&polar_dx,&polar_dy,&polar_dut);
	/* write them to DSM */
        dsm_status=dsm_write(DSM_HOST,"DSM_POLAR_MJD_L",&polar_mjd);
        dsm_status=dsm_write(DSM_HOST,"DSM_POLAR_DX_ARCSEC_D",&polar_dx);
        dsm_status=dsm_write(DSM_HOST,"DSM_POLAR_DY_ARCSEC_D",&polar_dy);
        dsm_status=dsm_write(DSM_HOST,"DSM_POLAR_DUT_SEC_D",&polar_dut);
	fclose(fp_polar);


    print_upper(sname);

    if (target_flag==1) strcpy(sname,"target");

    if (target_flag == 0)
    {

	 strcpy(messg, "                              ");
                SendMessageToDSM(messg);

	strcpy(sname2,sname);
	pad(sname, 20);

	if(newSourceFlag==0) is_planet(sname, &sol_sys_flag, &id);

	if (sol_sys_flag == 1)
	{
	  
	    nplanet = id;
	
		/* in the new ephemeris codes, moon is 10, not 301 */
		/*
		if(nplanet==301) nplanet=10;
		commented out on 11 jan 2001, now we are back to
		ansi-C codes from jpl and not using hoffman's package
		to read the jpl ephemeris files*/
	    ra_cat_disp = 0.;
	    dec_cat_disp = 0.;
	   Planet.type=0;
	   Planet.number=nplanet;
	  strcpy(Planet.name,sname);
	  strcpy(sptype,"----------");
	  sptype[9]=0x0;
	  magnitude=0.0;
	}
	if (sol_sys_flag == 0)
	{
            if(newSourceFlag==0) starcat(sname, &star_flag, &observe_source);
            if(newSourceFlag==1) star_flag=1;

	    if (star_flag == 0)
	    {
		nplanet = 0;

	/* unknown source. go into target mode with current

	position as commanded position */

	       strcpy(sname,"unknown");
		strcat(messg,"Source not found.");
                SendMessageToDSM(messg);
		fprintf(stderr,"%s\n",messg);
	  strcpy(operatorErrorMessage, "Source not found.");
	  sendOpMessage(OPMSG_WARNING, 10, 30, operatorErrorMessage);

        az_enc = tsshm->encAz;
        el_enc = tsshm->encEl;
        az_actual=(double)az_enc/MSEC_PER_DEG;
        el_actual=(double)el_enc/MSEC_PER_DEG;
       /* call local to get mount model corrections for the first
                commanded position */
        ra = 0.;
        dec = 0.;
        ra_disp = 0.;
        dec_disp = 0.;
        ra_cat_disp = 0.;
        dec_cat_disp = 0.;
        ra0 = 0.;
        dec0 = 0.;
        radot = 0.;
        decdot = 0.;
        hangle1 = 0.;
        hangle2 = 0.;
        lst_radian_prev=0;
        tjd_prev=0.;
        commanded_az=az_actual;
        commanded_el=el_actual;
        target_flag=1;
        local(&lst_radian_prev, &ra, &dec, &az, &el, &tjd_prev, &azoff,
                &eloff, &pressure, &temperature, &humidity, &radio_flag,
                &refraction, &pmdaz, &pmdel,
                &target_flag,&commanded_az,&commanded_el);
        commanded_el = el_actual-pmdel/3600.;
        commanded_az = az_actual-pmdaz/3600./cos(commanded_el*radian);


	    }

	    if (star_flag == 1)
	    {
		nplanet = 0;

		if(newSourceFlag==0) {

		dec_cat = fabs(observe_source.decd) + observe_source.decm / 60. + observe_source.decs / 3600.;
	dec_cat = dec_cat + decOffset/3600.;
		cosdec=cos(dec_cat*radian);
	
		ra_cat = observe_source.rah + observe_source.ram / 60. + observe_source.ras / 3600.;
		ra_cat = ra_cat + (raOffset/3600./15.0/cosdec);

		if (observe_source.decsign == '-') dec_cat = -dec_cat;

/* if the coordinates are B1950, precess them to J2000 first*/
		epoch = observe_source.epoch;
		} /* if newSourceFlag==0; source from catalog */

		if(newSourceFlag==1) {
		dsm_status=dsm_read(DSM_HOST,"DSM_CMD_SOURCE_C34",newCmdSourceName,&timeStamp);
		strcpy(sname,newCmdSourceName);
		dsm_status=dsm_read(DSM_HOST,"DSM_CMD_RA_HOURS_D",&ra_cat,&timeStamp);
		dsm_status=dsm_read(DSM_HOST,"DSM_CMD_DEC_DEG_D",&dec_cat,&timeStamp);
		dsm_status=dsm_read(DSM_HOST,"DSM_CMD_PMRA_MASPYEAR_D",&cmdpmra,&timeStamp);
		dsm_status=dsm_read(DSM_HOST,"DSM_CMD_PMDEC_MASPYEAR_D",&cmdpmdec,&timeStamp);
		dsm_status=dsm_read(DSM_HOST,"DSM_CMD_EPOCH_YEAR_D",&dummyDouble,&timeStamp);
		epoch=(float)dummyDouble;
		dsm_status=dsm_read(DSM_HOST,"DSM_CMD_SVEL_KMPS_D",&sourceVelocity,&timeStamp);
		strcpy(sname,newCmdSourceName);

		dec_cat =  dec_cat + decOffset/3600.;
		cosdec=cos(dec_cat*radian);
	
		ra_cat = ra_cat + (raOffset/3600./15.0/cosdec);

		}/* if source is not from catalog but given via observe command. */

		if (epoch == 1950.)
		{
			radec2vector(ra_cat,dec_cat,1.0,pos1950);
                        precession(T1950,pos1950,T2000,pos2000);
                        vector2radec(pos2000,&ra_cat,&dec_cat);

		}
		star.ra=ra_cat;
		star.dec=dec_cat;

		/* converting input pm-ra in mas/yr to sec/century */
/* possible bug- found 5 jul 2006
		star.promora= cmdpmra/15./cos(dec_cat*radian)/10.;
		star.promodec=cmdpmdec/10.;
		if(newSourceFlag==1) {
		star.promora=
		 (double)observe_source.pmr/15./cos(dec_cat*radian)/10.;
		star.promodec=(double)observe_source.pmd/10.;
		}
*/

		if(newSourceFlag==1) {
		star.promora= cmdpmra/15./cos(dec_cat*radian)/10.;
		star.promodec=cmdpmdec/10.;
		} else {
		star.promora=
		 (double)observe_source.pmr/15./cos(dec_cat*radian)/10.;
		star.promodec=(double)observe_source.pmd/10.;
		}

		ra_cat_disp = ra_cat;
		dec_cat_disp = dec_cat;
	    }			/* star_flag if */
	}			/* sol_sys_flag if */
    }				/* if target flag is 0 */
/* write sol_sys_flag to DSM */
        dummyshortint=(short)sol_sys_flag;
        dsm_status=dsm_write(DSM_HOST,"DSM_SOLSYS_FLAG_S",&dummyshortint);
        if(dsm_status != RM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_write() solsysflag");
                }







/************************************************************************/
    /* starting infinite loop */
/************************************************************************/


    /* getting previously written offsets from ref. mem. */

	dsm_status=dsm_read(DSM_HOST,"DSM_AZOFF_ARCSEC_D",&azoff,&timeStamp);	
	dsm_status=dsm_read(DSM_HOST,"DSM_ELOFF_ARCSEC_D",&eloff,&timeStamp);	
	dsm_status=dsm_read(DSM_HOST,"DSM_RAOFF_ARCSEC_D",&raOffset,&timeStamp);
	dsm_status=dsm_read(DSM_HOST,"DSM_DECOFF_ARCSEC_D",&decOffset,&timeStamp);


    /*
     * if offsets are huge, due to a bad initialization in ref. mem. then do
     * not use them if(fabs(azoff)>1800.) azoff=0.; if(fabs(eloff)>1800.)
     * azoff=0.;
     */

    /* begin while loop every 1 second */
    while (1)
    {

beginning:

	yield();


    /* read mount model parameters */
	if(read_mount_model_flag==1) {
	fp_mount_model=fopen("/instance/configFiles/pointingModel","r"); 
	if(fp_mount_model==NULL) {
	fprintf(stderr,"Failed to open the mount model file.\n");
	exit(QUIT_RTN);
	}
	end_of_file=0;
	numParameters=0;
	corruptedMountModelFile=0;

        while(fgets(line,sizeof(line),fp_mount_model) != NULL) {
        line[strlen(line)-1]='\0';
                if(line[0]!='#') {
        sscanf(line,"%s %s %s", pointingParameter, opticalValue, radioValue);
                     if(!strcmp(pointingParameter,"AzDC")) {
                     azdc=atof(opticalValue);
                     razdc=atof(radioValue);
                        numParameters++;
                     }

                     if(!strcmp(pointingParameter,"AzColl")) {
                     azcol=atof(opticalValue);
                     razcol=atof(radioValue);
                        numParameters++;
                     }

                     if(!strcmp(pointingParameter,"ElTilt")) {
                     eltilt=atof(opticalValue);
                     reltilt=atof(radioValue);
                        numParameters++;
                     }

                     if(!strcmp(pointingParameter,"AAzTltSin")) {
                     aztilt_sin=atof(opticalValue);
                     raztilt_sin=atof(radioValue);
                        numParameters++;
                     }

                     if(!strcmp(pointingParameter,"AAzTltCos")) {
                     aztilt_cos=atof(opticalValue);
                     raztilt_cos=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AAzTltSin2")) {
                     aztilt_sin2=atof(opticalValue);
                     raztilt_sin2=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AAzTltCos2")) {
                     aztilt_cos2=atof(opticalValue);
                     raztilt_cos2=atof(radioValue);
                        numParameters++;
                     }

                     if(!strcmp(pointingParameter,"AzEncSin")) {
                     azenc_sin=atof(opticalValue);
                     razenc_sin=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AzEncCos")) {
                     azenc_cos=atof(opticalValue);
                     razenc_cos=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AzEncSin2")) {
                     azenc_sin2=atof(opticalValue);
                     razenc_sin2=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AzEncCos2")) {
                     azenc_cos2=atof(opticalValue);
                     razenc_cos2=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AzEncSin3")) {
                     azenc_sin3=atof(opticalValue);
                     razenc_sin3=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AzEncCos3")) {
                     azenc_cos3=atof(opticalValue);
                     razenc_cos3=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"AzRms")) {
                     azmodelrms=atof(opticalValue);
                     razmodelrms=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"ElDC")) {
                     eldc=atof(opticalValue);
                     reldc=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"ElSag")) {
                     elsag=atof(opticalValue);
                     relsag=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"EAzTltSin")) {
                     eaztilt_sin=atof(opticalValue);
                     reaztilt_sin=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"EAzTltCos")) {
                     eaztilt_cos=atof(opticalValue);
                     reaztilt_cos=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"EAzTltSin2")) {
                     eaztilt_sin2=atof(opticalValue);
                     reaztilt_sin2=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"EAzTltCos2")) {
                     eaztilt_cos2=atof(opticalValue);
                     reaztilt_cos2=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"ElRms")) {
                     elmodelrms=atof(opticalValue);
                     relmodelrms=atof(radioValue);
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"TiltFlag")) {
                     defaultTiltFlag=(int)atof(opticalValue);
                     rdefaultTiltFlag=(int)atof(radioValue);
		     setTiltflag=defaultTiltFlag;
                        numParameters++;
                     }
                     if(!strcmp(pointingParameter,"Date")) {
                     strcpy(modeldate,opticalValue);
                     strcpy(rmodeldate,radioValue);
                        numParameters++;
                     }

                }

        }

	if(numParameters!=23) corruptedMountModelFile=1;

	fclose(fp_mount_model);
	if(corruptedMountModelFile==1) {
	strcpy(messg, "Mount model file is corrupted.");
        SendMessageToDSM(messg);
        fprintf(stderr,"%s\n",messg);
        strcpy(operatorErrorMessage, "Mount model file is corrupted.");
          sendOpMessage(OPMSG_WARNING, 10, 30, operatorErrorMessage);
	exit(QUIT_RTN);
	}

	read_mount_model_flag=0;
	}

/*
printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s",
razdc,razcol,reltilt,raztilt_sin,raztilt_cos,raztilt_sin2,raztilt_cos2,
razenc_sin,razenc_cos,razenc_sin2,razenc_cos2,razenc_sin3,razenc_cos3,
razmodelrms,reldc,relsag,reaztilt_sin,reaztilt_cos,reaztilt_sin2,reaztilt_cos2,relmodelrms,rmodeldate);
*/


	/* get sun's position */

	dsm_status=dsm_read(DSM_HOST,"DSM_SUN_AZ_DEG_F",&sunazf,&timeStamp);
	sunaz=(double)sunazf;
	sunaz=sunaz*radian;

	dsm_status=dsm_read(DSM_HOST,"DSM_SUN_EL_DEG_F",&sunelf,&timeStamp);
	sunel=(double)sunelf;
	sunel=sunel*radian;


	if (source_skip_flag == 1) {
	    icount = 0;
	    source_skip_flag = 0;
	    source++;
	}

	icount++;

	if ((icount % BIG_TIME_STEP) == 0) {
	    app_pos_flag = 1;
	} else {
	    app_pos_flag = 0;
	}


	/*-------------------------------time-------------------------*/

		

/*
	hr=(double)ts.hour+((double)ts.min)/60.+ 
			(((double)ts.sec +(double)ts.usec)/1.0e6)/3600.;
*/

	   if(sc32fd>0) {

		 if((irig_status = read(sc32fd, &sctime, sizeof(sctime))) < 0) {
            	fprintf(stderr, "Error %d reading Syncclock32\n", irig_status);
            	}


	    hours = sctime.hour;
	    minutes = sctime.min;
	    seconds = (double) sctime.sec;
	    museconds = (double) sctime.usec;

	   } else {


	      irig_status = read(device_fd,&ts,1);
	        if(irig_status==-1) {
		perror("read()");
		exit(-1);
	       }
 
	    hours = ts.hour;
	    minutes = ts.min;
	    seconds = (double) ts.sec;
	    museconds = (double) ts.usec;
	    }

	
	    seconds += (museconds/1.0e6);

	hr = (double)hours + ((double)minutes)/60. + seconds/3600.;
	    utc_disp = hr;
	dmilliseconds=hr * 3600000.;
	milliseconds = (int) dmilliseconds;
/* calculating julian day */
/*
	   tjd_upper=(double)julianDay-0.5;
*/


	if(sc32fd>0) {
  tjd_upper = (double)((int)(365.25*(sctime.year+4711))) +
                        (double)sctime.yday + 351.5;
	} else {
	   tjd_upper = (double)((int)(365.25*(ts.year+4711))) + 
			(double)ts.yday + 351.5;
	}

	   tjd_lower=hr/24.;

 	   tjd_lower += polar_dut/86400.;

	   tjd=tjd_upper+tjd_lower;


/*
printf("tjd=%lf %d %d %d %lf \n",tjd, ts.year,hours, minutes, seconds);
*/

   utcsec = (tjd-2451545.)* 86400.0;
	
/*
	  fprintf(stderr,"The GPS (UTC) time is: %d:%d:%d:%lf\n",
		julianDay,hours,minutes,seconds);
*/


/*
	    delta=32.184+LEAPSECONDS+polar_dut;
*/
	    delta=32.184+LEAPSECONDS;

	    et = utcsec+delta ;

	 /* obtain equation of equinoxes which is needed for LST */
	cel_pole(polar_dx,polar_dy);
	earthtilt(tjd_upper,&mobl, &tobl, &equinoxes,&dpsi, &deps);
	sidereal_time(tjd_upper, tjd_lower,equinoxes, &gst);
	d1 = gst *dtupi / 24.+longrad;
	lst_radian = fmod(d1, dtupi);
	if (lst_radian < 0.) {
	    lst_radian += dtupi;
	}
	/* converting lst to hours */
	lst = lst_radian * 24. / dtupi;
/*
	lst += polar_dut/3600.;
*/
	lst_disp = lst;
	tjd_disp = tjd;

/*
printf("lst=%lf\n",lst_disp);
printf("tjd_disp=%d\n",tjd_disp);
*/


	/*--------------------end of time part------------------------*/

	/*--------------------beginning of apparent calculations------*/

	if (target_flag == 0) {

	    if ((app_pos_flag == 1) || (icount == 1))
	    {

		if (nplanet != 0)
		{
		   topo_planet(tjd, &Planet,&earth, delta, 
			&location, &ra0, &dec0, &distance,&radialVelocity0);

	/*increment ra0 and dec0 with offsets */
	dec0 = dec0 + decOffset/3600.;
	cosdec=cos(dec0*radian);
	ra0=ra0+(raOffset/3600./15.0/cosdec);
	
		radialVelocity0 *= AUPERDAY_KMPERSEC;
		}
		if (nplanet == 0)
		{
		    topo_star(tjd,&earth,delta, &star,&location, &ra0, &dec0);

		}
		if (icount == 1)
		{
		    tjd_prev = tjd - secunit * 30.;
		    et_prev = et - 30.;
		    lst_prev = lst - 8.3333333333e-3; 
		    lst_radian_prev = lst_prev * dtupi / 24.;
		    if (lst_radian_prev < 0.)
		    {
			lst_radian_prev += dtupi;
		    }
		    if (nplanet != 0)
		    {
		   topo_planet(tjd_prev, &Planet,&earth, delta, 
		&location, &ra_prev, &dec_prev, &distance,&radialVelocity_prev);
		radialVelocity_prev *= AUPERDAY_KMPERSEC;


	if(Planet.number <=11)  {
	planetdiameter=2.0*planetradius[Planet.number]/distance/149597900.0;
	} else {
	switch (Planet.number) {
	case 301:
	planetdiameter=2.0*planetradius301/distance/149597900.0;
	break;
	case 501:
	planetdiameter=2.0*planetradius501/distance/149597900.0;
	break;
	case 502:
	planetdiameter=2.0*planetradius502/distance/149597900.0;
	break;
	case 503:
	planetdiameter=2.0*planetradius503/distance/149597900.0;
	break;
	case 504:
	planetdiameter=2.0*planetradius504/distance/149597900.0;
	break;
	case 601:
	planetdiameter=2.0*planetradius601/distance/149597900.0;
	break;
	case 602:
	planetdiameter=2.0*planetradius602/distance/149597900.0;
	break;
	case 603:
	planetdiameter=2.0*planetradius603/distance/149597900.0;
	break;
	case 604:
	planetdiameter=2.0*planetradius604/distance/149597900.0;
	break;
	case 605:
	planetdiameter=2.0*planetradius605/distance/149597900.0;
	break;
	case 606:
	planetdiameter=2.0*planetradius606/distance/149597900.0;
	break;
	case 607:
	planetdiameter=2.0*planetradius607/distance/149597900.0;
	break;
	case 608:
	planetdiameter=2.0*planetradius608/distance/149597900.0;
	break;
	case 801:
	planetdiameter=2.0*planetradius801/distance/149597900.0;
	break;
	case 802:
	planetdiameter=2.0*planetradius802/distance/149597900.0;
	break;
	case 375:
	planetdiameter=2.0*planetradius375/distance/149597900.0;
	break;
	case 376:
	planetdiameter=2.0*planetradius376/distance/149597900.0;
	break;
	case 377:
	planetdiameter=2.0*planetradius377/distance/149597900.0;
	break;
	case 378:
	planetdiameter=2.0*planetradius378/distance/149597900.0;
	break;
	}

	}
	planetdiameter=(planetdiameter/radian)*3600.;

		    } else
		    {
		    topo_star(tjd_prev,&earth,delta, &star,&location, &ra_prev, &dec_prev);
		    }
		}

		et_time_interval = et - et_prev_big_time_step;

		if(icount==1) et_time_interval=30.;

		if(radec_offset_flag==0) {
		radot = (ra0 - ra_prev) / 30.;
		decdot = (dec0 - dec_prev) /30. ;
		} else {
		radot = radot_prev;
		decdot = decdot_prev;
		radec_offset_flag=0;
		}
		radialVelocitydot=(radialVelocity0-radialVelocity_prev)/30.;

		    ra_prev = ra0;
		    dec_prev = dec0;
		    radot_prev=radot;
		    decdot_prev=decdot;
		    radialVelocity_prev=radialVelocity0;
		    et_prev_big_time_step=et;

		
	    }			/* ((app_pos_flag == 1) || (icount == 1)) */
	}			/* if target_flag=0 */

	/* read the weather parameters from ref. mem. */
	dsm_status=dsm_read(DSM_HOST,"DSM_WEATHER_TEMP_F",&temperature,&timeStamp);
	dsm_status=dsm_read(DSM_HOST,"DSM_WEATHER_HUMIDITY_F",&humidity,&timeStamp);
	dsm_status=dsm_read(DSM_HOST,"DSM_WEATHER_MBAR_F",&pressure,&timeStamp);
	dsm_status=dsm_read(DSM_HOST,"DSM_WEATHER_WINDSPEED_F",&windspeed,&timeStamp);
	windspeed *= 0.44704; /* convert from mph to m/s */
	dsm_status=dsm_read(DSM_HOST,"DSM_WEATHER_WINDDIR_F",&winddirection,&timeStamp);


	/* check for limits on weather parameters */
	if (
	    (pressure < 600.) || (pressure > 1200.)
	    || (temperature < -30.) || (temperature > 70.)
	    || (humidity < 0.) || (humidity > 110.)
	    ) {
		strcpy(messg, "                                   ");
		SendMessageToDSM(messg);
		strcpy(messg, "Check the weather station.");
		SendMessageToDSM(messg);

	}
	/*
	 * ra and dec in hours and degrees, apparent coordinates output from
	 * stars or planets functions are converted below into azimuth and
	 * elevation. First, linearly interpolating between the ra and dec
	 * apparent positions, calculated every BIG_TIME_STEPs
	 */
	if (target_flag == 0) {
	    if(icount!=1)
            {		
	    ra = ra0 + radot * (et-et_prev_big_time_step);
	    dec = dec0 + decdot * (et-et_prev_big_time_step);
	    radialVelocity=radialVelocity0+
			radialVelocitydot*(et-et_prev_big_time_step);
            }		

	    if(icount==1)
	    {
	    ra=ra0;
	    dec=dec0;
	    radialVelocity=radialVelocity0;
	    }		

	    ra_disp = ra;
	    dec_disp = dec;

	    /* if it is a solar system object, replace
                ra_cat, dec_cat by the apparent coordinates
                instead of making them zeroes
                */

                if(sol_sys_flag==1) {
                ra_cat_disp=ra_disp;
                dec_cat_disp=dec_disp;
                }



	    ra = ra * 15.0 * pi / 180.;
	    dec = dec * pi / 180.;


	    if (icount == 1)
	    {
		tjdn = tjd - secunit;
		local(&lst_radian_prev, &ra, &dec, &az, &el, &tjd_prev, &azoff, &eloff, &pressure, &temperature, &humidity, &radio_flag, &refraction, &pmdaz, &pmdel,&target_flag,&commanded_az,
	&commanded_el);
		hangle1 = lst_radian_prev - ra;

		az1 = az;
		el1 = el;
		tjd0 = tjd;
	    }
	}			/* if target flag = 0 */

    if (target_flag == 1) {
	ra = 0.;
	dec = 0.;
	ra_disp = 0.;
	dec_disp = 0.;
	ra_cat_disp = 0.;
	dec_cat_disp = 0.;
	ra0 = 0.;
	dec0 = 0.;
	radot = 0.;
	decdot = 0.;
	strcpy(sptype, "----------");
	sptype[9]=0x0;
	magnitude=0.0;
	hangle1 = 0.;
	hangle2 = 0.;
	sol_sys_flag=0;
	local(&lst_radian_prev, &ra, &dec, &az, &el, &tjd_prev, &azoff, &eloff, &pressure, &temperature, &humidity, &radio_flag, &refraction, &pmdaz, &pmdel,&target_flag,&commanded_az,&commanded_el);
    }

	/* adding offsets for scanning/mapping */
	if (azscan_flag == 1)
	    azoff = azoff + scan_unit * TIME_STEP ;
	if (elscan_flag == 1)
	    eloff = eloff + scan_unit * TIME_STEP ;

	/* position switching */

	if (position_switching_flag == 1) {

	    if (source_on_off_flag != INVALID)
	    {
		if (on_source)
		    on_source_timer--;
		if (off_source)
		    off_source_timer--;
	    }
	    if (on_source_timer == 0)
	    {
		off_position_counter++;
		on_source = 0;
		off_source = 1;
		on_source_timer = integration * 10;
		if ((off_position_counter % 2) == 0)
		    off_position_sign = -1;
		else
		    off_position_sign = 1;
		azoff = azoff + (off_position_sign * scan_unit) ;
	        az_offset_flag=1;
		strcpy(messg, "                                       ");
		SendMessageToDSM(messg);
		strcpy(messg, "position switching - OFF source");
		SendMessageToDSM(messg);
		source_on_off_flag = OFFSOURCE;
		send_spectrometer_interrupt = 1;
	    }
	    if (off_source_timer == 0)
	    {
		on_source = 1;
		off_source = 0;
		off_source_timer = integration * 10;
		azoff = azoff - (off_position_sign * scan_unit);
		az_offset_flag=1;
		strcpy(messg, "                                       ");
		SendMessageToDSM(messg);
		strcpy(messg, "position switching - ON source");
		SendMessageToDSM(messg);
		source_on_off_flag = ONSOURCE;
		send_spectrometer_interrupt = 1;
	    }
	}


	if (target_flag == 0) {
	    local(&lst_radian, &ra, &dec, &az, &el, &tjd, &azoff, &eloff, &pressure, &temperature, &humidity, &radio_flag, &refraction, &pmdaz, &pmdel,&target_flag,&commanded_az,&commanded_el);
	    hangle2 = lst_radian - ra;
	}
	/* these variables are for the display */



	el_disp = el-pmdel/3600.*radian;
	az_disp = az-pmdaz/3600.*radian/cos(el_disp);


/* Remove the mount model from the commanded positions for display,
since these should be the true positions (also remove similarly,
for the actual positions*/

	/* Compare az_disp and el_disp with Sun's az and el
	and go into simulation mode with an error message
	if the commanded position is within SUNLIMIT degrees
	of the Sun's position */

	sundistance=sunDistance(az_disp,el_disp,sunaz,sunel);
	dsm_write(DSM_HOST,"DSM_SUN_DISTANCE_DEG_D",&sundistance);

	if(sunDistance(az_disp,el_disp,sunaz,sunel)<=SUNLIMIT) {

		strcpy(lastCommand,"Standby - Sun Limit (cmd)");
		SendLastCommandToRM(lastCommand);
            	strcpy(messg, " Standing by. Sun Limit (cmd)");
		SendMessageToDSM(messg);
	 } /* sun limit check  with commanded position */
	

	az_disp_rm = (float)az_disp / radian;
	el_disp_rm = (float)el_disp / radian;
	dsm_status=dsm_write(DSM_HOST,"DSM_COMMANDED_AZ_DEG_F",&az_disp_rm);
	dsm_status=dsm_write(DSM_HOST,"DSM_COMMANDED_EL_DEG_F",&el_disp_rm);

	hour_angle = hangle2 * 12.0 / pi;

	hour_angle = hour_angle * 100.;
	hangleint = (int) hour_angle;

        /* if an offset has been commanded, then add it also
        to the previous values of az and el so that the
        computed rate does not have a jump in it */

        if(az_offset_flag==0) {
		azrate = az - az1;
/* added these following two lines on 23 sep 2002 */
		if(azrate < -6.0) azrate += 2.0* pi;
		if(azrate > 6.0) azrate -= 2.0* pi;
		}

        if(az_offset_flag==1) {
        azrate = prev_azrate;
        az_offset_flag=0;
        }

        if(el_offset_flag==0) elrate = el - el1;
        if(el_offset_flag==1) {
        elrate = prev_elrate;
        el_offset_flag=0;
        }                                         

	prev_azrate = azrate;
        prev_elrate = elrate;
                                 
	/* to detect the transit for a northern source */


/*debug
printf("azrate=%f hangle1=%f hangle2=%f az1=%f\n", azrate,hangle1,hangle2,az1);
*/

	azint = az * milliarcsec;
	elint = el * milliarcsec;
	azrateint = azrate * milliarcsec;
	elrateint = elrate * milliarcsec;
	tjdint = (tjd - tjd0) * 24.0 * 3600.0 * 1000.0;
	az1 = az;
	el1 = el;
	hangle1 = hangle2;

/*debug
printf("%d %d %.12f %.12f %.12f %.12f %d %d\n",icount,milliseconds,(lst_radian*24*1800/pi),tjd,az*180./pi,el*180./pi,azrateint,elrateint);
*/

	/*
	 * converting cmd az,el and rates to counts, and adjusting these for
	 * the azimuth lap ambiguity
	 */


	Az_cmd = (double) azint;
	az_cmd = (int) Az_cmd;	

	El_cmd = (double) elint;
	el_cmd = (int) El_cmd;

	Az_cmd_rate = (double) azrateint;

	El_cmd_rate = (double) elrateint;

	/* check if servo is running */
	if((tsshm->msec)==checkmsec) {
	strcpy(messg,"servo is not running.");
	strcpy(lastCommand,"Servo is not running.");
	SendLastCommandToDSM(lastCommand);
	}

	az_enc = tsshm->encAz;
        el_enc = tsshm->encEl;   
	az_actual=(double)az_enc/MSEC_PER_DEG;
        el_actual=(double)el_enc/MSEC_PER_DEG;


                                                  
	/* read subreflector x,y,z positions */

	el_actual_disp = el_actual-pmdel/3600.;
	az_actual_disp = az_actual-pmdaz/3600./cos(el_actual_disp*radian);

	az_actual_msec = az_actual_disp * 3600000.;
	el_actual_msec = el_actual_disp * 3600000.;


	az_actual_msec_int = (unsigned long) az_actual_msec;
	el_actual_msec_int = (unsigned long) el_actual_msec;


	/* now adjust az_cmd, if encoder has rolled over */
	/*
	 * The commanded azimuth in counts should not be a smaller number
	 * than the AZ_NEG_HARDSTOP_CTS which is the negative hardstop
	 */


/* Cable-wrap logic for command azimuth */
/*  -269.902 is the CCW prelimit from
the "settings" file  for the new servo, corresponding
to the az enc value for antenna-1 at Haystack */

/* changed the hardcoded limit numbers below for the
new style antennas, to values reported by servo through
shared memory */


if ((Az_cmd>=180.*MSEC_PER_DEG)&&
	(Az_cmd<=(tsshm->cwLimit))&&(az_actual < 90.)) 
			{Az_cmd -= (360.*MSEC_PER_DEG);
			az_disp-=(2*pi);
			}
if(Az_cmd < (tsshm->ccwLimit)) {Az_cmd += 360.*MSEC_PER_DEG;
			if(az_disp<0.) az_disp+=2*pi;}


/* getting az and el tracking errors from shared memory */
/* in mas */

        az_error = (tsshm->azTrError)/MSEC_PER_DEG;
        el_error = (tsshm->elTrError)/MSEC_PER_DEG;
                                                         

/* correct the azimuth tracking error for cosine elevation */
	az_error = az_error * cos(el);

	posn_error = pow(((double) az_error * (double) az_error + (double) el_error * (double) el_error), 0.5);

	/* pass tracking errors through reflective memory */
	
        az_tracking_error=(float)az_error *3600.;
        el_tracking_error=(float)el_error *3600.;


/*********** compute the running average of tracking errors (c. katz)*****/


	tracking_error = sqrt((az_error * 3600.) * (az_error * 3600.)
			      + (el_error * 3600.) * (el_error * 3600.));
	tracking_error_accum -= *dp;
	tracking_error_accum += tracking_error;
	*dp = tracking_error;
	dp++;
	if ((dp - tracking_error_buffer) > (SMOOTH_LENGTH - 1))
	    dp = tracking_error_buffer;
	smoothed_tracking_error = tracking_error_accum / SMOOTH_LENGTH;
   /* write smoothed tracking error to DSM */ 


	if ((smoothed_tracking_error<18000.)&&(smoothed_tracking_error > 10.))
	{
	if(beep_flag==1) printf("");
	}
	
	if(smoothed_tracking_error>=10) waitflag=1;
	else waitflag=0;

#if 0

/* uncomment this when we are ready for single-dish autocorrelation 
spectroscopy in position switching mode */

/***************************spectrometer part****************************/
	/* Pass flag to Charlie's spectrometer program for tracking_OK */

	if (smoothed_tracking_error > 10.)
	    source_on_off_flag = INVALID;

	else if (previous_source_on_off_flag == INVALID)

	{

	    if (on_source == 1)
	    {
		source_on_off_flag = ONSOURCE;
		on_source_timer = integration * 10;
	    }
	    if (off_source == 1)
	    {
		source_on_off_flag = OFFSOURCE;
		off_source_timer = integration * 10;
	    }
	}

	/*
	 * if Charlie's program is alive, send an interrupt to inform about
	 * the above flag changes
	 */
	/* first check if the spectrometer program is alive */
	dsm_status=dsm_read(DSM_HOST,"DSM_SPECTROMETER_ANTENNA_S", 
			&spectrometer,&timeStamp);
	/* if it is, then send interrupt */
	if (previous_source_on_off_flag != source_on_off_flag)
	{
	dsm_status=dsm_write_notify(DSM_HOST,"DSM_BLANKING_SOURCE_S",
				&source_on_off_flag);
	    send_spectrometer_interrupt = 0;
	}
	previous_source_on_off_flag = source_on_off_flag;
#endif

/******************************end of spectrometer part********************/

/************************* motion control ****************************/
/*
 * If scbComm = 1, then command motion through SCB else just
 * display the calculated and monitored values
 */

        if (scbComm == 1)
        {
                /* load the position and rate and turn on the drives */

                if(servoOnFlag==0)
                {
                tsshm->elCmd = ON_CMD;                    
                tsshm->azCmd = ON_CMD;
 		servoOnFlag=1;
                }

	if((azelCommandFlag==1)&&(azscan_flag==0))
	{
	Az_cmd_rate=0.;
	}
	if((azelCommandFlag==1)&&(elscan_flag==0))
	{
	El_cmd_rate=0.;
	}

	servomilliseconds=tsshm->msec;

	/* check for bad values of position*/
	if(Az_cmd < (tsshm->ccwLimit)) Az_cmd+=(360.*MSEC_PER_DEG);
	if(Az_cmd > (tsshm->cwLimit)) Az_cmd-=(360.*MSEC_PER_DEG);


                tsshm->az = Az_cmd;
                tsshm->azVel = Az_cmd_rate;

/* added on 18 apr 2003 */
if(sun_avoid_flag==1) {
	if(suneloff>0.) El_cmd=HIGH_EL_TO_AVOID_SUN_MAS;
	if(suneloff<0.) El_cmd=LOW_EL_TO_AVOID_SUN_MAS;
	El_cmd_rate=0.;
}
                tsshm->el = El_cmd;
                tsshm->elVel = El_cmd_rate;
        tsshm->msecCmd = milliseconds;
        } /* if scbcomm=1 */
/****************************end of motion control*********************/

   
	/* displaying everything */

	if (icount == 1)
	    initflag = 0;
	if (icount != 1)
	    initflag = 1;

	azoff_int=(short)azoff;
	eloff_int=(short)eloff;
	dsm_status=dsm_write(DSM_HOST,"DSM_AZOFF_ARCSEC_D",&azoff);
	dsm_status=dsm_write(DSM_HOST,"DSM_ELOFF_ARCSEC_D",&eloff);

	/* fill in the RM variables for ccd header info if optical mode*/
	time1 = time(NULL);
        tval = localtime(&time1);
        if(antennaNumber==1) strcpy(antdir,"ant1");
        if(antennaNumber==2) strcpy(antdir,"ant2");
        if(antennaNumber==3) strcpy(antdir,"ant3");
        if(antennaNumber==4) strcpy(antdir,"ant4");
        if(antennaNumber==5) strcpy(antdir,"ant5");
        if(antennaNumber==6) strcpy(antdir,"ant6");
        if(antennaNumber==7) strcpy(antdir,"ant7");
        if(antennaNumber==8) strcpy(antdir,"ant8");

	sprintf(snamefits, "/opticalPointing/%s/%s_%02d%02d%02d_%02d%02d%02d.fits", 
				antdir,sname2,
                                (tval->tm_year)+1900, tval->tm_mon + 1,
                                tval->tm_mday,
                                tval->tm_hour,
                                tval->tm_min,
                                tval->tm_sec);

	dsm_status=dsm_write(DSM_HOST,"DSM_CCD_FITS_FILENAME_C100",snamefits);
	if(dsm_status != RM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_write() filename");
                }

	dsm_status=dsm_write(DSM_ANT_0,"RM_SPECTRAL_TYPE_C10",sptype);
	if(dsm_status != DSM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_write() sptype");
                }
	dsm_status=dsm_write(DSM_HOST,"DSM_VISUAL_MAGNITUDE_F",&magnitude);
	if(dsm_status != RM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_write() magnitude");
                }



	switch (user)
	{
	case 'q':
	strcpy(lastCommand,"Exit from Track");
	SendLastCommandToDSM(lastCommand);



          if (scbComm == 1)
            {
                tsshm->azCmd = OFF_CMD;
                tsshm->elCmd = OFF_CMD;
		scbComm = 0;
            }                   

          	dsm_status=rm_clear_monitor();
                if(dsm_status != DSM_SUCCESS) {
                dsm_error_message(dsm_status,"rm_clear()");
                exit(1);
                }
	    fprintf(stderr,"\nReceived signal: %d. Exiting track. Bye.\n",
			receivedSignal);
            dsm_close();
	    if (receivedSignal==SIGINT) exit(0);
	    if (receivedSignal==SIGTERM) exit(QUIT_RTN);
		user = -1;
	    break;
	case '0':
	strcpy(lastCommand,"Reset offsets and  stop scans");
	SendLastCommandToDSM(lastCommand);
	    azscan_flag = 0;
	    elscan_flag = 0;
	    azoff = 0.;
	    eloff = 0.;
	    az_offset_flag=1;
            el_offset_flag=1;     
	    strcpy(messg, "                                       ");
		SendMessageToDSM(messg);
		user = -1;
	    break;

	case 'X':
	strcpy(lastCommand,"Load the new pointing model");
        SendLastCommandToDSM(lastCommand);
            azscan_flag = 0;
            elscan_flag = 0;
            azoff = 0.;
            eloff = 0.;
            az_offset_flag=1;
            el_offset_flag=1;
            strcpy(messg, "Loading new pointing model             ");
                SendMessageToDSM(messg);
	   read_mount_model_flag=1; 
                user = -1;
	   goto beginning;
	break;

	case 'O':
	strcpy(lastCommand,"Azoff commanded ");
	SendLastCommandToDSM(lastCommand);
        dsm_status=dsm_read(DSM_HOST,"DSM_COMMANDED_AZOFF_ARCSEC_D",&azoff,&timeStamp);
	az_offset_flag=1;
		user = -1;
	break;

/* ignore command '5', which is handled by the encoder-server
for holography mapping */

	case '5':
	radio_flag=1;
		user = -1;
	break;

	case '4':
	radio_flag=0;
		user = -1;
	break;

	case 'P':
	strcpy(lastCommand,"Eloff commanded ");
	SendLastCommandToDSM(lastCommand);
        dsm_status=dsm_read(DSM_HOST,"DSM_COMMANDED_ELOFF_ARCSEC_D",&eloff,&timeStamp);
        el_offset_flag=1;
		user = -1;
	break;

	case 'T':
	strcpy(lastCommand,"Az commanded ");
	SendLastCommandToDSM(lastCommand);
        dsm_status=dsm_read(DSM_HOST,"DSM_COMMANDED_AZ_DEG_D",&commanded_az,&timeStamp,&timeStamp);
        dsm_status=dsm_read(DSM_HOST,"DSM_COMMANDED_EL_DEG_D",&commanded_el,&timeStamp,&timeStamp);
	azelCommandFlag=1;

		icount=0;
		interrupt_command_flag=0;
		target_flag = 1;
		user = -1;
		goto new_source;
	break;

	case 'U':
	strcpy(lastCommand,"Offset unit commanded ");
	SendLastCommandToDSM(lastCommand);
        dsm_status=dsm_read(DSM_HOST,"DSM_OFFSET_UNIT_ARCSEC_S",&scan_unit_int,&timeStamp);
	scan_unit=(double)scan_unit_int;
		user = -1;
	break;

	case 'i':
	strcpy(lastCommand,"Integration time set ");
	SendLastCommandToDSM(lastCommand);
        dsm_status=dsm_read(DSM_HOST,"DSM_INTEGRATION_TIME_SEC_S",&integration_short,&timeStamp);
	integration=(int)integration_short;
		user = -1;
	break;

	case 'p':
	strcpy(lastCommand,"Start position switching");
	SendLastCommandToDSM(lastCommand);
	    position_switching_flag = 1;
	    off_source_timer = integration;
	    on_source_timer = integration ;
	    on_source = 1;
	    off_source = 0;
		user = -1;
	    break;

	case 'a':
	strcpy(lastCommand,"Azimuth scan");
	SendLastCommandToDSM(lastCommand);
	    azscan_flag = 1;
	    strcpy(messg, "                                       ");
		SendMessageToDSM(messg);
	    strcpy(messg, "azimuth scan");
		SendMessageToDSM(messg);
		user = -1;
	    break;
	case 'z':
	strcpy(lastCommand,"Stop az and el scans");
	SendLastCommandToDSM(lastCommand);
	    azscan_flag = 0;
	    elscan_flag = 0;
		user = -1;
	    break;
	case 'e':
	strcpy(lastCommand,"Elevation scan");
	SendLastCommandToDSM(lastCommand);
	    elscan_flag = 1;
	    strcpy(messg, "                                       ");
		SendMessageToDSM(messg);
	    strcpy(messg, "elevation scan");
		SendMessageToDSM(messg);
		user = -1;
	    break;

	case '7':
	strcpy(lastCommand,"Azoff by minus chopper beam");
	SendLastCommandToDSM(lastCommand);
	    azoff = azoff - CHOPPER_BEAM ;
		user = -1;
	    break;
	case '9':
	strcpy(lastCommand,"Azoff by plus chopper beam");
	SendLastCommandToDSM(lastCommand);
	    azoff = azoff + CHOPPER_BEAM ;
		user = -1;
	    break;

	case 'r':
	case 'R':
	strcpy(lastCommand,"Reset azoff and eloff");
	SendLastCommandToDSM(lastCommand);
	    azoff = 0.0;
	    eloff = 0.0;
	    az_offset_flag=1;
	    el_offset_flag=1;
		user = -1;
	    break;

	case 's':
	case 'S':
		user = -1;
	break;

	/* adding a command for ra-dec offsets */
	case '1':
	strcpy(lastCommand,"Adding ra/dec offset");
	SendLastCommandToDSM(lastCommand);
	dsm_status=dsm_read(DSM_HOST,"DSM_RAOFF_ARCSEC_D",&raOffset,&timeStamp);
	dsm_status=dsm_read(DSM_HOST,"DSM_DECOFF_ARCSEC_D",&decOffset,&timeStamp);
		icount=0;
		if(target_flag==1) target_flag=0;
		interrupt_command_flag=0;
	        if(errorflag==ERROR) errorflag=OK;
		radec_offset_flag=1;
		user = -1;
		goto new_source;

	    break;
	
	case 'n':
	az_offset_flag=1;
	el_offset_flag=1;
	strcpy(lastCommand,"Change source");
	SendLastCommandToDSM(lastCommand);
	dsm_status=dsm_read(DSM_HOST,"DSM_SOURCE_LENGTH_S",&slength,&timeStamp);

	dsm_status=dsm_read(DSM_HOST,"DSM_SOURCE_C34", sname);
	azelCommandFlag=0;

	dsm_status=dsm_read(DSM_HOST,"DSM_CMD_SOURCE_FLAG_L", &newSourceFlag,&timeStamp);
	if(newSourceFlag==1) sol_sys_flag=0;

		icount=0;
		if(target_flag==1) target_flag=0;
		interrupt_command_flag=0;
	        if(errorflag==ERROR) errorflag=OK;
		user = -1;
		goto new_source;

	    break;

	case '@':
	strcpy(lastCommand,"Standby - put track in simulation mode");
	SendLastCommandToDSM(lastCommand);
	
	    if(scbComm==1)
            {
            position_switching_flag = 0;
            strcpy(messg, "Standing by.                           ");
            tsshm->azCmd = OFF_CMD;
            tsshm->elCmd = OFF_CMD;
            scbComm=0;
	    servoOnFlag=0;
            }            

		user = -1;
	    break;

	case '!':
	strcpy(lastCommand,"Resume - put track in real mode");
	SendLastCommandToDSM(lastCommand);
	    if(scbComm==0)
	    {
		icount=0;
		scbComm = 1;
            strcpy(messg, " Resuming 				       ");
		SendMessageToDSM(messg);
		user = -1;
		goto beginning;
	    }
	    break;

	case ';':
	if(beep_flag==1) {
	beep_flag=0;
	strcpy(lastCommand,"Turning off beeping");
	SendLastCommandToDSM(lastCommand);
		user = -1;
	break;
	}
	if(beep_flag==0) {
	beep_flag=1;
	strcpy(lastCommand,"Turning on beeping");
	SendLastCommandToDSM(lastCommand);
	}
		user = -1;
	break;
	
	    
	}			/* end of switch */

	interrupt_command_flag=0;


#if DEBUG
	printf("source=%s\n", sname);
	printf("lst=%lf\n", lst_disp);
	printf("utc=%lf\n", utc_disp);
	printf("tjd=%d\n", tjd_disp);
	printf("ra=%lf\n", ra_disp);
	printf("dec=%lf\n", dec_disp);
	printf("ra0=%lf\n", ra0);
	printf("dec0=%lf\n", dec0);
	printf("radot=%lf\n", radot);
	printf("decdot=%lf\n", decdot);
	printf("target_flag=%d\n", target_flag);
	printf("icount=%d\n", icount);
	printf("ra_cat=%lf\n", ra_cat_disp);
	printf("dec_cat=%lf\n", dec_cat_disp);
	printf("az_disp=%lf\n", az_disp);
	printf("el_disp=%lf\n", el_disp);
	printf("temperature=%f\n",temperature);
	printf("humidity=%f\n",humidity);
	printf("pressure=%f\n",pressure);
	printf("refraction=%f\n",refraction);
#endif


/* transfer hour-angle and declination through refl.mem. (RPC)*/

#if COORDINATES_SVC
	hourangle=lst_disp - ra_disp;

	dsm_status=dsm_write(DSM_HOST,"DSM_HOUR_ANGLE_HR_D", &hourangle);
	dsm_status=dsm_write(DSM_HOST,"DSM_RA_APP_HR_D", &ra_disp);
	dsm_status=dsm_write(DSM_HOST,"DSM_DEC_APP_DEG_D",&dec_disp);
	dsm_status=dsm_write(DSM_HOST,"DSM_UTC_HR_D", &utc_disp);
/* magnitude is actually velocity in this single dish version- in case
we are observing sources other than stars for optical pointing */
        if(sol_sys_flag==0) dummyDouble=sourceVelocity;
        if(sol_sys_flag==1) {
                dummyDouble=radialVelocity;
                magnitude=(float)radialVelocity;
                }
	dsm_status=dsm_write(DSM_HOST,"DSM_SVEL_KMPS_D",&dummyDouble);
	if(sol_sys_flag==0) dummyshortint=0x1;
	if(sol_sys_flag==1) dummyshortint=0x2;
	dsm_status=dsm_write(DSM_HOST,"DSM_SVELTYPE_S",&dummyshortint);

	dummyDouble=latitude_degrees;
	dsm_status=dsm_write(DSM_HOST,"DSM_LATITUDE_DEG_D", &dummyDouble);

	dummyDouble= longitude_degrees;
	dsm_status=dsm_write(DSM_HOST,"DSM_LONGITUDE_DEG_D",&dummyDouble);
	drefraction=(double)refraction;
	dsm_status=dsm_write(DSM_HOST,"DSM_REFRACTION_ARCSEC_D",&drefraction);
	if((drefraction<0.0)||(drefraction>4000.)) {
	  strcpy(operatorErrorMessage, "Refraction correction failed.");
	  sendOpMessage(OPMSG_WARNING, 10, 30, operatorErrorMessage);
	}

	/* pointing model in RM*/
	
	if(radio_flag==1) {
        dsm_status=dsm_write(DSM_HOST,"DSM_AZDC_ARCSEC_D", &razdc);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZCOLLIMATION_ARCSEC_D", &razcol);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZELAXISTILT_ARCSEC_D", &reltilt);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTSIN_ARCSEC_D", &raztilt_sin);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTCOS_ARCSEC_D", &raztilt_cos);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTSIN2_ARCSEC_D", &raztilt_sin2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTCOS2_ARCSEC_D", &raztilt_cos2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCSIN_ARCSEC_D", &razenc_sin);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCCOS_ARCSEC_D", &razenc_cos);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCSIN2_ARCSEC_D", &razenc_sin2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCCOS2_ARCSEC_D", &razenc_cos2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCSIN3_ARCSEC_D", &razenc_sin3);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCCOS3_ARCSEC_D", &razenc_cos3);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZRMS_ARCSEC_D", &razmodelrms);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELDC_ARCSEC_D", &reldc);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELSAG_ARCSEC_D", &relsag);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTSIN_ARCSEC_D", &reaztilt_sin);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTCOS_ARCSEC_D", &reaztilt_cos);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTSIN2_ARCSEC_D", &reaztilt_sin2);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTCOS2_ARCSEC_D", &reaztilt_cos2);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELRMS_ARCSEC_D", &relmodelrms);
        dsm_status=dsm_write(DSM_HOST,"DSM_MODELDATE_C10",rmodeldate);
	}
	if(radio_flag==0) {
        dsm_status=dsm_write(DSM_HOST,"DSM_AZDC_ARCSEC_D", &azdc);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZCOLLIMATION_ARCSEC_D", &azcol);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZELAXISTILT_ARCSEC_D", &eltilt);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTSIN_ARCSEC_D", &aztilt_sin);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTCOS_ARCSEC_D", &aztilt_cos);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTSIN2_ARCSEC_D", &aztilt_sin2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZAZTILTCOS2_ARCSEC_D", &aztilt_cos2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCSIN_ARCSEC_D", &azenc_sin);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCCOS_ARCSEC_D", &azenc_cos);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCSIN2_ARCSEC_D", &azenc_sin2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCCOS2_ARCSEC_D", &azenc_cos2);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCSIN3_ARCSEC_D", &azenc_sin3);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZENCCOS3_ARCSEC_D", &azenc_cos3);
        dsm_status=dsm_write(DSM_HOST,"DSM_AZRMS_ARCSEC_D", &azmodelrms);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELDC_ARCSEC_D", &eldc);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELSAG_ARCSEC_D", &elsag);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTSIN_ARCSEC_D", &eaztilt_sin);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTCOS_ARCSEC_D", &eaztilt_cos);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTSIN2_ARCSEC_D", &eaztilt_sin2);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELAZTILTCOS2_ARCSEC_D", &eaztilt_cos2);
        dsm_status=dsm_write(DSM_HOST,"DSM_ELRMS_ARCSEC_D", &elmodelrms);
        dsm_status=dsm_write(DSM_HOST,"DSM_MODELDATE_C10",modeldate);
	}

#endif

	az_actual_corrected=az_actual_disp;
	
	if(sol_sys_flag==1)  planetdistance=distance;
	if(sol_sys_flag==0) {
	planetdistance=0.0;
	planetdiameter=0.0;
	}

        dsm_status=dsm_write(DSM_HOST,"DSM_PLANET_DISTANCE_AU_D", &planetdistance);
        dsm_status=dsm_write(DSM_HOST,"DSM_PLANET_DIAMETER_ARCSEC_D",&planetdiameter);

	
/* communicate some of the above variables to others through reflective
memory for monitoring purposes (azoff and eloff already written out
to RM ealier */



	 dummyByte=(char)radio_flag;
	dsm_status=dsm_write(DSM_HOST,"DSM_REFRACTION_RADIO_FLAG_B",&dummyByte);

	    ret = dsm_status=dsm_write(DSM_HOST,"DSM_SOURCE_C34",sname);

	lst_disp_float=(float)lst_disp;
	utc_disp_float=(float)utc_disp;
	dsm_status=dsm_write(DSM_HOST,"DSM_LST_HOURS_F",&lst_disp_float);
	dsm_status=dsm_write(DSM_HOST,"DSM_UTC_HOURS_F",&utc_disp_float);

	dsm_status=dsm_write(DSM_HOST,"DSM_TJD_D",&tjd_disp);

	dummyFloat=(float)ra_cat_disp;
	dsm_status=dsm_write(DSM_HOST,"DSM_RA_CAT_HOURS_F",&dummyFloat);
	dummyFloat=(float)dec_cat_disp;
	dsm_status=dsm_write(DSM_HOST,"DSM_DEC_CAT_DEG_F",&dummyFloat);

	dsm_status=dsm_write(DSM_HOST,"DSM_EPOCH_F",&epoch);

	el_actual_disp_rm=(float)el_actual_disp;
	az_actual_corrected_rm=(float)az_actual_corrected;
	dsm_status=dsm_write(DSM_HOST,"DSM_ACTUAL_AZ_DEG_F",&az_actual_corrected_rm);
	dsm_status=dsm_write(DSM_HOST,"DSM_ACTUAL_EL_DEG_F",&el_actual_disp_rm);
	dsm_status=dsm_write(DSM_HOST,"DSM_PMDAZ_ARCSEC_F",&pmdaz);
	dsm_status=dsm_write(DSM_HOST,"DSM_PMDEL_ARCSEC_F",&pmdel);

	fflush(stdout);

	checkmsec=tsshm->msec; /* for checking if servo is running*/


  /* timestamp for DERS etc..*/
        dsm_status=dsm_read(DSM_HOST,"DSM_UNIX_TIME_L",&timestamp,&timeStamp);
        dsm_status=dsm_write(DSM_HOST,"DSM_TRACK_TIMESTAMP_L",&timestamp);

	/* update milliseconds to servo's shared memory 
        tsshm->msecCmd = milliseconds;
	*/
/*debug
fprintf(fp_debug, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
tjd_disp,utc_disp,tsshm->limAz,tsshm->limEl,az_actual,el_actual,Az_cmd_rate,El_cmd_rate);
if(icount==1) 
fprintf(fp_debug, "tjd_disp utc_disp tsshm->msec tsshm->msecCmd milliseconds  tsshm->az  tsshm->el  tsshm->azVel  tsshm->elVel  scbComm  tsshm->azCmd tsshm->elCmd tsshm->azState tsshm->elState\n");
fprintf(fp_debug, "%lf %lf %d %d %d %lf %lf %lf %lf %d %d %d %d %d\n", tjd_disp,utc_disp,tsshm->msec,tsshm->msecCmd,milliseconds, tsshm->az, tsshm->el, tsshm->azVel, tsshm->elVel, scbComm, tsshm->azCmd,tsshm->elCmd,tsshm->azState,tsshm->elState);
*/

	sleep(1);
    }				/* this is the big while loop */


}				/* end of main Loop */

void
split(unsigned long * lw, unsigned short * sw1, unsigned short * sw2)
{
    *sw1 = *lw & 0xFFFF;
    *sw2 = (*lw & 0xFFF0000) / 0x10000;
}


/*------------name.c--------*/
void
print_upper(char *name)
{
    register int    t;

    for (t = 0; name[t]; ++t)
    {
	name[t] = tolower(name[t]);
	/*
	 * putchar (name[t]);
	 */

    }

}

void
pad(char *s, int length)
{
    int             l;

    l = strlen(s);
    while (l < length)
    {
	s[l] = ' ';
	l++;
    }

    s[l] = '\0';
}


/*
 * This table is to be extended as and when we acquire more ephemeris data
 * from JPL for other minor bodies
 */

void
is_planet(char *s, int *flag, int *id)
{
    if (!strcmp(s, "mercury             "))
    {
	*id = 1;
	*flag = 1;
    } else if (!strcmp(s, "venus               "))
    {
	*id = 2;
	*flag = 1;
    } else if (!strcmp(s, "earth               "))
    {
	*id = 3;
	*flag = 1;
    } else if (!strcmp(s, "mars                "))
    {
	*id = 4;
	*flag = 1;
    } else if (!strcmp(s, "jupiter             "))
    {
	*id = 5;
	*flag = 1;
    } else if (!strcmp(s, "saturn              "))
    {
	*id = 6;
	*flag = 1;
    } else if (!strcmp(s, "uranus              "))
    {
	*id = 7;
	*flag = 1;
    } else if (!strcmp(s, "neptune             "))
    {
	*id = 8;
	*flag = 1;
    } else if (!strcmp(s, "pluto               "))
    {
	*id = 9;
	*flag = 1;
    } else if (!strcmp(s, "moon                "))
    {
	*id = 301;
	*flag = 1;
    } else if (!strcmp(s, "sun                 "))
    {
	*id = 10;
	*flag = 1;
    } else if (!strcmp(s, "titan               "))
    {
	*id = 606;
	*flag = 1;
    } else if (!strcmp(s, "io                  "))
    {
	*id = 501;
	*flag = 1;
    } else if (!strcmp(s, "europa              "))
    {
	*id = 502;
	*flag = 1;
    } else if (!strcmp(s, "callisto            "))
    {
	*id = 504;
	*flag = 1;
    } else if (!strcmp(s, "ganymede            "))
    {
	*id = 503;
	*flag = 1;
    } else if (!strcmp(s, "mimas               "))
    {
	*id = 601;
	*flag = 1;
    } else if (!strcmp(s, "enceladus           "))
    {
	*id = 602;
	*flag = 1;
    } else if (!strcmp(s, "tethys              "))
    {
	*id = 603;
	*flag = 1;
    } else if (!strcmp(s, "dione               "))
    {
	*id = 604;
	*flag = 1;
    } else if (!strcmp(s, "rhea                "))
    {
	*id = 605;
	*flag = 1;
    } else if (!strcmp(s, "hyperion            "))
    {
	*id = 607;
	*flag = 1;
    } else if (!strcmp(s, "iapetus             "))
    {
	*id = 608;
	*flag = 1;
    } else if (!strcmp(s, "triton              "))
    {
	*id = 801;
	*flag = 1;
    } else if (!strcmp(s, "nereid              "))
    {
	*id = 802;
	*flag = 1;
    } else if (!strcmp(s, "ceres               "))
    {
        *id = 375;
        *flag = 1;
    } else if (!strcmp(s, "pallas              "))
    {
        *id = 376;
        *flag = 1;
    } else if (!strcmp(s, "hygiea              "))
    {
        *id = 377;
        *flag = 1;
    } else if (!strcmp(s, "vesta               "))
    {
	*id = 378;
	*flag = 1;
    } else if (!strcmp(s, "comet103p           "))
    {
  	*id = 385;
	*flag = 1;
    } else if (!strcmp(s, "2005yu55            "))
    {
  	*id = 386;
	*flag = 1;
    } else
    {
	*id = 0;
	*flag = 0;
    }
}

/* The following function does the standard star catalog look up */

void
starcat(char *s, int *star_flag, struct source * observe_source)
{

    FILE           *fp2;

    int             end_of_file=0;

    int             number;

    int             rah, ram, decd, decm;

    float           vel, epoch, ras, decs, pmr, pmd;

    char            vtype[20], source_name[20], comment[100];
    char            decsign;

    *star_flag = 0;

/*
bug: 3 aug 2009: 1st source is skipped
*/
	if(radio_flag==1) {
	fp2 = fopen("/global/catalogs/sma_catalog", "r");
        end_of_file = fscanf(fp2, 
	"%s %d %d %f %c%2d %d %f %f %f %f %s %f %s",
	 source_name, &rah, &ram, &ras, &decsign, &decd, &decm, &decs,
		    &pmr, &pmd, &epoch, vtype, &magnitude, comment);
	}

    	if(radio_flag==0) {
	fp2 = fopen("/global/catalogs/sma_optical_catalog", "r");
        end_of_file = fscanf(fp2, 
	"%d %s %d %d %f %c%2d %d %f %f %f %f %s %f %s %s",
	 &number,source_name, &rah, &ram, &ras, &decsign, &decd, &decm, &decs,
		    &pmr, &pmd, &epoch, vtype, &magnitude, comment,sptype);
	}

	
        while (end_of_file != EOF) {

	if(radio_flag==1) {
        end_of_file = fscanf(fp2, 
	"%s %d %d %f %c%2d %d %f %f %f %f %s %f %s",
	 source_name, &rah, &ram, &ras, &decsign, &decd, &decm, &decs,
		    &pmr, &pmd, &epoch, vtype, &magnitude, comment);
	}

    	if(radio_flag==0) {
        end_of_file = fscanf(fp2, 
	"%d %s %d %d %f %c%2d %d %f %f %f %f %s %f %s %s",
	 &number,source_name, &rah, &ram, &ras, &decsign, &decd, &decm, &decs,
		    &pmr, &pmd, &epoch, vtype, &magnitude, comment, sptype);
	}

	pad(source_name, 20);
	print_upper(source_name);

	if (!strcmp(s, source_name))
	{
	    *star_flag = 1;
	strcpy(observe_source->sourcename, source_name);
	observe_source->rah = rah;
	observe_source->ram = ram;
	observe_source->ras = ras;
	observe_source->decsign = decsign;
	observe_source->decd = decd;
	observe_source->decm = decm;
	observe_source->decs = decs;
	observe_source->pmr = pmr;
	observe_source->pmd = pmd;
	observe_source->epoch = epoch;
	strcpy(observe_source->veltype, vtype);
	observe_source->vel = magnitude;
	sourceVelocity=(double)magnitude;
	strcpy(observe_source->comment, comment);
	strcpy(messg, " ");
	break;
	}
	if(radio_flag==1) {
        if (end_of_file!=14) {
	strcpy(messg, "Source catalog is corrupted. ");
        SendMessageToRM(messg);
	fprintf(stderr,"%s\n",messg);
	strcpy(operatorErrorMessage, "Source catalog is corrupted.");
	  sendOpMessage(OPMSG_WARNING, 10, 30, operatorErrorMessage);
        break;
         }}
	if(radio_flag==0) {
        if (end_of_file!=16) {
	strcpy(messg, "Source catalog is corrupted. ");
        SendMessageToDSM(messg);
	fprintf(stderr,"%s\n",messg);
	strcpy(operatorErrorMessage, "Source catalog is corrupted.");
	  sendOpMessage(OPMSG_WARNING, 10, 30, operatorErrorMessage);
        break;
         }}

        end_of_file=0;
    }
    fclose(fp2);
}

void SendMessageToRM(char *messg)
{
int messagelength;
char blank[100];
messagelength=strlen(messg);
sprintf(blank,"                                                                                                   ");                      
dsm_status=dsm_write(DSM_HOST,"DSM_TRACK_MESSAGE_C100",blank);
dsm_status=dsm_write(DSM_HOST,"DSM_TRACK_MESSAGE_C100",messg);
}

void SendLastCommandToRM(char *lastCommand)
{
int messagelength;
char blank[100];
messagelength=strlen(lastCommand);

sprintf(blank,"                                                                                                   ");                      
dsm_status=dsm_write(DSM_HOST,"DSM_TRACK_LAST_COMMAND_C100",blank);
dsm_status=dsm_write(DSM_HOST,"DSM_TRACK_LAST_COMMAND_C100",lastCommand);
}

double sunDistance(double az1,double el1,double az2,double el2)
{
double cosd,sind,d;

cosd=sin(el1)*sin(el2)+cos(el1)*cos(el2)*cos(az1-az2);
sind=pow((1.0-cosd*cosd),0.5);
d=atan2(sind,cosd);
d=d/radian;
return d;

}

/* Interrupt handler for receiving commands from console command hrough DSM*/

void *CommandHandler()
{
char command[30];
char name[DSM_NAME_LENGTH];
int ant=DSM_HOST;

	sprintf(name,"DSM_CONSOLE_COMMAND_FLAG_S");

	while(1)
	{
	
	dsm_status=dsm_read_wait(&ant,name,&command_flag,&timeStamp);
        if(dsm_status != DSM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_read_wait()");
                exit(1);

        fprintf(stderr,"Interrupt received. command_flag=%d\n",command_flag);
	fflush(stderr);

	sleep(1);
	}
	
	if(command_flag==0)
	{
	dsm_status=dsm_read(DSM_HOST,"DSM_COMMANDED_TRACK_COMMAND_C30",command,&timeStamp);
        if(dsm_status != RM_SUCCESS) {
                dsm_error_message(dsm_status,"dsm_read()");
                exit(1);
        }

        user=(int)command[0];
	interrupt_command_flag=1;
	}

	} /* while */

	pthread_detach(&CommandHandlerTID);
	pthread_exit((void *) 0);
}

void handlerForSIGINT(int signum)
{
        interrupt_command_flag=1;
        user='q'; /* 'q' for quit command */
	receivedSignal=signum;
        fprintf(stderr,"Got the control C signal:%d. Quitting.\n",signum);
}

double tjd2et(double tjd)
{
  return((tjd-2451545.)* 86400.0+32.184+LEAPSECONDS);
}

/*********************************end of track.c*************************/
