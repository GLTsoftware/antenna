#include <stdio.h>
#include <math.h>
#define YOFFSETSCALE -64.6 /* arcsec/mm */

extern double          azdc,azcol,eltilt,aztilt_sin,aztilt_cos,aztilt_sin2,aztilt_cos2,
		azenc_sin,azenc_cos,azenc_sin2,azenc_cos2,
		azenc_sin3,azenc_cos3,
                eldc,elsag,eaztilt_sin,eaztilt_cos,eaztilt_sin2,eaztilt_cos2;
extern double          razdc,razcol,reltilt,raztilt_sin,raztilt_cos,raztilt_sin2,
                raztilt_cos2, reldc,relsag,reaztilt_sin,reaztilt_cos,
		razenc_sin,razenc_cos,razenc_sin2,razenc_cos2,
		razenc_sin3,razenc_cos3,
                reaztilt_sin2,reaztilt_cos2;
/*
extern double tiltx, tilty, tiltCoefficients[6];
extern double scaledTiltx, scaledTilty;
extern int tiltflag;
extern double tiltAzoffCorrection, tiltEloffCorrection;
extern double pmaztiltAzoff,pmaztiltEloff;
*/

extern double setFeedOffsetA1,setFeedOffsetA2;
extern float chopperYCorrection;
extern int chopperYZCorrFlag; 
 

void mount(double *az, double *el,int radio_flag,float *pmdaz, float *pmdel)
{

	double daz,del; 
	double xtilt, ytilt;
	double sec2rad=4.8481368111e-6;
	double cose,sine,cosa,sina;
	double sin2a, cos2a, sin3a, cos3a;
	int i;
	double at=0.0,tilt=300.0;
	static double pi, radian;
	int addTiltFlag=1;


	pi = 4.0 * atan(1.);
	radian = pi/180.;

	cose = cos(*el);
	sine = sin(*el);
	cosa = cos(*az);
	sina = sin(*az);

	sin2a = 2.0 * sina * cosa;

	cos2a = 2.0 * cosa * cosa - 1.0;

	sin3a = sin(3.0*(*az));
	cos3a = cos(3.0*(*az));


/*
	if(radio_flag==1){	
	pmaztiltAzoff=raztilt_sin*sine*sina -
		raztilt_cos*sine*cosa +
			raztilt_sin2*sine*sin2a-
				raztilt_cos2*sine*cos2a;
	pmaztiltEloff=reaztilt_sin*cosa + reaztilt_cos*sina +
			reaztilt_sin2*cos2a +reaztilt_cos2*sin2a;
	} else {
	pmaztiltAzoff=aztilt_sin*sine*sina -
		aztilt_cos*sine*cosa +
			aztilt_sin2*sine*sin2a-
				aztilt_cos2*sine*cos2a;
	pmaztiltEloff=eaztilt_sin*cosa + eaztilt_cos*sina +
			eaztilt_sin2*cos2a +eaztilt_cos2*sin2a;
	}
*/
	

/*
	if(tiltflag==0)
	{
	tiltAzoffCorrection=0.;
	tiltEloffCorrection=0.;
*/
	if(radio_flag==1){	


	daz=razdc*cose + razcol + reltilt*sine + raztilt_sin*sine*sina -raztilt_cos*sine*cosa +raztilt_sin2*sine*sin2a-raztilt_cos2*sine*cos2a
		+razenc_sin*sina*cose+razenc_cos*cosa*cose
		+razenc_sin2*sin2a*cose+razenc_cos2*cos2a*cose
		+razenc_sin3*sin3a*cose+razenc_cos3*cos3a*cose;
	del= reldc + relsag*cose + reaztilt_sin*cosa + reaztilt_cos*sina +reaztilt_sin2*cos2a +reaztilt_cos2*sin2a;
/*
	if(chopperYZCorrFlag==1) {
		    del+=YOFFSETSCALE*chopperYCorrection;
*/
/*
	            printf("chopperYCorrection is ON = %f\n",chopperYCorrection); 
*/
/*
	            }
*/
		}

	else {

	daz=azdc*cose + azcol + eltilt*sine + aztilt_sin*sine*sina -aztilt_cos*sine*cosa +aztilt_sin2*sine*sin2a-aztilt_cos2*sine*cos2a
		+azenc_sin*sina*cose+azenc_cos*cosa*cose
		+azenc_sin2*sin2a*cose+azenc_cos2*cos2a*cose
		+azenc_sin3*sin3a*cose+azenc_cos3*cos3a*cose;
	del= eldc + elsag*cose + eaztilt_sin*cosa + eaztilt_cos*sina +eaztilt_sin2*cos2a +eaztilt_cos2*sin2a;

		}
/*
	}
*/


#if 0
	tiltAzoffCorrection= scaledTilty*sine;
	tiltEloffCorrection = scaledTiltx; 

	if(tiltflag==1)
	{

/*
printf("tiltAzoffCorrection=%f, tiltEloffCorrection=%f stx=%f sty=%f\n",tiltAzoffCorrection, tiltEloffCorrection, scaledTiltx, scaledTilty);
*/
	if(radio_flag==1){	

	daz=razdc*cose + razcol + reltilt*sine + tiltAzoffCorrection
		+razenc_sin*sina*cose+razenc_cos*cosa*cose
		+razenc_sin2*sin2a*cose+razenc_cos2*cos2a*cose
		+razenc_sin3*sin3a*cose+razenc_cos3*cos2a*cose;

	if(addTiltFlag==1) daz=daz+raztilt_sin*sine*sina -raztilt_cos*sine*cosa +raztilt_sin2*sine*sin2a-raztilt_cos2*sine*cos2a;

	del= reldc + relsag*cose +  tiltEloffCorrection;

	if(addTiltFlag==1) del=del+reaztilt_sin*cosa + reaztilt_cos*sina +reaztilt_sin2*cos2a +reaztilt_cos2*sin2a;

	if(chopperYZCorrFlag==1) {
		    del+=YOFFSETSCALE*chopperYCorrection;
/*
	            printf("chopperYCorrection is ON = %f\n",chopperYCorrection); 
*/
	            }


			}

	else {

	daz=azdc*cose + azcol + eltilt*sine + tiltAzoffCorrection
		+azenc_sin*sina*cose+azenc_cos*cosa*cose
		+azenc_sin2*sin2a*cose+azenc_cos2*cos2a*cose
		+azenc_sin3*sin3a*cose+azenc_cos3*cos2a*cose;

	if(addTiltFlag==1) daz=daz+aztilt_sin*sine*sina -aztilt_cos*sine*cosa +aztilt_sin2*sine*sin2a-aztilt_cos2*sine*cos2a;

	del= eldc + elsag*cose + tiltEloffCorrection;
	if(addTiltFlag==1) del=del+eaztilt_sin*cosa + eaztilt_cos*sina +eaztilt_sin2*cos2a +eaztilt_cos2*sin2a;

		}
	}
#endif

	*pmdaz=(float)daz;
	
	/* feedOffsets*/
	if(radio_flag==1) {
	daz = daz + setFeedOffsetA1 * sine + setFeedOffsetA2 * cose;
	del = del + setFeedOffsetA1 * cose - setFeedOffsetA2 * sine;
	}
	  
	daz = daz / cose ; 
	
	*pmdel=(float)del;

	*az += daz * sec2rad;
	
	if(*az>=2.*pi) *az-=2.0*pi;

	*el += del * sec2rad; 

/*
printf("from within mount subroutine %.14f  %.14f \n",*az,*el);
printf ("tiltflag=%d \n",tiltflag);
printf ("tiltAzoffCorrection=%f tiltEloffCorrection=%f \n",tiltAzoffCorrection,tiltEloffCorrection);
printf ("tiltCoefficients[0]=%f tiltCoefficients[1]=%f\n",tiltCoefficients[0], tiltCoefficients[1]);
printf ("tiltCoefficients[2]=%f tiltCoefficients[3]=%f\n",tiltCoefficients[2], tiltCoefficients[3]);
printf ("tiltCoefficients[4]=%f tiltCoefficients[5]=%f\n",tiltCoefficients[4], tiltCoefficients[5]);
printf ("tiltx=%f tilty=%f\n",tiltx,tilty);


printf("pmdazM=%f pmdelM=%f\n",*pmdaz,*pmdel);
*/

}
