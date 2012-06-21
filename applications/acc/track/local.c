#include <math.h>
#include "track.h"

#define ARCSEC_TO_RAD 4.84813681e-6

void Refract(double *el, int radio_flag, double temp,
		double humid, double pres, float *refraction);
void mount(double *az, double *el, int radio_flag, float *pmdaz, float *pmdel);
void sidtim(double *tjdh, double *tjdl, int *k, double *gst);

extern double sinlat,coslat,longrad,height_m,latitude_degrees;
extern double tiltx,tilty,tiltCoeficients[6];


void local(double *lst,double *ra,double *dec,double *az,double *el, double *tjd,double *azoff, double *eloff, float *pressure,float *temperature,float *humidity,int *radio_flag, float *refraction,float *pmdaz,float *pmdel, 
	short *target_flag,double *commanded_az, double *commanded_el)
{
	int i;
	double rlong = longrad; 
	double d1;
	double c1=0.0;
	int e1=1;
	double xyzobs[3],pos[3],xyproj,gst;


	double constant,ha,cosha,sinha,cosphi;
	double  dpi =   4.0 * atan(1.0);                /* pi */
	double  dtupi = 2.0 * dpi;
	double  rsin =  sinlat;  
	double  rcos =  coslat; 
	double  rear =  6378.1492;                      /* radius of earth in km */
	double  t2000 = 2451545.0;                      /* julian day number for year 2000 */
	double COEFF = 1.5514037795505152259e-6;  /* 0.32" in radians for diurnal aberration.*/
	double au = 1.4959787061368887e8;                       /* AU in km */
	double phi;
	double height = rear*1000.0 + height_m ;/* geocentric height in meters */

	double cosh,sinh,cosd,sind,sinphi;
	double sina,cosa;
	double rad;
	double dist;
	int radio;
	double pres, temp, humid;
	int ret;

 /*  Diurnal aberration */

	rad = dpi / 180.0;

	phi = latitude_degrees*rad;

	constant = COEFF * (height)/(rear*1000.);

	ha = *lst - *ra ;
	sinphi = sin(phi);
	cosphi = cos(phi);

/*
	*ra+=  constant * cosphi * cos(ha) / cos(*dec) ;
	*dec+= constant * cosphi * sin(ha) * sin(*dec) ;

	ha = *lst - *ra ;
*/
	cosh = cos(ha);
	sinh = sin(ha);
	cosd = cos(*dec);
	sind = sin(*dec);

/*      azimuth and elevation   */

	if(*target_flag==0)
	{
	sina = - cosd * sinh;

	cosa = sind * cosphi - cosd * cosh * sinphi;

	*el = asin(sind*sinphi + cosd*cosphi*cosh);

	if ( cosa == 0.0)
	{*az = 0.0;}
	else
	{*az = atan2(sina,cosa);}

	if(*az < 0.0) {*az = dtupi + *az;}
	}


	if(*target_flag==1)
	{
	*az=*commanded_az*rad;
	*el=*commanded_el*rad;
	}

	*el += ARCSEC_TO_RAD*(*eloff);
	*az += ARCSEC_TO_RAD*(*azoff)/cos(*el);


/*      refraction correction   */

	temp=*temperature;
	humid=*humidity;
	pres=*pressure;
	radio=*radio_flag;

	if(*target_flag==0)
	{
	if(*el>=0.0) Refract(el, radio, temp, humid, pres,refraction);
		else	*refraction=0.0;
	}

	if(*target_flag==1) *refraction=0.0;


	/*in above, do not apply refraction correction if observing
	a terrestrial object. NAP 6sep99*/

/*      add mount errors        */

	mount(az,el,radio,pmdaz,pmdel);
/*
	if(*az < 0.0) {*az = dtupi + *az;}
*/

/*
printf("pmdazL=%f pmdelL=%f\n",*pmdaz,*pmdel);
*/

}
