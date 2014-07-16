/*
 * GeneralSettings.h
 *
 * Created: 12.04.2013 12:35:12
 *  Author: snfu
 */ 


#ifndef GENERALSETTINGS_H_
#define GENERALSETTINGS_H_

#define F_CPU		12000000UL
#define SCL_CLOCK	100000UL

#define PI			3.1415926535897932384626433832795

#define TRUE 1
#define FALSE 0

// Control Mode
//#define DIRECT_LAW	0
//#define NORMAL_LAW	1

typedef enum{	DIRECT_CTRL = 0,
				NORMAL_CTRL,
				DAMPED_CTRL,
				HOLD_CTRL}
	CTRL_LAWS_e;

typedef enum {
				GPS_RMC_MODE = 0,
				GPS_RMC_TIME,
				GPS_RMC_STATUS,
				GPS_RMC_LATITUDE,
				GPS_RMC_LATORIENT,
				GPS_RMC_LONGITUDE,
				GPS_RMC_LONGORIENT,
				GPS_RMC_GROUNDSPEED,
				GPS_RMC_PATH,
				GPS_RMC_DATE,
				GPS_RMC_MAGDEV,
				GPS_RMC_MAGDEVORIENT,
				GPS_RMC_CHECKSUM
} GPS_RMC_e ;

typedef enum {
				GPS_GGA_MODE = 0,
				GPS_GGA_TIME,
				GPS_GGA_LATITUDE,
				GPS_GGA_LATORIENT,
				GPS_GGA_LONGITUDE,
				GPS_GGA_LONGORIENT,
				GPS_GGA_QUAL,
				GPS_GGA_NOSAT,
				GPS_GGA_HORDEV,
				GPS_GGA_ALTMSL,
				GPS_GGA_ALTUNIT,
				GPS_GGA_GEOSEP,
				GPS_GGA_GEOSEPUNIT,
				GPS_GGA_DGPSAGE,
				GPS_GGA_DGPSREF,
				GPS_GGA_CHECKSUM
} GPS_GGA_e ;

struct three_elements_obj
{
	long p,q,r;
};

struct acc_readings_obj
{
	long a_x,a_y,a_z;	
};

#endif /* GENERALSETTINGS_H_ */

