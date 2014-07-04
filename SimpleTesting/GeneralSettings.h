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
				GPS_MODE = 0,
				GPS_TIME,
				GPS_STATUS,
				GPS_LATITUDE,
				GPS_LATORIENT,
				GPS_LONGITUDE,
				GPS_LONGORIENT,
				GPS_GROUNDSPEED,
				GPS_PATH,
				GPS_DATE,
				GPS_MAGDEV,
				GPS_MAGDEVORIENT,
				GPS_CHECKSUM
} GPS_params_e ;

struct three_elements_obj
{
	long p,q,r;
};

struct acc_readings_obj
{
	long a_x,a_y,a_z;	
};

#endif /* GENERALSETTINGS_H_ */

