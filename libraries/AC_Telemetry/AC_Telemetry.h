#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#include <AP_GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Baro.h>
#include <AP_Baro_Glitch.h>
#include <AP_Compass.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_BattMonitor.h>
//
// Wrapper for all Telemtry data
//
class Telem {
public:
	// Constructor
	Telem()
	{
		ins = new AP_InertialSensor();
		gps = new AP_GPS();
		gps_glitch = new GPS_Glitch(*gps);
		baro = new AP_Baro_HIL();
		baro_glitch = new Baro_Glitch(*baro);
		compass = new AP_Compass_HIL();
		battery = new AP_BattMonitor();
		ahrs = new AP_AHRS_DCM(*ins, *baro, *gps);
		inertial_nav =  new AP_InertialNav(*ahrs, *baro, *gps_glitch, *baro_glitch);
	}

	AP_InertialSensor& getIns() { return *ins;}

	AP_GPS& getGps() { return *gps;}
	GPS_Glitch& getGpsGlitch() { return *gps_glitch;}

	AP_Baro_HIL& getBaro() { return *baro;}
	Baro_Glitch& getBaroGlitch() { return *baro_glitch;}

	AP_Compass_HIL& getCompass() { return *compass;}
	AP_BattMonitor& getBattery() { return *battery;}

	AP_AHRS_DCM& getAhrs() { return *ahrs;}
	AP_InertialNav& getInertialNav() { return *inertial_nav;}

private:
	AP_InertialSensor* ins;

	AP_GPS* gps;
	GPS_Glitch* gps_glitch;

	AP_Baro_HIL* baro;
	Baro_Glitch*  baro_glitch;

	AP_Compass_HIL* compass;
	AP_BattMonitor* battery;

	AP_AHRS_DCM* ahrs;
	AP_InertialNav* inertial_nav;

};

#endif // __TELEMETRY_H__

