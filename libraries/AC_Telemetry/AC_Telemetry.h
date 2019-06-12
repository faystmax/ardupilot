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

	Telem(AP_InertialSensor &ins, AP_GPS &gps, GPS_Glitch &gps_glitch, AP_Baro &baro, Baro_Glitch &baro_glitch,
			Compass& compass, AP_BattMonitor &battery, AP_AHRS_DCM &ahrs,AP_InertialNav &inertial_nav) :
		_ins(ins),
        _gps(gps),
        _gps_glitch(gps_glitch),
        _baro(baro),
        _baro_glitch(baro_glitch),
        _compass(compass),
        _battery(battery),
        _ahrs(ahrs),
        _inertial_nav(inertial_nav)
        {
        }

	AP_InertialSensor& getIns() { return _ins;}

	AP_GPS& getGps() { return _gps;}
	GPS_Glitch& getGpsGlitch() { return _gps_glitch;}

	AP_Baro& getBaro() { return _baro;}
	Baro_Glitch& getBaroGlitch() { return _baro_glitch;}

	Compass& getCompass() { return _compass;}
	AP_BattMonitor& getBattery() { return _battery;}

	AP_AHRS_DCM& getAhrs() { return _ahrs;}
	AP_InertialNav& getInertialNav() { return _inertial_nav;}

private:
	AP_InertialSensor& _ins;

	AP_GPS& _gps;
	GPS_Glitch& _gps_glitch;

	AP_Baro& _baro;
	Baro_Glitch&  _baro_glitch;

	Compass& _compass;
	AP_BattMonitor& _battery;

	AP_AHRS_DCM& _ahrs;
	AP_InertialNav& _inertial_nav;

};

#endif // __TELEMETRY_H__

