/*
 *       Example of PID library.
 *       2012 Code by Jason Short, Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AC_POK.h>
#include <AP_Param.h>
#include <AP_Baro_Glitch.h>
#include <AP_GPS_Glitch.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Compass.h>
#include <AP_AHRS_DCM.h>
#include <AP_BattMonitor.h>
#include <AP_InertialNav.h>
#include <AC_Telemetry.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_Declination.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Buffer.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AC_POK_Stm.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <AP_Mission.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Compass.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_AVR_SITL.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define CONFIG_BARO HAL_BARO_DEFAULT

#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
#error Unrecognized CONFIG_BARO setting
#endif

#define CONFIG_COMPASS HAL_COMPASS_DEFAULT

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#else
#error Unrecognized CONFIG_COMPASS setting
#endif

// setup function
void setup() {
	hal.console->println("ArduPilot Mega AC_POK library test");
	hal.scheduler->delay(1000);
}

// main loop
void loop() {
	AP_InertialSensor ins;
	AP_GPS gps;
	GPS_Glitch gps_glitch(gps);
	Baro_Glitch baro_glitch(barometer);
	AP_BattMonitor battery;
	AP_AHRS_DCM ahrs(ins, barometer, gps);
	AP_InertialNav inertial_nav(ahrs, barometer, gps_glitch, baro_glitch);
	Telem telem(ins, gps, gps_glitch, barometer, baro_glitch, compass, battery,ahrs, inertial_nav);

	AC_POK_Stm pok;

	pok.init();

	struct local_data d;
	d.mode = 0;
	d.armed = 1;

	while (true) {
		pok.update(d, telem);
	}
}

AP_HAL_MAIN();
