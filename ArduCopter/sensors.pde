// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdint.h>

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
    sonar.init();
}
#endif

static void init_barometer(bool full_calibration)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    if (full_calibration) {
        telem.getBaro().calibrate();
    }else{
        telem.getBaro().update_calibration();
    }
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static void read_barometer(void)
{
    telem.getBaro().read();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = telem.getBaro().get_altitude() * 100.0f;
    baro_climbrate = telem.getBaro().get_climb_rate() * 100.0f;

    // run glitch protection and update AP_Notify if home has been initialised
    telem.getBaroGlitch().check_alt();
    bool report_baro_glitch = (telem.getBaroGlitch().glitching() && !ap.usb_connected && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    if (AP_Notify::flags.baro_glitching != report_baro_glitch) {
        if (telem.getBaroGlitch().glitching()) {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_BARO_GLITCH);
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_ERROR_RESOLVED);
        }
        AP_Notify::flags.baro_glitching = report_baro_glitch;
    }
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    sonar.update();

    // exit immediately if sonar is disabled
    if (!sonar_enabled || !sonar.healthy()) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar.distance_cm();

    if (temp_alt >= sonar.min_distance_cm() && 
        temp_alt <= sonar.max_distance_cm() * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = telem.getAhrs().cos_pitch() * telem.getAhrs().cos_roll();
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}

static void init_compass()
{
    if (!telem.getCompass().init() || !telem.getCompass().read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    telem.getAhrs().set_compass(&telem.getCompass());
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    optflow.init();
    if (!optflow.healthy()) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("Failed to Init OptFlow\n"));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }
#endif      // OPTFLOW == ENABLED
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
static void read_battery(void)
{
    telem.getBattery().read();

    // update compass with current value
    if (telem.getBattery().monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        telem.getCompass().set_current(telem.getBattery().current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && telem.getBattery().exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}

///
/// Addition by S.S. to invoke SPI transfer
///

static void init_POK(void)
{
	cliSerial->println_P(PSTR("Init POK"));
	pok.init();
	cliSerial->println_P(PSTR("POK Init successfuly"));
}

static void update_POK(void)
{
	// Prepare some data
	struct local_data d;
	d.mode = control_mode;
	d.armed = motors.armed() == true ? 1 : 0;
	d.rc1_in = g.rc_1.control_in;
	d.rc2_in = g.rc_2.control_in;
	d.rc3_in = g.rc_3.control_in;
	d.rc4_in = g.rc_4.control_in;

	pok.update(d, telem);
}
