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


//addition by S.S. to invoke SPI transfer

//demo function to calc checksum - note start from 4th byte of data
//static uint32_t calcSumCRC(void *data, int len) {
//    uint32_t sum = 0;
//    for (int i = 4; i < len; i++) sum += ((uint8_t *)data)[i]; //-4 byte for crc
//    return sum;
//}

static uint32_t calcCRC(void *data, int len) {
    uint32_t crc, byte, mask;

    crc = 0xFFFFFFFF;
    for (int i = 0; i < len - 4; i++) { // 4 byte for CRC
        byte = ((uint8_t *) data)[i];
        crc = crc ^ byte;
        for (int j = 7; j >= 0; j--) {      // Do eight times.
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
    }
    return ~crc;
}


//addition
#define SYNCHRONIZE_BYTE 0x02
AP_HAL::SPIDeviceDriver *_spi_pok;

typedef enum  {
  TRANSFER_OK,
  TRANSFER_ERROR
} TRANSFER_Status;

/// Exchange Structures
struct send_pack {
	uint32_t snc;
    uint32_t len;
    uint32_t velocity;
    float roll;
    float pitch;
	float yaw;
	uint32_t crc;
};

struct recive_pack {
	uint32_t snc;
    uint32_t code1;
    uint32_t code2;
    uint32_t code3;
    uint32_t code4;
    uint32_t code5;
    uint32_t code6;
    uint32_t  crc;
};

union message {
	struct send_pack send;
	struct recive_pack rcv;
};

const uint16_t message_size = sizeof(union message);
const uint16_t send_pack_size = sizeof(struct send_pack);
const uint16_t recive_pack_size = sizeof(struct recive_pack);

static void ReadPOK_Update(void)
{
    AP_HAL::Semaphore* _spi_sem_pok;
    uint8_t resp, read_len;
    static uint32_t  _timer = 0;
    uint32_t tnow = hal.scheduler->micros();

    // read rate to 100hz maximum.
    if (tnow - _timer < 10000) {
        return;
    }
    _timer = tnow;

    // get spi bus semaphore
    _spi_sem_pok = _spi_pok->get_semaphore();
    if (_spi_sem_pok == NULL || !_spi_sem_pok->take_nonblocking())
    {
    	hal.console->printf("ReadPOK() failed - sem error ");
        return;
    }

    uint32_t expected = 0;
	union message send;
	union message rcv;

	/// Preparing data to send
    memset(&send.send, 0, send_pack_size);
    send.send.snc = SYNCHRONIZE_BYTE;
    send.send.len = 6;
    send.send.velocity = 1010;
    send.send.roll = ToDeg(telem.getAhrs().roll);
    send.send.pitch = ToDeg(telem.getAhrs().pitch);
    send.send.yaw = ToDeg(telem.getAhrs().yaw);
    send.send.crc = calcCRC(&send.send, send_pack_size);

    /// Send to POK
    _spi_pok->cs_assert();
    _spi_pok->transaction((uint8_t *) &send.send, (uint8_t *) &rcv.rcv, message_size);

	expected = calcCRC(&rcv.rcv, recive_pack_size);
	if (expected == rcv.rcv.crc) {
		hal.console->printf("CRC OK and we rcv: %lu %lu %lu %lu %lu %lu\n",
				rcv.rcv.code1, rcv.rcv.code2, rcv.rcv.code3,
				rcv.rcv.code4, rcv.rcv.code5, rcv.rcv.code6);
	} else {
		hal.console->printf("CRC not valid %lu / %lu \n", expected, rcv.rcv.crc);
	}

    /// release
    _spi_pok->cs_release();
    _spi_sem_pok->give();
    return;
}

static void ReadPOK_Init(void)
{
    _spi_pok = hal.spi->device(AP_HAL::SPIDevice_POK);
    if (_spi_pok == NULL)
    {
    	hal.console->printf("ReadPOKInit() failed - device error ");
        return;
    }
    /// Set the SPI bus speed to high (2MHz on APM2)
    _spi_pok->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    return;
}
