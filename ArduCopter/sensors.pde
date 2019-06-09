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

static uint32_t calcSumCRC(void *data, int len) {
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
#define SPI_TRANSACTION_DELAY 1
#define SYNCHRONIZE_BYTE 0x02

AP_HAL::SPIDeviceDriver *_spi_pok;

static int velo = 1; //demo variable to send

static void ReadPOK_Update(void)
{
    uint32_t expected = 0;

    uint16_t max_size;
    struct to_send {
    	uint32_t snc;
        uint32_t len;
        uint32_t velocity;
    	uint32_t gps;
    	uint32_t crc;
    };

    struct to_receive {
    	uint32_t snc;
        uint32_t code1;
        uint32_t code2;
        uint32_t code3;
        uint32_t code4;
        uint32_t code5;
        uint32_t code6;
        uint32_t  crc;
    };

    union send_and_receive {
        struct to_send send;
        struct to_receive rcv;
    };

    union send_and_receive sendme;
    union send_and_receive receiveme;

    uint8_t *uk_send;
    uint8_t *uk_rcv;


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
        //cliSerial->println_P(PSTR("ReadPOK() failed - sem error "));
        //periodically we will get an error, maybe normal
        goto spi_error_retrun;
    }


    hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);

    //generate a demo packet to send
    memset(&sendme.send, 0, sizeof (struct to_send));
    sendme.send.snc = SYNCHRONIZE_BYTE;
    sendme.send.len = 6;
    sendme.send.velocity = velo;
    sendme.send.gps = 555;
    sendme.send.crc = calcSumCRC(&sendme.send, sizeof (struct to_send));
    if(velo>100){
        velo = 0;
    }

    //hal.console->printf("Ardupolot CRC = %lu   %lu \n",  sendme.send.crc, sendme.send.crc);

    max_size = sizeof(union send_and_receive);

    _spi_pok->cs_assert();

    //pointers to send and receive structs
    uk_send = (uint8_t *) &sendme.send;
    uk_rcv = (uint8_t *) &receiveme.rcv;

    //start with a magic number
//    _spi_pok->transfer('S');
//    _spi_pok->transfer('0');
//    _spi_pok->transfer('s');
//    uint8_t tx[3];
//    uint8_t rx[3];
//    tx[0] = 'S';
//    tx[1] = '0';
//    tx[2] = 's';
//    _spi_pok->transaction(tx, rx, 3);


    hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);

    //two - way transfer
//    for (int i = 0; i < max_size; i++) {
//        uk_rcv[i] = _spi_pok->transfer(uk_send[i]);
//        hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);
//    }
    _spi_pok->transaction(uk_send, uk_rcv, max_size);
    hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);

    expected = calcSumCRC(&receiveme.rcv, sizeof (struct to_receive));

    if (receiveme.rcv.crc != 0) {
        //have data, otherwise the other way is sleeping
        if (expected == receiveme.rcv.crc) {
            //correct crc
            hal.console->printf("OK->>");
            hal.console->printf("CRC %lu / %lu ",  expected , receiveme.rcv.crc);
            hal.console->printf("and we rcv: %d %d %d %d %d %d\n",  (int)receiveme.rcv.code1, (int)receiveme.rcv.code2, (int)receiveme.rcv.code3,  (int)receiveme.rcv.code4, (int)receiveme.rcv.code5, (int)receiveme.rcv.code6);
        } else{
            hal.console->printf("FAIL->>");
            hal.console->printf("CRC %lu / %lu \n", expected , receiveme.rcv.crc);
            hal.console->printf("and we rcv: %d %d %d %d %d %d\n",  (int)receiveme.rcv.code1, (int)receiveme.rcv.code2, (int)receiveme.rcv.code3,  (int)receiveme.rcv.code4, (int)receiveme.rcv.code5, (int)receiveme.rcv.code6);
        }
    }
    // release slave select
    _spi_pok->cs_release();


    // release the spi bus
    _spi_sem_pok->give();



    return;

    spi_error_retrun:
    return;

    spi_error_sem_and_cs_retrun:
    // release slave select
    _spi_pok->cs_release();

    // release the spi bus
    _spi_sem_pok->give();

    //cliSerial->println_P(PSTR("ReadRPM() failed - spi Nak"));
}

static void ReadPOK_Init(void)
{
    AP_HAL::Semaphore* _spi_sem_rpm;
    uint8_t resp, read_len;
    int i;

    _spi_pok = hal.spi->device(AP_HAL::SPIDevice_POK);
    if (_spi_pok == NULL)
    {
        cliSerial->println_P(PSTR("ReadPOKInit() failed - device error "));
        return;
    }

    // now that we have initialised, we set the SPI bus speed to high
    // (2MHz on APM2)
    _spi_pok->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);


    cliSerial->println_P(PSTR("Init variables update proc\n"));

    /*
      _spi_pok->cs_assert();

      resp = _spi_pok->transfer(6);
      if(resp != ACK)
      {
    	 cliSerial->println_P(PSTR("no ACK response!\n"));
      }

 	 cliSerial->println_P(PSTR("transfer data!\n"));

      hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);
      _spi_pok->transfer('H');
      _spi_pok->transfer('e');
      _spi_pok->transfer('l');
      _spi_pok->transfer('l');
      _spi_pok->transfer('o');
      _spi_pok->transfer('\n');


      // release slave select

  	 cliSerial->println_P(PSTR("spi release"));

      _spi_pok->cs_release();


  	 cliSerial->println_P(PSTR("give sem!\n"));

      // release the spi bus
      _spi_sem_rpm->give();

*/
    cliSerial->println_P(PSTR("registering timer process\n"));


   // hal.scheduler->register_timer_process((AP_HAL::MemberProc)&ReadPOK_Update);

    return;
}
