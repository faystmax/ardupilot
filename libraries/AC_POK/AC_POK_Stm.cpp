/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AC_POK.h"
#include "AC_POK_Stm.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

uint32_t AC_POK_Stm::_timer;

/// Public Methods
bool AC_POK_Stm::init() {

	_spi = hal.spi->device(AP_HAL::SPIDevice_POK);
	if (_spi == NULL) {
		hal.scheduler->panic(
				PSTR("PANIC: AP_Pok_Stm did not get valid SPI device driver!"));
		return false; /* never reached */
	}
	_spi_sem = _spi->get_semaphore();
	if (_spi_sem == NULL) {
		hal.scheduler->panic(
				PSTR("PANIC: AP_Pok_Stm did not get valid SPI semaphroe!"));
		return false; /* never reached */
	}

	/// Set the SPI bus speed to high (2MHz on APM2)
	_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
	return true;
}

void AC_POK_Stm::update(struct local_data &d, Telem &telem) {

	uint32_t tnow = hal.scheduler->micros();
	// Read rate to 100hz maximum.
	if (tnow - _timer < 10000) {
		return;
	}
	_timer = tnow;

	if (!_spi_sem->take_nonblocking()) {
		hal.console->printf("Update POK failed - sem error ");
		return;
	}
	_timer = tnow;

	uint32_t expected = 0;
	union message send;
	union message rcv;

	/// Preparing data to send
	memset(&send.send, 0, send_pack_size);
	send.send.snc = SYNCHRONIZE_BYTE;
	send.send.mode = d.mode;
	send.send.armed = 1/*d.armed*/;
	send.send.rc1_in = 522/*d.rc1_in*/;
	send.send.rc2_in = 200/*d.rc2_in*/;
	send.send.rc3_in = 320/*d.rc3_in*/;
	send.send.rc4_in = 400/*d.rc4_in*/;
	send.send.limit_roll_pitch = d.limit_roll_pitch;         // we have reached roll or pitch limit
	send.send.limit_yaw = d.limit_yaw;                       // we have reached yaw limit
	send.send.limit_throttle_lower = d.limit_throttle_lower; // we have reached throttle's lower limit
	send.send.limit_throttle_upper = d.limit_throttle_upper; // we have reached throttle's upper limit
	send.send.roll_sensor = telem.getAhrs().roll_sensor;
    send.send.pitch_sensor = telem.getAhrs().pitch_sensor;
    send.send.yaw_sensor = telem.getAhrs().yaw_sensor;
    send.send.throttle_min = d.throttle_min;
    send.send.throttle_max = d.throttle_max;
	send.send.gyro_x = telem.getAhrs().get_gyro().x;
	send.send.gyro_y = telem.getAhrs().get_gyro().y;
	send.send.gyro_z = telem.getAhrs().get_gyro().z;
	send.send.cos_roll = telem.getAhrs().cos_roll();
	send.send.cos_pitch = telem.getAhrs().cos_pitch();
	send.send.cos_yaw = telem.getAhrs().cos_yaw();
	send.send.sin_roll = telem.getAhrs().sin_roll();
	send.send.sin_pitch = telem.getAhrs().sin_pitch();
	send.send.sin_yaw = telem.getAhrs().sin_yaw();
	send.send.roll = ToDeg(telem.getAhrs().roll);
	send.send.pitch = ToDeg(telem.getAhrs().pitch);
	send.send.yaw = ToDeg(telem.getAhrs().yaw);
	send.send.velociry_xy = telem.getInertialNav().get_velocity_xy();
	send.send.latitude = telem.getInertialNav().get_latitude();
	send.send.longitude = telem.getInertialNav().get_longitude();
	send.send.preassure = telem.getBaro().get_pressure();
	send.send.temperature = telem.getBaro().get_temperature();
	send.send.altitude = telem.getBaro().get_altitude();
	send.send.climb_rate = telem.getBaro().get_climb_rate();
	send.send.batteryPct = 15/*telem.getBattery().capacity_remaining_pct()*/;
	send.send.crc = calcCRC(&send.send, send_pack_size);

	/// Send to POK
	_spi->transaction((uint8_t *) &send.send, (uint8_t *) &rcv.rcv, message_size);

	expected = calcCRC(&rcv.rcv, receive_pack_size);
	if (expected == rcv.rcv.crc) {
		hal.console->printf("CRC OK and we rcv: command=%lu %lu %lu %lu %lu\n",
				rcv.rcv.command, rcv.rcv.motor_pitch, rcv.rcv.motor_roll, rcv.rcv.motor_yaw, rcv.rcv.motor_throttle);
		_last_rcv = rcv.rcv;
		_last_state = TRANSFER_OK;
	} else {
		hal.console->printf("CRC not valid %lu / %lu\n", expected, rcv.rcv.crc);
		_last_state = TRANSFER_ERROR;
	}
	_spi_sem->give();
	return;
}

/// Private Methods
uint32_t AC_POK_Stm::calcCRC(void *data, int len) {
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

