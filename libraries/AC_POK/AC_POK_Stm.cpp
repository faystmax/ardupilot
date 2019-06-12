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

void AC_POK_Stm::update(Telem *telem) {

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
	send.send.mode = telem->getMode();
	send.send.roll = ToDeg(telem->getAhrs().roll);
	send.send.pitch = ToDeg(telem->getAhrs().pitch);
	send.send.yaw = ToDeg(telem->getAhrs().yaw);
	send.send.velociry_xy = telem->getInertialNav().get_velocity_xy();
	send.send.latitude = telem->getInertialNav().get_latitude();
	send.send.longitude = telem->getInertialNav().get_longitude();
	send.send.preassure = telem->getBaro().get_pressure();
	send.send.temperature = telem->getBaro().get_temperature();
	send.send.altitude = telem->getBaro().get_altitude();
	send.send.climb_rate = telem->getBaro().get_climb_rate();
	send.send.batteryPct = telem->getBattery().capacity_remaining_pct();
	send.send.crc = calcCRC(&send.send, send_pack_size);

	/// Send to POK
	_spi->transaction((uint8_t *) &send.send, (uint8_t *) &rcv.rcv, message_size);

	expected = calcCRC(&rcv.rcv, receive_pack_size);
	if (expected == rcv.rcv.crc) {
		hal.console->printf("CRC OK and we rcv: %lu %lu %lu %lu %lu %lu\n",
				rcv.rcv.command, rcv.rcv.data, rcv.rcv.code3, rcv.rcv.code4,
				rcv.rcv.code5, rcv.rcv.code6);
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

