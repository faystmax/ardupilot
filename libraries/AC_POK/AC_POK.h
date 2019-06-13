/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AC_POK_H__
#define __AC_POK_H__

#define SYNCHRONIZE_BYTE 0x02

#include <AP_Param.h>
#include <AC_Telemetry.h>

typedef enum  {
  TRANSFER_OK,
  TRANSFER_ERROR
} TRANSFER_Status;

/// Exchange Structures
struct send_pack {
	uint32_t snc;
	uint32_t mode;
	uint32_t armed;
	int32_t  rc1_in;
	int32_t  rc2_in;
	int32_t  rc3_in;
	int32_t  rc4_in;
    uint8_t limit_roll_pitch;     // we have reached roll or pitch limit
    uint8_t limit_yaw;            // we have reached yaw limit
    uint8_t limit_throttle_lower; // we have reached throttle's lower limit
    uint8_t limit_throttle_upper; // we have reached throttle's upper limit
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;
    int32_t throttle_min;
    int32_t throttle_max;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float cos_roll;
    float cos_pitch;
    float cos_yaw;
    float sin_roll;
    float sin_pitch;
    float sin_yaw;
    float roll;
    float pitch;
	float yaw;
	float velociry_xy;   // current horizontal velocity in cm/s
	uint32_t latitude;   // latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
	uint32_t longitude;  // longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
	float preassure;     // pressure in Pascal. Divide by 100 for millibars or hectopascals
	float temperature;   // temperature in degrees C
	float altitude;      // get current altitude in meters relative to altitude at the time of the last calibrate() in baro call
	float climb_rate;    // get current climb rate in meters/s. A positive number means going up
	uint32_t batteryPct; // returns the % battery capacity remaining (0 ~ 100)
	uint32_t crc;
};

struct receive_pack {
	uint32_t snc;
	uint32_t command;
	uint32_t motor_pitch;
	uint32_t motor_roll;
	uint32_t motor_yaw;
	uint32_t motor_throttle;
    uint32_t  crc;
};

union message {
	struct send_pack send;
	struct receive_pack rcv;
};

// Struct for data from Parameters.h to send
struct local_data {
	uint32_t mode;
	uint32_t armed;
	int32_t  rc1_in;
	int32_t  rc2_in;
	int32_t  rc3_in;
	int32_t  rc4_in;
    uint8_t limit_roll_pitch;
    uint8_t limit_yaw;
    uint8_t limit_throttle_lower;
    uint8_t limit_throttle_upper;
    int32_t throttle_min;
    int32_t throttle_max;
};

class AC_POK {
public:
	AC_POK() {
		message_size = sizeof(union message);
		send_pack_size = sizeof(struct send_pack);
		receive_pack_size = sizeof(struct receive_pack);
	}

	// initialize connection
	virtual bool init() = 0;

	// send data to POK and recive commands
	virtual void update(struct local_data &d, Telem &telem) = 0;

	TRANSFER_Status getLastState() { return _last_state; }

protected:
	uint16_t message_size;
	uint16_t send_pack_size;
	uint16_t receive_pack_size;

	TRANSFER_Status _last_state;
	AP_HAL::Semaphore *_spi_sem;
	AP_HAL::SPIDeviceDriver *_spi;
};

#include "AC_POK_Stm.h"

#endif // __AC_POK_H__
