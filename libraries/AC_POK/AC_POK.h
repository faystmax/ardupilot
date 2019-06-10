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
    uint32_t len;
    uint32_t velocity;
    float roll;
    float pitch;
	float yaw;
	uint32_t crc;
};

struct receive_pack {
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
	struct receive_pack rcv;
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
	virtual void update(Telem *telem) = 0;

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
