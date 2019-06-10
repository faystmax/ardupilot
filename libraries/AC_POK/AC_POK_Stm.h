/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_POK__STM_H__
#define __AP_POK__STM_H__

#include "AP_POK.h"

class AC_POK_Stm: public AP_POK {
public:
	/// initialize connection
	bool init();

	/// send data to POK and recive commands
	TRANSFER_Status update(Telem *telem);

private:
	static uint32_t _timer;
	uint32_t _last_update;

	/// Calculate CRC
	uint32_t calcCRC(void *data, int len);
};

#endif //  __AP_POK__STM_H__
