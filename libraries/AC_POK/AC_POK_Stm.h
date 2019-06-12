/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_POK__STM_H__
#define __AP_POK__STM_H__

#include "AC_POK.h"

class AC_POK_Stm: public AC_POK {
public:
	/// initialize connection
	bool init();

	/// send data to POK and receive commands
	void update(struct local_data &d, Telem &telem);

private:
	static uint32_t _timer;

	/// Calculate CRC
	uint32_t calcCRC(void *data, int len);
};

#endif //  __AP_POK__STM_H__
