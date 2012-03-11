#ifndef JAGUAR_BROADCASTER_H_
#define JAGUAR_BROADCASTER_H_

#include "can_bridge.h"
#include "jaguar_api.h"
#include "jaguar_helper.h"

namespace jaguar {

class JaguarBroadcaster {
public:
	JaguarBroadcaster(can::CANBridge &can);
	virtual ~JaguarBroadcaster(void);
	void system_reset(void);
	void system_halt(void);
	void system_resume(void);
	void heartbeat(void);
	void device_assignment(uint8_t id);
	void synchronous_update(uint8_t group);

private:
	can::CANBridge &can_;	

	void broadcast(SystemControl::Enum api);
	template <typename T>
	void broadcast(SystemControl::Enum api, T const &payload);
};

};

#endif