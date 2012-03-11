#include "jaguar_broadcaster.h"

namespace jaguar {

JaguarBroadcaster::JaguarBroadcaster(can::CANBridge &can)
	: can_(can)
{
}	

JaguarBroadcaster::~JaguarBroadcaster(void)
{
}

void JaguarBroadcaster::system_reset(void)
{
	broadcast(SystemControl::kSystemReset);
}

void JaguarBroadcaster::system_halt(void)
{
	broadcast(SystemControl::kSystemHalt);
}

void JaguarBroadcaster::system_resume(void)
{
	broadcast(SystemControl::kSystemResume);
}

void JaguarBroadcaster::heartbeat(void)
{
	broadcast(SystemControl::kHeartbeat);
}

void JaguarBroadcaster::device_assignment(uint8_t id)
{
	broadcast(SystemControl::kDeviceAssignment, id);
}

void JaguarBroadcaster::synchronous_update(uint8_t group)
{
	broadcast(SystemControl::kSynchronousUpdate, group);
}

void JaguarBroadcaster::broadcast(SystemControl::Enum api)
{
    uint32_t id = pack_id(0, Manufacturer::kBroadcastMessage, DeviceType::kBroadcastMessage,
    	                  APIClass::kBroadcastMessage, api);
	can_.send(id, NULL, 0);
}

template <typename T>
void JaguarBroadcaster::broadcast(SystemControl::Enum api, T const &payload)
{
    uint32_t id = pack_id(0, Manufacturer::kBroadcastMessage, DeviceType::kBroadcastMessage,
    	                  APIClass::kBroadcastMessage, api);
	can_.send(id, &payload, sizeof payload);
}

};