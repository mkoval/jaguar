#include <boost/thread.hpp>


class JaguarReceiveThread {
public:	
	void operator()(void);
}

class JaguarReceiver {
public:	
	JaguarReceiver(can::CANBridge &can);
	~JaguarReceiver(void);

	void get_ack(uint8_t num, Manufacturer::Enum man, DeviceType::Enum type);

private:
	can::CANBridge &can_;
};

void JaguarReceiver::get_ack(uint8_t num, Manufacturer::Enum man, DeviceType::Enum type)
{

}