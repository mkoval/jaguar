#include <iostream>
#include <unistd.h>
#include <sstream>
#include <string>
#include <stdint.h>
#include "jaguar.h"
#include "jaguar_bridge.h"

template <typename T>
static T convert(std::string str)
{
	std::stringstream ss(str);
	T x;
	ss >> x;
	return x;
}

int main(int argc, char *argv[])
{
    if (argc <= 2) {
        std::cerr << "err: incorrect number of arguments\n"
                  << "usage: ./assign_id <port> <device id>"
                  << std::endl;
        return 1;
    }

    std::string const port = argv[1];
    uint8_t const new_id = convert<uint16_t>(argv[2]);

    try {
        can::JaguarBridge can(port);
        can::Jaguar jaguar(can, 5);
        jaguar.set_voltage();
#if 0
        jaguar.device_assignment(new_id);

        std::cout << "Press the button on the desired Jaguar.\n"
                  << ">>> Waiting... 5" << std::flush;
        for (int i = 4; i > 0; --i) {
            sleep(1);
            std::cout << " " << i << std::flush;
        }

        std::cout << " ...Done." << std::endl;
#endif
        return 0;
    } catch (can::CANException const &e) {
        std::cerr << "err: " << e.what() << std::endl;
        return 1;
    }
}
