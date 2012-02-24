#include <iostream>
#include <unistd.h>
#include <sstream>
#include <string>
#include <stdint.h>
#include "jaguar.h"
#include "ntcan_bridge.h"

namespace can {

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
                  << "usage: ./assign_id <network> <device id>"
                  << std::endl;
        return 1;
    }

	int net = convert<int>(argv[1]);
    uint16_t new_id = convert<uint16_t>(argv[2]);

    try {
        NTCANBridge can(20);
        Jaguar jaguar(can, 0);
        jaguar.device_assignment(new_id);

        std::cout << "Press the button on the desired Jaguar.\n"
                  << ">>> Waiting... 5" << std::flush;
        for (int i = 4; i > 0; i++) {
            sleep(1);
            std::cout << " " << i << std::flush;
        }

        std::cout << " ...Done." << std::endl;
        return 0;
    } catch (CANException const &e) {
        std::cerr << "err: " << e.what() << std::endl;
        return 1;
    }
}

};
