#include <iostream>
#include <unistd.h>
#include <sstream>
#include <string>
#include <stdint.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_broadcaster.h>

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
    try {
        if (argc <= 2) {
            std::cerr << "err: incorrect number of arguments\n"
                      << "usage: ./assign_id <path> <device id>"
                      << std::endl;
            return 1;
        }

        std::string const path(argv[1]);
        uint8_t const new_id = convert<uint16_t>(argv[2]);

        can::JaguarBridge can(path);
        jaguar::JaguarBroadcaster broadcaster(can);
        broadcaster.device_assignment(new_id);

        std::cout << "Press the button on the desired Jaguar.\n"
                  << ">>> Waiting... 5" << std::flush;
        for (int i = 4; i > 0; --i) {
            sleep(1);
            std::cout << " " << i << std::flush;
        }

        std::cout << " ...Done." << std::endl;
    } catch (can::CANException &e) {
        std::cerr << "error " << e.code() << ": " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
