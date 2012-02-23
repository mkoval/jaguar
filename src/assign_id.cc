#include <unistd.h>
#include "jaguar.h"
#include "ntcan_bridge.h"

namespace can {

int main(int argc, char *argv[])
{
    try {
        NTCANBridge can;
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
