#include <iostream>
#include <fstream>
#include <boost/lambda/lambda.hpp>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_helper.h>


static uint32_t upd_id(uint16_t api)
{
    return jaguar::pack_id(
            0,
            jaguar::Manufacturer::kTexasInstruments,
            jaguar::DeviceType::kFirmwareUpdate,
            api);
}

__attribute__((__unused__))
static uint32_t upd_ack_id(void)
{
    return jaguar::pack_ack(
            0,
            jaguar::Manufacturer::kTexasInstruments,
            jaguar::DeviceType::kFirmwareUpdate);
}

static void upd_send(can::CANBridge &can, uint16_t api)
{
    can.send(can::CANMessage(upd_id(api)));
}

__attribute__((__unused__))
static void upd_send(can::CANBridge &can, uint16_t api,
    std::vector<uint8_t> const &payload)
{
    can.send(can::CANMessage(upd_id(api), payload));
}

int main(int argc, char *argv[])
{
    try {
        if (argc <= 2) {
            char const *n = argc?argv[0]:"./unbrick";
            std::cerr << "err: lack args\n"
                      << "usage: " << n << " <serial> <fw.bin>" << std::endl;
            return 1;
        }

        std::string const io_path(argv[1]);
        std::string const fw_path(argv[2]);

        can::JaguarBridge can(io_path);
        /* XXX: the hell is this, C++? */
        std::ifstream     fw(fw_path.c_str(), std::ifstream::in);

        can.attach_callback(0, 0, std::cerr << boost::lambda::_1);

        /* send PING */
        can::TokenPtr ping_token = can.recv(upd_id(jaguar::FirmwareUpdate::kPing));

        do {
            upd_send(can, jaguar::FirmwareUpdate::kPing);
            std::cout << "p" << std::endl;
        } while(!ping_token->timed_block(boost::posix_time::millisec(50)));

        /* TODO: set starting address */
        upd_send(can, jaguar::FirmwareUpdate::kDownload);
        /* TODO: ack? */

        /* TODO: send data block */

    } catch (can::CANException &e) {
        std::cerr << "error " << e.code() << ": " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

/* vim: set ts=4 et sts=4 sw=4: */

