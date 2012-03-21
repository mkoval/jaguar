#include <stdint.h>
#include <iostream>
#include <fstream>
#include <boost/assert.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/spirit/include/karma.hpp>
#include <boost/foreach.hpp>
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

static void upd_send(can::CANBridge &can, uint16_t api)
{
    can.send(can::CANMessage(upd_id(api)));
}

static void upd_send(can::CANBridge &can, uint16_t api,
    std::vector<uint8_t> const &payload)
{
    can.send(can::CANMessage(upd_id(api), payload));
}


static can::TokenPtr send_ack(can::CANBridge &can,
        uint16_t api,
        std::vector<uint8_t> const &data,
        uint16_t ack_api = jaguar::FirmwareUpdate::kAck)
{
    can::TokenPtr tp = can.recv(upd_id(ack_api));
    upd_send(can, api, data);
    return tp;
}

template <typename G>
static can::TokenPtr send_ack(can::CANBridge &can,
        uint16_t api,
        G generator,
        uint16_t ack_api = jaguar::FirmwareUpdate::kAck)
{
    std::vector<uint8_t> obuf;
    BOOST_VERIFY(boost::spirit::karma::generate(obuf.end(), generator));
    return send_ack(can, api, obuf, ack_api);
}

#if 0
class JaguarBootloader
{
public:
    JaguarBootloader(can::CANBridge &can)
    : can_(can)
    {}

    can::TokenPtr ping(void);
    can::TokenPtr prepare(uint32_t len, uint32_t start_addr);
    can::TokenPtr send_data(std::vector<uint8_t> const &data);

    can::TokenPtr send_ack(uint16_t api, std::vector<uint8_t> const &data,
                           uint16_t ack_api = jaguar::FirmwareUpdate::kAck)
    {
    }

    template <typename G>
    can::TokenPtr send_ack(uint16_t api, G generator,
                           uint16_t ack_api = jaguar::FirmwareUpdate::kAck)
    {
    }


private:
    can::CANBridge &can_;
}
#endif

int main(int argc, char *argv[])
{
    try {
        if (argc <= 3) {
            char const *n = argc?argv[0]:"./unbrick";
            std::cerr << "err: lack args\n"
                      << "usage: " << n << " <serial> <fw.bin> <start addr>" << std::endl;
            return 1;
        }

        std::string const io_path(argv[1]);
        std::string const fw_path(argv[2]);
        std::stringstream sa_stm;
        uint32_t fw_start;
        sa_stm << argv[3];
        sa_stm >> fw_start;

        can::JaguarBridge can(io_path);
        /* XXX: the hell is this, C++? */
        std::ifstream     fw_stream(fw_path.c_str());
        std::stringstream fw_buf;
        fw_buf << fw_stream.rdbuf();
        std::string const fw(fw_buf.str());

        can.attach_callback(0, 0, std::cerr << boost::lambda::_1);

        /* send PING */
        can::TokenPtr ping_token = can.recv(upd_id(jaguar::FirmwareUpdate::kPing));

        do {
            upd_send(can, jaguar::FirmwareUpdate::kPing);
            std::cout << "p" << std::endl;
        } while(!ping_token->timed_block(boost::posix_time::millisec(50)));

        /* set starting address and length */
        can::TokenPtr ack = send_ack(can, jaguar::FirmwareUpdate::kDownload,
                boost::spirit::karma::little_dword(fw_start) <<
                boost::spirit::karma::little_dword(fw.size()));

        std::cout << 's';

        /* TODO: examine ack */
        ack->block();

        std::cout << 'a';

        /* send data block */
        std::vector<uint8_t> data;
        BOOST_FOREACH(uint8_t byte, fw) {
            if (data.size() == 8) {
                /* send `data` and reset */
                ack = send_ack(can, jaguar::FirmwareUpdate::kSendData,
                        data);

                std::cout << 'd';

                /* TODO: look at ack payload (== status) */
                ack->block();

                std::cout << 'a';

                data.clear();
            }

            data.push_back(byte);
        }

        if (!data.empty()) {
            ack = send_ack(can, jaguar::FirmwareUpdate::kSendData, data);
            ack->block();
        }

        std::cout << std::endl << "Programming complete" << std::endl;

    } catch (can::CANException &e) {
        std::cerr << "error " << e.code() << ": " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

/* vim: set ts=4 et sts=4 sw=4: */

