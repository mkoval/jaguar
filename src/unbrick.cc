#include <stdint.h>
#include <iostream>
#include <fstream>
#include <boost/assert.hpp>
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

class AckToken {
public:
    AckToken(can::TokenPtr &token_)
    : token(token_)
    {}

    can::TokenPtr &token;

    int get_status(void)
    {
        std::vector<uint8_t> data = token->message()->payload;
        if (data.size() == 1)
            return data[0];
        else
            return -1;
    }

    int status_block(void)
    {
        token->block();
        return get_status();
    }

    int timed_status_block(boost::posix_time::time_duration const &duration)
    {
        token->timed_block(duration);
        return get_status();
    }
};

class Bootloader {
public:
    Bootloader(can::CANBridge &can)
    : can_(can)
    {}

    can::TokenPtr recv(uint16_t api)
    {
        return can_.recv(upd_id(api));
    }

    can::TokenPtr send_ack(uint16_t api,
                           std::vector<uint8_t> const &data,
                           uint16_t ack_api = jaguar::FirmwareUpdate::kAck)
    {
        can::TokenPtr tp = recv(ack_api);
        send(api, data);
        return tp;
    }

    template <typename G>
    can::TokenPtr send_ack(uint16_t api,
                           G generator,
                           uint16_t ack_api = jaguar::FirmwareUpdate::kAck)
    {
        std::vector<uint8_t> obuf;
        std::back_insert_iterator<std::vector<uint8_t> > payload(obuf);
        BOOST_VERIFY(boost::spirit::karma::generate(payload, generator));
        return send_ack(api, obuf, ack_api);
    }

    can::TokenPtr send_ack(uint16_t api)
    {
        return send_ack(api, boost::spirit::karma::eps);
    }

    can::TokenPtr ping(void)
    {
        return send_ack(jaguar::FirmwareUpdate::kPing);
    }

    bool timed_ping(boost::posix_time::time_duration const &duration)
    {
        return send_ack(jaguar::FirmwareUpdate::kPing)->timed_block(duration);
    }

    can::TokenPtr prepare(uint32_t start_addr, uint32_t size)
    {
        return send_ack(jaguar::FirmwareUpdate::kDownload,
                boost::spirit::karma::little_dword(start_addr) <<
                boost::spirit::karma::little_dword(size));
    }

    void wait_for_request(void)
    {
        can_.recv(jaguar::FirmwareUpdate::kRequest)->block();
    }

    can::TokenPtr send_data(std::vector<uint8_t> const &data)
    {
        assert(data.size() <= 8 && data.size() > 0);
        return send_ack(jaguar::FirmwareUpdate::kSendData, data);
    }

    void send(uint16_t api)
    {
        can_.send(can::CANMessage(upd_id(api)));
    }

    void send(uint16_t api, std::vector<uint8_t> const &payload)
    {
        can_.send(can::CANMessage(upd_id(api), payload));
    }

private:
    can::CANBridge &can_;
};

int main(int argc, char *argv[])
{
    try {
        if (argc <= 3) {
            char const *n = argc?argv[0]:"./unbrick";
            std::cerr << "err: lack args\n"
                      << "usage: " << n << " <serial> <fw.bin> <start addr>"
                        << std::endl;
            return 1;
        }

        std::string const io_path(argv[1]);
        std::string const fw_path(argv[2]);

        //std::stringstream sa_stm;
        uint32_t fw_start;
        //sa_stm << argv[3];

        /* FIXME: */
        //sa_stm >> std::hex >> fw_start;
        fw_start = 0x800;

        can::JaguarBridge     can(io_path);
        Bootloader bl(can);

        /* XXX: the hell is this, C++? */
        std::ifstream     fw_stream(fw_path.c_str());
        std::stringstream fw_buf;
        fw_buf << fw_stream.rdbuf();
        std::string const fw(fw_buf.str());

        using boost::phoenix::arg_names::arg1;
        using boost::phoenix::arg_names::arg2;
        using boost::phoenix::arg_names::arg3;
        using boost::phoenix::arg_names::arg4;

        /* XXX: spy on all recv'd data */
        can.attach_callback(0, 0, std::cerr << arg1);
        can.attach_callback(std::cerr
                << arg1 << ":" << arg2 << ":" << arg3 << ":" << arg4);

        /* wait for Request & ping ack */
        can::TokenPtr req_token  = bl.recv(jaguar::FirmwareUpdate::kRequest);

        std::cout << "waiting for request." << std::endl;
        req_token->block();

        std::cout << "recv'd req, pinging." << std::endl;

        do {
            std::cout << 'p' << std::flush;
        } while (bl.timed_ping(boost::posix_time::millisec(100)));

        std::cout << std::endl << "pinged, waiting for ack." << std::endl;

        std::cout << "recv'd ack." << std::endl;

        /* set starting address and length */
        can::TokenPtr ack = bl.prepare(fw_start, fw.size());

        std::cout << 's' << std::flush;

        /* TODO: examine ack */
        ack->block();
        if (ack->message()->payload[0] == 1) {
            std::cout << std::endl << "Prepare failed" << std::endl;
        }

        std::cout << 'a' << std::flush;

        /* send data block */
        std::vector<uint8_t> data;
        BOOST_FOREACH(uint8_t byte, fw) {
            if (data.size() == 8) {
                /* send `data` and reset */
                ack = bl.send_data(data);

                std::cout << 'd' << std::flush;

                /* TODO: look at ack payload (== status) */
                ack->block();
                if (ack->message()->payload[0] == 1) {
                    std::cout << std::endl << "send data failed" << std::endl;
                }

                std::cout << 'a' << std::flush;

                data.clear();
            }

            data.push_back(byte);
        }

        if (!data.empty()) {
            ack = bl.send_data(data);
            ack->block();
            if (ack->message()->payload[0] == 1) {
                std::cout << std::endl << "send data failed" << std::endl;
            }
        }

        std::cout << std::endl << "Programming complete" << std::endl;

    } catch (can::CANException &e) {
        std::cerr << "error " << e.code() << ": " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

/* vim: set ts=4 et sts=4 sw=4: */

