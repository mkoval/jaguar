#include <stdint.h>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
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
        return ping()->timed_block(duration);
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

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    uint32_t fw_start;
    std::string io_path;
    std::string fw_path;
    bool wait_for_req;
    bool help;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("serial port,s",   po::value<std::string>(&io_path)->required(),
            "serial port the Jaguar is connected to")
        ("firmware,f",      po::value<std::string>(&fw_path)->required(),
            "firmware binary to flash")
        ("wait for req,w",  po::value<bool>(&wait_for_req)->zero_tokens(),
            "wait for a request for a firmware update")
        ("start address,a", po::value<uint32_t>(&fw_start)->default_value(0x800),
            "set the firmware start address")
        ("help", po::value<bool>(&help)->zero_tokens(),
            "show this message")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (help) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    try {
        can::JaguarBridge     can(io_path);
        Bootloader bl(can);

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
        if (wait_for_req) {
            can::TokenPtr req_token  = bl.recv(jaguar::FirmwareUpdate::kRequest);

            std::cout << "waiting for request." << std::endl;
            req_token->block();

            std::cout << "recv'd req." << std::endl;
        }

        std::cout << "pinging." << std::endl;

        do {
            std::cout << 'p' << std::flush;
        } while (bl.timed_ping(boost::posix_time::millisec(100)));

        std::cout << std::endl << "recv'd ack."     << std::endl;
        std::cout              << "sending prepare" << std::endl;

        /* set starting address and length */
        can::TokenPtr ack = bl.prepare(fw_start, fw.size());

        ack->block();
        if (ack->message()->payload[0] == 1) {
            std::cout << "Prepare failed" << std::endl;
        }

        std::cout << "prepare acked." << std::endl;

        std::cout << 'a' << std::flush;

        /* send data block */
        std::vector<uint8_t> data;
        BOOST_FOREACH(uint8_t byte, fw) {
            if (data.size() == 8) {
                /* send `data` and reset */
                ack = bl.send_data(data);

                std::cout << 'd' << std::flush;

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

