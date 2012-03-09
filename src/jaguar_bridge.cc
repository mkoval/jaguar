#include <cassert>
#include <endian.h>
#include <vector>
#include "jaguar_bridge.h"

#include <iomanip>
#include <iostream>

namespace asio = boost::asio;

namespace can {

uint8_t const JaguarBridge::m_sof = 0xFF;
uint8_t const JaguarBridge::m_esc = 0xFE;
uint8_t const JaguarBridge::m_sof_esc = 0xFE;
uint8_t const JaguarBridge::m_esc_esc = 0xFD;


JaguarBridge::JaguarBridge(std::string port)
    : m_serial(m_io, port), m_last(0)
{
    using asio::serial_port_base;

    m_serial.set_option(serial_port_base::baud_rate(115200u));
    m_serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    m_serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
}

JaguarBridge::~JaguarBridge(void)
{
    m_serial.close();
}

void JaguarBridge::send(uint32_t id, void const *data, size_t length)
{
    assert(length <= 8);
    assert((id & 0xE0000000) == 0);

    // Each message consists of two bytes of framing, a 29-bit CAN identifier
    // packed into four bytes, and a maximum of eight bytes of data. All of
    // these, except the start of frame byte, may need to be escaped. In all,
    // this is: 2 + (4 + 8)*2 = 26 bytes.
    std::vector<uint8_t> buffer;
    buffer.reserve(26);

    // 29-bit CAN id encoded as a 32-bit integer. Note the Endian-ness
    // conversion because the integer is being treated as an array of bytes.
    union {
        uint32_t id;
        uint8_t  bytes[4];
    } id_conversion = { htole32(id) };

    buffer.push_back(m_sof);
    buffer.push_back(length + 4);
    encode_bytes(id_conversion.bytes, 4, buffer);
    encode_bytes(static_cast<uint8_t const *>(data), length, buffer);

    std::cout << "send: " << std::hex << std::setfill('0');
    for (size_t i = 0; i < buffer.size(); ++i) {
        std::cout << std::setw(2) << (int)buffer[i];
    }
    std::cout << std::endl;

    asio::write(m_serial, asio::buffer(&buffer[0], buffer.size()));
}

void JaguarBridge::recv_test(void)
{
    for (;;) {
        uint8_t byte;
        asio::read(m_serial, asio::buffer(&byte, 1));
        std::cout << "recv: "
                  << std::hex << std::setfill('0') << std::setw(2)
                  << (int)byte
                  << std::endl;
    }
}

uint32_t JaguarBridge::recv(void *data, size_t length)
{

    // Four bytes for the CAN id plus a payload of at most eight bytes.
    std::vector<uint8_t> buffer;
    buffer.reserve(12);

    enum {
        kWaiting,
        kLength,
        kPayload,
        kComplete
    } state = kWaiting;
    size_t count  = 0;
    bool   escape = false;

    while (state != kComplete) {
        uint8_t byte;
        asio::read(m_serial, asio::buffer(&byte, 1));

        std::cout << "recv: "
                  << std::hex << std::setfill('0') << std::setw(2)
                  << (int)byte
                  << std::endl;

        // Due to escaping, the SOF byte only appears at frame starts.
        if (byte == m_sof) {
            state = kLength;
            count = 0;
        }
        // Packet length can never be SOF or ESC, so we can ignore escaping.
        else if (state == kLength) {
            assert(byte >= 4);
            state = kPayload;
            count = byte - 4;
        }
        // This is the second byte in a two-byte escape code.
        else if (state == kPayload && escape) {
            switch (byte) {
            case m_sof_esc:
                buffer.push_back(m_sof);
                break;

            case m_esc_esc:
                buffer.push_back(m_esc);
                break;

            default:
                // TODO: Print a warning because this should never happen.
                state = kWaiting;
            }
            escape = false;
        }
        // Escape character, so the next byte has special meaning.
        else if (state == kPayload && byte == m_esc) {
            escape = true;
        }
        // Normal data.
        else {
            buffer.push_back(byte);
        }

        if (state == kPayload && buffer.size() >= count) {
            state = kComplete;
        }
    }

    // Fix the endian-ness on the CAN id.
    uint32_t id;
    memcpy(&id, &buffer[0], 4);
    id = le32toh(id);

    assert(count <= length);
    if (length > 0) {
        memcpy(data, &buffer[4], count - 4);
    }
    return id;
}

size_t JaguarBridge::encode_bytes(uint8_t const *bytes, size_t length, std::vector<uint8_t> &buffer)
{
    size_t emitted = 0;

    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = bytes[i];
        switch (byte) {
        case m_sof:
            buffer.push_back(m_esc);
            buffer.push_back(m_sof_esc);
            emitted += 2;
            break;

        case m_esc:
            buffer.push_back(m_esc);
            buffer.push_back(m_esc_esc);
            emitted += 2;
            break;

        default:
            buffer.push_back(byte);
            ++emitted;
        }
    }

    return emitted;
}

};
