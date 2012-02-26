#include <cassert>
#include <endian.h>
#include <vector>
#include "jaguar_bridge.h"

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
    buffer.push_back(2 + length);
    encode_bytes(id_conversion.bytes, 4, buffer);
    encode_bytes(static_cast<uint8_t const *>(data), length, buffer);

    asio::write(m_serial, asio::buffer(buffer));
}

uint32_t JaguarBridge::recv(void *data, size_t length)
{
    // See JaguarBridge::send() for a detailed explaination of this capacity.
    std::vector<uint8_t> buffer;
    buffer.reserve(26);

    // TODO
    return 0;
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

#if 0
static uint8_t decode_byte(boost::asio::SyncReadStream stream, uint8_t *)
{
    while (read < length) {
        uint8_t byte;
        asio::read(m_serial, asio::buffer(&byte, 1));

        // Start a new packet whenever we receive a SOF byte. This is necessary
        // for resynchronization and is possible because of escaping.
        if (byte == m_sof) {
            state    = kHeader;
            consumed = 0;
        } else if (state == kLength) {
        } else if (state == kIdentifier) {
            packet_length <<= 8;
            packet_length  |= byte;
            ++consumed;

            if (consumed == 4) {
                state = kIdentifier;
            }
        } else if (state == kPayload) {
            // TODO: read (length - 4) decoded bytes
        }
    }
}
#endif

};
