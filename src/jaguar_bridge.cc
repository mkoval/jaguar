#include <cassert>
#include "jaguar_bridge.h"

#include <iostream>

namespace asio = boost::asio;

namespace can {

uint8_t const JaguarBridge::kSOF = 0xFF;
uint8_t const JaguarBridge::kESC = 0xFE;
uint8_t const JaguarBridge::kSOFESC = 0xFE;
uint8_t const JaguarBridge::kESCESC = 0xFD;
size_t const JaguarBridge::kReceiveBufferLength = 1024;
size_t const JaguarBridge::kReceiveQueueLength  = 16;

JaguarBridge::JaguarBridge(std::string port)
    : serial_(io_, port),
      recv_queue_(kReceiveQueueLength),
      recv_buffer_(kReceiveBufferLength),
      state_(kWaiting),
      length_(0),
      escape_(false)
{
    using asio::serial_port_base;

    serial_.set_option(serial_port_base::baud_rate(115200u));
    serial_.set_option(serial_port_base::character_size(8));
    serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

    // Four byte ID plus at most eight bytes of payload.
    packet_.reserve(12);

    // Schedule an asynchronous read. This will persist for the entire
    // lifetime of the program.
    serial_.async_read_some(asio::buffer(recv_buffer_),
        boost::bind(&JaguarBridge::recv_handle, this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred
        )
    );
    recv_thread_ = boost::thread(boost::bind(&asio::io_service::run, &io_));
}

JaguarBridge::~JaguarBridge(void)
{
    serial_.cancel();
    recv_thread_.join();
    serial_.close();
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

    buffer.push_back(kSOF);
    buffer.push_back(length + 4);
    encode_bytes(id_conversion.bytes, 4, buffer);
    encode_bytes(static_cast<uint8_t const *>(data), length, buffer);

    asio::write(serial_, asio::buffer(&buffer[0], buffer.size()));
}

void JaguarBridge::recv_block(uint32_t id, void *data, size_t length)
{
    // FIXME: not implemented
}

void JaguarBridge::recv_async(uint32_t id, recv_callback cb)
{
    // FIXME: not implemented
}

boost::optional<CANMessage> JaguarBridge::recv_byte(uint8_t byte)
{
    // Due to escaping, the SOF byte only appears at frame starts.
    if (byte == kSOF) {
        state_  = kLength;
        length_ = 0;
        escape_ = 0;
        packet_.clear();
    }
    // Packet length can never be SOF or ESC, so we can ignore escaping.
    else if (state_ == kLength) {
        assert(4 <= byte && byte <= 12);
        state_  = kPayload;
        length_ = byte - 4;
    }
    // This is the second byte in a two-byte escape code.
    else if (state_ == kPayload && escape_) {
        switch (byte) {
        case kSOFESC:
            packet_.push_back(kSOF);
            break;

        case kESCESC:
            packet_.push_back(kESC);
            break;

        default:
            // TODO: Print a warning because this should never happen.
            state_ = kWaiting;
        }
        escape_ = false;
    }
    // Escape character, so the next byte has special meaning.
    else if (state_ == kPayload && byte == kESC) {
        escape_ = true;
    }
    // Normal data.
    else {
        packet_.push_back(byte);
    }

    // Emit a packet as soon as it is finished.
    boost::optional<CANMessage> packet = boost::optional<CANMessage>();

    if (state_ == kPayload && packet_.size() >= length_) {
        CANMessage msg = unpack_packet(packet_);
        packet = boost::optional<CANMessage>(msg);

        state_  = kWaiting;
        length_ = 0;
        escape_ = 0;
        packet_.clear();
    }

    return packet;
}

void JaguarBridge::recv_handle(boost::system::error_code const& error, size_t count)
{
    if (error == boost::system::errc::success) {
        for (size_t i = 0; i < count; ++i) {
            boost::optional<CANMessage> msg = recv_byte(recv_buffer_[i]);
            if (msg) {
                // FIXME: This isn't thread safe.
                recv_queue_.push_back(*msg);
            }
        }
    } else {
        // TODO: Log an error message.
    }

    // Start the next asynchronous read. This chaining is necessary to avoid
    // explicit threading.
    // TODO: Do I need to check for an abort error code?
    serial_.async_read_some(asio::buffer(recv_buffer_),
        boost::bind(&JaguarBridge::recv_handle, this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred
        )
    );
}

CANMessage JaguarBridge::unpack_packet(std::vector<uint8_t> const &packet)
{
    assert(4 <= packet.size() && packet.size() <= 12);

    uint32_t id;
    memcpy(&id, &packet[0], sizeof(uint32_t));
    id = le32toh(id);

    std::vector<uint8_t> payload(packet.size() - 4);
    if (packet.size() > 4) {
        memcpy(&payload[0], &packet[4], packet.size() - 4);
    }
    return std::make_pair(id, payload);
}

size_t JaguarBridge::encode_bytes(uint8_t const *bytes, size_t length, std::vector<uint8_t> &buffer)
{
    size_t emitted = 0;

    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = bytes[i];
        switch (byte) {
        case kSOF:
            buffer.push_back(kESC);
            buffer.push_back(kSOFESC);
            emitted += 2;
            break;

        case kESC:
            buffer.push_back(kESC);
            buffer.push_back(kESCESC);
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
