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

JaguarBridge::JaguarBridge(std::string port)
    : serial_(io_, port),
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

void JaguarBridge::send(CANMessage const &message)
{
    assert(message.payload.size() <= 8);
    assert((message.id & 0xE0000000) == 0);

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
    } id_conversion = { htole32(message.id) };

    buffer.push_back(kSOF);
    buffer.push_back(message.payload.size() + 4);
    encode_bytes(id_conversion.bytes, 4, buffer);
    encode_bytes(&message.payload[0], message.payload.size(), buffer);

    asio::write(serial_, asio::buffer(&buffer[0], buffer.size()));
}

TokenPtr JaguarBridge::recv(uint32_t id)
{
    // We can't use boost::make_shared because JaguarToken's constructor is
    // private, so we can only call it from a friend class.
    std::pair<token_table::iterator, bool> it = tokens_.insert(
        std::make_pair(id, boost::shared_ptr<JaguarToken>(new JaguarToken()))
    );
    assert(it.second);
    return it.first->second;
}

void JaguarBridge::attach_callback(uint32_t id, recv_callback cb)
{
    boost::mutex::scoped_lock lock(callback_mutex_);

    // Calling map::insert() is equivalent to map::find() if the key already
    // exists; i.e. the map is not changed in any way.
    std::pair<callback_table::iterator, bool> old_callback = callbacks_.insert(
        std::make_pair(
            id,
            boost::make_shared<callback_signal>()
        )
    );

    callback_signal_ptr signal = old_callback.first->second;
    signal->connect(cb);
}

boost::shared_ptr<CANMessage> JaguarBridge::recv_byte(uint8_t byte)
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
        length_ = byte;
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
    else if (state_ == kPayload) {
        packet_.push_back(byte);
    }

    // Emit a packet as soon as it is finished.
    boost::shared_ptr<CANMessage> message;

    if (state_ == kPayload && packet_.size() >= length_) {
        message = unpack_packet(packet_);
        state_  = kWaiting;
        length_ = 0;
        escape_ = 0;
        packet_.clear();
    }
    return message;
}

void JaguarBridge::recv_handle(boost::system::error_code const& error, size_t count)
{
    if (error == boost::system::errc::success) {
        for (size_t i = 0; i < count; ++i) {
            boost::shared_ptr<CANMessage> msg = recv_byte(recv_buffer_[i]);
            if (msg) {
                recv_message(msg);
            }
        }
    } else if (error == asio::error::operation_aborted) {
        return;
    } else {
        // TODO: properly log this error message
        std::cerr << "error: " << error.message() << std::endl;
        return;
    }

    // Start the next asynchronous read. This chaining is necessary to avoid
    // explicit threading.
    serial_.async_read_some(asio::buffer(recv_buffer_),
        boost::bind(&JaguarBridge::recv_handle, this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred
        )
    );
}

void JaguarBridge::recv_message(boost::shared_ptr<CANMessage> msg)
{
    boost::mutex::scoped_lock lock(callback_mutex_);

    // Invoke callbacks registered to this CAN identifier.
    callback_table::iterator callback_it = callbacks_.find(msg->id);
    if (callback_it != callbacks_.end()) {
        (*callback_it->second)(msg);
    }    

    // Wake anyone who is blocking for a response.
    token_table::iterator token_it = tokens_.find(msg->id);
    if (token_it != tokens_.end()) {
        token_ptr token = token_it->second;
        token->unblock(msg);
        tokens_.erase(token_it);
    }
}

boost::shared_ptr<CANMessage> JaguarBridge::unpack_packet(std::vector<uint8_t> const &packet)
{
    assert(4 <= packet.size() && packet.size() <= 12);

    uint32_t id;
    memcpy(&id, &packet[0], sizeof(uint32_t));
    id = le32toh(id);

    std::vector<uint8_t> payload(packet.size() - 4);
    if (packet.size() > 4) {
        memcpy(&payload[0], &packet[4], packet.size() - 4);
    }
    return boost::make_shared<CANMessage>(id, payload);
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

/*
 * JaguarToken
 */
JaguarToken::JaguarToken(void)
    : done_(false) 
{
}

JaguarToken::~JaguarToken(void)
{
}

void JaguarToken::block(void)
{
    if (done_) return;

    boost::unique_lock<boost::mutex> lock(mutex_);
    while (!done_) {
        cond_.wait(lock);
    }
}

bool JaguarToken::ready(void) const
{
    return done_;
}

boost::shared_ptr<CANMessage const> JaguarToken::message(void) const
{
    assert(done_);
    return message_;
}

void JaguarToken::unblock(boost::shared_ptr<CANMessage> message)
{
    assert(!done_);
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        message_ = message;
        done_ = true;
    }
    cond_.notify_all();
}

};