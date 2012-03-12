#ifndef JAGUAR_BRIDGE_H_
#define JAGUAR_BRIDGE_H_

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <stdint.h>
#include "can_bridge.h"
#include "jaguar_helper.h"

typedef boost::asio::buffers_iterator<
    boost::asio::streambuf::const_buffers_type> asio_iterator;


namespace can {

typedef std::pair<uint32_t, std::vector<uint8_t> > CANMessage;

enum ReceiveState {
    kWaiting,
    kLength,
    kPayload,
    kComplete
};

class JaguarBridge : public CANBridge
{
public:
    typedef boost::function<void (uint32_t, std::vector<uint8_t> payload)> recv_callback;

    JaguarBridge(std::string port);
    virtual ~JaguarBridge(void);

    virtual void send(uint32_t id, void const *data, size_t length);
    virtual void recv_block(uint32_t id, void *data, size_t length);
    virtual void recv_async(uint32_t id, recv_callback cb);

private:
    static uint8_t const kSOF, kESC;
    static uint8_t const kSOFESC, kESCESC;
    static size_t const kReceiveBufferLength;
    static size_t const kReceiveQueueLength;

    boost::asio::io_service  io_;
    boost::asio::serial_port serial_;
    boost::thread recv_thread_;
    boost::circular_buffer<CANMessage> recv_queue_;
    std::vector<uint8_t> recv_buffer_;

    std::vector<uint8_t> packet_;
    ReceiveState state_;
    size_t length_;
    bool escape_;

    boost::optional<CANMessage> recv_byte(uint8_t byte);
    void recv_handle(boost::system::error_code const& error, size_t count);

    CANMessage unpack_packet(std::vector<uint8_t> const &packet);
    size_t encode_bytes(uint8_t const *bytes, size_t length, std::vector<uint8_t> &buffer);
};

};

#endif
