#ifndef JAGUAR_BRIDGE_H_
#define JAGUAR_BRIDGE_H_

#include <boost/asio.hpp>
#include <vector>
#include <stdint.h>
#include "can_bridge.h"

namespace can {

class JaguarBridge : public CANBridge
{
public:
    JaguarBridge(std::string port);
    virtual ~JaguarBridge(void);

    virtual void send(uint32_t id, void const *data, size_t length);
    virtual uint32_t recv(void *data, size_t length);

    void recv_test(void);

private:
    boost::asio::io_service  m_io;
    boost::asio::serial_port m_serial;
    uint8_t m_last;

    static uint8_t const m_sof, m_esc;
    static uint8_t const m_sof_esc, m_esc_esc;

    static size_t encode_bytes(uint8_t const *bytes, size_t length, std::vector<uint8_t> &buffer);
};

};

#endif
