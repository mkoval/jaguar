#include <ntcan.h>
#include <stdint.h>
#include "can_bridge.h"

namespace can {

class NTCANBridge : public CANBridge
{
public:
    NTCANBridge(int net);
    virtual ~NTCANBridge();

    virtual void send(uint32_t id, void const *data, size_t length);
    virtual uint32_t recv(void *data, size_t length);

private:
    NTCAN_HANDLE m_handle;
    CMSG         m_buffer;

    static int const m_tx_queue = 16; // messages
    static int const m_rx_queue = 16; // messages
    static int const m_tx_timeout = 100; // ms
    static int const m_rx_timeout = 100; // ms
};

};
