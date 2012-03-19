#include <stdint.h>
#include "can_bridge.h"
extern "C" {
#include <ntcan.h>
};

namespace can {

class NTCANBridge : public CANBridge
{
public:
    NTCANBridge(int net);
    virtual ~NTCANBridge();

    virtual void send(uint32_t id, void const *data, size_t length);
    virtual uint32_t recv(void *data, size_t length);

//private:
    NTCAN_HANDLE m_handle;
    CMSG         m_buffer;

    static int32_t const m_tx_queue;
    static int32_t const m_rx_queue;
    static int32_t const m_tx_timeout;
    static int32_t const m_rx_timeout;
    static uint32_t const m_baud;

    static void handle_error(int code);
};

};
