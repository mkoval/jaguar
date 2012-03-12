#include <cassert>
#include <cstring>
#include <iostream>
#include "ntcan_bridge.h"

namespace can {

int32_t const NTCANBridge::m_tx_queue = 8; // messages
int32_t const NTCANBridge::m_rx_queue = 8; // messages
int32_t const NTCANBridge::m_tx_timeout = 100; // ms
int32_t const NTCANBridge::m_rx_timeout = 10000; // ms
uint32_t const NTCANBridge::m_baud = NTCAN_BAUD_1000; // 500 kb/s

NTCANBridge::NTCANBridge(int net)
{
    NTCAN_RESULT result;
    result = canOpen(net, 0, m_tx_queue, m_rx_queue, m_tx_timeout, m_rx_timeout, &m_handle);
    handle_error(result);

    result = canSetBaudrate(m_handle, m_baud);
    handle_error(result);

    result = canIdAdd(m_handle, NTCAN_20B_BASE);
    handle_error(result);
}

NTCANBridge::~NTCANBridge(void)
{
    canClose(m_handle);
}

void NTCANBridge::send(uint32_t id, void const *data, size_t payload_length)
{
    assert((id & ~0x1fffffff) == 0);
    assert(payload_length <= 8);

    CMSG ntcan_msg;
    ntcan_msg.id  = id | NTCAN_20B_BASE;
    ntcan_msg.len = payload_length;
    if (payload_length > 0) {
        memcpy(ntcan_msg.data, data, payload_length);
    }

    int32_t msg_length = 1;
    NTCAN_RESULT result = canWrite(m_handle, &ntcan_msg, &msg_length, NULL);
    handle_error(result);
}

uint32_t NTCANBridge::recv(void *data, size_t payload_length)
{
    int32_t msg_length = 1;
    NTCAN_RESULT result = canRead(m_handle, &m_buffer, &msg_length, NULL);
    handle_error(result);

    // Any timeout should be indicated in the return code, so we can assume
    // that one message was received.
    assert(msg_length == sizeof(CMSG));
    assert(m_buffer.len == payload_length);
    memcpy(data, m_buffer.data, payload_length);
    return m_buffer.id;
}

void NTCANBridge::handle_error(int result)
{
    switch (result) {
    case NTCAN_SUCCESS:
        break;

    case NTCAN_CONTR_BUSY:
        throw CANException(result, "The capacity of the internal transmit-FIFO has been exceeded.");

    case NTCAN_CONTR_OFF_BUS:
        throw CANException(result, "Transmission error.");

    case NTCAN_CONTR_WARN:
        throw CANException(result, "The CAN controller has changed into Error "
                                   "Passive status during a transmit operation, "
                                   "because too many CAN error frames have been received.");

    case NTCAN_ID_ALREADY_ENABLED:
        throw CANException(result, "The CAN-ID for this handle has already been activated.");

    case NTCAN_ID_NOT_ENABLED:
        throw CANException(result, "The CAN-ID has not been activated for this handle.");

    case NTCAN_INSUFFICIENT_RESOURCES:
        throw CANException(result, "Insufficient internal resources.");

    case NTCAN_INVALID_DRIVER:
        throw CANException(result, "Driver and NTCAN library are not compatible.");

    case NTCAN_INVALID_FIRMWARE:
        throw CANException(result, "Driver and firmware are incompatible.");

    case NTCAN_INVALID_HANDLE:
        throw CANException(result, "Invalid CAN handle.");

    case NTCAN_INVALID_HARDWARE:
        throw CANException(result, "Driver and hardware are incompatible.");

    case NTCAN_INVALID_PARAMETER:
        throw CANException(result, "Operation has not yet been terminated (Win32 only).");

    case NTCAN_NET_NOT_FOUND:
        throw CANException(result, "Invalid logical network number.");

    case NTCAN_NO_ID_ENABLED:
        throw CANException(result, "Read handle without any enabled CAN identifier.");

    case NTCAN_OPERATION_ABORTED:
        throw CANException(result, "Abortion of a blocking transmit/receive operation.");

    case NTCAN_HANDLE_FORCED_CLOSE:
        throw CANException(result, "Abortion of a blocking transmit/receive operation.");

    case NTCAN_PENDING_READ:
        throw CANException(result, "Receive operation could not be executed.");

    case NTCAN_PENDING_WRITE:
        throw CANException(result, "Transmit operation could not be executed.");

    case NTCAN_RX_TIMEOUT:
        throw CANException(result, "Timeout event for blocking receive operation.");

    case NTCAN_TX_ERROR:
        throw CANException(result, "Timeout event for blocking transmit operation.");

    case NTCAN_TX_TIMEOUT:
        throw CANException(result, "Timeout event for blocking transmit operation.");

    case NTCAN_WRONG_DEVICE_STATE:
        throw CANException(result, "The actual device state prevents I/O-operations (Win32 only).");

    case NTCAN_NOT_IMPLEMENTED:
        throw CANException(result, "Command for canIoctl() is not implemented.");

    case NTCAN_NOT_SUPPORTED:
        throw CANException(result, "The argument of the call is valid but not supported.");

    case NTCAN_SOCK_CONN_TIMEOUT:
        throw CANException(result, "No network connection established connection runtime to exceeded.");

    case NTCAN_SOCK_CMD_TIMEOUT:
        throw CANException(result, "Runtime of TCP/IP-packages exceeded.");

    case NTCAN_SOCK_HOST_NOT_FOUND:
        throw CANException(result, "Wrong name, incorrect name server configuration, etc.");

    default:
        throw CANException(result, "An unknown error has occurred.");
    }
}

};
