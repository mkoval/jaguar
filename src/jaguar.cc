#include <cassert>
#include <arpa/inet.h>
#include <stdint.h>
#include "jaguar.h"

namespace can {

Jaguar::Jaguar(CANBridge &can, uint8_t device_num)
    : m_can(can), m_num(device_num)
{
}

/*
 * Broadcast
 */
void Jaguar::system_reset(void)
{
    uint32_t request_id = pack_id(0, kBroadcastMessage, kBroadcastMessage, kSystemReset);
    m_can.send(request_id, NULL, 0);
}

void Jaguar::system_halt(void)
{
    uint32_t request_id = pack_id(0, kBroadcastMessage, kBroadcastMessage, kSystemHalt);
    m_can.send(request_id, NULL, 0);
}

void Jaguar::system_resume(void)
{
    uint32_t request_id = pack_id(0, kBroadcastMessage, kBroadcastMessage, kSystemResume);
    m_can.send(request_id, NULL, 0);
}

void Jaguar::heartbeat(void)
{
    uint32_t request_id = pack_id(0, kBroadcastMessage, kBroadcastMessage, kHeartbeat);
    m_can.send(request_id, NULL, 0);
}

void Jaguar::device_assignment(uint8_t id)
{
    assert((id & 0xC0) == 0);
    uint32_t request_id = pack_id(id, kBroadcastMessage, kBroadcastMessage, kDeviceAssignment);
    m_can.send(request_id, NULL, 0);
}

void Jaguar::synchronous_update(uint8_t group)
{
    uint32_t request_id = pack_id(0, kBroadcastMessage, kBroadcastMessage, kSynchronousUpdate);
    m_can.send(request_id, &group, 1);
}

/*
 * Motor Control Configuration
 */
void Jaguar::set_encoder_lines(uint16_t lines)
{
    uint16_t payload = htons(lines);
    m_can.send(pack_id(m_num, m_manufacturer, m_type, kConfiguration, kNumberOfEncodersLines), &payload, 2);
}

void Jaguar::set_fault_time(uint16_t ms)
{
    uint16_t payload = htons(ms);
    m_can.send(pack_id(m_num, m_manufacturer, m_type, kConfiguration, kFaultTime), &payload, 2);
}

/*
 * Motor Control Status
 */
double Jaguar::get_bus_voltage(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, m_type, kStatus, kBusVoltage);
    m_can.send(request_id, NULL, 0);

    int16_t payload;
    uint32_t response_id = m_can.recv(&payload, 2);
    assert(response_id == request_id);

    return s8p8_to_double(payload);
}

double Jaguar::get_current(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, m_type, kStatus, kCurrent);
    m_can.send(request_id, NULL, 0);

    int16_t payload;
    uint32_t response_id = m_can.recv(&payload, 2);
    assert(response_id == request_id);

    return s8p8_to_double(payload);
}

double Jaguar::get_temperature(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, m_type, kStatus, kTemperature);
    m_can.send(request_id, NULL, 0);

    int16_t payload;
    uint32_t response_id = m_can.recv(&payload, 2);
    assert(response_id == request_id);

    return s8p8_to_double(payload);
}

double Jaguar::get_position(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, m_type, kStatus, kPosition);
    m_can.send(request_id, NULL, 0);

    int32_t payload;
    uint32_t response_id = m_can.recv(&payload, 4);
    assert(response_id == request_id);

    return s16p16_to_double(payload);
}

double Jaguar::get_speed(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, m_type, kStatus, kSpeed);
    m_can.send(request_id, NULL, 0);

    int32_t payload;
    uint32_t response_id = m_can.recv(&payload, 4);
    assert(response_id == request_id);

    return s16p16_to_double(payload);
}

Fault Jaguar::get_fault(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, m_type, kStatus, kFault);
    m_can.send(request_id, NULL, 0);

    uint16_t payload;
    uint32_t response_id = m_can.recv(&payload, 2);
    assert(response_id == request_id);

    return static_cast<Fault>(ntohs(payload));
}

/*
 * Speed Control
 */
void Jaguar::enable_pid(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, kMotorController, kSpeedControl, kSpeedModeEnable);
    m_can.send(request_id, NULL, 0);
}

void Jaguar::disable_pid(void)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, kMotorController, kSpeedControl, kSpeedModeDisable);
    m_can.send(request_id, NULL, 0);
}

void Jaguar::set_p_constant(double p)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, kMotorController, kSpeedControl, kSpeedProportionalConstant);
    int32_t payload = double_to_s16p16(p);
    m_can.send(request_id, &payload, 4);
}

void Jaguar::set_i_constant(double i)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, kMotorController, kSpeedControl, kSpeedIntegralConstant);
    int32_t payload = double_to_s16p16(i);
    m_can.send(request_id, &payload, 4);
}

void Jaguar::set_d_constant(double d)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, kMotorController, kSpeedControl, kSpeedDifferentialConstant);
    int32_t payload = double_to_s16p16(d);
    m_can.send(request_id, &payload, 4);
}

void Jaguar::set_speed_reference(SpeedReference reference)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, kMotorController, kSpeedControl, kSpeedReference);
    uint8_t payload = static_cast<uint8_t>(reference);
    m_can.send(request_id, &payload, 1);
}

void Jaguar::set_speed(double speed)
{
    uint32_t request_id = pack_id(m_num, m_manufacturer, kMotorController, kSpeedControl, kSpeedSet);
    int32_t payload = double_to_s16p16(speed);
    m_can.send(request_id, &payload, 4);
}

void Jaguar::set_speed(double speed, uint8_t group)
{
    struct {
        int32_t speed;
        uint8_t group;
    } __attribute__((__packed__)) payload;

    payload.speed = double_to_s16p16(speed);
    payload.group = group;

    uint32_t request_id = pack_id(m_num, m_manufacturer, m_type, kSpeedControl, kSpeedSet);
    m_can.send(request_id, &payload, 5);
}

/*
 * Helpers
 */
int16_t Jaguar::double_to_s8p8(double x)
{
    return htons(static_cast<int16_t>(x * 256));
}

int32_t Jaguar::double_to_s16p16(double x)
{
    return htonl(static_cast<int32_t>(x * 65536));
}

double Jaguar::s8p8_to_double(int16_t x)
{
    return ntohs(x) / 256.;
}

double Jaguar::s16p16_to_double(int32_t x)
{
    return ntohl(x) / 65536.;
}

uint16_t Jaguar::pack_api(uint8_t api_class, uint8_t api_index)
{
    assert((api_class & ~0x3F) == 0);
    assert((api_index & ~0x0F) == 0);
    return api_class | (api_index << 6);
}

uint32_t Jaguar::pack_id(uint8_t device_num, uint8_t manufacturer, uint8_t type, uint16_t api)
{
    assert((device_num   & ~0x1F)  == 0);
    assert((api          & ~0x3FF) == 0);
    assert((manufacturer & ~0xFF)  == 0);
    assert((type         & ~0x1F)  == 0);
    return (static_cast<uint32_t>(device_num)   << 0)
         | (static_cast<uint32_t>(api)          << 6)
         | (static_cast<uint32_t>(manufacturer) << 16)
         | (static_cast<uint32_t>(type)         << 24);
}

uint32_t Jaguar::pack_id(uint8_t device_num, uint8_t manufacturer, uint8_t type, uint8_t api_class, uint8_t api_index)
{
    return pack_id(device_num, pack_api(api_class, api_index), manufacturer, type);
}

};
