#include <cassert>
#include <limits>
#include <stdint.h>
#include <vector>
#include <iomanip>
#include <iostream>

#include "jaguar.h"
#include "jaguar_helper.h"

namespace jaguar {

Manufacturer::Enum const Jaguar::kManufacturer = Manufacturer::kTexasInstruments;
DeviceType::Enum   const Jaguar::kDeviceType   = DeviceType::kMotorController;

Jaguar::Jaguar(can::CANBridge &can, uint8_t device_num)
    : num_(device_num), can_(can)
{
}

/*
 * Motor Control Configuration
 */
can::TokenPtr Jaguar::set_num_brushes(uint8_t brushes)
{
    return send_ack(APIClass::kConfiguration, Configuration::kNumberOfBrushes, brushes);
}

can::TokenPtr Jaguar::set_num_encoders(uint16_t lines)
{
    uint16_t const payload = htole16(lines);
    return send_ack(APIClass::kConfiguration, Configuration::kNumberOfEncodersLines, payload);
}

can::TokenPtr Jaguar::set_fault_time(uint16_t ms)
{
    uint16_t const payload = htole16(ms);
    return send_ack(APIClass::kConfiguration, Configuration::kFaultTime, payload);
}

/*
 * Voltage Control
 */
can::TokenPtr Jaguar::enable_voltage(void)
{
    return send_ack(APIClass::kVoltageControl, VoltageControl::kVoltageModeEnable);
}

can::TokenPtr Jaguar::disable_voltage(void)
{
    return send_ack(APIClass::kVoltageControl, VoltageControl::kVoltageModeDisable);
}

can::TokenPtr Jaguar::set_voltage(double scale)
{
    double constrained = std::min(std::max(scale, -1.0), +1.0);
    int16_t output;

    if (constrained < 0) {
        output = static_cast<int16_t>(std::numeric_limits<int16_t>::min() * -constrained);
    } else if (constrained > 0) {
        output = static_cast<int16_t>(std::numeric_limits<int16_t>::max() * +constrained);
    } else {
        output = 0;
    }

    int16_t const payload = htole16(output);
    return send_ack(APIClass::kVoltageControl, VoltageControl::kVoltageSet, payload);
}

/*
 * Speed Control
 */
can::TokenPtr Jaguar::enable_pid(void)
{
    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedModeEnable);
}

can::TokenPtr Jaguar::disable_pid(void)
{
    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedModeDisable);
}

can::TokenPtr Jaguar::set_p_constant(double p)
{
    int32_t const payload = double_to_s16p16(p);
    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedProportionalConstant, payload);
}

can::TokenPtr Jaguar::set_i_constant(double i)
{
    int32_t const payload = double_to_s16p16(i);
    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedIntegralConstant, payload);
}

can::TokenPtr Jaguar::set_d_constant(double d)
{
    int32_t const payload = double_to_s16p16(d);
    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedDifferentialConstant, payload);
}

can::TokenPtr Jaguar::set_speed_reference(SpeedReference::Enum reference)
{
    uint8_t const payload = static_cast<uint8_t>(reference);
    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedReference, payload);
}

can::TokenPtr Jaguar::set_speed(double speed)
{
    int32_t const payload = double_to_s16p16(speed);
    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedSet, payload);
}

can::TokenPtr Jaguar::set_speed(double speed, uint8_t group)
{
    struct {
        int32_t speed;
        uint8_t group;
    } __attribute__((__packed__)) payload;

    payload.speed = double_to_s16p16(speed);
    payload.group = group;

    return send_ack(APIClass::kSpeedControl, SpeedControl::kSpeedSet, payload);
}

/*
 * Helpers
 */
can::TokenPtr Jaguar::recv_ack(void)
{
    token_ = can_.recv(pack_ack(num_, kManufacturer, kDeviceType));
    return token_;
}

void Jaguar::send(APIClass::Enum api_class, uint8_t api_index)
{
    uint32_t id = pack_id(num_, kManufacturer, kDeviceType, api_class, api_index);
    can_.send(can::CANMessage(id));
}

template <typename T>
void Jaguar::send(APIClass::Enum api_class, uint8_t api_index, T const &payload)
{
    uint32_t id = pack_id(num_, kManufacturer, kDeviceType, api_class, api_index);
    std::vector<uint8_t> payload_raw(sizeof payload);
    memcpy(&payload_raw[0], &payload, sizeof payload);
    can_.send(can::CANMessage(id, payload_raw));
}

can::TokenPtr Jaguar::send_ack(APIClass::Enum api_class, uint8_t api_index)
{
    assert(!token_ || token_->ready());
    send(api_class, api_index);
    uint32_t const id = pack_ack(num_, kManufacturer, kDeviceType);
    return can_.recv(id);
}

template <typename T>
can::TokenPtr Jaguar::send_ack(APIClass::Enum api_class, uint8_t api_index, T const &payload)
{
    assert(!token_ || token_->ready());
    send(api_class, api_index, payload);
    uint32_t const id = pack_ack(num_, kManufacturer, kDeviceType);
    return can_.recv(id);
}

};

