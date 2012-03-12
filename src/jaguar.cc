#include <cassert>
#include <limits>
#include <stdint.h>

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
void Jaguar::set_num_brushes(uint8_t brushes)
{
    send(APIClass::kConfiguration, Configuration::kNumberOfBrushes, brushes);
}

void Jaguar::set_num_encoders(uint16_t lines)
{
    uint16_t const payload = htole16(lines);
    send(APIClass::kConfiguration, Configuration::kNumberOfEncodersLines, payload);
}

void Jaguar::set_fault_time(uint16_t ms)
{
    uint16_t const payload = htole16(ms);
    send(APIClass::kConfiguration, Configuration::kFaultTime, payload);
}

/*
 * Voltage Control
 */
void Jaguar::enable_voltage(void)
{
    send(APIClass::kVoltageControl, VoltageControl::kVoltageModeEnable);
}

void Jaguar::disable_voltage(void)
{
    send(APIClass::kVoltageControl, VoltageControl::kVoltageModeDisable);
}

void Jaguar::set_voltage(double scale)
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
    send(APIClass::kVoltageControl, VoltageControl::kVoltageSet, payload);
}

/*
 * Speed Control
 */
void Jaguar::enable_pid(void)
{
    send(APIClass::kSpeedControl, SpeedControl::kSpeedModeEnable);
}

void Jaguar::disable_pid(void)
{
    send(APIClass::kSpeedControl, SpeedControl::kSpeedModeDisable);
}

void Jaguar::set_p_constant(double p)
{
    int32_t const payload = double_to_s16p16(p);
    send(APIClass::kSpeedControl, SpeedControl::kSpeedProportionalConstant, payload);
}

void Jaguar::set_i_constant(double i)
{
    int32_t const payload = double_to_s16p16(i);
    send(APIClass::kSpeedControl, SpeedControl::kSpeedIntegralConstant, payload);
}

void Jaguar::set_d_constant(double d)
{
    int32_t const payload = double_to_s16p16(d);
    send(APIClass::kSpeedControl, SpeedControl::kSpeedDifferentialConstant, payload);
}

void Jaguar::set_speed_reference(SpeedReference::Enum reference)
{
    uint8_t const payload = static_cast<uint8_t>(reference);
    send(APIClass::kSpeedControl, SpeedControl::kSpeedReference, payload);
}

void Jaguar::set_speed(double speed)
{
    int32_t const payload = double_to_s16p16(speed);
    send(APIClass::kSpeedControl, SpeedControl::kSpeedSet, payload);
}

void Jaguar::set_speed(double speed, uint8_t group)
{
#if 0
    struct {
        int32_t speed;
        uint8_t group;
    } __attribute__((__packed__)) payload;

    payload.speed = double_to_s16p16(speed);
    payload.group = group;

    send(APIClass::kSpeedControl, SpeedControl::kSpeedSet, payload);
#endif 
    // FIXME: Implement this.
}

/*
 * Helpers
 */
void Jaguar::send(APIClass::Enum api_class, uint8_t api_index)
{
    uint32_t id = pack_id(num_, kManufacturer, kDeviceType, api_class, api_index);
    can_.send(id, NULL, 0);
}

template <typename T>
void Jaguar::send(APIClass::Enum api_class, uint8_t api_index, T const &payload)
{
    uint32_t id = pack_id(num_, kManufacturer, kDeviceType, api_class, api_index);
    can_.send(id, &payload, sizeof payload);
}

};

