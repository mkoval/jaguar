#include <cassert>
#include <limits>
#include <stdint.h>
#include <vector>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <boost/spirit/include/karma.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <jaguar/jaguar.h>
#include <jaguar/jaguar_helper.h>

using boost::spirit::byte_;
using boost::spirit::little_word;
using boost::spirit::little_dword;
typedef boost::spirit::unused_type no_payload;

namespace boost { namespace spirit { namespace traits
{
    template <>
    struct create_generator<no_payload>
    {
        typedef spirit::karma::eps_type type;

        static type call()
        {
            return spirit::karma::eps;
        }
    };
}}}

namespace jaguar {

Manufacturer::Enum const Jaguar::kManufacturer = Manufacturer::kTexasInstruments;
DeviceType::Enum   const Jaguar::kDeviceType   = DeviceType::kMotorController;

struct speed_group_t {
    int32_t speed;
    uint8_t group;
} __attribute__((__packed__));

Jaguar::Jaguar(can::CANBridge &can, uint8_t device_num)
    : num_(device_num), can_(can)
{
}

/*
 * Motor Control Configuration
 */
can::TokenPtr Jaguar::set_num_brushes(uint8_t brushes)
{
    return send_ack(
        APIClass::kConfiguration, Configuration::kNumberOfBrushes,
        byte_(brushes)
    );
}

can::TokenPtr Jaguar::set_num_encoders(uint16_t lines)
{
    return send_ack(
        APIClass::kConfiguration, Configuration::kNumberOfEncodersLines,
        little_word(lines)
    );
}

can::TokenPtr Jaguar::set_fault_time(uint16_t ms)
{
    return send_ack(
        APIClass::kConfiguration,Configuration::kFaultTime,
        little_word(ms)
    );
}

/*
 * Voltage Control
 */
can::TokenPtr Jaguar::enable_voltage(void)
{
    return send_ack(
        APIClass::kVoltageControl, VoltageControl::kVoltageModeEnable,
        no_payload()
    );
}

can::TokenPtr Jaguar::disable_voltage(void)
{
    return send_ack(
        APIClass::kVoltageControl, VoltageControl::kVoltageModeDisable,
        no_payload()
    );
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

    return send_ack(
        APIClass::kVoltageControl, VoltageControl::kVoltageSet,
        little_word(output)
    );
}
    
/*
 * Speed Control
 */
can::TokenPtr Jaguar::enable_pid(void)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedModeEnable,
        no_payload()
    );
}

can::TokenPtr Jaguar::disable_pid(void)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedModeDisable,
        no_payload()
    );
}

can::TokenPtr Jaguar::set_p_constant(double p)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedProportionalConstant,
        little_dword(double_to_s16p16(p))
    );
}

can::TokenPtr Jaguar::set_i_constant(double i)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedIntegralConstant,
        little_dword(double_to_s16p16(i))
    );
}

can::TokenPtr Jaguar::set_d_constant(double d)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedDifferentialConstant,
        little_dword(double_to_s16p16(d))
    );
}

can::TokenPtr Jaguar::set_speed_reference(SpeedReference::Enum reference)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedReference,
        byte_(reference)
    );
}

can::TokenPtr Jaguar::set_speed(double speed)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedSet,
        little_dword(double_to_s16p16(speed))
    );
}

can::TokenPtr Jaguar::set_speed(double speed, uint8_t group)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedSet,
        little_dword(double_to_s16p16(speed)) << byte_(group)
    );
}

/*
 * Helpers
 */
template <typename G>
void Jaguar::send(APIClass::Enum api_class, uint8_t api_index, G const &generator)
{
    uint32_t const id = pack_id(num_, kManufacturer, kDeviceType, api_class, api_index);
    can::CANMessage msg(id);

    std::back_insert_iterator<std::vector<uint8_t> > payload(msg.payload);
    bool success = boost::spirit::karma::generate(payload, generator);
    assert(success);

    can_.send(msg);
}

template <typename G>
can::TokenPtr Jaguar::send_ack(APIClass::Enum api_class, uint8_t api_index, G const &generator)
{
    assert(!token_ || token_->ready());
    send(api_class, api_index, generator);
    uint32_t const ack_id = pack_ack(num_, kManufacturer, kDeviceType);
    return can_.recv(ack_id);
}

can::TokenPtr Jaguar::recv_ack(void)
{
    token_ = can_.recv(pack_ack(num_, kManufacturer, kDeviceType));
    return token_;
}


};

