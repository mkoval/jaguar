#include <cassert>
#include <limits>
#include <stdint.h>
#include <vector>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <boost/bind.hpp>
#include <boost/spirit/include/karma.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <jaguar/jaguar.h>
#include <jaguar/jaguar_helper.h>

using boost::spirit::eps;
using boost::spirit::byte_;
using boost::spirit::little_word;
using boost::spirit::little_dword;


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

can::TokenPtr Jaguar::config_brushes_set(uint8_t brushes)
{
    return send_ack(
        APIClass::kConfiguration, Configuration::kNumberOfBrushes,
        byte_(brushes)
    );
}

can::TokenPtr Jaguar::config_encoders_set(uint16_t lines)
{
    return send_ack(
        APIClass::kConfiguration, Configuration::kNumberOfEncodersLines,
        little_word(lines)
    );
}

can::TokenPtr Jaguar::config_fault_set(uint16_t ms)
{
    assert(ms >= 500);
    return send_ack(
        APIClass::kConfiguration,Configuration::kFaultTime,
        little_word(ms)
    );
}

/*
 * Voltage Control
 */
can::TokenPtr Jaguar::voltage_enable(void)
{
    return send_ack(
        APIClass::kVoltageControl, VoltageControl::kVoltageModeEnable,
        eps
    );
}

can::TokenPtr Jaguar::voltage_disable(void)
{
    return send_ack(
        APIClass::kVoltageControl, VoltageControl::kVoltageModeDisable,
        eps
    );
}

can::TokenPtr Jaguar::voltage_set(double voltage)
{
    return send_ack(
        APIClass::kVoltageControl, VoltageControl::kVoltageSet,
        little_word(rescale<int16_t>(voltage))
    );
}

can::TokenPtr Jaguar::voltage_set(double voltage, uint8_t group)
{
    return send_ack(
        APIClass::kVoltageControl, VoltageControl::kVoltageSet,
        little_word(rescale<int16_t>(voltage)) << byte_(group)
    );
}

void Jaguar::voltage_set_noack(double voltage)
{
    send(
        APIClass::kVoltageControl, VoltageControl::kVoltageSetNoACK,
        little_word(rescale<int16_t>(voltage))
    );
}

void Jaguar::voltage_set_noack(double voltage, uint8_t group)
{
    send(
        APIClass::kVoltageControl, VoltageControl::kVoltageSetNoACK,
        little_word(rescale<int16_t>(voltage)) << byte_(group)
    );
}

/*
 * Speed Control
 */
can::TokenPtr Jaguar::speed_enable(void)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedModeEnable,
        eps
    );
}

can::TokenPtr Jaguar::speed_disable(void)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedModeDisable,
        eps
    );
}

can::TokenPtr Jaguar::speed_set_p(double p)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedProportionalConstant,
        little_dword(double_to_s16p16(p))
    );
}

can::TokenPtr Jaguar::speed_set_i(double i)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedIntegralConstant,
        little_dword(double_to_s16p16(i))
    );
}

can::TokenPtr Jaguar::speed_set_d(double d)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedDifferentialConstant,
        little_dword(double_to_s16p16(d))
    );
}

can::TokenPtr Jaguar::speed_set_reference(SpeedReference::Enum reference)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedReference,
        byte_(reference)
    );
}

can::TokenPtr Jaguar::speed_set(double speed)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedSet,
        little_dword(double_to_s16p16(speed))
    );
}

can::TokenPtr Jaguar::speed_set(double speed, uint8_t group)
{
    return send_ack(
        APIClass::kSpeedControl, SpeedControl::kSpeedSet,
        little_dword(double_to_s16p16(speed)) << byte_(group)
    );
}

void Jaguar::speed_set_noack(double speed)
{
    send(
        APIClass::kSpeedControl, SpeedControl::kSpeedSetNoACK,
        little_dword(double_to_s16p16(speed))
    );
}

void Jaguar::speed_set_noack(double speed, uint8_t group)
{
    send(
        APIClass::kSpeedControl, SpeedControl::kSpeedSetNoACK,
        little_dword(double_to_s16p16(speed)) << byte_(group)
    );
}

/*
 * Periodic Status Updates
 */
can::TokenPtr Jaguar::periodic_enable(uint8_t index, uint16_t rate_ms, periodic_callback cb)
{
    return send_ack(
        APIClass::kPeriodicStatus, PeriodicStatus::kEnableMessage + index,
        little_dword(rate_ms)
    );
}

can::TokenPtr Jaguar::periodic_disable(uint8_t index)
{
    // TODO: Unregister the callback.

    return send_ack(
        APIClass::kPeriodicStatus, PeriodicStatus::kEnableMessage + index,
        byte_(0)
    );
}

can::TokenPtr Jaguar::periodic_config(uint8_t index, AggregateStatus statuses)
{
    // Tell the Jaguar which status fields we're interested in. Due to CAN
    // limitations, we can only receive eight bytes per update message.
    uint32_t const config_id = pack_id(num_,
        kManufacturer, kDeviceType,
        APIClass::kPeriodicStatus, PeriodicStatus::kConfigureMessage + index
    );
    uint32_t const ack_id = pack_ack(num_, kManufacturer, kDeviceType);
    can::CANMessage msg(config_id);

    std::back_insert_iterator<std::vector<uint8_t> > payload(msg.payload);
    statuses.write(payload);

    // Terminate short methods the EOM item.
    assert(msg.payload.size() <= 8);
    if (msg.payload.size() < 8) {
        msg.payload.push_back(PeriodicStatusItem::kEndOfMessage);
    }

    // Register a callback to process the periodic status updates.
    uint32_t const status_id = pack_id(num_,
        kManufacturer, kDeviceType,
        APIClass::kPeriodicStatus, PeriodicStatus::kPeriodicStatus + index
    );
    can_.attach_callback(status_id, boost::bind(&Jaguar::periodic_unpack, this, _1, statuses));

    // Wait for an ACK in response to the config message.
    can_.send(msg);
    return can_.recv(ack_id);
}

/*
 * Helpers
 */
void Jaguar::periodic_unpack(boost::shared_ptr<can::CANMessage> message, AggregateStatus statuses)
{
    std::vector<uint8_t> const &payload = message->payload;
    statuses.read(&payload.front(), &payload.back() + 1);
}

template <typename T>
T Jaguar::rescale(double x)
{
    assert(-1.0 <= x && x <= +1.0);

    if (x < 0) {
        return static_cast<T>(std::numeric_limits<T>::min() * -x);
    } else if (x > 0) {
        return static_cast<T>(std::numeric_limits<T>::max() *  x);
    } else {
        return 0;
    }

}

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

AggregateStatus operator<<(AggregateStatus aggregate, Status::Ptr const &status) {
    aggregate.statuses_.push_back(status);
    return aggregate;
}

};