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
    : num_(device_num)
    , can_(can)
    , sig_diag_(4)
    , sig_odom_(4)
{
    for (size_t i = 0; i < 4; i++) {
        sig_diag_[i] = boost::make_shared<DiagSignal>();
        sig_odom_[i] = boost::make_shared<OdomSignal>();
    }
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

can::TokenPtr Jaguar::config_brake_set(BrakeCoastSetting::Enum brake)
{
    return send_ack(
        APIClass::kConfiguration, Configuration::kBrakeCoastSetting,
        byte_(brake)
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
 * Position Mode
 */
can::TokenPtr Jaguar::position_enable(void) {
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionModeEnable,
        eps
    );
}

can::TokenPtr Jaguar::position_disable(void) {
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionModeDisable,
        eps
    );
}

can::TokenPtr Jaguar::position_set_p(double p) {
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionProportionalConstant,
        little_dword(double_to_s16p16(p))
    );
}

can::TokenPtr Jaguar::position_set_i(double i) {
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionIntegralConstant,
        little_dword(double_to_s16p16(i))
    );
}

can::TokenPtr Jaguar::position_set_d(double d) {
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionDifferentialConstant,
        little_dword(double_to_s16p16(d))
    );
}

can::TokenPtr Jaguar::position_set_reference(PositionReference::Enum reference)
{
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionReference,
        byte_(reference)
    );
}

can::TokenPtr Jaguar::position_set(double position) {
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionSet,
        little_dword(double_to_s16p16(position))
    );
}

can::TokenPtr Jaguar::position_set(double position, uint8_t group) {
    return send_ack(
        APIClass::kPositionControl, PositionControl::kPositionSet,
        little_dword(double_to_s16p16(position)) << byte_(group)
    );
}

void Jaguar::position_set_noack(double position) {
    send(
        APIClass::kPositionControl, PositionControl::kPositionSetNoACK,
        little_dword(double_to_s16p16(position))
    );
}

void Jaguar::position_set_noack(double position, uint8_t group) {
    send(
        APIClass::kPositionControl, PositionControl::kPositionSetNoACK,
        little_dword(double_to_s16p16(position)) << byte_(group)
    );
}


/*
 * Periodic Status Updates
 */
can::TokenPtr Jaguar::periodic_enable(uint8_t index, uint16_t rate_ms)
{
    return send_ack(
        APIClass::kPeriodicStatus, PeriodicStatus::kEnableMessage + index,
        little_word(rate_ms)
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

can::TokenPtr Jaguar::periodic_config_diag(uint8_t index, boost::function<DiagCallback> callback)
{
    // Tell the Jaguar which status fields we're interested in. Due to CAN
    // limitations, we can only receive eight bytes per update message.
    uint32_t const config_id = pack_id(num_,
        kManufacturer, kDeviceType,
        APIClass::kPeriodicStatus, PeriodicStatus::kConfigureMessage + index
    );
    uint32_t const ack_id = pack_ack(num_, kManufacturer, kDeviceType);
    can::CANMessage msg(config_id);

    // Request the limit switch status, fault status, temperature
    msg.payload.resize(8);
    msg.payload[0] = PeriodicStatusItem::kLimitNonClearing;
    msg.payload[1] = PeriodicStatusItem::kStickyFaultsNonClearing;
    msg.payload[2] = PeriodicStatusItem::kBusVoltageBase + 0;
    msg.payload[3] = PeriodicStatusItem::kBusVoltageBase + 1;
    msg.payload[4] = PeriodicStatusItem::kTemperatureBase + 0;
    msg.payload[5] = PeriodicStatusItem::kTemperatureBase + 1;
    msg.payload[6] = PeriodicStatusItem::kEndOfMessage;

    // Register a callback to process the periodic status updates.
    uint32_t const status_id = pack_id(num_,
        kManufacturer, kDeviceType,
        APIClass::kPeriodicStatus, PeriodicStatus::kPeriodicStatus + index
    );
    sig_diag_[index]->connect(callback);
    can_.attach_callback(status_id, boost::bind(&Jaguar::diag_unpack, this, _1, index));

    // Wait for an ACK in response to the config message.
    can::TokenPtr token =  can_.recv(ack_id);
    can_.send(msg);
    return token;
}

can::TokenPtr Jaguar::periodic_config_odom(uint8_t index, boost::function<OdomCallback> callback)
{
    // Tell the Jaguar which status fields we're interested in. Due to CAN
    // limitations, we can only receive eight bytes per update message.
    uint32_t const config_id = pack_id(num_,
        kManufacturer, kDeviceType,
        APIClass::kPeriodicStatus, PeriodicStatus::kConfigureMessage + index
    );
    uint32_t const ack_id = pack_ack(num_, kManufacturer, kDeviceType);
    can::CANMessage msg(config_id);

    // Request the 16.16 position and 16.16 velocity.
    msg.payload.reserve(8);
    for (int i = 0; i < 4; i++) {
        msg.payload.push_back(PeriodicStatusItem::kPositionBase + i);
    }
    for (int i = 0; i < 4; i++) {
        msg.payload.push_back(PeriodicStatusItem::kSpeedBase + i);
    }

    // Register a callback to process the periodic status updates.
    uint32_t const status_id = pack_id(num_,
        kManufacturer, kDeviceType,
        APIClass::kPeriodicStatus, PeriodicStatus::kPeriodicStatus + index
    );
    sig_odom_[index]->connect(callback);
    can_.attach_callback(status_id, boost::bind(&Jaguar::odom_unpack, this, _1, index));

    // Wait for an ACK in response to the config message.
    can::TokenPtr token =  can_.recv(ack_id);
    can_.send(msg);
    return token;
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
    can::TokenPtr token =  can_.recv(ack_id);
    can_.send(msg);
    return token;
}


/*
 * Helpers
 */
void Jaguar::diag_unpack(boost::shared_ptr<can::CANMessage> msg, uint8_t index)
{
    uint8_t raw_limits = 0, raw_faults = 0;
    uint16_t raw_bus_voltage = 0, raw_temperature = 0;
    boost::spirit::qi::parse(msg->payload.begin(), msg->payload.end(),
        byte_ >> byte_ >> little_word >> little_word,
        raw_limits, raw_faults, raw_bus_voltage, raw_temperature
    );

#if 0
    std::cout << std::hex << static_cast<int>(raw_limits) << ", "
              << std::hex << static_cast<int>(raw_faults) << std::endl;
#endif

    LimitStatus::Enum const limits = static_cast<LimitStatus::Enum>(raw_limits);
    Fault::Enum const faults = static_cast<Fault::Enum>(raw_faults);
    double const bus_voltage = s8p8_to_double(raw_bus_voltage);
    double const temperature = s8p8_to_double(raw_temperature);
    (*sig_diag_[index])(limits, faults, bus_voltage, temperature);
}

void Jaguar::odom_unpack(boost::shared_ptr<can::CANMessage> msg, uint8_t index)
{
    int32_t raw_position = 0, raw_speed = 0;
    boost::spirit::qi::parse(msg->payload.begin(), msg->payload.end(),
        little_dword >> little_dword, raw_position, raw_speed
    );
    double const position = s16p16_to_double(raw_position);
    double const speed = s16p16_to_double(raw_speed);
    (*sig_odom_[index])(position, speed);
}


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
    uint32_t const ack_id = pack_ack(num_, kManufacturer, kDeviceType);
    can::TokenPtr token =  can_.recv(ack_id);
    send(api_class, api_index, generator);
    return token;
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
