#ifndef JAGUAR_H_
#define JAGUAR_H_

#include <list>
#include <vector>
#include <stdint.h>
#include <boost/spirit/include/karma.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_binary.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/signals2.hpp>
#include <boost/assert.hpp>

#include "can_bridge.h"
#include "jaguar.h"
#include "jaguar_api.h"
#include "jaguar_helper.h"

namespace jaguar {

typedef boost::function<void (void)> periodic_callback;

class Status;
class AggregateStatus;

class Jaguar {
public:
    typedef void DiagCallback(LimitStatus::Enum, Fault::Enum, double, double);
    typedef void OdomCallback(double, double);

    Jaguar(can::CANBridge &can, uint8_t device_num);

    // Motor Control Configuration
    can::TokenPtr config_brushes_set(uint8_t brushes);
    can::TokenPtr config_encoders_set(uint16_t lines);
    can::TokenPtr config_brake_set(BrakeCoastSetting::Enum setting);
    can::TokenPtr config_fault_set(uint16_t ms);

    // Voltage Control
    can::TokenPtr voltage_enable(void);
    can::TokenPtr voltage_disable(void);
    can::TokenPtr voltage_set(double scale);
    can::TokenPtr voltage_set(double scale, uint8_t group);
    void          voltage_set_noack(double scale);
    void          voltage_set_noack(double scale, uint8_t group);

    // Speed Control
    can::TokenPtr speed_enable(void);
    can::TokenPtr speed_disable(void);
    can::TokenPtr speed_set_p(double p);
    can::TokenPtr speed_set_i(double i);
    can::TokenPtr speed_set_d(double d);
    can::TokenPtr speed_set_reference(SpeedReference::Enum reference);
    can::TokenPtr speed_set(double speed);
    can::TokenPtr speed_set(double speed, uint8_t group);
    void          speed_set_noack(double speed);
    void          speed_set_noack(double speed, uint8_t group);

    // Position Control
    can::TokenPtr position_enable(void);
    can::TokenPtr position_disable(void);
    can::TokenPtr position_set_p(double p);
    can::TokenPtr position_set_i(double i);
    can::TokenPtr position_set_d(double d);
    can::TokenPtr position_set_reference(PositionReference::Enum reference);
    can::TokenPtr position_set(double position);
    can::TokenPtr position_set(double position, uint8_t group);
    void          position_set_noack(double position);
    void          position_set_noack(double position, uint8_t group);

    // Periodic Status Updates
    can::TokenPtr periodic_enable(uint8_t index, uint16_t rate_ms);
    can::TokenPtr periodic_disable(uint8_t index);
    can::TokenPtr periodic_config(uint8_t index, AggregateStatus statuses);
    can::TokenPtr periodic_config_diag(uint8_t index, boost::function<DiagCallback> callback);
    can::TokenPtr periodic_config_odom(uint8_t index, boost::function<OdomCallback> callback);

private:
    typedef boost::signals2::signal<DiagCallback> DiagSignal;
    typedef boost::signals2::signal<OdomCallback> OdomSignal;
    typedef boost::shared_ptr<DiagSignal> DiagSignalPtr;
    typedef boost::shared_ptr<OdomSignal> OdomSignalPtr;

    void diag_unpack(boost::shared_ptr<can::CANMessage> msg, uint8_t index);
    void odom_unpack(boost::shared_ptr<can::CANMessage> msg, uint8_t index);
    void periodic_unpack(boost::shared_ptr<can::CANMessage> message, AggregateStatus statuses);

    template <typename T> T rescale(double x);

    template <typename G>
    void send(APIClass::Enum api_class, uint8_t api_index, G const &generator);
    template <typename G>
    can::TokenPtr send_ack(APIClass::Enum api_class, uint8_t api_index, G const &generator);
    can::TokenPtr recv_ack(void);

    uint8_t const num_;
    can::CANBridge &can_;
    can::TokenPtr token_;

    std::vector<DiagSignalPtr> sig_diag_;
    std::vector<OdomSignalPtr> sig_odom_;

    static Manufacturer::Enum const kManufacturer;
    static DeviceType::Enum   const kDeviceType;
};

class AggregateStatus;

class Status {
public:
    typedef boost::shared_ptr<Status> Ptr;

    virtual uint8_t const *read(uint8_t const *begin, uint8_t const *end) = 0;
    virtual void write(std::back_insert_iterator<std::vector<uint8_t> > &data) = 0;

    friend class Jaguar;
    friend class AggregateStatus;
};

class AggregateStatus : public Status {
public:
    typedef boost::shared_ptr<AggregateStatus> Ptr;

    AggregateStatus(AggregateStatus const &other)
        : statuses_(other.statuses_) {}

    AggregateStatus(Status::Ptr const &status) {
        statuses_.push_back(status);
    }

    virtual ~AggregateStatus(void) {}

    virtual uint8_t const *read(uint8_t const *begin, uint8_t const *end)
    {
        BOOST_FOREACH(Status::Ptr &status, statuses_) {
            begin = status->read(begin, end);
        }
        return begin;
    }

    virtual void write(std::back_insert_iterator<std::vector<uint8_t> > &data)
    {
        BOOST_FOREACH(Status::Ptr &status, statuses_) {
            status->write(data);
        }
    }

private:
    std::vector<Status::Ptr> statuses_;

    friend class Jaguar;
    friend AggregateStatus operator<<(
        AggregateStatus aggregate,
        Status::Ptr const &status
    );
};

// This must be declared outside of the class because the first argument needs
// the above-defined implicit conversion from Status::Ptr.
AggregateStatus operator<<(AggregateStatus aggregate, Status::Ptr const &status);

#define JAGUAR_MAKE_STATUS(_name_, _Toutput_,  _generator_, _parser_)          \
class _name_##_ : public Status {                                              \
public:                                                                        \
    typedef boost::shared_ptr<_name_##_> Ptr;                                  \
    typedef boost::function<void (_Toutput_)> Callback;                        \
                                                                               \
    explicit _name_##_(Callback callback) : callback_(callback) {}             \
                                                                               \
    virtual uint8_t const *read(uint8_t const *begin, uint8_t const *end) {    \
        _Toutput_ output;                                                      \
        BOOST_VERIFY(boost::spirit::qi::parse(begin, end, _parser_, output));  \
        callback_(output);                                                     \
        return begin;                                                          \
    }                                                                          \
                                                                               \
    virtual void write(std::back_insert_iterator<std::vector<uint8_t> > &data) \
    {                                                                          \
        BOOST_VERIFY(boost::spirit::karma::generate(data, _generator_));       \
    }                                                                          \
                                                                               \
private:                                                                       \
    Callback callback_;                                                        \
};                                                                             \
                                                                               \
inline Status::Ptr _name_(_name_##_::Callback callback) {                      \
    return boost::make_shared<_name_##_>(callback);                            \
}

namespace PeriodicStatus {
using boost::spirit::byte_;
using boost::spirit::little_word;
using boost::spirit::little_dword;

// TOOD: Replace this with proper parsers that automatically convert fixed
// point numbers to floating point numbers.
//#define little_s8p8   attr_cast(boost::spirit::little_word)
//#define little_s16p16 attr_cast(boost::spirit::little_dword)
#define little_s8p8   boost::spirit::little_word
#define little_s16p16 boost::spirit::little_dword

JAGUAR_MAKE_STATUS(OutputVoltagePercent, int16_t, byte_(1)  << byte_(2), little_s8p8);
JAGUAR_MAKE_STATUS(BusVoltage, int16_t,  byte_(3) << byte_(4), little_s8p8)
JAGUAR_MAKE_STATUS(Current, int16_t, byte_(5) << byte_(6), little_s8p8)
JAGUAR_MAKE_STATUS(Temperature, int16_t,  byte_(7) << byte_(8), little_s8p8)
JAGUAR_MAKE_STATUS(Position, int32_t, byte_(9)  << byte_(10) << byte_(11) << byte_(12), little_s16p16)
JAGUAR_MAKE_STATUS(Speed, int32_t,  byte_(13) << byte_(14) << byte_(15) << byte_(16), little_s16p16);
JAGUAR_MAKE_STATUS(LimitNonClearing, uint8_t, byte_(17), byte_)
JAGUAR_MAKE_STATUS(LimitClearing, uint8_t, byte_(18), byte_)
JAGUAR_MAKE_STATUS(OutputVoltageVolts, int16_t, byte_(22) << byte_(23), little_s8p8)
JAGUAR_MAKE_STATUS(CurrentFaultCounter, uint8_t, byte_(24), byte_)
JAGUAR_MAKE_STATUS(TemperatureFaultCounter, uint8_t, byte_(25), byte_)
JAGUAR_MAKE_STATUS(BusVoltageFaultCounter, uint8_t, byte_(26), byte_)
JAGUAR_MAKE_STATUS(GateFaultCounter, uint8_t, byte_(27), byte_)
JAGUAR_MAKE_STATUS(CommunicationFaultCounter, uint8_t, byte_(28), byte_)

};

};

#endif
