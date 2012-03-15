#ifndef JAGUAR_H_
#define JAGUAR_H_

#include <list>
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

#include "can_bridge.h"
#include "jaguar.h"
#include "jaguar_api.h"


namespace jaguar {

typedef boost::function<void (void)> periodic_callback;

class Status;
class AggregateStatus;

class Jaguar {
public:
    Jaguar(can::CANBridge &can, uint8_t device_num);

    // Motor Control Configuration
    can::TokenPtr config_brushes_set(uint8_t brushes);
    can::TokenPtr config_encoders_set(uint16_t lines);
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

    // Periodic Status Updates
    can::TokenPtr perioic_enable(uint8_t index, uint16_t rate_ms, periodic_callback cb);
    can::TokenPtr periodic_disable(uint8_t index);
    can::TokenPtr periodic_config(uint8_t index, AggregateStatus statuses);

private:
    template <typename T>
    T rescale(double x);

    template <typename G>
    void send(APIClass::Enum api_class, uint8_t api_index, G const &generator);
    template <typename G>
    can::TokenPtr send_ack(APIClass::Enum api_class, uint8_t api_index, G const &generator);
    can::TokenPtr recv_ack(void);

    uint8_t const num_;
    can::CANBridge &can_;
    can::TokenPtr token_;

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
class _name_##Status_ : public Status {                                        \
public:                                                                        \
    typedef boost::shared_ptr<_name_##Status_> Ptr;                            \
    typedef boost::function<void (_Toutput_)> Callback;                        \
                                                                               \
    explicit _name_##Status_(Callback callback) : callback_(callback) {}       \
                                                                               \
    virtual uint8_t const *read(uint8_t const *begin, uint8_t const *end) {    \
        _Toutput_ output;                                                      \
        assert(boost::spirit::qi::parse(begin, end, _parser_, output));        \
        callback_(output);                                                     \
        return begin;                                                          \
    }                                                                          \
                                                                               \
    virtual void write(std::back_insert_iterator<std::vector<uint8_t> > &data) \
    {                                                                          \
        assert(boost::spirit::karma::generate(data, _generator_));             \
    }                                                                          \
                                                                               \
private:                                                                       \
    Callback callback_;                                                        \
};                                                                             \
                                                                               \
inline Status::Ptr _name_##Status(_name_##Status_::Callback callback) {        \
    return boost::make_shared<_name_##Status_>(callback);                      \
}

namespace StatusUpdates {
using boost::spirit::byte_;
using boost::spirit::little_word;
using boost::spirit::little_dword;

//JAGUAR_MAKE_STATUS(OutputVoltagePercent, double,  byte_(1)  << byte_(2), little_8p8)
//JAGUAR_MAKE_STATUS(BusVoltage, double,  byte_(3)  << byte_(4), little_8p8)
//JAGUAR_MAKE_STATUS(MotorCurrent, double,  byte_(5)  << byte_(6), little_8p8)
//JAGUAR_MAKE_STATUS(Temperature, double,  byte_(7)  << byte_(8), little_8p8)
//JAGUAR_MAKE_STATUS(Position, double,  byte_(9)  << byte_(10) << byte_(11) << byte_(12), little_16p16)
//JAGUAR_MAKE_STATUS(Speed, double,  byte_(13) << byte_(14) << byte_(15) << byte_(16), little_16p16);
//JAGUAR_MAKE_STATUS(LimitNonClearing, uint8_t, byte_(17), byte_)
//JAGUAR_MAKE_STATUS(LimitClearing, uint8_t, byte_(18), byte_)
//JAGUAR_MAKE_STATUS(OutputVoltageVolts, double,  byte_(22) << byte_(23), little_8p8)
//JAGUAR_MAKE_STATUS(CurrentFaultCounter, uint8_t, byte_(24), byte_)
//JAGUAR_MAKE_STATUS(TemperatureFaultCounter, uint8_t, byte_(25), byte_)
//JAGUAR_MAKE_STATUS(BusVoltageFaultCounter, uint8_t, byte_(26), byte_)
//JAGUAR_MAKE_STATUS(GateFaultCounter, uint8_t, byte_(27), byte_)
JAGUAR_MAKE_STATUS(CommunicationFaultCounter, uint8_t, byte_(28), byte_)

};

};

#endif