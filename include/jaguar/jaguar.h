#ifndef JAGUAR_H_
#define JAGUAR_H_

#include <stdint.h>
#include "can_bridge.h"
#include "jaguar_api.h"

namespace jaguar {

class Jaguar {
public:
    Jaguar(can::CANBridge &can, uint8_t device_num);

    // Motor Control Configuration
    can::TokenPtr set_num_brushes(uint8_t brushes);
    can::TokenPtr set_num_encoders(uint16_t lines);
    can::TokenPtr set_fault_time(uint16_t ms);

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


};

#endif
