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
    can::TokenPtr enable_voltage(void);
    can::TokenPtr disable_voltage(void);
    can::TokenPtr set_voltage(double scale);

    // Speed Control
    can::TokenPtr enable_pid(void);
    can::TokenPtr disable_pid(void);
    can::TokenPtr set_p_constant(double p);
    can::TokenPtr set_i_constant(double i);
    can::TokenPtr set_d_constant(double d);
    can::TokenPtr set_speed_reference(SpeedReference::Enum reference);
    can::TokenPtr set_speed(double speed);
    can::TokenPtr set_speed(double speed, uint8_t group);

private:
    static Manufacturer::Enum const kManufacturer;
    static DeviceType::Enum   const kDeviceType;

    uint8_t const num_;
    can::CANBridge &can_;

    void send(APIClass::Enum api_class, uint8_t api_index);
    template <typename T>
    void send(APIClass::Enum api_class, uint8_t api_index, T const &payload);

    can::TokenPtr send_ack(APIClass::Enum api_class, uint8_t api_index);
    template <typename T>
    can::TokenPtr send_ack(APIClass::Enum api_class, uint8_t api_index, T const &payload);
};


};

#endif
