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
    void set_num_brushes(uint8_t brushes);
    void set_num_encoders(uint16_t lines);
    void set_fault_time(uint16_t ms);

    // Voltage Control
    void enable_voltage(void);
    void disable_voltage(void);
    void set_voltage(double scale);

    // Speed Control
    void enable_pid(void);
    void disable_pid(void);
    void set_p_constant(double p);
    void set_i_constant(double i);
    void set_d_constant(double d);
    void set_speed_reference(SpeedReference::Enum reference);
    void set_speed(double speed);
    void set_speed(double speed, uint8_t group);

private:
    static Manufacturer::Enum const kManufacturer;
    static DeviceType::Enum   const kDeviceType;

    uint8_t const num_;
    can::CANBridge &can_;

    void send(APIClass::Enum api_class, uint8_t api_index);
    template <typename T>
    void send(APIClass::Enum api_class, uint8_t api_index, T const &payload);
};


};

#endif
