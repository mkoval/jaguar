#ifndef JAGUAR_H_
#define JAGUAR_H_

#include <stdint.h>
#include "can_bridge.h"
#include "jaguar_api.h"

namespace can {

class Jaguar {
public:
    Jaguar(CANBridge &can, uint8_t device_num);

    // Broadcast: These apply to all devices on the network.
    void system_reset(void);
    void system_halt(void);
    void system_resume(void);
    void heartbeat(void);
    void device_assignment(uint8_t id);
    void synchronous_update(uint8_t group);

    void set_voltage(void);

    // Motor Control Configuration
    void set_encoder_lines(uint16_t lines);
    void set_fault_time(uint16_t ms);

    // Motor Control Status
    double get_bus_voltage(void);
    double get_current(void);
    double get_temperature(void);
    double get_speed(void);
    double get_position(void);
    double get_output_voltage(void);
    Fault get_fault(void);

    // Voltage Control
    void enable_voltage(void);
    void disable_voltage(void);
    void set_voltage(double scale);
    void set_voltage_ramp(double rate);

    // Speed Control
    void enable_pid(void);
    void disable_pid(void);
    void set_p_constant(double p);
    void set_i_constant(double i);
    void set_d_constant(double d);
    void set_speed_reference(SpeedReference reference);
    void set_speed(double speed);
    void set_speed(double speed, uint8_t group);

private:
    CANBridge &m_can;
    uint32_t m_num;

    static uint8_t const m_manufacturer;
    static uint8_t const m_type;

    // Conversion between floating and fixed point numbers
    static int16_t double_to_s8p8(double x);
    static int32_t double_to_s16p16(double x);
    static double s8p8_to_double(int16_t x);
    static double s16p16_to_double(int32_t x);

    static uint32_t pack_id(uint8_t device_number, uint8_t manufacturer, uint8_t type, uint16_t api);
    static uint32_t pack_id(uint8_t device_number, uint8_t manufacturer, uint8_t type,
                            uint8_t api_class, uint8_t api_index);
};

};

#endif
