#ifndef JAGUAR_H_
#define JAGUAR_H_

#include <stdint.h>
#include "can_bridge.h"
#include "jaguar_api.h"

namespace can {

class Jaguar {
public:
    Jaguar(CANBridge &can);

    // Broadcast: These apply to all devices on the network.
    void system_reset(void);
    void system_halt(void);
    void system_resume(void);
    void heartbeat(void);
    void device_assignment(uint8_t id);
    void synchronous_update(uint8_t group);

    // Motor Control Configuration
    void set_encoder_lines(uint16_t lines);
    void set_fault_time(uint16_t ms);

    // Motor Control Status
    double get_bus_voltage(void);
    double get_current(void);
    double get_temperature(void);
    double get_speed(void);
    Fault get_fault(void);

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
    CAN_Bridge &m_can;
    uint32_t m_num;

    static uint8_t m_manufacturer = kTexasInstruments;
    static uint8_t m_type = kMotorController,

    // Conversion between floating and fixed point numbers
    static inline uint32_t double_to_u16p16(double x);
    static inline int32_t  double_to_s16p16(double x);
    static inline double u16p16_to_double(uint32_t x);
    static inline double s16p16_to_double(int32_t x);

    // Packing and unpacking of CAN ids
    static inline uint8_t unpack_device_number(uint32_t id);
    static inline uint8_t unpack_manufacturer(uint32_t id);
    static inline uint8_t unpack_type(uint32_t id);
    static inline uint16_t unpack_type(uint32_t id);

    static uint16_t pack_api(uint8_t api_class, uint8_t api_index);
    static uint32_t pack_id(uint8_t device_number, uint8_t manufacturer, uint8_t type, uint16_t api);
    static uint32_t pack_id(uint8_t device_number, uint8_t manufacturer, uint8_t type,
                            uint8_t api_class, uint8_t api_index);
};

};

#endif
