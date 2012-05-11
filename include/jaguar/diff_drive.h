#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include <string>
#include <stdint.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signal.hpp>
#include <boost/thread/thread.hpp>
#include <jaguar/jaguar.h>
#include <jaguar/jaguar_api.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_broadcaster.h>

namespace jaguar {

struct DiffDriveSettings {
    // CAN bus Configuration
    std::string port;
    int id_left, id_right;
    // Periodic Messages
    int heartbeat_ms, status_ms;
    // Robot Model Parameters
    uint16_t ticks_per_rev;
    double wheel_radius_m;
    double robot_radius_m;
    double accel_max_mps2;
    BrakeCoastSetting::Enum brake;
};

class DiffDriveRobot
{
public:
    enum Side { kNone, kLeft, kRight };
    typedef void DiagnosticsCallback(LimitStatus::Enum, Fault::Enum,
                                     double, double);
    typedef void OdometryCallback(double, double, double, double, double);

    DiffDriveRobot(DiffDriveSettings const &settings);
    virtual ~DiffDriveRobot(void);

    virtual void heartbeat(void);
    virtual void drive(double v, double omega);
    virtual void drive_raw(double v_left, double v_right);
    virtual void drive_brake(bool braking);
    virtual void drive_spin(double dt);

    virtual void speed_set_p(double p);
    virtual void speed_set_i(double i);
    virtual void speed_set_d(double d);

    virtual void odom_attach(boost::function<OdometryCallback> callback);

    virtual void robot_set_encoders(uint16_t ticks_per_rev);
    virtual void robot_set_radii(double wheel_radius, double robot_radius);

private:
    // Wheel Odometry
    virtual void odom_init(void);
    virtual void odom_update(Side side, double &last_pos, double &curr_pos,
                             double new_pos, double velocity);

    // Speed Control
    virtual void speed_init(void);

    virtual void block(can::TokenPtr t1, can::TokenPtr t2);

    
    // Diagnostics
    void diag_init(void);
    void diag_update(Side side, LimitStatus::Enum limits, Fault::Enum faults,
                     double voltage, double temperature);

    can::JaguarBridge bridge_;
    jaguar::JaguarBroadcaster jag_broadcast_;
    jaguar::Jaguar jag_left_, jag_right_;
    boost::mutex mutex_;

    uint32_t status_ms_;

    // Odometry
    Side odom_state_;
    double velocity_left_, velocity_right_;
    double odom_curr_left_, odom_curr_right_;
    double odom_last_left_, odom_last_right_;
    double x_, y_, theta_;
    boost::signal<OdometryCallback> odom_signal_;
    double robot_radius_;
    double wheel_circum_;

    // Acceleration limiting code.
    double current_rpm_left_, current_rpm_right_;
    double target_rpm_left_, target_rpm_right_;
    double accel_max_;
};

};

#endif
