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
#include <robot_kf/WheelOdometry.h>

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
    typedef void EStopCallback(bool);
    typedef void DiagnosticsCallback(double, double);
    typedef void OdometryCallback(double, double, double, double, double, double, double);

    DiffDriveRobot(DiffDriveSettings const &settings);
    virtual ~DiffDriveRobot(void);

    virtual void heartbeat(void);
    virtual void drive(double v, double omega);
    virtual void drive_raw(double v_left, double v_right);
    virtual void drive_brake(bool braking);
    virtual void drive_spin(double dt);

    virtual void odom_set_circumference(double circum_m);
    virtual void odom_set_separation(double separation_m);
    virtual void odom_set_encoders(uint16_t cpr);
    virtual void odom_set_rate(uint8_t rate_ms);
    virtual void odom_attach(boost::function<OdometryCallback> callback);

    virtual void speed_set_p(double p);
    virtual void speed_set_i(double i);
    virtual void speed_set_d(double d);

    virtual void diag_attach(
        boost::function<DiagnosticsCallback> callback_left,
        boost::function<DiagnosticsCallback> callback_right);
    virtual void diag_set_rate(uint8_t rate_ms);

    virtual void estop_attach(boost::function<EStopCallback> callback);

private:
    // Wheel Odometry
    struct Odometry {
        Odometry(void)
            : init(false)
            , pos_curr(0.0)
            , pos_prev(0.0)
            , vel(0.0)
        {}

        Side side;
        bool init;
        double pos_curr, pos_prev;
        double vel;
    };

    virtual void odom_init(void);
    virtual void odom_update(Odometry &side, double pos, double vel);

    // Speed Control
    virtual void speed_init(void);
    
    // Diagnostics
    struct Diagnostics {
        bool init;
        bool stopped;
        double voltage;
        double temperature;
    };

    void diag_init(void);
    void diag_update(Side side, Diagnostics &diag,
                     LimitStatus::Enum limits, Fault::Enum faults,
                     double voltage, double temperature);

    virtual void block(can::TokenPtr t1, can::TokenPtr t2);

    can::JaguarBridge bridge_;
    jaguar::JaguarBroadcaster jag_broadcast_;
    jaguar::Jaguar jag_left_, jag_right_;
    boost::mutex mutex_;

    // Odometry
    Side odom_state_;
    Odometry odom_left_, odom_right_;
    double x_, y_, theta_;
    boost::signal<OdometryCallback> odom_signal_;
    double wheel_circum_, wheel_sep_;

    // Status message
    bool diag_init_;
    Diagnostics diag_left_, diag_right_;
    boost::signal<EStopCallback> estop_signal_;
    boost::signal<DiagnosticsCallback> diag_left_signal_, diag_right_signal_;

    // Acceleration limiting code.
    double current_rpm_left_, current_rpm_right_;
    double target_rpm_left_, target_rpm_right_;
    double accel_max_;
};

};

#endif
