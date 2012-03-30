#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include <string>
#include <stdint.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signal.hpp>
#include <boost/thread/thread.hpp>
#include <jaguar/jaguar.h>
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
    BrakeCoastSetting::Enum brake;
};

class DiffDriveRobot
{
public:
    enum Side { kNone, kLeft, kRight };
    typedef void OdometryCallback(double, double, double, double, double, double);
    typedef void SpeedCallback(Side, double);

    DiffDriveRobot(DiffDriveSettings const &settings);
    virtual ~DiffDriveRobot(void);

    virtual void heartbeat(void);
    virtual void drive(double v, double omega);
    virtual void drive_raw(double v_left, double v_right);

    virtual void speed_set_p(double p);
    virtual void speed_set_i(double i);
    virtual void speed_set_d(double d);

    virtual void odom_attach(boost::function<OdometryCallback> callback);
    virtual void speed_attach(boost::function<SpeedCallback> cb_left);

    virtual void robot_set_encoders(uint16_t ticks_per_rev);

private:
    // Wheel Odometry
    virtual void odom_init(void);
    virtual void odom_update(Side side, int32_t &last_pos, int32_t &curr_pos, int32_t new_pos);

    // Speed Control
    virtual void speed_init(void);
    virtual void speed_update(Side side, int32_t speed);

    virtual void block(can::TokenPtr t1, can::TokenPtr t2);

    can::JaguarBridge bridge_;
    jaguar::JaguarBroadcaster jag_broadcast_;
    jaguar::Jaguar jag_left_, jag_right_;
    boost::mutex mutex_;

    uint32_t status_ms_;

    Side odom_state_;
    int32_t odom_curr_left_, odom_curr_right_;
    int32_t odom_last_left_, odom_last_right_;
    double x_, y_, theta_;
    boost::signal<OdometryCallback> odom_signal_;
    boost::signal<SpeedCallback> speed_signal_;

    double robot_radius_;
};

};

#endif
