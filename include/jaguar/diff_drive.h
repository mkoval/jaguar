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
    std::string port;
    int id_left, id_right;
    int heartbeat_ms;
    int status_ms;
    double ticks_per_m;
    double wheel_radius_m;
    double robot_radius_m;
};

class DiffDriveRobot
{
public:
    typedef void OdometryCallback(double, double, double, double, double, double);

    DiffDriveRobot(DiffDriveSettings const &settings);
    virtual ~DiffDriveRobot(void);

    virtual void drive(double v, double omega);
    virtual void drive_raw(double rpm_left, double rpm_right);

    virtual void speed_set_p(double p);
    virtual void speed_set_i(double i);
    virtual void speed_set_d(double d);

    virtual void odom_attach(boost::function<OdometryCallback> callback);

private:
    enum Side { kNone, kLeft, kRight };

    // Wheel Odometry
    virtual void odom_init(void);
    virtual void odom_update(Side side, int32_t pos);
    virtual double odom_get_delta(double before, double after);

    // Speed Control
    virtual void speed_init(void);

    virtual void heartbeat(void);
    virtual void block(can::TokenPtr t1, can::TokenPtr t2);

    can::JaguarBridge bridge_;
    jaguar::JaguarBroadcaster jag_broadcast_;
    jaguar::Jaguar jag_left_, jag_right_;
    boost::mutex mutex_;

    uint32_t status_ms_;
    boost::posix_time::time_duration timer_period_;
    boost::thread timer_;

    Side odom_state_;
    double odom_left_, odom_right_;
    double odom_last_left_, odom_last_right_;
    double x_, y_, theta_;
    boost::signal<OdometryCallback> odom_signal_;

    double robot_radius_;
    double wheel_radius_;
};

};

#endif
