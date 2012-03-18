#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include <string>
#include <stdint.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <jaguar/jaguar.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_broadcaster.h>

namespace jaguar {

struct DiffDriveSettings {
    uint8_t id_left, id_right;
    uint32_t heartbeat_ms;
    uint32_t status_ms;
};

class DiffDriveRobot
{
public:
    DiffDriveRobot(std::string port, uint8_t left_id, uint8_t right_id, uint32_t heartbeat_ms, uint32_t status_ms);
    virtual ~DiffDriveRobot(void);

    virtual void drive(double v, double omega);
    virtual void drive_raw(double rpm_left, double rpm_right);

    virtual void speed_set_p(double p);
    virtual void speed_set_i(double i);
    virtual void speed_set_d(double d);

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

    double radius_robot_;
    double radius_wheel_;
};

};

#endif
