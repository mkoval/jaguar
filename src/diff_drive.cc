#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <jaguar/diff_drive.h>

using can::JaguarBridge;

namespace jaguar {

DiffDriveRobot::DiffDriveRobot(DiffDriveSettings const &settings)
    : bridge_(settings.port),
      jag_broadcast_(bridge_),
      jag_left_(bridge_, settings.id_left),
      jag_right_(bridge_, settings.id_right),
      status_ms_(settings.status_ms),
      robot_radius_(settings.robot_radius_m)
{
    // This is necessary for the Jaguars to work after a fresh boot, even if
    // we never called system_halt() or system_reset().
    // FIXME: Convert from revolutions to meters using the robot model.
    std::cout << "set encoder ticks" << std::endl;
    block(
        jag_left_.config_encoders_set(settings.ticks_per_rev),
        jag_right_.config_encoders_set(settings.ticks_per_rev)
    );
    std::cout << "set coast mode" << std::endl;
    block(
        jag_left_.config_brake_set(settings.brake),
        jag_right_.config_brake_set(settings.brake)
    );

    // FIXME: DEBUG
    status_ms_ = 100;

    speed_init();
    odom_init();

    std::cout << "system resume" << std::endl;
    jag_broadcast_.system_resume();

#if 0
    jag_left_.voltage_enable()->block();
    jag_left_.voltage_set(1.0)->block();
    jag_right_.voltage_enable()->block();
    jag_right_.voltage_set(1.0)->block();
    jag_left_.speed_enable()->block();
    jag_left_.speed_set(100)->block();

    jag_right_.speed_enable()->block();
    jag_right_.speed_set(100)->block();
#endif
}

DiffDriveRobot::~DiffDriveRobot(void)
{
}

void DiffDriveRobot::drive(double v, double omega)
{
    double const v_left  = v - robot_radius_ * omega;
    double const v_right = v + robot_radius_ * omega;
    drive_raw(v_left, v_right);
}

void DiffDriveRobot::drive_raw(double v_left, double v_right)
{
    block(
        jag_left_.speed_set(v_left),
        jag_right_.speed_set(v_right)
    );
}

void DiffDriveRobot::heartbeat(void)
{
    jag_broadcast_.heartbeat();
}

void foo(std::string side, uint32_t speed)
{
    std::cout << side << " speed = " << s16p16_to_double(speed) << std::endl;
}

/*
 * Wheel Odometry
 */
void DiffDriveRobot::odom_init(void)
{
    using jaguar::PeriodicStatus::Position;
    using jaguar::PeriodicStatus::Speed;

    odom_curr_left_  = 0;
    odom_curr_right_ = 0;
    odom_last_left_  = 0;
    odom_last_right_ = 0;
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    // Configure the Jaguars to use optical encoders. They are used as both a
    // speed reference for velocity control and position reference for
    // odometry. As such, they must be configured for position control even
    // though we are are using speed control mode.
    std::cout << "set position reference" << std::endl;
    block(
        jag_left_.position_set_reference(PositionReference::kQuadratureEncoder),
        jag_right_.position_set_reference(PositionReference::kQuadratureEncoder)
    );

    // FIXME: This fails if I only listen for position.
    // TODO: Combine position and speed into one callback.
    std::cout << "config periodic updates" << std::endl;
    block(
        jag_left_.periodic_config(0,
            Position(boost::bind(
                &DiffDriveRobot::odom_update, this,
                kLeft, boost::ref(odom_last_left_), boost::ref(odom_curr_left_), _1
            ))
            << Speed(boost::bind(&DiffDriveRobot::speed_update, this,
                kLeft, _1
            ))
        ),
        jag_right_.periodic_config(0,
            Position(boost::bind(
                &DiffDriveRobot::odom_update, this,
                kRight, boost::ref(odom_last_right_), boost::ref(odom_curr_right_), _1
            ))
            << Speed(boost::bind(&DiffDriveRobot::speed_update, this,
                kRight, _1
            ))
        )
    );
    std::cout << "enable periodic updates" << std::endl;
    block(
        jag_left_.periodic_enable(0, status_ms_),
        jag_right_.periodic_enable(0, status_ms_)
    );
}

void DiffDriveRobot::odom_attach(boost::function<OdometryCallback> callback)
{
    odom_signal_.connect(callback);
}

void DiffDriveRobot::odom_update(Side side, int32_t &last_pos, int32_t &curr_pos, int32_t new_pos)
{
#if 0
    // Keep track of the last two encoder readings to measure the change.
    last_pos = curr_pos;
    curr_pos = new_pos;

    // Update the state variables to indicate which odometry readings we already
    // have. Once we've received a pair of distinct readings, update!
    if (odom_state_ == kNone) {
        odom_state_ = side;
    } else if (odom_state_ != side) {
        odom_state_ = kNone;

        // Update the robot's pose using the new odometry data.
        double const curr_left = s16p16_to_double(odom_curr_left_);
        double const last_left = s16p16_to_double(odom_last_left_);
        double const curr_right = s16p16_to_double(odom_curr_right_);
        double const last_right = s16p16_to_double(odom_last_right_);
        double const delta_left  = curr_left - last_left;
        double const delta_right = curr_right - last_right;

        double const delta_linear  = (delta_left + delta_right) / 2;
        theta_ += (delta_right - delta_left) / (2 * robot_radius_);
        x_ += delta_linear * cos(theta_);
        y_ += delta_linear * sin(theta_);

        // FIXME: Also needs the measured velocities.
        odom_signal_(x_, y_, theta_, 0.0, 0.0, 0.0);
    } else {
        std::cerr << "war: periodic update message was dropped" << std::endl;
    }
#endif
}

/*
 * Speed Control
 */
void DiffDriveRobot::speed_set_p(double p)
{
    block(
        jag_left_.speed_set_p(p),
        jag_right_.speed_set_p(p)
    );
}

void DiffDriveRobot::speed_set_i(double i)
{
    block(
        jag_left_.speed_set_p(i),
        jag_right_.speed_set_p(i)
    );
}

void DiffDriveRobot::speed_set_d(double d)
{
    block(
        jag_left_.speed_set_p(d),
        jag_right_.speed_set_p(d)
    );
}

void DiffDriveRobot::speed_attach(boost::function<DiffDriveRobot::SpeedCallback> cb)
{
    speed_signal_.connect(cb);
}

void DiffDriveRobot::speed_init(void)
{
    std::cout << "set speed reference" << std::endl;
    block(
        jag_left_.speed_set_reference(SpeedReference::kQuadratureEncoder),
        jag_right_.speed_set_reference(SpeedReference::kQuadratureEncoder)
    );
    block(
        jag_left_.speed_enable(),
        jag_right_.speed_enable()
    );
}

void DiffDriveRobot::speed_update(DiffDriveRobot::Side side, int32_t speed)
{
    speed_signal_(side, s16p16_to_double(speed));
}

/*
 * Helper Methods
 */
void DiffDriveRobot::block(can::TokenPtr t1, can::TokenPtr t2)
{
    t1->block();
    t2->block();
}

};
