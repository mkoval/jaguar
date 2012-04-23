#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <jaguar/diff_drive.h>

using can::JaguarBridge;

namespace jaguar {

DiffDriveRobot::DiffDriveRobot(DiffDriveSettings const &settings)
    : bridge_(settings.port)
    , jag_broadcast_(bridge_)
    , jag_left_(bridge_, settings.id_left)
    , jag_right_(bridge_, settings.id_right)
    , status_ms_(settings.status_ms)
    , robot_radius_(settings.robot_radius_m)
    , current_v_left_(0.0), current_v_right_(0.0)
    , target_v_left_(0.0), target_v_right_(0.0)
{
    // This is necessary for the Jaguars to work after a fresh boot, even if
    // we never called system_halt() or system_reset().
    // FIXME: Convert from revolutions to meters using the robot model.
    block(
        jag_left_.config_brake_set(settings.brake),
        jag_right_.config_brake_set(settings.brake)
    );

    speed_init();
    odom_init();
    jag_broadcast_.system_resume();
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
    // Convert from rev/sec to RPM, which the Jaguar expects.
    block(
        jag_left_.speed_set(v_left * 60),
        jag_right_.speed_set(v_right * 60)
    );
}

void DiffDriveRobot::drive_brake(bool braking)
{
    jaguar::BrakeCoastSetting::Enum value;
    if (braking) {
        value = jaguar::BrakeCoastSetting::kOverrideBrake;
    } else {
        value = jaguar::BrakeCoastSetting::kOverrideCoast;
    }

    block(
        jag_left_.config_brake_set(value),
        jag_right_.config_brake_set(value)
    );
}

void DiffDriveRobot::heartbeat(void)
{
    jag_broadcast_.heartbeat();
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
    block(
        jag_left_.position_set_reference(PositionReference::kQuadratureEncoder),
        jag_right_.position_set_reference(PositionReference::kQuadratureEncoder)
    );

    // FIXME: This fails if I only listen for position.
    // TODO: Combine position and speed into one callback.
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
        jag_left_.speed_set_i(i),
        jag_right_.speed_set_i(i)
    );
}

void DiffDriveRobot::speed_set_d(double d)
{
    block(
        jag_left_.speed_set_d(d),
        jag_right_.speed_set_d(d)
    );
}

void DiffDriveRobot::speed_attach(boost::function<DiffDriveRobot::SpeedCallback> cb)
{
    speed_signal_.connect(cb);
}

void DiffDriveRobot::speed_init(void)
{
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
    speed_signal_(side, s16p16_to_double(speed) / 60);
}

/*
 * Robot Parameters
 */
void DiffDriveRobot::robot_set_encoders(uint16_t ticks_per_rev)
{
    block(
        jag_left_.config_encoders_set(ticks_per_rev),
        jag_right_.config_encoders_set(ticks_per_rev)
    );
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
