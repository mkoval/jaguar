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
      timer_period_(boost::posix_time::milliseconds(settings.heartbeat_ms)),
      timer_(boost::bind(&DiffDriveRobot::heartbeat, this)),
      odom_last_left_(0.), odom_last_right_(0.),
      x_(0.), y_(0.), theta_(0.),
      robot_radius_(settings.robot_radius_m),
      wheel_radius_(settings.wheel_radius_m)
{
    speed_init();
    odom_init();

    // This is necessary for the Jaguars to work atfter a fresh boot, even if
    // we never called system_halt() or system_reset().
    jag_broadcast_.system_resume();
}

DiffDriveRobot::~DiffDriveRobot(void)
{
    timer_.interrupt();
    timer_.join();
}

void DiffDriveRobot::drive(double v, double omega)
{
    // TODO: Figure out the inverse kinematics for this.
    assert(false);
}

void DiffDriveRobot::drive_raw(double rpm_left, double rpm_right)
{
    block(
        jag_left_.speed_set(rpm_left),
        jag_right_.speed_set(rpm_right)
    );
}

/*
 * Wheel Odometry
 */
void DiffDriveRobot::odom_init(void)
{
    using jaguar::PeriodicStatus::Position;

    // Configure the Jaguars to use optical encoders. They are used as both a
    // speed reference for velocity control and position reference for
    // odometry. As such, they must be configured for position control even
    // though we are are using speed control mode.
    block(
        jag_left_.position_set_reference(PositionReference::kQuadratureEncoder),
        jag_right_.position_set_reference(PositionReference::kQuadratureEncoder)
    );
    block(
        jag_left_.config_encoders_set(800),
        jag_right_.config_encoders_set(800)
    );

    boost::function<void (uint32_t)> cb_l = boost::bind(&DiffDriveRobot::odom_update, this, kLeft,  _1);
    boost::function<void (uint32_t)> cb_r = boost::bind(&DiffDriveRobot::odom_update, this, kRight, _1);
    block(
        jag_left_.periodic_config(0, Position(cb_l)),
        jag_right_.periodic_config(0, Position(cb_r))
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

void DiffDriveRobot::odom_update(Side side, int32_t pos)
{
    double const position = s16p16_to_double(pos);

    assert(side == kLeft || side == kRight);
    if (side == kLeft) {
        odom_left_ = position;
    } else if (side == kRight) {
        odom_right_ = position;
    }

    // Update the state variables to indicate which odometry readings we are
    // waiting for. When we've received a pair of distinct readings we will
    // use them to update our pose estimate.
    if (odom_state_ == kNone) {
        odom_state_ = side;
    } else if (odom_state_ != side) {
        odom_state_ = kNone;

        // Update the robot's pose using the new odometry data.
        double const delta_left  = odom_get_delta(odom_last_left_, odom_left_);
        double const delta_right = odom_get_delta(odom_last_right_, odom_right_);

        double const delta_linear  = (delta_left + delta_right) / 2;
        theta_ += (delta_right - delta_left) / (2 * robot_radius_);
        x_ += delta_linear * cos(theta_);
        y_ += delta_linear * sin(theta_);

        // TODO: Also include the estimated linear and angular velocity.
        odom_signal_(x_, y_, theta_, 0.0, 0.0, 0.0);

        odom_last_left_  = odom_left_;
        odom_last_right_ = odom_right_;
    } else {
        std::cerr << "war: periodic update message was dropped" << std::endl;
    }
}

double DiffDriveRobot::odom_get_delta(double before, double after)
{
    // Calculate the achievable range of position values.
    double const min_s16p16 = s16p16_to_double(std::numeric_limits<int16_t>::max());
    double const max_s16p16 = s16p16_to_double(std::numeric_limits<int16_t>::min());
    double const delta_max = max_s16p16 - min_s16p16;

    // Assume that the the sample rate is high enough so the position counter
    // will overflow at most once between subsequent samples. Therefore, an
    // overflow occured if the position changes by "enough".
    double delta = after - before;

    if (delta >= delta_max / 2) {
        // TODO: Verify that this math actually compensates for overflow.
        return delta_max - delta;
    } else {
        return delta;
    }
}

/*
 * Speed Control
 */
void DiffDriveRobot::speed_set_p(double p)
{
    block(jag_left_.speed_set_p(p), jag_right_.speed_set_p(p));
}

void DiffDriveRobot::speed_set_i(double i)
{
    block(jag_left_.speed_set_p(i), jag_right_.speed_set_p(i));
}

void DiffDriveRobot::speed_set_d(double d)
{
    block(jag_left_.speed_set_p(d), jag_right_.speed_set_p(d));
}

void DiffDriveRobot::speed_init(void)
{
    block(
        jag_left_.speed_set_reference(SpeedReference::kQuadratureEncoder),
        jag_right_.speed_set_reference(SpeedReference::kQuadratureEncoder)
    );
    speed_set_p(1000.0);
    speed_set_i(0.0);
    speed_set_d(0.0);
}

/*
 * Helper Methods
 */
void DiffDriveRobot::heartbeat(void)
{
    for (;;) {
        jag_broadcast_.heartbeat();

        // We must gracefully handle interruption because the destructor
        // interrupts this thread to cleanly exit.
        try {
            boost::this_thread::sleep(timer_period_);
        } catch (boost::thread_interrupted const &e) {
            break;
        }
    }
}

void DiffDriveRobot::block(can::TokenPtr t1, can::TokenPtr t2)
{
    t1->block();
    t2->block();
}

};
