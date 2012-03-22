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
      robot_radius_(settings.robot_radius_m)
{
    double const ticks_per_meter = settings.ticks_per_rev / (2. * M_PI * settings.wheel_radius_m);
    speed_init();
    odom_init();

    // This is necessary for the Jaguars to work after a fresh boot, even if
    // we never called system_halt() or system_reset().
    boost::mutex::scoped_lock lock(mutex_);
    block(
        jag_left_.config_encoders_set(ticks_per_meter),
        jag_right_.config_encoders_set(ticks_per_meter)
    );
    jag_broadcast_.system_resume();
}

DiffDriveRobot::~DiffDriveRobot(void)
{
    timer_.interrupt();
    timer_.join();
}

void DiffDriveRobot::drive(double v, double omega)
{
    double const v_left  = v - robot_radius_ * omega;
    double const v_right = v + robot_radius_ * omega;
    drive_raw(v_left, v_right);
}

void DiffDriveRobot::drive_raw(double v_left, double v_right)
{
    boost::mutex::scoped_lock lock(mutex_);
    block(
        jag_left_.speed_set(v_left),
        jag_right_.speed_set(v_right)
    );
}

/*
 * Wheel Odometry
 */
void DiffDriveRobot::odom_init(void)
{
    using jaguar::PeriodicStatus::Position;

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
    boost::mutex::scoped_lock lock(mutex_);
    block(
        jag_left_.position_set_reference(PositionReference::kQuadratureEncoder),
        jag_right_.position_set_reference(PositionReference::kQuadratureEncoder)
    );

    block(
        jag_left_.periodic_config(0,
            Position(boost::bind(
                &DiffDriveRobot::odom_update, this,
                kLeft, boost::ref(odom_last_left_), boost::ref(odom_curr_left_), _1
            ))
        ),
        jag_right_.periodic_config(0,
            Position(boost::bind(
                &DiffDriveRobot::odom_update, this,
                kRight, boost::ref(odom_last_right_), boost::ref(odom_curr_right_), _1
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
    boost::mutex::scoped_lock lock(mutex_);
    block(jag_left_.speed_set_p(p), jag_right_.speed_set_p(p));
}

void DiffDriveRobot::speed_set_i(double i)
{
    boost::mutex::scoped_lock lock(mutex_);
    block(jag_left_.speed_set_p(i), jag_right_.speed_set_p(i));
}

void DiffDriveRobot::speed_set_d(double d)
{
    boost::mutex::scoped_lock lock(mutex_);
    block(jag_left_.speed_set_p(d), jag_right_.speed_set_p(d));
}

void DiffDriveRobot::speed_init(void)
{
    {
        boost::mutex::scoped_lock lock(mutex_);
        block(
            jag_left_.speed_set_reference(SpeedReference::kQuadratureEncoder),
            jag_right_.speed_set_reference(SpeedReference::kQuadratureEncoder)
        );
    }
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
        {
            boost::mutex::scoped_lock lock(mutex_);
            jag_broadcast_.heartbeat();
        }

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
