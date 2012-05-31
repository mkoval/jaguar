#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <angles/angles.h>
#include <jaguar/diff_drive.h>

using can::JaguarBridge;

namespace jaguar {

template <typename T>
inline T sgn(T x)
{
    if      (x > 0) return  1;
    else if (x < 0) return -1;
    else            return  0;
}

DiffDriveRobot::DiffDriveRobot(DiffDriveSettings const &settings)
    : bridge_(settings.port)
    , jag_broadcast_(bridge_)
    , jag_left_(bridge_, settings.id_left)
    , jag_right_(bridge_, settings.id_right)
    , diag_init_(false)
    // These are set by dynamic_reconfigure. However, there is a race condition
    // in waiting for the callback. These are sane defaults to prevent
    // generating +/-infinity or NaN during the race.
    , accel_max_(settings.accel_max_mps2)
    , wheel_circum_(0.0), wheel_sep_(0.0)
    , flip_left_((settings.flip_left) ? -1.0 : 1.0)
    , flip_right_((settings.flip_right) ? -1.0 : 1.0)
{
    // This is necessary for the Jaguars to work after a fresh boot, even if
    // we never called system_halt() or system_reset().
    block(
        jag_left_.config_brake_set(settings.brake),
        jag_right_.config_brake_set(settings.brake)
    );

    speed_init();
    odom_init();
    diag_init();
    jag_broadcast_.system_resume();
}

DiffDriveRobot::~DiffDriveRobot(void)
{
}

void DiffDriveRobot::drive(double v, double omega)
{
    if (wheel_circum_ == 0 || wheel_sep_ == 0) return;
    double const v_left  = v - 0.5 * wheel_sep_ * omega;
    double const v_right = v + 0.5 * wheel_sep_ * omega;
    drive_raw(v_left, v_right);
}

void DiffDriveRobot::drive_raw(double v_left, double v_right)
{
    if (wheel_circum_ == 0 || wheel_sep_ == 0) return;
    target_rpm_left_  = v_left  * 60 / wheel_circum_;
    target_rpm_right_ = v_right * 60 / wheel_circum_;
}

void DiffDriveRobot::drive_spin(double dt)
{
    if (wheel_circum_ == 0 || wheel_sep_ == 0) return;
    double const residual_rpm_left  = target_rpm_left_  - current_rpm_left_;
    double const residual_rpm_right = target_rpm_right_ - current_rpm_right_;

    // Cap the acceleration at the limiting value.
    double const drpm_max = accel_max_ * dt * 60 / wheel_circum_;

    if (fabs(residual_rpm_left) <= drpm_max) {
        current_rpm_left_ = target_rpm_left_;
    } else {
        current_rpm_left_ += sgn(residual_rpm_left) * drpm_max;
    }

    if (fabs(residual_rpm_right) <= drpm_max) {
        current_rpm_right_ = target_rpm_right_;
    } else {
        current_rpm_right_ += sgn(residual_rpm_right) * drpm_max;
    }

    block(
        jag_left_.speed_set(current_rpm_left_),
        jag_right_.speed_set(current_rpm_right_)
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

void DiffDriveRobot::odom_set_circumference(double circum_m)
{
    wheel_circum_ = circum_m;
}

void DiffDriveRobot::odom_set_separation(double separation_m)
{
    wheel_sep_ = separation_m;
}

void DiffDriveRobot::odom_set_encoders(uint16_t cpr)
{
    block(
        jag_left_.config_encoders_set(cpr),
        jag_right_.config_encoders_set(cpr)
    );
}

void DiffDriveRobot::odom_set_rate(uint8_t rate_ms)
{
    block(
        jag_left_.periodic_enable(0, rate_ms),
        jag_right_.periodic_enable(0, rate_ms)
    );
}

void DiffDriveRobot::diag_set_rate(uint8_t rate_ms)
{
    block(
        jag_left_.periodic_enable(1, rate_ms),
        jag_right_.periodic_enable(1, rate_ms)
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
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    // Ignore the first odometry message to establish the reference point.:
    // TODO: This is wrong.
    odom_left_.side = kLeft;
    odom_left_.init = false;
    odom_right_.side = kRight;
    odom_right_.init = false;

    // Just in case. This shouldn't really matter.
    odom_left_.pos_prev = 0;
    odom_left_.pos_curr = 0;
    odom_right_.pos_prev = 0;
    odom_right_.pos_curr = 0;

    // Configure the Jaguars to use optical encoders. They are used as both a
    // speed reference for velocity control and position reference for
    // odometry. As such, they must be configured for position control even
    // though we are are using speed control mode.
    block(
        jag_left_.position_set_reference(PositionReference::kQuadratureEncoder),
        jag_right_.position_set_reference(PositionReference::kQuadratureEncoder)
    );

    block(
        jag_left_.periodic_config_odom(0,
            boost::bind(&DiffDriveRobot::odom_update, this,
                boost::ref(odom_left_), _1, _2)),
        jag_right_.periodic_config_odom(0,
            boost::bind(&DiffDriveRobot::odom_update, this,
                boost::ref(odom_right_), _1, _2))
    );
}

void DiffDriveRobot::odom_attach(boost::function<OdometryCallback> callback)
{
    odom_signal_.connect(callback);
}

void DiffDriveRobot::diag_attach(
    boost::function<DiagnosticsCallback> callback_left,
    boost::function<DiagnosticsCallback> callback_right)
{
    diag_left_signal_.connect(callback_left);
    diag_right_signal_.connect(callback_right);
}

void DiffDriveRobot::estop_attach(boost::function<EStopCallback> callback)
{
    estop_signal_.connect(callback);
}

void DiffDriveRobot::odom_update(Odometry &odom, double pos, double vel)
{
    if (wheel_circum_ == 0 || wheel_sep_ == 0) return;

    odom.pos_prev = odom.pos_curr;
    odom.pos_curr = pos;
    odom.vel = vel;

    // Skip the first sample from each wheel. This is necessary in case the
    // encoders came up in an unknown state.
    if (!odom.init) {
        odom.pos_prev = pos;
        odom.pos_curr = pos;
        odom.init = true;
        return;
    }

    // Update the state variables to indicate which odometry readings we
    // already have. Trigger a callback once we've received a pair of readings.
    if (odom_state_ == kNone) {
        odom_state_ = odom.side;
    } else if (odom.side != odom_state_) {
        // Compute the difference between the last two updates. Speed is
        // measured in RPMs, so all of these values are measured in
        // revolutions.
        double revs_left  = odom_left_.pos_curr  - odom_left_.pos_prev;
        double revs_right = odom_right_.pos_curr - odom_right_.pos_prev;
        std::swap(revs_left, revs_right);

        // Convert from revolutions to meters.
        double const meters_left  = revs_left * wheel_circum_;
        double const meters_right = revs_right * wheel_circum_;

        // Use the robot model to convert from wheel odometry to
        // two-dimensional motion.
        // TODO: Switch to a better odometry model.
        double const meters  = (meters_left + meters_right) / 2;
        double const radians = (meters_left - meters_right) / wheel_sep_;
        x_ += meters * cos(theta_ + radians / 2);
        y_ += meters * sin(theta_ + radians / 2);
        theta_ = angles::normalize_angle(theta_ + radians);

        // Estimate the robot's current velocity.
        double const vl = odom_left_.vel * wheel_circum_ / 60;
        double const vr = odom_right_.vel * wheel_circum_ / 60;
        double const v_linear = (vr + vl) / 2;
        double const omega    = (vr - vl) / wheel_sep_;

        odom_signal_(x_, y_, theta_, v_linear, omega, meters_left, meters_right, vl, vr);
        odom_state_ = kNone;
    } else {
        std::cerr << "war: periodic update message was dropped" << std::endl;
    }
}

/*
 * Diagnostics
 */
void DiffDriveRobot::diag_init(void)
{
    block(
        jag_left_.periodic_config_diag(1,
            boost::bind(&DiffDriveRobot::diag_update, this,
                kLeft, boost::ref(diag_left_), _1, _2, _3, _4)
        ),
        jag_right_.periodic_config_diag(1,
            boost::bind(&DiffDriveRobot::diag_update, this,
                kRight, boost::ref(diag_right_), _1, _2, _3, _4)
        )
    );

    // TODO: Make this a parameter.
    block(
        jag_left_.periodic_enable(1, 500),
        jag_right_.periodic_enable(1, 500)
    );
}

void DiffDriveRobot::diag_update(
    Side side, Diagnostics &diag,
    LimitStatus::Enum limits, Fault::Enum faults,
    double voltage, double temperature)
{
    bool const estop_before = diag_left_.stopped || diag_right_.stopped;

    // TODO: Check for a fault.
    diag.stopped = !(limits & 0x03);
    diag.voltage = voltage;
    diag.temperature = temperature;

    bool const estop_after = diag_left_.stopped || diag_right_.stopped;

    // Only trigger an e-stop callback if the state changed. We don't know the
    // initial state, so the first update always triggers a callback.
    if (!diag_init_ || estop_after != estop_before) {
        estop_signal_(estop_after);
    }
    diag_init_ = true;

    // Other diagnostics (i.e. bus voltage and temperature) use separate left
    // and right callbacks.
    if (side == kLeft) {
        diag_left_signal_(voltage, temperature);
    } else if (side == kRight) {
        diag_right_signal_(voltage, temperature);
    }
}

/*
 * Speed Control
 */
void DiffDriveRobot::speed_set_p(double p)
{
    block(
        jag_left_.speed_set_p(flip_left_ * p),
        jag_right_.speed_set_p(flip_right_ * p)
    );
}

void DiffDriveRobot::speed_set_i(double i)
{
    block(
        jag_left_.speed_set_i(flip_left_ * i),
        jag_right_.speed_set_i(flip_right_ * i)
    );
}

void DiffDriveRobot::speed_set_d(double d)
{
    block(
        jag_left_.speed_set_d(flip_left_ * d),
        jag_right_.speed_set_d(flip_right_ * d)
    );
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

/*
 * Helper Methods
 */
void DiffDriveRobot::block(can::TokenPtr t1, can::TokenPtr t2)
{
    t1->block();
    t2->block();
}

};
