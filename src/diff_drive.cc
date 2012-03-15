#include <jaguar/diff_drive.h>

using can::JaguarBridge;

namespace jaguar {

DiffDriveRobot::DiffDriveRobot(
        std::string port,
        uint8_t left_id, uint8_t right_id,
        uint32_t heartbeat_ms, uint32_t status_ms
    )
    : bridge_(port),
      jag_broadcast_(bridge_),
      jag_left_(bridge_, left_id),
      jag_right_(bridge_, right_id),
      timer_period_(boost::posix_time::milliseconds(heartbeat_ms)),
      timer_(boost::bind(&DiffDriveRobot::heartbeat, this)),
      ticks_left_(0),
      ticks_right_(0)
{
    init(status_ms);
}

DiffDriveRobot::~DiffDriveRobot(void)
{
    timer_.interrupt();
    timer_.join();
}

void DiffDriveRobot::drive(double rpm_left, double rpm_right)
{
    can::TokenPtr cmd_left, cmd_right;
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        cmd_left  = jag_left_.speed_set(rpm_left);
        cmd_right = jag_right_.speed_set(rpm_right);
    }
    block(cmd_left, cmd_right);
    }

void DiffDriveRobot::init(uint16_t status_ms)
{
    using namespace jaguar::PeriodicStatus;

    // Configure callbacks for the periodic status updates.
    boost::function<void (uint32_t)> cb_enc_left  = boost::bind(
        &DiffDriveRobot::update_encoders_left, this, _1);
    boost::function<void (uint32_t)> cb_enc_right = boost::bind(
        &DiffDriveRobot::update_encoders_right, this, _1);
    boost::function<void (uint16_t)> cb_temp      = boost::bind(
        &DiffDriveRobot::update_temperature, this, _1);
    boost::function<void (uint8_t)>  cb_estop     = boost::bind(
        &DiffDriveRobot::update_estop, this, _1);
    block(
        jag_left_.periodic_config(0, Position(cb_enc_left)
                                  << Temperature(cb_temp)
                                  << LimitNonClearing(cb_estop)),
        jag_right_.periodic_config(0, Position(cb_enc_right))
    );
    block(
        jag_left_.periodic_enable(0, status_ms),
        jag_right_.periodic_enable(0, status_ms)
    );

    // Configure speed control to use quadrature encoders.
    block(
        jag_left_.speed_set_reference(SpeedReference::kQuadratureEncoder),
        jag_right_.speed_set_reference(SpeedReference::kQuadratureEncoder)
    );

    // Enable speed control.
    block(
        jag_left_.speed_enable(),
        jag_right_.speed_enable()
    );

    jag_broadcast_.system_resume();
}

void DiffDriveRobot::heartbeat(void)
{
    for (;;) {
        boost::lock_guard<boost::mutex> lock(mutex_);
        jag_broadcast_.heartbeat();

        try {
            boost::this_thread::sleep(timer_period_);
        } catch (boost::thread_interrupted const &e) {
            break;
        }
    }
}

void DiffDriveRobot::update_encoders_left(double pos)
{
    std::cout << "x_left  = " << s16p16_to_double(pos) << std::endl; 
}

void DiffDriveRobot::update_encoders_right(double pos)
{
    std::cout << "x_right = " << s16p16_to_double(pos) << std::endl; 
}

void DiffDriveRobot::update_temperature(double temp)
{
    std::cout << "temp    = " << s8p8_to_double(temp) << std::endl; 
}

void DiffDriveRobot::update_estop(uint8_t state)
{
    std::cout << "estop   = " << state << std::endl; 
}

void DiffDriveRobot::block(can::TokenPtr t1, can::TokenPtr t2)
{
    t1->block();
    t2->block();
}

};

int main(int argc, char **argv)
{
    std::string port;
    uint8_t id_left(1), id_right(2);
    uint32_t heartbeat_ms(50), status_ms(100);

    jaguar::DiffDriveRobot robot(port, id_left, id_right, heartbeat_ms, status_ms);
    robot.drive(1.0, 1.0);

    for (;;);

    return 0;
}
