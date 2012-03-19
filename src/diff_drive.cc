#include <sstream>
#include <string>
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
    std::cout << "drive before lock" << std::endl;
    can::TokenPtr cmd_left, cmd_right;
    {
        //boost::lock_guard<boost::mutex> lock(mutex_);
        std::cout << "drive after lock)" << std::endl;
        cmd_left  = jag_left_.speed_set(rpm_left);
        //cmd_right = jag_right_.speed_set(rpm_right);
    }
    std::cout << "drive unlock" << std::endl;
    block(cmd_left); //, cmd_right);
    std::cout << "drive ack" << std::endl;
}

void DiffDriveRobot::init(uint16_t status_ms)
{
    using namespace jaguar::PeriodicStatus;

    // Configure speed control to use quadrature encoders.
    std::cout << "set speed reference" << std::endl;
    block(
        jag_left_.speed_set_reference(SpeedReference::kQuadratureEncoder)
        //jag_right_.speed_set_reference(SpeedReference::kQuadratureEncoder)
    );

    std::cout << "set position reference" << std::endl;
    block(
        jag_left_.position_set_reference(PositionReference::kQuadratureEncoder)
    );

    std::cout << "set encoder lines" << std::endl;
    block(
        jag_left_.config_encoders_set(800)
    );

    // Configure callbacks for the periodic status updates.
    boost::function<void (uint32_t)> cb_enc_left  = boost::bind(
        &DiffDriveRobot::update_encoders_left, this, _1);
    boost::function<void (uint32_t)> cb_enc_right = boost::bind(
        &DiffDriveRobot::update_encoders_right, this, _1);
    boost::function<void (uint16_t)> cb_temp      = boost::bind(
        &DiffDriveRobot::update_temperature, this, _1);
    boost::function<void (uint8_t)>  cb_estop     = boost::bind(
        &DiffDriveRobot::update_estop, this, _1);


    std::cout << "periodic config" << std::endl;
    block(
        jag_left_.periodic_config(0, Position(cb_enc_left)
                                  << Temperature(cb_temp)
                                  << LimitNonClearing(cb_estop))
        //jag_right_.periodic_config(0, Position(cb_enc_right))
    );

    std::cout << "periodic enable" << std::endl;
    block(
        jag_left_.periodic_enable(0, status_ms)
        //jag_right_.periodic_enable(0, status_ms)
    );

    // Enable speed control.
    std::cout << "enable speed control" << std::endl;
    block(
        jag_left_.speed_enable()
        //jag_right_.speed_enable()
    );

    std::cout << "set p" << std::endl;
    block(
        jag_left_.speed_set_p(10000.0)
        //jag_right_.speed_set_p(10000.0)
    );

    std::cout << "resume" << std::endl;
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

void DiffDriveRobot::update_encoders_left(int32_t pos)
{
    std::cout << "x_left  = " << s16p16_to_double(pos) << "(" << pos << "(" << std::endl;
}

void DiffDriveRobot::update_encoders_right(int32_t pos)
{
    std::cout << "x_right = " << s16p16_to_double(pos) << std::endl;
}

void DiffDriveRobot::update_temperature(int16_t temp)
{
    std::cout << "temp    = " << s8p8_to_double(temp) << std::endl;
}

void DiffDriveRobot::update_estop(uint8_t state)
{
    std::cout << "estop   = " << (int)state << std::endl;
}

void DiffDriveRobot::block(can::TokenPtr t)
{
    t->block();
}

void DiffDriveRobot::block(can::TokenPtr t1, can::TokenPtr t2)
{
    t1->block();
    t2->block();
}

};

template <typename T> T convert(std::string const &str)
{
    T output;
    std::stringstream ss(str);
    ss >> output;
    return output;
}

void test(int32_t x)
{
    std::cout << "update: " << x << std::endl;
}

int main(int argc, char **argv)
{
    if (argc <= 3) {
        std::cerr << "err: incorrect number of arguments\n"
                  << "usage: ./diff_drive <port> <left id> <right id>"
                  << std::endl;
        return 0;
    }

    std::string const port = argv[1];
    uint8_t const id_left  = convert<int>(argv[2]);
    uint8_t const id_right = convert<int>(argv[3]);
    uint32_t const heartbeat_ms = 50;
    uint32_t const status_ms = 100;

    jaguar::DiffDriveRobot robot(port, id_left, id_right, heartbeat_ms, status_ms);
    robot.drive(1000.0, 1000.0);

    // Configure callbacks for the periodic status updates.

    for (;;);

    return 0;
}
