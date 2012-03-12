#include <iomanip>
#include <iostream>
#include <string>
#include "jaguar.h"
#include "jaguar_bridge.h"

static std::string const kPath = "/dev/ttyUSB0";
static int const id_left  = 1;
static int const id_right = 2;

class JaguarDiffDrive {
public:
    JaguarDiffDrive(can::Jaguar &left, can::Jaguar &right);
    virtual ~JaguarDiffDrive(void);

    void heartbeat(void);
    void set_voltage(double left, double right);

private:
    can::Jaguar &m_left, &m_right;
};

JaguarDiffDrive::JaguarDiffDrive(can::Jaguar &left, can::Jaguar &right)
    : m_left(left), m_right(right)
{
    //m_left.enable_voltage();
    //m_right.enable_voltage();
}

JaguarDiffDrive::~JaguarDiffDrive(void)
{
}

void JaguarDiffDrive::heartbeat(void)
{
    m_left.heartbeat();
}

void JaguarDiffDrive::set_voltage(double left, double right)
{
    m_left.set_voltage(left);
    //m_right.set_voltage(right);
}


int main(int argc, char *argv[])
{
    can::JaguarBridge can(kPath);
    can::Jaguar jag1(can, 1);
    can::Jaguar jag2(can, 2);

    jag1.system_resume();
    jag1.enable_voltage();
    jag1.set_voltage(1.0);

    jag2.system_resume();
    jag2.enable_voltage();
    jag2.set_voltage(-1.0);


    for (;;) {
        jag1.heartbeat();
        usleep(100000);
    }

    return 0;
}
