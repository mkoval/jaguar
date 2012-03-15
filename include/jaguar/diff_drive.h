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

class DiffDriveRobot
{
public:
    DiffDriveRobot(std::string port, uint8_t left_id, uint8_t right_id, uint32_t heartbeat_ms, uint32_t status_ms);
    virtual ~DiffDriveRobot(void);

    virtual void drive(double rpm_left, double rpm_right);

private:
    virtual void init(uint16_t status_ms);
    virtual void heartbeat(void);
    virtual void update_encoders_left(int32_t pos);
    virtual void update_encoders_right(int32_t pos);
    virtual void update_temperature(int16_t temp);
    virtual void update_estop(uint8_t state);

    virtual void block(can::TokenPtr t1);
    virtual void block(can::TokenPtr t1, can::TokenPtr t2);

    can::JaguarBridge bridge_;
    jaguar::JaguarBroadcaster jag_broadcast_;
    jaguar::Jaguar jag_left_, jag_right_;
    boost::mutex mutex_;

    boost::posix_time::time_duration timer_period_;
    boost::thread timer_;

    uint32_t ticks_left_, ticks_right_;
};

};

#endif
