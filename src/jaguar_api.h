#ifndef JAGUAR_API_H_
#define JAGUAR_API_H_

namespace can {

enum {
	kBroadcastMessage = 0
};

enum DeviceType {
    kRobotController     = 1,
    kMotorController     = 2,
    kRelayController     = 3,
    kGyroSensor          = 4,
    kAccelerometerSensor = 5,
    kUltrasonicSensor    = 6,
    kGearToothSensor     = 7
};

enum Manufacturer {
    kNationalInstruments = 1,
    kTexasInstruments    = 2,
    kDEKA                = 3
};

enum APIClass {
    kVoltageControl             = 0,
    kSpeedControl               = 1,
    kVoltageCompensationControl = 2,
    kPositionControl            = 3,
    kCurrentControl             = 4,
    kStatus                     = 5,
    kConfiguration              = 7,
    kAcknowledge                = 8
};

enum SystemControlInterface {
    kSystemHalt        = 0,
    kSystemReset       = 1,
    kDeviceAssignment  = 2,
    kDeviceQuery       = 3,
    kHeartbeat         = 5,
    kSynchronousUpdate = 6,
    kFirmwareVersion   = 8,
    kEnumeration       = 9,
    kSystemResume      = 10
};

enum SpeedControlInterface {
    kSpeedModeEnable = 0,
    kSpeedModeDisable,
    kSpeedSet,
    kSpeedProportionalConstant,
    kSpeedIntegralConstant,
    kSpeedDifferentialConstant,
    kSpeedReference
};

enum VoltageControl {
    kVoltageSet = 2
};

enum MotorControlStatus {
    kOutputVoltagePercent = 0,
    kBusVoltage = 1,
    kCurrent = 2,
    kTemperature = 3,
    kPosition = 4,
    kSpeed = 5,
    kLimit = 6,
    kFault = 7,
    kPower = 8,
    kControlMode = 9,
    kOutputVoltage = 10
};

enum MotorControlConfiguration {
    kNumberOfBrushes = 0,
    kNumberOfEncodersLines,
    kNumberOfPotentiometerTurns,
    kBreakCoastSetting,
    kLimitMode,
    kForwardDirectionLimit,
    kReverseDirectionLimit,
    kMaximumOutputVoltage,
    kFaultTime
};

enum SpeedReference {
    kPositiveEncoder = 0,
    kNegativeEncoder = 2,
    kQuadratureEncoder
};

enum Fault {
    kCurrentFault = 0,
    kTemperatureFault,
    kBusVoltageFault
};

};

#endif
