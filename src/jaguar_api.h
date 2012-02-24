#ifndef JAGUAR_API_H_
#define JAGUAR_API_H_

namespace can {

enum {
	kBroadcastMessage = 0
};

enum DeviceType {
    kRobotController,
    kMotorController,
    kRelayController,
    kGyroSensor,
    kAccelerometerSensor,
    kUltrasonicSensor
};

enum Manufacturer {
    kNationalInstruments = 1,
    kTexasInstruments,
    kDEKA
};

enum APIClass {
    kVoltageControl = 0,
    kSpeedControl,
    kVoltageCompensationControl,
    kPositionControl,
    kCurrentControl,
    kStatus,
    kConfiguration = 7,
    kAcknowledge
};

enum SystemControlInterface {
    kSystemHalt = 0,
    kSystemReset,
    kDeviceAssignment,
    kDeviceQuery,
    kHeartbeat,
    kSynchronousUpdate,
    kFirmwareUpdate,
    kFirmwareVersion,
    kEnumeration,
    kSystemResume
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

enum MotorControlStatus {
    kOutputVoltage = 0,
    kBusVoltage,
    kCurrent,
    kTemperature,
    kPosition,
    kSpeed,
    kLimit,
    kFault,
    kPower,
    kControlMode
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
