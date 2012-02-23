#ifndef JAGUAR_API_H_
#define JAGUAR_API_H_

namespace can {

enum DeviceType {
    kBroadcastMessage = 0,
    kRobotController,
    kMotorController,
    kRelayController,
    kGyroSensor,
    kAccelerometerSensor,
    kUltrasonicSensor,
    kFirmwareUpdate = 31
};

enum Manufacturer {
    kBroadcastMessage    = 0,
    kNationalInstruments = 1,
    kTexasInstruments    = 2,
    kDEKA                = 3
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

enum VoltageControlInterface {
    kVoltageModeEnable = 0,
    kVoltageModeDisable,
    kVoltageSet,
    kVoltageRampSet
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

enum VoltageCompensationInterface {
    kVoltageCompensationModeEnable = 0,
    kVoltageCompensationModeDisable,
    kVoltageSet,
    kVoltageRampSet,
    kVoltageCompensationRateSet
};

enum PositionControlInterface {
    kPositionModeEnable = 0,
    kPositionModeDisable,
    kPositionSet,
    kPositionProportionalConstant,
    kPositionIntegralConstant,
    kPositionDifferentialConstant,
    kPositionReference
};

enum CurrentControlInterface {
    kCurrentModeEnable = 0,
    kCurrentModeDisable,
    kCurrentSet,
    kCurrentProportionalConstant,
    kCurrentIntegralConstant,
    kCurrentDifferentialConstant
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
    kControlMode,
    kOutputVoltage
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
