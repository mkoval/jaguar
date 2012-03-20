        #ifndef JAGUAR_API_H_
#define JAGUAR_API_H_

namespace jaguar {

namespace DeviceType {
    enum Enum {
        kBroadcastMessage    = 0,
        kRobotController     = 1,
        kMotorController     = 2,
        kRelayController     = 3,
        kGyroSensor          = 4,
        kAccelerometerSensor = 5,
        kUltrasonicSensor    = 6,
        kGearToothSensor     = 7,
        kFirmwareUpdate      = 31
    };
};

namespace Manufacturer {
    enum Enum {
        kBroadcastMessage    = 0,
        kNationalInstruments = 1,
        kTexasInstruments    = 2,
        kDEKA                = 3
    };
};

/*
 * API Classes
 */
namespace APIClass {
    enum Enum {
        kBroadcastMessage           = 0,
        kVoltageControl             = 0,
        kSpeedControl               = 1,
        kVoltageCompensationControl = 2,
        kPositionControl            = 3,
        kCurrentControl             = 4,
        kStatus                     = 5,
        kPeriodicStatus             = 6,
        kConfiguration              = 7,
        kAcknowledge                = 8
    };
};

/*
 * API Indexes
 */
namespace SystemControl {
    enum Enum {
        kSystemHalt        = 0,
        kSystemReset       = 1,
        kDeviceAssignment  = 2,
        kDeviceQuery       = 3,
        kHeartbeat         = 5,
        kSynchronousUpdate = 6,
        kFirmwareUpdate    = 7,
        kFirmwareVersion   = 8,
        kEnumeration       = 9,
        kSystemResume      = 10
    }; 
};

namespace VoltageControl {
    enum Enum {
        kVoltageModeEnable  = 0,
        kVoltageModeDisable = 1,
        kVoltageSet         = 2,
        kVoltageRampSet     = 3,
        kVoltageSetNoACK    = 8
    }; 
};

namespace SpeedControl {
    enum Enum {
        kSpeedModeEnable           = 0,
        kSpeedModeDisable          = 1,
        kSpeedSet                  = 2,
        kSpeedProportionalConstant = 3,
        kSpeedIntegralConstant     = 4,
        kSpeedDifferentialConstant = 5,
        kSpeedReference            = 6,
        kSpeedSetNoACK             = 11
    }; 
};

namespace VoltageCompensationControl {
    enum Enum {
        kVoltageCompensationModeEnable  = 0,
        kVoltageCompensationModeDisable = 1,
        kVoltageSet                     = 2,
        kVoltageRampSet                 = 3,
        kVoltageCompensationRateSet     = 4,
        kVoltageSetNoACK                = 9
    }; 
};

namespace PositionControl {
    enum Enum {
        kPositionModeEnable           = 0,
        kPositionModeDisable          = 1,
        kPositionSet                  = 2,
        kPositionProportionalConstant = 3,
        kPositionIntegralConstant     = 4,
        kPositionDifferentialConstant = 5,
        kPositionReference            = 6,
        kPositionSetNoACK             = 11
    }; 
}

namespace CurrentControl {
    enum Enum {
        kCurrentModeEnable      = 0,
        kCurrentModeDisable     = 1,
        kCurrentSet             = 2,
        kCurrentProportionalSet = 3,
        kCurrentIntegralSet     = 4,
        kCurrentDifferentialSet = 5,
        kCurrentSetNoACK        = 10
    }; 
}

namespace MotorControlStatus {
    enum Enum {
        kOutputVoltagePercent = 0,
        kBusVoltage           = 1,
        kCurrent              = 2,
        kTemperature          = 3,
        kPosition             = 4,
        kSpeed                = 5,
        kLimit                = 6,
        kFault                = 7,
        kPower                = 8,
        kControlMode          = 9,
        kOutputVoltageVolts   = 10,
        kStickyFault          = 11,
        kFaultCount           = 12
    }; 
};

namespace Configuration {
    enum Enum {
        kNumberOfBrushes            = 0,
        kNumberOfEncodersLines      = 1,
        kNumberOfPotentiometerTurns = 2,
        kBrakeCoastSetting          = 3,
        kLimitMode                  = 4,
        kForwardDirectionLimit      = 5,
        kReverseDirectionLimit      = 6,
        kMaximumOutputVoltage       = 7,
        kFaultTime                  = 8
    }; 
};

namespace PeriodicStatus {
    enum Enum {
        kEnableMessage    = 0,
        kConfigureMessage = 4,
        kPeriodicStatus   = 8
    }; 
};

namespace FirmwareUpdate {
    enum Enum {
        kPing     = 0,
        kDownload = 1, /* sets starting address */
        kSendData = 2,
        kReset    = 3,
        kAck      = 4,
        kRequest  = 6
    };
}

/*
 * Miscellaneous Constants
 */
namespace SpeedReference {
    enum Enum {
        kPositiveEncoder   = 0,
        kNegativeEncoder   = 2,
        kQuadratureEncoder = 3
    }; 
};

namespace PositionReference {
    enum Enum {
        kPotentiometer     = 1,
        kQuadratureEncoder = 3
    }; 
};

namespace Fault {
    enum Enum {
        kCurrentFault       = 1 << 0,
        kTemperatureFault   = 1 << 1,
        kBusVoltageFault    = 1 << 2,
        kGateDriverFault    = 1 << 3,
        kCommunicationFault = 1 << 4
    }; 
};

namespace ControlMode {
    enum Enum {
        kVoltageMode             = 0,
        kCurrentMode             = 1,
        kSpeedMode               = 2,
        kPositionMode            = 3,
        kVoltageCompensationMode = 4
    }; 
};

namespace LimitStatus {
    enum Enum {
        kForwardLimitReached           = 0,
        kReverseLimitReached           = 1,
        kSoftForwardLimitReached       = 2,
        kSoftReverseLimitReached       = 3,
        kStickyForwardLimitReached     = 4,
        kStickyReverseLimitReached     = 5,
        kStickySoftForwardLimitReached = 6,
        kStickySoftReverseLimitReached = 7
    }; 
};

namespace PeriodicStatusItem {
    enum Enum {
        kEndOfMessage              = 0,
        kOutputVoltagePercentBase  = 1,
        kBusVoltageBase            = 3,
        kMotorCurrentBase          = 5,
        kTemperatureBase           = 7,
        kPositionBase              = 9,
        kSpeedBase                 = 13,
        kLimitNonClearing          = 17,
        kLimitClearing             = 17,
        kFaults                    = 19,
        kStickyFaultsNonClearing   = 20,
        kStickyFaultsClearing      = 21,
        kOutputVoltageVolts        = 22,
        kCurrentFaultCounter       = 24,
        kTemperatureFaultCounter   = 25,
        kBusVoltageFaultCounter    = 26,
        kGateFaultCounter          = 27,
        kCommunicationFaultCounter = 28
    };
};

};

#endif

/* vim: set ts=4 et sts=4 sw=4: */
