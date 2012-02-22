import struct
from collections import namedtuple
from datetime import datetime, timedelta

GenericCANMsg = namedtuple('GenericCANMsg', [ 'device_type', 'manufacturer',
                                              'api_class', 'api_key',
                                              'device_number', 'payload' ])
MotorCtrl     = namedtuple('MotorCtrl',     [ 'device_type', 'manufacturer',
                                              'device_number' ])
class CAN:
    class DeviceType:
        BroadcastMessages   = 0
        RobotController     = 1
        MotorController     = 2
        RelayController     = 3
        GyroSensor          = 4
        AccelerometerSensor = 5
        UltrasonicSensor    = 6
        GearToothSensor     = 7
        FirmwareUpdate      = 31

    class Manufacturer:
        BroadcastMessages   = 0
        NationalInstruments = 1
        TexasInstruments    = 2
        DEKA                = 3

    class APIClass:
        VoltageControl      = 0
        SpeedControl        = 1
        VoltageCompControl  = 2
        PositionControl     = 3
        CurrentControl      = 4
        Status              = 5
        Configuration       = 7
        Acknowledge         = 8

    class SystemControl:
        SystemHalt          = 0
        SystemReset         = 1
        DeviceAssignment    = 2
        DeviceQuery         = 3
        Heartbeat           = 5
        SynchronousUpdate   = 6
        FirmwareUpdate      = 7
        FirmwareVersion     = 8
        Enumeration         = 9
        SystemResume        = 10

    class SpeedControl:
        SpeedModeEnable     = 0
        SpeedModeDisable    = 1
        SpeedSet            = 2
        SpeedPConstant      = 3
        SpeedIConstant      = 4
        SpeedDConstant      = 5
        SpeedReference      = 6

    class MotorControlStatus:
        OutputVoltagePerc   = 0
        BusVoltage          = 1
        Current             = 2
        Temperature         = 3
        Position            = 4
        Speed               = 5
        Limit               = 6
        Fault               = 7
        Power               = 8
        ControlMode         = 9
        OutputVoltageVolts  = 10

    class MotorControlConfiguration:
        NumBrushes          = 0
        NumEncoderLines     = 1
        NumPotTurns         = 2
        BreakCoastSetting   = 3
        LimitMode           = 4
        ForwardDirLimit     = 5
        ReverseDirLimit     = 6
        MaximumOutputVolt   = 7
        FaultTime           = 8

    class SpeedReference:
        Positive   = 0
        Negative   = 2
        Quadrature = 3

class BroadcastCAN:

    enumerate_timeout = timeval(milliseconds=80)

    def __init__(self, bridge):
        self.bridge = bridge

    def _broadcast(self, api_key, payload):
        self.bridge.send_message(GenericCANMsg(
            device_type   = CAN.DeviceType.BroadcastMessages,
            manufacturer  = CAN.Manufacturer.BroadcastMessages,
            api_class     = 0, # arbitrary
            api_key       = api_key,
            device_number = 0, # arbitrary
            payload       = payload
        ))

    def halt(self):
        '''
        Stop driving all motors and go into a neutral state. No motors can not
        be driven again until either a resume has been issued.
        '''
        self._broadcast(CAN.SystemControl.Halt, b'')

    def resume(self):
        '''
        Enable motor control. See halt() for more information.
        '''
        self._broadcast(CAN.SystemControl.Resume, b'')

    def update(self, mask):
        '''
        Trigger a synchronous update all motor controllers with a group that
        matches the bitmask.
        '''
        self._broadcast(CAN.SynchronousUpdate, struct.pack('B', mask))

    def heartbeat(self):
        '''
        Reset the watchdogs on all connected motor controllers. Every motor
        controller must receive a minimum of one CAN message every 100 ms to
        avoid entering a fault state.
        '''
        self._broadcast(CAN.Heartbeat, b'')

    def enumerate(self):
        '''
        Obtain a list of motor controllers connected to this CAN bus bridge by
        requesting each controller to enumerate itself. This enumeration takes
        a minimum of 80 ms.
        '''
        self._broadcast(CAN.Enumeration, b'')

        controllers = list()
        end = datetime.now() + self.enumerate_timeout
        while datetime.now() < end:
            remaining = (end - datetime.now()).total_seconds()
            msg = self.bridge.recv_message(timeout=remaining)

            if msg:
                ctrl = MotorCtrl(
                    device_type=msg.device_type,
                    manufacturer=msg.manufacturer,
                    device_number=msg.device_number
                )
                controllers.append(ctrl)

        return ids

class JaguarCAN:
    def __init__(self, bridge, device_number):
        self.bridge = bridge
        self.number = device_number

    def enable(self):
        '''
        Enables speed control and sets the output to neutral.
        '''
        self._send_message(CAN.SpeedControl.SpeedModeEnable, b'')

    def disable(self):
        '''
        Disables speed control, returns to the default control mode, and sets
        the output to neutral.
        '''
        self._send_message(CAN.SpeedControl.SpeedModeDisable, b'')

    def set_speed(self, speed, group=0):
        '''
        Sets the target rotational speed in revolutions per minute.
        '''
        payload = self._to16p16(speed) + struct.pack('B', group)
        self._send_message(self.SpeedControl.SpeedSet, payload)

    def set_constants(self, kp, ki, kd):
        '''
        Sets the PID control constants.
        '''
        self._send_message(CAN.SpeedControl.SetPConstant, self._to16p16(kp))
        self._send_message(CAN.SpeedControl.SetIConstant, self._to16p16(ki))
        self._send_message(CAN.SpeedControl.SetDConstant, self._to16p16(kp))

    def set_reference(self, reference):
        payload = struct.pack('B', reference)
        self._send_message(CAN.SpeedControl.SpeedReference, payload)

    def set_encoder_lines(self, ticks_per_rev):
        payload = struct.pack('<H', ticks_per_rev)
        self._send_message(CAN.Configuration.EncoderLines, payload)

    def _send_message(self, api_key, payload):
        self.bridge.send_message(GenericCANMsg(
            device_type   = CAN.DeviceType.MotorController,
            manufacturer  = CAN.Manufacturer.TexasInstruments,
            api_class     = CAN.APIClass.SpeedControl,
            api_key       = api_key,
            device_number = self.number,
            payload       = payload
        ))

    @staticmethod
    def _to16p16(x):
        return struct.pack('<i', x * (2 ** 16))
