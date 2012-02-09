import bitstring, serial, struct
from collections import namedtuple, OrderedDict
from datetime import datetime, timedelta

SerialPayload = namedtuple('SerialPayload', [ 'device_id', 'payload' ])
GenericCANMsg = namedtuple('GenericCANMsg', [ 'device_type', 'manufacturer',
                                              'api_class', 'api_key',
                                              'device_number', 'payload' ])
MotorCtrl     = namedtuple('MotorCtrl',     [ 'device_type', 'manufacturer',
                                              'device_number' ])

class Packetizer:
    def __init__(self, sof, esc, sof_esc, esc_esc):
        self.sof = bytearray([ sof ])
        self.esc = bytearray([ esc ])
        self.esc_sof = bytearray([ sof_esc ])
        self.esc_esc = bytearray([ esc_esc ])
        self._reset()

    def recv_byte(self, byte):
        byte_dec = self._decode(self.last, byte)

        # Every start of frame (SOF) byte starts a new frame because it is
        # otherwise escaped.
        if byte == self.sof:
            self._reset()
        # Next byte is the total number of bytes in the packet. Note that this
        # could be encoded if it equals 255, although this should never occur
        # in practice when framing CANbus messages.
        elif self.offset == 1:
            self.count   = ord(byte_dec)
            self.offset += 1
        elif byte_dec:
            self.packet.extend(byte_dec)
            self.offset += 1

        self.last = byte

        # Done receiving the entire packet, so add it to the queue. Padding
        # compensates for fields in the packet (e.g. device id) that don't
        # contribute to the byte count.
        if self.valid and self.offset == self.count + 2:
            new_packet = self.packet
            self._reset()
            return new_packet
        else:
            return None

    def frame_bytes(self, payload):
        count = len(payload)
        contents = struct.pack('B{}s'.format(count), count, payload)
        return self.sof + self._encode(contents)

    def _reset(self):
        self.offset = 1
        self.count  = 0
        self.last   = None
        self.packet = bytearray()
        self.valid  = True

    def _error(self):
        self.valid = False

    def _decode(self, last, curr):
        if last == self.esc:
            if curr == self.esc_esc:
                return self.esc
            elif curr == self.esc_sof:
                return self.sof
        elif curr == self.esc:
            return None
        else:
            return curr

    def _encode(self, raw):
        encoded = bytearray()
        for curr in raw:
            if curr == self.sof:
                encoded.extend(self.esc)
                encoded.extend(self.esc_sof)
            elif curr == self.esc:
                encoded.extend(self.esc)
                encoded.extend(self.esc_esc)
            else:
                encoded.extend(curr)
        return encoded

class JaguarUART:
    header_fields = OrderedDict([
        ('resv',          'uint:3'),
        ('device_type',   'uint:5'),
        ('manufacturer',  'uint:8'),
        ('api_class',     'uint:6'),
        ('api_key',       'uint:4'),
        ('device_number', 'uint:6')
    ])
    header_fmt = [ field + '=' + name for name, field in header_fields.items() ]

    def __init__(self, serial, packetizer):
        self.serial = serial
        self.packetizer = packetizer
        self.alive = True

    def close(self):
        self.alive = False

    @classmethod
    def parse_message(cls, packet):
        header_raw = bitstring.BitArray(bytes=packet[0:4])
        header_raw.byteswap()

        header_list = header_raw.unpack(cls.header_fmt)
        header = dict(zip(cls.header_fields.keys(), header_list))
        del header['resv']
        return GenericCANMsg(payload=packet[4:], **header)

    @classmethod
    def generate_message(cls, msg):
        header = bitstring.pack(cls.header_fmt, resv=0, **msg._asdict())
        header.byteswap()
        return header.bytes + msg.payload

    def send_message(self, msg):
        packed = self.generate_message(msg)
        framed = self.packetizer.frame_bytes(packed)
        self.serial.write(framed)

    def recv_message(self, timeout=None):
        self.timeout = timeout
        while True:
            # This may timeout and return zero bytes, so timeout here as well.
            byte = self.serial.read(1)
            if byte:
                return None

            # Feed the packetizing state machine the next character.
            packet = self.packetizer.recv_byte(byte)
            if packet:
                return self.parse_message(packet)

    def pending_data(self):
        return self.serial.inWaiting() > 0

class JaguarCAN:
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

    enumerate_timeout = timeval(milliseconds=80)

    def __init__(self, bridge):
        self.bridge = bridge

    def halt(self):
        '''
        Stop driving all motors and go into a neutral state. No motors can not
        be driven again until either a resume has been issued.
        '''
        self.bridge.send_message(GenericCANMsg(
            device_type   = self.DeviceType.BroadcastMessages,
            manufacturer  = self.Manufacturer.BroadcastMessages,
            api_class     = 0, # arbitrary
            api_key       = self.SystemControl.Halt,
            device_number = 0, # arbitrary
            payload       = b''
        ))

    def resume(self):
        '''
        Enable motor control. See halt() for more information.
        '''
        self.bridge.send_message(GenericCANMsg(
            device_type   = self.DeviceType.BroadcastMessages,
            manufacturer  = self.Manufacturer.BroadcastMessages,
            api_class     = 0, # arbitrary
            api_key       = self.SystemControl.Resume,
            device_number = 0, # arbitrary
            payload       = b''
        ))

    def update(self, mask):
        '''
        Trigger a synchronous update all motor controllers with a group that
        matches the bitmask.
        '''
        self.bridge.send_message(GenericCANMsg(
            device_type   = self.DeviceType.BroadcastMessages,
            manufacturer  = self.Manufacturer.BroadcastMessages,
            api_class     = 0, # arbitrary
            api_key       = self.SystemControl.SynchronousUpdate,
            device_number = 0, # arbitrary
            payload       = struct.pack('B', mask)
        ))

    def heartbeat(self):
        '''
        Reset the watchdogs on all connected motor controllers. Every motor
        controller must receive a minimum of one CAN message every 100 ms to
        avoid entering a fault state.
        '''
        self.bridge.send_message(GenericCANMsg(
            device_type   = self.DeviceType.BroadcastMessages,
            manufacturer  = self.Manufacturer.BroadcastMessages,
            api_class     = 0, # arbitrary
            api_key       = self.SystemControl.Heartbeat,
            device_number = 0, # arbitrary
            payload       = b''
        ))

    def enumerate(self):
        '''
        Obtain a list of motor controllers connected to this CAN bus bridge by
        requesting each controller to enumerate itself. This enumeration takes
        a minimum of 80 ms.
        '''
        self.bridge.send_message(GenericCANMsg(
            device_type   = self.DeviceType.BroadcastMessages,
            manufacturer  = self.Manufacturer.BroadcastMessages,
            api_class     = 0, # arbitrary
            api_key       = self.SystemControl.Enumeration,
            device_number = 0, # arbitrary
            payload       = b''
        ))

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

class Jaguar:
    def __init__(self, bridge, device_number):
        self.bridge = bridge
        self.number = device_number

        # Use a quadrature encoder to measure velocity.
        payload = struct.pack('B', JaguarCAN.SpeedReference.Quadrature)
        self._send_message(JaguarCAN.SpeedControl.SpeedReference, payload)

    def enable(self):
        '''
        Enables speed control and sets the output to neutral.
        '''
        self._send_message(JaguarCAN.SpeedControl.SpeedModeEnable, b'')

    def disable(self):
        '''
        Disables speed control, returns to the default control mode, and sets
        the output to neutral.
        '''
        self._send_message(JaguarCAN.SpeedControl.SpeedModeDisable, b'')

    def set_speed(self, speed, group=0):
        '''
        Sets the target rotational speed in revolutions per minute.
        '''
        payload_speed = struct.pack('<i', ???)
        payload_group = struct.pack('B', group)
        payload       = payload_speed + payload_group
        self._send_message(self.SpeedControl.SpeedSet, payload)

    def set_constants(self, kp, ki, kd):
        payload_kp = struct.pack('<i', ???)
        payload_ki = struct.pack('<i', ???)
        payload_kd = struct.pack('<i', ???)
        self._send_message(self.SpeedControl.SetPConstant, payload_kp)
        self._send_message(self.SpeedControl.SetIConstant, payload_ki)
        self._send_message(self.SpeedControl.SetDConstant, payload_kd)

    def _send_message(self, api_key, payload):
        self.bridge.send_message(GenericCANMsg(
            device_type   = self.DeviceType.MotorController,
            manufacturer  = self.Manufacturer.TexasInstruments,
            api_class     = self.APIClass.SpeedControl,
            api_key       = api_key,
            device_number = self.number,
            payload       = payload
        ))

def main():
    serial = serial.Serial(
        baudrate = 115200,
        bytesize = serial.EIGHTBITS,
        parity   = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        timeout  = None
    )
    packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd)
    jaguar = JaguarUART(serial, packetizer)

if __name__ == '__main__':
    main()
