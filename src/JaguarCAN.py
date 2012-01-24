import serial
import struct
from collections import deque, namedtuple
from Threading import Condition, Thread

SerialPayload = namedtuple('SerialPayload', [ 'device_id', 'payload' ])

class Packetizer:
    def __init__(self, sof, esc, sof_esc, esc_esc, padding):
        self.sof = sof
        self.esc = esc
        self.esc_sof = sof_esc
        self.esc_esc = esc_esc
        self.padding = padding
        self.packets = deque()
        self.reset()

    def reset(self):
        self.offset = 1
        self.count  = 0
        self.last   = None
        self.packet = bytearray()
        self.valid  = True

    def error(self):
        self.valid = False

    def add_data(self, data_raw):
        data_raw = ord(curr_raw)
        data_dec = None

        # Last character was the escape byte, so this character must be
        # interpreted as an escape code.
        if self.last == self.esc:
            if data == self.esc_esc:
                data_dec = self.esc
            elif data == self.esc_sof:
                data_dec = self.sof
            else:
                self.error()
        # Non-escaped byte.
        else:
            data_dec = curr

        # Every start of frame (SOF) byte starts a new frame because it is
        # otherwise escaped.
        if data_dec == self.sof:
            self.reset()
        # Next byte is the total number of bytes in the packet. Note that this
        # could be encoded if it equals 255.
        elif self.offset == 1:
            (self.count, ) = struct.unpack('B', data_dec)
            self.offset += 1
        else:
            self.packet.append(data_dec)

        # Done receiving the dataent packet, so add it to the queue.
        if self.valid and self.offset == self.count + self.padding + 1:
            self.add_packet(self.packet)
            self.reset()

    def add_packet(self, packet):
        (device_id, ) = struct.unpack('>I', packet[0:4])
        payload       = packet[5:]
        return SerialPayload(device_id, self.packet)

class JaguarUART:
    def __init__(self, device):
        """
        self.serial = serial.Serial(
            baudrate = 115200,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = None
        )
        """
        self.packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd)

    def close(self):
        self.fp.close()

jaguar = JaguarUART('/dev/ttyUSB0')
print jaguar.send(0, [ 0xFF ])

