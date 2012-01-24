import serial
import struct
from collections import deque, namedtuple
from Threading import Condition, Thread

SerialPayload = namedtuple('SerialPayload', [ 'device_id', 'payload' ])
GenericCANMsg = namedtuple('GenericCANMsg', [ 'device_type', 'manufacturer',
                                              'api', 'device_number',
                                              'payload' ])

class Packetizer:
    def __init__(self, sof, esc, sof_esc, esc_esc);
        self.sof = sof
        self.esc = esc
        self.esc_sof = sof_esc
        self.esc_esc = esc_esc
        self.packets = deque()
        self.reset()

    def recv_byte(self, byte):
        byte_dec = self._decode(self.last, byte)

        if byte_dec == None:
            self._error()
        # Every start of frame (SOF) byte starts a new frame because it is
        # otherwise escaped.
        if byte_dec == self.sof:
            self._reset()
        # Next byte is the total number of bytes in the packet. Note that this
        # could be encoded if it equals 255, although this should never occur
        # in practice when framing CANbus messages.
        elif self.offset == 1:
            (self.count, ) = struct.unpack('B', byte_dec)
            self.offset += 1
        else:
            self.packet.append(byte_dec)

        # Done receiving the entire packet, so add it to the queue. Padding
        # compensates for fields in the packet (e.g. device id) that don't
        # contribute to the byte count.
        if self.valid and self.offset == self.count + 2:
            new_packet = self.packet
            self._reset()
            return new_packet
        else:
            return None

    def send_bytes(self, ):
        raise NotImplemented('Sending is not implemented!')

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
            else:
                return None
        else:
            return curr

class JaguarUART:
    def __init__(self, serial, packetizer):
        self.packets    = deque()
        self.serial     = serial
        self.packetizer = packetizer
        self.condition  = Condition()
        self.producer   = Thread(target=self._producer)
        self.producer.run()

    def close(self):
        # TODO: Kill the producer thread before closing the connection.
        self.fp.close()

    def recv_message(self):
        self.condition.acquire()
        while not self.packets:
            self.condition.wait()

        packet = self.packets.popleft()
        self.condition.release()

        (message_id, ) = struct.unpack('<I', packet[0:4])
        return GenericCANMessage(
            device_type   = message_id & 0b00011111000000000000000000000000,
            manufacturer  = message_id & 0b00000000111111110000000000000000,
            api           = message_id & 0b00000000000000001111111111000000,
            device_number = message_id & 0b00000000000000000000000000111111,
            payload       = packet[5:]
        )

    def send_message(self, message):
        message_id = ((message.device_id     & 0b11111)      << 28) |
                     ((message.manufacturer  & 0b11111111)   << 23) |
                     ((message.api           & 0b1111111111) << 15) |
                     ((message.device_number & 0b111111)     <<  0)
        packed_id = struct.pack('<I', message_id)

        packed_message = packed_id + message.payload
        self.serial.send_bytes(packed_message)

    def _producer(self):
        while True:
            pending = self.serial.inWaiting()
            data    = self.serial.read(pending)
            
            self.condition.acquire()

            for datum in data:
                packet = self.packetizer.recv_byte(datum)
                if packet:
                    self.packets.append(packet)
                    self.condition.notify()

            self.condition.release()

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
