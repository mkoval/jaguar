import bitstring, logging, serial, struct
from collections import deque, namedtuple, OrderedDict
from threading import Condition, Thread

SerialPayload = namedtuple('SerialPayload', [ 'device_id', 'payload' ])
GenericCANMsg = namedtuple('GenericCANMsg', [ 'device_type', 'manufacturer',
                                              'api_class', 'api_key',
                                              'device_number', 'payload' ])

class Packetizer:
    def __init__(self, sof, esc, sof_esc, esc_esc, logger):
        self.sof = sof
        self.esc = esc
        self.esc_sof = sof_esc
        self.esc_esc = esc_esc
        self.packets = deque()
        self.logger  = logger
        self._reset()

    def recv_byte(self, byte_raw):
        byte     = ord(byte_raw)
        byte_dec = self._decode(self.last, byte)

        # Every start of frame (SOF) byte starts a new frame because it is
        # otherwise escaped.
        if byte == self.sof:
            self.logger.debug('packet start')
            self._reset()
        # Next byte is the total number of bytes in the packet. Note that this
        # could be encoded if it equals 255, although this should never occur
        # in practice when framing CANbus messages.
        elif self.offset == 1:
            self.count   = byte
            self.offset += 1
        elif byte_dec:
            self.packet.append(byte_dec)
            self.offset += 1

        self.last = byte

        # Done receiving the entire packet, so add it to the queue. Padding
        # compensates for fields in the packet (e.g. device id) that don't
        # contribute to the byte count.
        if self.valid and self.offset == self.count + 2:
            self.logger.debug('packet end')
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
                self.logger.warning('Unexpected escape sequence.')
        elif curr == self.esc:
            self.logger.debug('packet esc')
            return None
        else:
            return curr

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

    def __init__(self, serial, packetizer, timeout=0.25):
        self.packets = deque()
        self.alive   = True

        self.serial = serial
        self.serial.timeout = timeout
        self.packetizer = packetizer

        self.condition = Condition()
        self.producer = Thread(target=self._producer)
        self.producer.start()

    def close(self):
        self.alive = False
        self.producer.join()
        self.fp.close()

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
        self.serial.send_bytes(packed)

    def recv_message(self):
        self.condition.acquire()
        while not self.packets:
            self.condition.wait()

        packet = self.packets.popleft()
        self.condition.release()
        return self.parse_message(packet)

    def _producer(self):
        while self.alive:
            pending = max(self.serial.inWaiting(), 1)
            data    = self.serial.read(size=pending)
            
            if data != None:
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
