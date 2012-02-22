import bitstring, struct
from collections import namedtuple, OrderedDict

SerialPayload = namedtuple('SerialPayload', [ 'device_id', 'payload' ])

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

class JaguarBridge:
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
        self.serial.timeout = timeout
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

