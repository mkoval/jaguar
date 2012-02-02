import unittest, struct
from collections import deque, namedtuple
from mock import Mock
from JaguarCAN import GenericCANMsg, JaguarUART, Packetizer

class PacketizerTests(unittest.TestCase):
    def setUp(self):
        self.packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd)

    def test_RecvByte_ReturnsNone_WhenNoPacket(self):
        packet = self.packetizer.recv_byte('\xff')
        self.assertIsNone(packet)
    
    def test_RecvByte_EmptyPacket(self):
        packet  = self.build_packet('', 0)
        payloads = self.recv_bytes(packet)
        self.assertEqual(payloads, [ None, '' ])

    def test_RecvByte_NonEmptyPacket(self):
        packet   = self.build_packet('ab', 2)
        payloads = self.recv_bytes(packet)
        self.assertEqual(payloads, [ None, None, None, 'ab' ])

    def test_RecvByte_MultiplePackets(self):
        packet1  = self.build_packet('a', 1)
        packet2  = self.build_packet('b', 1)
        payloads = self.recv_bytes(packet1 + packet2)
        self.assertEqual(payloads, [ None, None, 'a', None, None, 'b' ])

    def test_RecvByte_EscapesSOF(self):
        packet   = self.build_packet('\xFE\xFE', 1)
        payloads = self.recv_bytes(packet)
        self.assertEqual(payloads, [ None, None, None, '\xFF' ])

    def test_RecvByte_EscapesESC(self):
        packet   = self.build_packet('\xFE\xFD', 1)
        payloads = self.recv_bytes(packet)
        self.assertEqual(payloads, [ None, None, None, '\xFE' ])

    def test_FrameBytes_EmptyPayload(self):
        framed = self.packetizer.frame_bytes(b'')
        self.assertEqual(framed, b'\xff\x00')

    def test_FrameBytes_AddsHeader(self):
        payload = b'ab'
        framed = self.packetizer.frame_bytes(payload)
        self.assertEqual(framed, b'\xff\x02ab')

    def test_FrameBytes_EscapesSOFInLength(self):
        payload = b'a' * 0xff
        framed = self.packetizer.frame_bytes(payload)
        self.assertEqual(framed, '\xff\xfe\xfe' + payload)

    def test_FrameBytes_EscapesESCInLength(self):
        payload = b'a' * 0xfe
        framed = self.packetizer.frame_bytes(payload)
        self.assertEqual(framed, '\xff\xfe\xfd' + payload)

    def test_FrameBytes_EscapesSOFInPayload(self):
        payload = b'a\xffb'
        framed = self.packetizer.frame_bytes(payload)
        self.assertEqual(framed, '\xff\x03a\xfe\xfeb')

    def test_FrameBytes_EscapesESCInPayload(self):
        payload = b'a\xfeb'
        framed = self.packetizer.frame_bytes(payload)
        self.assertEqual(framed, '\xff\x03a\xfe\xfdb')

    def build_packet(self, payload_raw, count):
        payload = map(ord, payload_raw)
        return struct.pack('BB{0}B'.format(len(payload)), 0xff, count, *payload)

    def recv_bytes(self, data):
        return map(self.packetizer.recv_byte, data)

class SerialMock:
    def __init__(self):
        self._buf_read  = deque()
        self._buf_write = deque()

    def _add_data(self, data):
        self._buf_read.extend(data)

    def read(self, size=1):
        assert self._buf_read
        return self._buf_read.popleft()

    def write(self, data):
        self._buf_write.extend(data)

    def inWaiting(self):
        return len(self._buf_read)


class JaguarUARTTests(unittest.TestCase):
    SampleMsg = namedtuple('SampleMsg', [ 'raw', 'dec', 'msg' ])
    samples = [
        SampleMsg(
            raw = b'\xff\x06\x85\x00\x02\x02\x00\x08',
            dec = b'\x85\x00\x02\x02\x00\x08',
            msg = GenericCANMsg(2, 2, 0, 2, 5, b'\x00\x08')
        ),
        SampleMsg(
            raw = b'\xff\x06\x85\x00\x02\x02\xfe\xfe\xfe\xfe',
            dec = b'\x85\x00\x02\x02\xff\xff',
            msg = GenericCANMsg(2, 2, 0, 2, 5, payload=b'\xff\xff')
        ),
        SampleMsg(
            raw = b'\xff\x04\x85\x00\x02\x02',
            dec = b'\x85\x00\x02\x02',
            msg = GenericCANMsg(2, 2, 0, 2, 5, payload=b'')
        )
    ]

    def setUp(self):
        self.serial = SerialMock()
        self.packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd)
        self.jaguar = JaguarUART(self.serial, self.packetizer)

    def test_ParseMessage(self):
        for sample in self.samples:
            msg = self.jaguar.parse_message(sample.dec)
            self.assertEqual(msg, sample.msg)

    def test_GenerateMessage(self):
        for sample in self.samples:
            data = self.jaguar.generate_message(sample.msg)
            self.assertEqual(data, sample.dec)

    def test_RecvMessage(self):
        for sample in self.samples:
            # Arrange:
            def mock_recv_byte(byte):
                return None if self.serial.inWaiting() else sample.msg

            self.packetizer.recv_byte = Mock(side_effect=mock_recv_byte)
            self.serial.read = Mock(side_effect=self.serial.read)
            self.serial._add_data(sample.raw)

            # Act:
            msg = self.jaguar.recv_message()

            # Assert:
            expected_read_args = [ ((1,), {}) ] * len(sample.raw)
            self.assertEqual(self.serial.read.call_args_list, expected_read_args)
            self.assertEqual(msg, sample.msg)

    def test_SendMessage(self):
        for sample in self.samples:
            self.packetizer.send_bytes = Mock()
            self.jaguar.send_message(sample.msg)
            self.packetizer.send_bytes.called_once_with(sample.dec)

if __name__ == '__main__':
    unittest.main()
