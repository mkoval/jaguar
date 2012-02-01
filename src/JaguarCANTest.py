from __future__ import nested_scopes
import logging, unittest, struct
from collections import deque
from mock import Mock
from threading import Condition,Thread
from JaguarCAN import GenericCANMsg, JaguarUART, Packetizer
import time

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
    raw_v0 = b'\xff\x06\x85\x00\x02\x02\x00\x08'
    dec_v0 = b'\x85\x00\x02\x02\x00\x08'
    msg_v0 = GenericCANMsg(
        manufacturer=2,
        device_type=2,
        device_number=5,
        api_class=0,
        api_key=2,
        payload='\x00\x08'
    )

    def setUp(self):
        self.serial = SerialMock()
        self.packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd)
        self.jaguar = JaguarUART(self.serial, self.packetizer)

    def test_ParseMessage(self):
        data    = b'\x85\x00\x02\x02\x00\x08'
        message = self.jaguar.parse_message(data)
        self.assertEqual(message.manufacturer, 2)
        self.assertEqual(message.device_type, 2)
        self.assertEqual(message.device_number, 5)
        self.assertEqual(message.api_class, 0)
        self.assertEqual(message.api_key, 2)
        self.assertEqual(message.payload, '\x00\x08')

    def test_GenerateMessage(self):
        message = GenericCANMsg(
            manufacturer=2,
            device_type=2,
            device_number=5,
            api_class=0,
            api_key=2,
            payload='\x00\x08'
        )
        data = self.jaguar.generate_message(message)
        self.assertEqual(data, b'\x85\x00\x02\x02\x00\x08')

    def test_RecvMessage(self):
        # Arrange:
        def mock_recv_byte(byte):
            return None if self.serial.inWaiting() else self.msg_v0

        self.packetizer.recv_byte = Mock(side_effect=mock_recv_byte)
        self.serial.read = Mock(side_effect=self.serial.read)
        self.serial._add_data(self.raw_v0)

        # Act:
        msg = self.jaguar.recv_message()

        # Assert:
        expected_read_args = [ ((1,), {}) ] * len(self.raw_v0)
        self.assertEqual(self.serial.read.call_args_list, expected_read_args)
        self.assertEqual(msg, self.msg_v0)

    def test_SendMessage(self):
        self.packetizer.send_bytes = Mock()
        self.jaguar.send_message(self.msg_v0)
        self.packetizer.send_bytes.called_once_with(self.dec_v0)

if __name__ == '__main__':
    unittest.main()
