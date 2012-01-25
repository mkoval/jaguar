import logging, unittest, struct
from JaguarCAN import JaguarUART, Packetizer

class PacketizerTests(unittest.TestCase):
    def setUp(self):
        logging.basicConfig(format='%(message)s')
        logger = logging.getLogger('Packetizer')
        logger.setLevel(logging.WARNING)

        self.packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd, logger)

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

if __name__ == '__main__':
    unittest.main()
