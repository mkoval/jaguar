
class CANROSBridge(CANBridge)
    def send_message(self, msg):
        msg_id, msg_payload = self.generate_message(msg)

    def recv_message(self, timeout=None):
        
