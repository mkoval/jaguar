import roslib; roslib.load_manifest('navi_jaguar')
import rospy
from threading import Timer

id_left  = 1
id_right = 2
ticks_per_rev = 800
pid_p = 0.0
pid_i = 0.0
pid_d = 0.0

robot_radius = 0.381 # m
wheel_radius = 0.100 # m
heartbeat_period = 1.0 # s

class JaguarNode:
    def __init__(self, bridge):
        self.broadcaster = CANBroadcaster(bridge)
        self.motor_left  = JaguarCAN(bridge, id_left)
        self.motor_right = JaguarCAN(bridge, id_right)
        self.seq_num = 1

        self._setup(self.motor_left)
        self._setup(self.motor_right)
        self._heartbeat()

    def drive(self, v, omega):
        v_left = v + 0.5 * robot_radius * omega
        v_right = 2*v - v_left
        self.direct_drive(v_left, v_right)

    def direct_drive(self, v_left, v_right):
        ratio = 1 / (2 * math.pi * wheel_radius)
        self.motor_left.set_speed(v_left * ratio, group=self.seq_num)
        self.motor_right.set_speed(v_right * ratio, group=self.seq_num)
        self.broadcaster.update(self.seq_num)

        self.seq_num = self.seq_num << 1
        if self.seq_num > 0xFF:
            self.seq_num = 1

    def _setup(self, jaguar):
        jaguar.set_reference(SpeedReference.Quadrature)
        jaguar.set_encoder_lines(ticks_per_rev)
        jaguar.set_constants(pid_p, pid_i, pid_d)
        jaguar.set_encoder_lines(ticks_per_rev)
        jaguar.enable()

    def _heartbeat(self):
        self.broadcaster.heartbeat()
        self.heartbeat_timer = Timer(heartbeat_period, heartbeat)
        self.heartbeat_timer.start()

if __name__ == '__main__':
    rospy.init_node('jaguar_node')
    node = JaguarNode()
