serial_config = {
    'baudrate' = 115200,
    'bytesize' = serial.EIGHTBITS,
    'parity'   = serial.PARITY_NONE,
    'stopbits' = serial.STOPBITS_ONE,
    'timeout'  = None
}
num_left = 1
num_right = 2
ticks_per_rev = 800
pid_p = 0.0
pid_i = 0.0
pid_d = 0.0

def init_jaguar(jaguar):
    jaguar.set_reference(SpeedReference.Quadrature)
    jaguar.set_encoder_lines(ticks_per_rev)
    jaguar.set_constants(pid_p, pid_i, pid_d)
    jaguar.enable()

serial     = serial.Serial(**serial_config)
packetizer = Packetizer(0xff, 0xfe, 0xfe, 0xfd)
bridge     = JaguarBridge(serial, packetizer)

broadcaster = JaguarBroadcaster(bridge)
motor_left  = Jaguar(bridge, num_left)
motor_right = Jaguar(bridge, num_right)

motor_left.set_encoder_lines(ticks_per_rev)
motor_right.set_encoder_lines(ticks_per_rev)


