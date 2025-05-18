#!/usr/bin/env python3
import time
import serial
from serial.tools import list_ports
import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrentBrake
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# Constants
HEARTBEAT_DT = 0.05  # s between VESC heartbeats
DESIRED_IDS = {1, 2, 3, 4}  # CAN IDs for each wheel motor
BRAKE_CURRENT = 10.0  # Brake current in amps (adjust as needed)

class VescController(Node):
    def __init__(self):
        super().__init__('vesc_controller')

        # Subscribe to VESC control commands
        self.create_subscription(
            Int32MultiArray,
            'vesc_ctrl',
            self.vesc_callback,
            10
        )

        # Discover and open serial ports for VESCs
        self.port_map = self.find_vescs()
        if not self.port_map:
            self.get_logger().error(f'No VESCs with CAN IDs {DESIRED_IDS} found')
        self.serial_map = {}
        for dev in self.port_map:
            ser = serial.Serial(dev, baudrate=115200, timeout=0.1)
            time.sleep(0.1)
            self.serial_map[dev] = ser

        # Desired RPM for all drive motors
        self.desired_rpm = 0

        # Heartbeat timer to continuously send commands
        self.create_timer(HEARTBEAT_DT, self.heartbeat)

    def scan_ids_on_port(self, dev):
        found = []
        try:
            with serial.Serial(dev, 115200, timeout=0.1) as ser:
                time.sleep(0.1)
                for can_id in DESIRED_IDS:
                    setattr(GetValues, 'can_id', can_id)
                    ser.write(pyvesc.encode_request(GetValues))
                    time.sleep(HEARTBEAT_DT)
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)
                        msg, _ = pyvesc.decode(data)
                        if msg:
                            found.append(can_id)
        except Exception as e:
            self.get_logger().warn(f'[scan] error on {dev}: {e}')
        return found

    def find_vescs(self):
        ports = [p.device for p in list_ports.comports() if 'ACM' in p.device or 'USB' in p.device]
        res = {}
        for dev in ports:
            ids = self.scan_ids_on_port(dev)
            if ids:
                res[dev] = ids
                self.get_logger().info(f'Found VESCs {ids} on {dev}')
        return res

    def vesc_callback(self, msg: Int32MultiArray):
        # Use first RPM value (all wheels have same RPM)
        self.desired_rpm = msg.data[0] if msg.data else 0

    def heartbeat(self):
        # Send commands to each VESC
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                if self.desired_rpm == 0:
                    cmd = SetCurrentBrake(BRAKE_CURRENT)
                else:
                    cmd = SetRPM(self.desired_rpm)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))

    def destroy_node(self):
        # Apply brake on shutdown
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetCurrentBrake(BRAKE_CURRENT)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))
        for ser in self.serial_map.values():
            ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VescController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()