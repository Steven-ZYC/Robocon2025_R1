#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

# Constants for steering servos
ENCODER_RATIO = 19.20321
GEAR_RATIO = 35.0 / 61.0
TWO_PI = 2.0 * math.pi
PERIOD_RAD = TWO_PI * (ENCODER_RATIO / GEAR_RATIO)  # ~210 rad

# Constants for drive motors
DESIRED_IDS = {1, 2, 3, 4}  # CAN IDs for each wheel motor
INPUT_MAX = 8192.0          # Max input for plane_speed
OUTPUT_MAX = 10000.0        # Max RPM output for VESC

# Geometry (units consistent with control inputs)
a = 19.0  # front axle to CG distance
c = 19.0  # CG to rear axle distance
b = 19.0  # half track width
L = a + c

# Wheel positions relative to CG
WHEEL_POS = {
    1: ( a,  b),  # front-left
    2: ( a, -b),  # front-right
    3: (-c,  b),  # rear-left
    4: (-c, -b),  # rear-right
}

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Subscribe to driving commands: [steer_deg, plane_speed, yaw]
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # Publisher for steering servos
        self.pos_pub = self.create_publisher(
            Float32MultiArray,
            'damiao_control',
            10
        )

        # Publisher for VESC control
        self.vesc_pub = self.create_publisher(
            Int32MultiArray,
            'vesc_ctrl',
            10
        )

        # Track last steering positions to wrap angles
        self.last_pos_rad = {i: 0.0 for i in WHEEL_POS}

        # Desired RPM for each drive motor
        self.desired_rpms = {i: 0 for i in WHEEL_POS}

    def driving_callback(self, msg: Float32MultiArray):
        # Unpack inputs
        steer_deg = msg.data[0]
        plane_speed = msg.data[1]  # Range: [0, 8192]
        # yaw_cmd = msg.data[2]  # Ignored for RPM calculation

        # --- 1) Steering angle calculation ---
        phi = math.radians(steer_deg)
        t = math.tan(phi)

        # Compute per-wheel steering angles (rad)
        delta = {
            1: math.atan(t / (2 - t)),
            2: math.atan(t / (2 + t)),
            3: -math.atan(t / (2 - t)),
            4: -math.atan(t / (2 + t)),
        }

        # Publish servo commands
        for i, angle in delta.items():
            # Convert to encoder radians
            wheel_rev = math.degrees(angle) / 360.0
            motor_rev = wheel_rev / GEAR_RATIO
            encoder_rev = motor_rev * ENCODER_RATIO
            target_rad = encoder_rev * TWO_PI

            # Wrap to nearest
            diff = target_rad - self.last_pos_rad[i]
            if diff > PERIOD_RAD / 2:
                target_rad -= PERIOD_RAD
            elif diff < -PERIOD_RAD / 2:
                target_rad += PERIOD_RAD

            msg_out = Float32MultiArray()
            msg_out.data = [float(i), 2.0, 40.0, target_rad]
            self.pos_pub.publish(msg_out)
            self.last_pos_rad[i] = target_rad

        # --- 2) Drive RPM calculation ---
        # Map plane_speed [0, 8192] to RPM [0, 10000]
        rpm = int(plane_speed * (OUTPUT_MAX / INPUT_MAX))

        # Assign same RPM to all wheels
        for i in WHEEL_POS.keys():
            self.desired_rpms[i] = rpm

        # Publish VESC RPM commands
        msg_vesc = Int32MultiArray()
        msg_vesc.data = [self.desired_rpms[i] for i in sorted(WHEEL_POS.keys())]
        self.vesc_pub.publish(msg_vesc)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()