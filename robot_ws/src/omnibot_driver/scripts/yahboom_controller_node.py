#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from tf2_ros import TransformBroadcaster
import serial
import math
import numpy as np
from tf_transformations import quaternion_from_euler
import time
import struct
import traceback

class YahboomControllerNode(Node):
    def __init__(self):
        super().__init__('yahboom_controller_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_separation_width', 0.215)
        self.declare_parameter('wheel_separation_length', 0.165)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation_width = self.get_parameter('wheel_separation_width').value
        self.wheel_separation_length = self.get_parameter('wheel_separation_length').value
        self.port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Robot state
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        
        # Store latest command for throttling
        self.current_twist = Twist()
        
        self.last_beep_time = 0
        
        # Ramping State
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wa = 0.0

        # Odometry velocity (from board feedback, used by publish_odometry)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.last_odom_time = time.time()
        
        # Create publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer: 20Hz (0.05s)
        self.update_timer = self.create_timer(0.05, self.update_callback)
        
        # Serial Setup
        self.serial_port = None
        self.connect_serial()
        
        self.get_logger().info('Yahboom controller node initialized')
        
    def log_to_file(self, msg):
        try:
            with open('/home/varunvaidhiya/yahboom_debug.log', 'a') as f:
                f.write(f"{time.time()}: {msg}\n")
        except:
            pass
    
    def connect_serial(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=1.0)
            self.get_logger().info(f'Connected to {self.port_name}')
            
            # Initialize CAR_TYPE = 1 (Mecanum X3) - must be set each session
            time.sleep(0.3)
            for _ in range(5):
                self.send_packet(0x15, struct.pack('<b', 1))
                time.sleep(0.05)
            self.get_logger().info('CAR_TYPE set to 1 (Mecanum X3)')
        except Exception as e:
            self.get_logger().error(f'Serial Connection Error: {e}')
            self.serial_port = None

    def calculate_checksum(self, data):
        return (sum(data) + 5) & 0xFF

    def send_packet(self, msg_type, payload):
        if self.serial_port is None:
            return
            
        try:
            HEAD = 0xFF
            DEVICE_ID = 0xFC
            
            # [HEAD, ID, LEN, TYPE] + Payload
            packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
            packet[2] = len(packet) - 1 # LEN value
            
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # Log successful packet generation for debugging
            # if msg_type == 0x12: 
            #    self.get_logger().info(f"TX: {[hex(x) for x in packet]}")
            
            self.serial_port.write(bytearray(packet))
            time.sleep(0.002) # Critical delay
            
        except Exception as e:
            self.get_logger().error(f'Tx Error (Closing): {e}')
            if self.serial_port:
                self.serial_port.close()
            self.serial_port = None

    def joy_callback(self, msg):
        try:
            # Button A (Index 0) -> Beep
            # DEBOUNCE: Only allow 1 beep every 0.5 seconds
            now = time.time()
            if len(msg.buttons) > 0 and msg.buttons[0] == 1:
                if (now - self.last_beep_time) > 0.5:
                    self.get_logger().info("Button A: BEEP")
                    payload = struct.pack('<h', 100) # 100ms
                    self.send_packet(0x02, payload)
                    self.last_beep_time = now
        except Exception as e:
            self.get_logger().error(f'Joy Error: {e}')

    def cmd_vel_callback(self, msg):
        # Just store the command. Do NOT send to serial here.
        # This prevents flooding the serial bus if joy publisher is fast.
        self.current_twist = msg

    def send_motion_command(self):
        try:
            msg = self.current_twist
            
            # CLAMP SPEED to prevent Brownout/Over-current - SAFE MODE
            MAX_VAL = 0.2  # m/s
            RAMP_STEP = 0.05  # m/s per 0.05s tick
            
            target_vx = np.clip(msg.linear.x, -MAX_VAL, MAX_VAL)
            target_vy = np.clip(msg.linear.y, -MAX_VAL, MAX_VAL)
            target_w  = np.clip(msg.angular.z, -1.0, 1.0)
            
            # Ramping Logic
            if self.cmd_vx < target_vx:
                self.cmd_vx = min(self.cmd_vx + RAMP_STEP, target_vx)
            elif self.cmd_vx > target_vx:
                self.cmd_vx = max(self.cmd_vx - RAMP_STEP, target_vx)
                
            if self.cmd_vy < target_vy:
                self.cmd_vy = min(self.cmd_vy + RAMP_STEP, target_vy)
            elif self.cmd_vy > target_vy:
                self.cmd_vy = max(self.cmd_vy - RAMP_STEP, target_vy)

            self.cmd_wa = target_w
            
            # Send onboard Mecanum kinematics command (0x12)
            # Board computes wheel speeds internally using CAR_TYPE=1 algorithm
            vx_int = int(self.cmd_vx * 1000)  # mm/s
            vy_int = int(self.cmd_vy * 1000)
            w_int  = int(self.cmd_wa * 1000)
            
            CAR_TYPE = 1  # Mecanum X3
            payload = struct.pack('<bhhh', CAR_TYPE, vx_int, vy_int, w_int)
            self.send_packet(0x12, payload)
            
        except Exception as e:
            self.get_logger().error(f'CmdVel Error: {e}')
            self.log_to_file(f'CmdVel Error: {e}')
            self.log_to_file(traceback.format_exc())

    def update_callback(self):
        try:
            if self.serial_port is None:
                self.connect_serial()
                return

            # 1. Read Odom
            self.read_yahboom_odometry()
            
            # 2. Publish Odom
            self.publish_odometry(self.get_clock().now())
            
            # 3. Send Motor Command (Throttled to 10Hz)
            self.send_motion_command()
            
        except Exception as e:
            self.get_logger().error(f'Update Error: {e}')
            self.log_to_file(f'Update Error: {e}')

    def read_yahboom_odometry(self):
        if self.serial_port is None:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting == 0:
                return

            data = self.serial_port.read(waiting)

            # RX packet format: [0xFF, 0xFB, LEN, TYPE, PAYLOAD..., CS]
            #   LEN = 3 + len(payload)  (covers LEN itself, TYPE, PAYLOAD, CS)
            #   total packet bytes = 2 + LEN
            #   CS = sum(LEN, TYPE, PAYLOAD...) & 0xFF
            #
            # TYPE 0x0C — velocity feedback (3 x int16, scaled x1000):
            #   payload[0:2] = vx  (mm/s)
            #   payload[2:4] = vy  (mm/s)
            #   payload[4:6] = vz  (mrad/s)
            # This is the holonomic motion result computed by the board after CAR_TYPE is set.

            vx_ms = vy_ms = vz_rads = None
            idx = 0
            while idx < len(data) - 2:
                if data[idx] != 0xFF or data[idx + 1] != 0xFB:
                    idx += 1
                    continue
                if idx + 3 > len(data):
                    break
                length = data[idx + 2]
                total = 2 + length
                if idx + total > len(data):
                    break  # incomplete packet — wait for next cycle
                pkt = data[idx:idx + total]
                pkt_type = pkt[3]
                payload = pkt[4:-1]  # between TYPE and CS
                if pkt_type == 0x0C and len(payload) >= 6:
                    vx_raw, vy_raw, vz_raw = struct.unpack_from('<hhh', payload, 0)
                    vx_ms   = vx_raw  / 1000.0
                    vy_ms   = vy_raw  / 1000.0
                    vz_rads = vz_raw  / 1000.0
                idx += total

            if vx_ms is None:
                return  # no velocity packet this cycle

            # Dead-band: suppress noise near zero
            if abs(vx_ms)   < 0.005: vx_ms   = 0.0
            if abs(vy_ms)   < 0.005: vy_ms   = 0.0
            if abs(vz_rads) < 0.005: vz_rads = 0.0

            # Integrate velocity into pose (robot frame → odom frame)
            now = time.time()
            dt = now - self.last_odom_time
            self.last_odom_time = now
            if dt > 0.5:  # skip large gaps on startup or after reconnect
                return

            cos_th = math.cos(self.theta)
            sin_th = math.sin(self.theta)
            self.x_pos += (vx_ms * cos_th - vy_ms * sin_th) * dt
            self.y_pos += (vx_ms * sin_th + vy_ms * cos_th) * dt
            self.theta += vz_rads * dt
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # wrap to [-pi, pi]

            self.current_vx = vx_ms
            self.current_vy = vy_ms
            self.current_vz = vz_rads

        except Exception as e:
            self.get_logger().error(f'Rx Error: {e}')
            # Do not close connection on RX error


    def publish_odometry(self, stamp):
        try:
            q = quaternion_from_euler(0, 0, self.theta)

            ts = TransformStamped()
            ts.header.stamp = stamp.to_msg()
            ts.header.frame_id = 'odom'
            ts.child_frame_id = 'base_link'
            ts.transform.translation.x = self.x_pos
            ts.transform.translation.y = self.y_pos
            ts.transform.translation.z = 0.0
            ts.transform.rotation.x = q[0]
            ts.transform.rotation.y = q[1]
            ts.transform.rotation.z = q[2]
            ts.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(ts)

            odom = Odometry()
            odom.header.stamp = stamp.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x_pos
            odom.pose.pose.position.y = self.y_pos
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x  = self.current_vx
            odom.twist.twist.linear.y  = self.current_vy
            odom.twist.twist.angular.z = self.current_vz
            self.odom_pub.publish(odom)
        except Exception as e:
            self.get_logger().error(f'Pub Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = YahboomControllerNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"CRITICAL NODE FAILURE: {e}")
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
