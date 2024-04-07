#!/usr/bin/env python3

import rclpy
from drive_interface.msg import DriveCmd, LimitBools, ManipCmd
from std_msgs.msg import Int32, String
from tamproxy import ROS2Sketch
from tamproxy.devices import DigitalOutput, Servo, FeedbackMotor, DigitalInput, TimeOfFlight
import math




class KitBotNode(ROS2Sketch):
    """ROS2 Node that controls the KitBot via the Teensy and tamproxy"""

    # Pin mappings
    LMOTOR_PINS = (14, 13)  # DIR, PWM
    RMOTOR_PINS = (16, 15)  # DIR, PWM
    LENCODER_PINS = (36, 35)  # DIR, PWM
    RENCODER_PINS = (32, 31)  # DIR, PWM

    COMMON_PINS = (8, 10)
    LIMIT_PINS = (9, 11) # bottom, top

    IR_PIN = 2

    MANIP_PINS = (24, 25, 26, 27, 28) # lift, pivot, grip, sort, door

    def setup(self):
        """
        One-time method that sets up the robot, like in Arduino
        Code is run when run_setup() method is called
        """
        # Create a subscriber to listen for drive motor commands
        self.drive_sub = self.create_subscription(
            DriveCmd,
            'drive_cmd',
            self.drive_callback,
            10)
        
        self.manip_sub = self.create_subscription(
            ManipCmd,
            'manip_cmd',
            self.manip_callback,
            10)
        
        self.sort_sub = self.create_subscription(
            String,
            'sort_cmd',
            self.sort_callback,
            10)
        
        self.door_sub = self.create_subscription(
            String,
            'door_cmd',
            self.door_callback,
            10)

        self.drive_sub  # prevent unused variable warning
        self.manip_sub  #

        self.limit_pub = self.create_publisher(LimitBools, '/limit_bools', 10)

        self.ir_pub = self.create_publisher(Int32, '/ir_dist', 10)

        self.timer_period = 0.01
        self.limit_timer = self.create_timer(self.timer_period, self.limit_callback)

        self.ir_timer = self.create_timer(self.timer_period, self.ir_callback)

        # Create the motor and encoder objects
        self.lmotor = FeedbackMotor(self.tamp, self.LMOTOR_PINS[0], self.LMOTOR_PINS[1], self.LENCODER_PINS[0], self.LENCODER_PINS[1], True)
        self.rmotor = FeedbackMotor(self.tamp, self.RMOTOR_PINS[0], self.RMOTOR_PINS[1], self.RENCODER_PINS[0], self.RENCODER_PINS[1], True)

        self.bottom_com = DigitalOutput(self.tamp, self.COMMON_PINS[0])
        self.top_com = DigitalOutput(self.tamp, self.COMMON_PINS[1])
        self.bottom_com.write(False)
        self.top_com.write(False)

        self.bottom_limit = DigitalInput(self.tamp, self.LIMIT_PINS[0], pullup=True)
        self.top_limit = DigitalInput(self.tamp, self.LIMIT_PINS[1], pullup=True)

        self.bottom_limit_trig = False
        self.top_limit_trig = False

        self.bottom_count = 0
        self.top_count = 0

        self.lift_servo = Servo(self.tamp, self.MANIP_PINS[0], lower_uS=900, upper_uS=2100)
        self.pivot_servo = Servo(self.tamp, self.MANIP_PINS[1], lower_uS=500, upper_uS=2500)
        self.grip_servo = Servo(self.tamp, self.MANIP_PINS[2], lower_uS=500, upper_uS=2500)
        self.sort_servo = Servo(self.tamp, self.MANIP_PINS[3], lower_uS=1000, upper_uS=2000)
        self.door_servo = Servo(self.tamp, self.MANIP_PINS[4], lower_uS=1000, upper_uS=2000)

        self.ir = TimeOfFlight(self.tamp, self.IR_PIN, 1)

        self.ir.enable()

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        self.lmotor.write(msg.l_speed)
        self.rmotor.write(-msg.r_speed)

    def manip_callback(self, msg):
        """Processes a new lift command and controls motors appropriately"""
        lift_speed = msg.lift_cmd
        if self.bottom_limit_trig and msg.lift_cmd > 90:
            lift_speed = 92
        if self.top_limit_trig and msg.lift_cmd < 90:
            lift_speed = 92
            
        self.lift_servo.write(lift_speed)
        self.pivot_servo.write(msg.pivot_cmd)
        self.grip_servo.write(msg.grip_cmd)

    def sort_callback(self, msg):
        if msg.data == "stack":
            self.sort_servo.write(42.5)
        else:
            self.sort_servo.write(110)

    def door_callback(self, msg):
        if msg.data == "open":
            self.door_servo.write(150)
        else:
            self.door_servo.write(30)

    def limit_callback(self):
        limit_msg = LimitBools()
        limit_msg.bottom_limit = self.bottom_limit.val == b'\x00'
        limit_msg.top_limit = self.top_limit.val == b'\x00'

        self.bottom_limit_trig = limit_msg.bottom_limit
        self.top_limit_trig = limit_msg.top_limit

        self.limit_pub.publish(limit_msg)

    def ir_callback(self):
        ir_msg = Int32()
        ir_msg.data = self.ir.dist
        self.ir_pub.publish(ir_msg)




def main():
    rclpy.init()

    kb = KitBotNode(rate=100)  # Run at 100Hz (10ms loop)
    kb.run_setup()     # Run tamproxy setup and code in setup() method
        
    rclpy.spin(kb)

    kb.destroy()       # Shuts down tamproxy
    kb.destroy_node()  # Destroys the ROS node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
