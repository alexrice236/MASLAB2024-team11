#!/usr/bin/env python3

from std_msgs.msg import Bool, Int32, String
from drive_interface.msg import DriveCmd, ManipCmd, RotationGoals, LimitBools

import rclpy
from rclpy.node import Node

import pygame
import sys

import math

from enum import Enum

PADDING = 32
BOX_SIZE = 64
SCREEN_SIZE = BOX_SIZE * 3 + PADDING * 2
BG_COLOR = (255, 255, 255)
BOX_COLOR = (0, 0, 0)

KEY_SPEEDS = {
    pygame.K_w: (-2*math.pi, -2*math.pi),
    pygame.K_a: (math.pi, -math.pi),
    pygame.K_s: (2*math.pi, 2*math.pi),
    pygame.K_d: (-math.pi, math.pi)
}

BOX_OFFSETS = {
    pygame.K_w: (0, -1),
    pygame.K_a: (-1, 0),
    pygame.K_s: (0, 1),
    pygame.K_d: (1, 0)
}

LIFT_SPEEDS = (0.0, 92.0, 155.0) # up, hold, down
PIVOT_SPEEDS = (150.0, 85.0, 30.0, 110.0) # left, middle, right, check
GRIP_SPEEDS = (20.0, 90.0) # open, closed

def pygame_setup():
    # Setup the GUI
    pygame.init()
    pygame.display.set_caption('Keyboard Driver')
    screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE), 0, 32)
    surface = pygame.Surface(screen.get_size()).convert()
    return screen, surface

class DriverNode(Node):

    
    def __init__(self):
        super().__init__('driver')

        self.screen, self.surface = pygame_setup()

        self.declare_parameter('color', rclpy.Parameter.Type.STRING)
        self.color = self.get_parameter('color').get_parameter_value().string_value
        
        self.l_goal = 0.0
        self.r_goal = 0.0

        self.drive_command_publisher = self.create_publisher(
                DriveCmd,
                'drive_cmd',
                10)
        
        self.manip_command_publisher = self.create_publisher(
                ManipCmd,
                'manip_cmd',
                10)
        
        self.sort_command_publisher = self.create_publisher(
                String,
                'sort_cmd',
                10)
        
        self.door_command_publisher = self.create_publisher(
                String,
                'door_cmd',
                10)
        
        self.goal_sub = self.create_subscription(RotationGoals, '/rotation_goals', self.goal_callback, 10)
        self.goal_sub

        self.block_seen_sub = self.create_subscription(Bool, '/block_seen', self.block_callback, 10) 
        self.block_seen_sub
        self.block_seen = False

        self.block_color_sub = self.create_subscription(String, '/block_color', self.color_callback, 10) 
        self.block_color_sub
        self.block_color = "red"

        self.ir_sub = self.create_subscription(Int32, '/ir_dist', self.ir_callback, 10) 
        self.ir_sub
        self.ir_dist = 100

        # self.wall_ir_sub

        self.limit_sub = self.create_subscription(LimitBools, '/limit_bools', self.limit_callback, 10)
        self.limit_sub
        self.bottom_limit = False
        self.top_limit = False

        self.block_count = 0
        self.block_held = False
        
        self.cur_drive_state = DriveState.FOLLOW

        self.cur_manip_state = ManipState.IDLE

        self.cur_sort_state = SortState.STORE

        self.cur_door_state = DoorState.CLOSED

        
        self.timer_period = 0.01
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.last_key = None

        self.collection_start_time = None
        self.game_start_time = None

    
    def timer_callback(self):
        pressed = pygame.key.get_pressed()
        key = pressed[pygame.K_SPACE]

        if self.i % 2:
            """
            start auto driver on key press
            """
            drive_cmd_msg = DriveCmd()
            manip_cmd_msg = ManipCmd()
            sort_cmd_msg = String()
            door_cmd_msg = String()

            drive_cmd_msg.l_speed, drive_cmd_msg.r_speed = self.drive_state_callback()
            manip_cmd_msg.lift_cmd, manip_cmd_msg.pivot_cmd, manip_cmd_msg.grip_cmd = self.manip_state_callback()
            sort_cmd_msg.data = self.sort_state_callback()
            door_cmd_msg.data = self.door_state_callback()


            self.drive_command_publisher.publish(drive_cmd_msg)
            self.manip_command_publisher.publish(manip_cmd_msg)
            self.sort_command_publisher.publish(sort_cmd_msg)
            self.door_command_publisher.publish(door_cmd_msg)
            # self.get_logger().info('Publishing left:"%f"' % drive_cmd_msg.l_speed)
            # self.get_logger().info('Publishing right:"%f"' % drive_cmd_msg.r_speed)

        else:
            """
            run keyboard driver otherwise
            """
            drive_speed = 0, 0
            manip_speeds = [LIFT_SPEEDS[1], PIVOT_SPEEDS[1], GRIP_SPEEDS[0]]
            sort_cmd = "store"
            door_cmd = "close"

            box_pos = PADDING + BOX_SIZE, PADDING + BOX_SIZE
            show_dir = False
            for keycode in [pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d]:
                if pressed[keycode]:
                    drive_speed = (drive_speed[0] + KEY_SPEEDS[keycode][0],
                                drive_speed[1] + KEY_SPEEDS[keycode][1])
                    box_pos = (box_pos[0] + BOX_OFFSETS[keycode][0] * BOX_SIZE,
                            box_pos[1] + BOX_OFFSETS[keycode][1] * BOX_SIZE)
                    show_dir = True
                if pressed[pygame.K_UP]:
                    manip_speeds[0] = LIFT_SPEEDS[0]
                if pressed[pygame.K_DOWN]:
                    manip_speeds[0] = LIFT_SPEEDS[2] 
                if pressed[pygame.K_LEFT]:
                    manip_speeds[1] = PIVOT_SPEEDS[0]
                if pressed[pygame.K_RIGHT]:
                    manip_speeds[1] = PIVOT_SPEEDS[2] 
                if pressed[pygame.K_g]:
                    manip_speeds[2] = GRIP_SPEEDS[1]
                if pressed[pygame.K_o]:
                    door_cmd = "open"
                if pressed[pygame.K_b]:
                    sort_cmd = "stack"


            ## Update the GUI
            self.surface.fill(BG_COLOR)
            if show_dir:
                r = pygame.Rect(box_pos, (BOX_SIZE, BOX_SIZE))
                pygame.draw.rect(self.surface, BOX_COLOR, r)
            self.screen.blit(self.surface, (0, 0))
            pygame.display.flip()
            pygame.display.update()

            drive_cmd_msg = DriveCmd()
            drive_cmd_msg.l_speed = float(drive_speed[0])
            drive_cmd_msg.r_speed = float(drive_speed[1])
            self.drive_command_publisher.publish(drive_cmd_msg)

            manip_cmd_msg = ManipCmd()
            manip_cmd_msg.lift_cmd = float(manip_speeds[0])
            manip_cmd_msg.pivot_cmd = float(manip_speeds[1])
            manip_cmd_msg.grip_cmd = float(manip_speeds[2])
            self.manip_command_publisher.publish(manip_cmd_msg)

            sort_cmd_msg = String()
            sort_cmd_msg.data = sort_cmd
            self.sort_command_publisher.publish(sort_cmd_msg)

            door_cmd_msg = String()
            door_cmd_msg.data = door_cmd
            self.door_command_publisher.publish(door_cmd_msg)

        if self.last_key is False and key is True:
            self.i += 1

        self.last_key = key
    
        # Process event queue and sleep between loops
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()


    def goal_callback(self, msg):
        self.l_goal = msg.l_goal
        self.r_goal = msg.r_goal

    def block_callback(self, msg):
        self.block_seen = msg.data

    def color_callback(self, msg):
        self.block_color = msg.data

    def ir_callback(self, msg):
        self.ir_dist = msg.data

    def limit_callback(self, msg):
        self.bottom_limit = msg.bottom_limit
        self.top_limit = msg.top_limit

    def drive_state_callback(self):
        """
        Updates state and returns setpoints for robot acutators.
        """
        #######################
        ###     FOLLOW      ###
        #######################
        if self.game_start_time == None:
            self.game_start_time = self.get_clock().now().nanoseconds / 1e9
        
        now = self.get_clock().now().nanoseconds / 1e9
        game_time_elapsed = now - self.game_start_time
        if game_time_elapsed > 0.0:
            self.cur_drive_state = DriveState.DROP

        if self.cur_drive_state == DriveState.FOLLOW:
            if abs(self.l_goal) < 0.5 and abs(self.r_goal) < 0.5 and self.cur_manip_state == ManipState.IDLE:
                self.cur_drive_state = DriveState.COLLECT
                self.cur_manip_state = ManipState.COLLECT
                self.collection_start_time = self.get_clock().now().nanoseconds / 1e9
                return (0.0, 0.0)

            if self.block_seen:
                return (self.l_goal, self.r_goal)
            
            return (-math.pi, math.pi)

        #######################
        ###     COLLECT     ###
        #######################
        elif self.cur_drive_state == DriveState.COLLECT:
            now = self.get_clock().now().nanoseconds / 1e9
            time_elapsed = now - self.collection_start_time
            if game_time_elapsed > 50.0:
                self.cur_drive_state = DriveState.DROP
                return (0.0, 0.0)
            if time_elapsed < 1.0:
            # if time_elapsed < 2.0 and self.ir_dist > 50: # may change distance
                return (-3.0, -5.5)
            elif time_elapsed > 1.5 and time_elapsed < 3.0:
                return (0.0, 0.0)
            # elif self.ir_dist < 50:
            #     self.cur_drive_state = DriveState.REPOSITION
            #     self.cur_manip_state = ManipState.IDLE
            #     return (0.0, 0.0) 
            else:
                self.cur_drive_state = DriveState.FOLLOW
                return (0.0, 0.0)
            
        elif self.cur_drive_state == DriveState.DROP:
            if game_time_elapsed < 52.0:
                self.cur_door_state = DoorState.OPEN
                return (0.0, 0.0) ## prep
            elif game_time_elapsed > 52.0 and game_time_elapsed < 53.0:
                return (3.0, 0.0) ## grab
            elif game_time_elapsed > 53.0 and game_time_elapsed < 55.0:
                return (-4.0, -4.0) ## lift and check color
            else:
                return (0.0, 0.0)


        
    def manip_state_callback(self):
        if self.cur_manip_state == ManipState.IDLE:
            return (LIFT_SPEEDS[1], PIVOT_SPEEDS[1], GRIP_SPEEDS[0])
        
        ########################
        ###   COLLECT_OPP    ###
        ########################
        elif self.cur_manip_state == ManipState.COLLECT:
            now = self.get_clock().now().nanoseconds / 1e9
            time_elapsed = now - self.collection_start_time
            if time_elapsed < 1.0:
                return (LIFT_SPEEDS[2], PIVOT_SPEEDS[1], GRIP_SPEEDS[0]) ## prep
            elif time_elapsed > 1.0 and time_elapsed < 1.5:
                return (LIFT_SPEEDS[1], PIVOT_SPEEDS[1], GRIP_SPEEDS[1]) ## grab
            elif time_elapsed > 1.5 and time_elapsed < 2.5:
                return (LIFT_SPEEDS[0], PIVOT_SPEEDS[3], GRIP_SPEEDS[1]) ## lift and check color
            elif time_elapsed > 2.5 and time_elapsed < 6.5:
                return (LIFT_SPEEDS[0], PIVOT_SPEEDS[1], GRIP_SPEEDS[1]) ## lift
            elif time_elapsed > 6.5 and time_elapsed < 7.0:
                return (LIFT_SPEEDS[1], PIVOT_SPEEDS[0], GRIP_SPEEDS[1]) ## turn
            elif time_elapsed > 7.0 and time_elapsed < 7.5:
                return (LIFT_SPEEDS[1], PIVOT_SPEEDS[0], GRIP_SPEEDS[0]) ## release
            elif time_elapsed > 7.5 and time_elapsed < 10.5:
                return (LIFT_SPEEDS[2], PIVOT_SPEEDS[1], GRIP_SPEEDS[0]) ## lower
            elif time_elapsed > 10.5 and time_elapsed < 11.0:
                return (LIFT_SPEEDS[1], PIVOT_SPEEDS[1], GRIP_SPEEDS[0]) ## open
            else:
                self.cur_manip_state = ManipState.IDLE
                return (LIFT_SPEEDS[1], PIVOT_SPEEDS[1], GRIP_SPEEDS[0])
    
    def sort_state_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.collection_start_time:
            time_elapsed = now - self.collection_start_time
            if time_elapsed > 2.0 and time_elapsed < 2.5:
                if self.block_color == self.color:
                    self.cur_sort_state = SortState.STACK
                else:
                    self.cur_sort_state = SortState.STORE
        if self.cur_sort_state == SortState.STACK:
            return "stack"
        else:
            return "store"

    def door_state_callback(self):
        if self.cur_door_state == DoorState.OPEN:
            return "open"
        else:
            return "close"
        

class DriveState(Enum):
    FOLLOW = 1
    COLLECT = 2
    DROP = 3

class ManipState(Enum):
    IDLE = 1
    COLLECT = 2

class SortState(Enum):
    STORE = 1
    STACK = 2

class DoorState(Enum):
    CLOSED = 1
    OPEN = 2


def main():
    rclpy.init()
    # Create an instance of DriverNode
    driver_node = DriverNode()

    # Continue to run the node until a stop command is given
    rclpy.spin(driver_node)

    # Destroy the node!
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
