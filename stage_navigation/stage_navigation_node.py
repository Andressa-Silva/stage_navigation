#!/usr/bin/env python3

import math
import rclpy
import rclpy.executors
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from time import sleep
import copy

class stage_navigation_node(Node):
    """
    Class to start the navigation node for multiple targets
    """
    
    def __init__(self):
        super().__init__('stage_navigation_node')

        self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/odom', self.position_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        data = Odometry()
        self.goal_position = data.pose.pose.position
        self.targets = [(5, 4), (4, -4)]      # List of target coordinates
        self.current_target_index = 0         
        self.update_goal_position()           

        self.state = 0  
        self.state_wall = 0
        self.state_go_to_point = 0
        self.yaw_precision = math.pi / 180    # Yaw angle precision (1 degree)
        self.dist_precision = 0.3             
        self.initial_position = 0.
        self.aux_initial = True
        self.done = False                     # Flag indicating if a target has been reached
        self.reverse_done = False             # Flag indicating if reverse and turn maneuver is completed

        rclpy.get_default_context().on_shutdown(self.shutdown)

    def update_goal_position(self):
        """
        Updates the goal position using the current target coordinates.
        Corrects the origin and angle of the robot's position in the map.
        """        
        
        if self.current_target_index < len(self.targets):
            position_alvo_x, position_alvo_y = self.targets[self.current_target_index]
           
            position_origem_robot_map_x = -7
            position_origem_robot_map_y = -7
            position_alvo_x_corrigido = position_alvo_x - position_origem_robot_map_x
            position_alvo_y_corrigido = position_alvo_y - position_origem_robot_map_y
            angle_origem_robot = 45
            angle_origem_corrigido = -angle_origem_robot * math.pi / 180
            
            self.goal_position.x = position_alvo_x_corrigido * math.cos(angle_origem_corrigido) - \
                                   position_alvo_y_corrigido * math.sin(angle_origem_corrigido)
            self.goal_position.y = position_alvo_x_corrigido * math.sin(angle_origem_corrigido) + \
                                   position_alvo_y_corrigido * math.cos(angle_origem_corrigido)
            print('Atualizando para alvo:', self.goal_position)

    def go_to_point(self):
        """
        Manages the robot's movement towards the target point.
        It switches between adjusting yaw, moving straight, and handling reaching the target.
        """
        
        if self.state_go_to_point == 0:
            self.fix_yaw()
        elif self.state_go_to_point == 1:
            self.straight()
        elif self.state_go_to_point == 2:
            if self.done and not self.reverse_done:
                print(f"Chegou ao alvo {self.current_target_index + 1}")
                self.reverse_and_turn()  
            elif self.done and self.reverse_done:
                self.next_target() 

    def next_target(self):
        """
        Sets the next target position if available. Stops the robot if no more targets.
        """
        
        cmd = Twist()
        cmd.linear.x = 0.0  
        self.velocity_publisher.publish(cmd)
        self.initial_position = copy.deepcopy(self.position)
        self.done = False
        self.reverse_done = False
        
        self.current_target_index += 1
        if self.current_target_index < len(self.targets):
            self.update_goal_position()
            self.state_go_to_point = 0
        else:
            print("Todos os alvos alcançados!")

    def reverse_and_turn(self):
        """
        Performs a reverse maneuver and a turn for collision avoidance.
        """
        
        cmd = Twist()
        cmd.linear.x = -0.2 
        self.velocity_publisher.publish(cmd)
        sleep(2)  
        cmd.linear.x = 0.0  
        self.velocity_publisher.publish(cmd)
        sleep(0.5)
        
        cmd.angular.z = 0.7
        self.velocity_publisher.publish(cmd)
        sleep(1.5)  
        cmd.angular.z = 0.0  
        self.velocity_publisher.publish(cmd)
        sleep(0.5)

        # Check if the front is clear after the maneuver
        if self.regions['front'] > 0.6:  
            self.reverse_done = True
        else:
            print("Caminho ainda bloqueado, ajustando mais...")
            self.adjust_position()

    def adjust_position(self):
        """
        Makes additional adjustments to avoid obstacles if needed.
        """
        
        cmd = Twist()
        cmd.linear.x = -0.1 
        self.velocity_publisher.publish(cmd)
        sleep(1) 
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5 
        self.velocity_publisher.publish(cmd)
        sleep(1)
        cmd.angular.z = 0.0
        self.velocity_publisher.publish(cmd)

        # Check again if the front is clear
        if self.regions['front'] > 0.6:
            self.reverse_done = True
        else:
            print("Caminho ainda bloqueado após ajuste.")

    def fix_yaw(self):
        """
        Adjusts the robot's yaw (heading) to face the goal position.
        """
        
        cmd = Twist()
        if math.fabs(self.error_angle) > self.yaw_precision:
            cmd.angular.z = 0.3 if self.error_angle > 0 else -0.3
        self.velocity_publisher.publish(cmd)

        if math.fabs(self.error_angle) <= self.yaw_precision:
            self.state_go_to_point = 1

    def straight(self):
        """
        Moves the robot straight towards the goal, with obstacle avoidance checks.
        """
        
        if self.regions['front'] < 0.6:
            print("Caminho bloqueado enquanto vai reto, ajustando...")
            self.state = 1         # Switch to wall following
        elif self.distance_goal() > self.dist_precision:
            cmd = Twist()
            cmd.linear.x = 0.3
            self.velocity_publisher.publish(cmd)
        else:
            self.done = True
            self.state_go_to_point = 2

        if math.fabs(self.error_angle) > self.yaw_precision:
            self.state_go_to_point = 0

    def follow_wall(self):
        """
        Follows the wall by switching between states: finding, turning, and following the wall.
        """
        
        if self.state_wall == 0:
            self.find_wall()
        elif self.state_wall == 1:
            self.turn_left()
        elif self.state_wall == 2:
            self.follow_the_wall()

    def normalize(self, angle):
        """
        Normalizes an angle to keep it within the range -pi to pi.
        """
        
        if(math.fabs(angle) > math.pi):
            angle -= 2 * math.pi
        elif(math.fabs(angle) < -math.pi):
            angle += 2 * math.pi
        return angle

    def find_wall(self):
        """
        Moves the robot forward while turning right to find a wall.
        """
        
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = -0.3
        self.velocity_publisher.publish(cmd)

    def turn_left(self):
        """
        Turns the robot left to follow the wall.
        """
        cmd = Twist()
        cmd.angular.z = 0.3
        self.velocity_publisher.publish(cmd)

    def follow_the_wall(self):
        """
        Moves the robot forward while following a wall, adjusting its position to stay at a safe distance.
        """
        cmd = Twist()
        cmd.linear.x = 0.15  # Reduz a velocidade para melhorar o controle ao seguir a parede
        cmd.angular.z = 0.2  # Ajusta a velocidade angular para se manter afastado
        self.velocity_publisher.publish(cmd)

    def distance_goal(self):
        """
        Computes the Euclidean distance from the robot's current position to the goal position.
        """
        
        return round(math.hypot(self.goal_position.x - self.position.x, self.goal_position.y - self.position.y), 2)

    def shutdown(self):
        print('Shutting!')
        linear_velocity = Twist()

        linear_velocity.linear.x = 0.0  
        linear_velocity.linear.y = 0.0  
        linear_velocity.linear.z = 0.0  

        self.velocity_publisher.publish(linear_velocity)
        rclpy.sleep(1)

    def laser_callback(self, data):
        """
        Callback for LaserScan messages, updates the distance readings in front, left, and right of the robot.
        """
        
        self.laser_reading = data

        self.regions = {
            'right': min(min(data.ranges[30:50]), 5),
            'front_right': min(min(data.ranges[80:100]), 5),
            'front': min(min(data.ranges[125:145]), 5),
            'front_left': min(min(data.ranges[170:190]), 5),
            'left': min(min(data.ranges[215:235]), 5)
        }

        self.action_wall()

    def action_wall(self):
        """
        Determines the robot's wall-following state based on the distances measured by the LIDAR 
        in different directions: front, front-left, and front-right. Adjusts the robot's behavior 
        depending on the proximity to obstacles in these regions.
        
        The function uses a threshold distance 'd' to classify the robot's surroundings as either 
        clear or obstructed, and sets the state of the robot accordingly:
        
        state_wall = 0: Continue searching for a wall.
        state_wall = 1: Turn left or adjust to avoid an obstacle ahead.
        state_wall = 2: Follow the wall closely on the right side.
        
        Regions:
        - front: LIDAR reading directly in front of the robot.
        - front_left: LIDAR reading diagonally to the left of the robot.
        - front_right: LIDAR reading diagonally to the right of the robot.
        """
        
        distance = 0.6   # Safety distance threshold 
        if self.regions['front'] > distance and self.regions['front_left'] > distance and self.regions['front_right'] > distance:
            self.state_wall = 0
        elif self.regions['front'] < distance and self.regions['front_left'] > distance and self.regions['front_right'] > distance:
            self.state_wall = 1
        elif self.regions['front'] > distance and self.regions['front_left'] > distance and self.regions['front_right'] < distance:
            self.state_wall = 2
        elif self.regions['front'] > distance and self.regions['front_left'] < distance and self.regions['front_right'] > distance:
            self.state_wall = 0
        elif self.regions['front'] < distance and self.regions['front_left'] > distance and self.regions['front_right'] < distance:
            self.state_wall = 1
        elif self.regions['front'] < distance and self.regions['front_left'] < distance and self.regions['front_right'] > distance:
            self.state_wall = 1
        elif self.regions['front'] < distance and self.regions['front_left'] < distance and self.regions['front_right'] < distance:
            self.state_wall = 1
        elif self.regions['front'] > distance and self.regions['front_left'] < distance and self.regions['front_right'] < distance:
            self.state_wall = 0
        else:
            print('not found situation')

    def distance_to_line(self):
        """
        Calculates the shortest distance from the robot's current position to the line formed between
        the initial position (starting point) and the goal position (target).

        This function uses the formula for the distance between a point (robot's position) and a line 
        defined by two points (initial and goal positions). The formula is derived from the 
        general equation of a line.

        Returns:
            float: The shortest distance from the robot's current position to the line connecting 
                the initial and goal positions.
        """
        up_eq = math.fabs((self.goal_position.y - self.initial_position.y) * self.position.x - (self.goal_position.x - self.initial_position.x)
                          * self.position.y + (self.goal_position.x * self.initial_position.y) - (self.goal_position.y * self.initial_position.x))
        lo_eq = math.sqrt(pow(self.goal_position.y - self.initial_position.y,
                              2) + pow(self.goal_position.x - self.initial_position.x, 2))

        distance = up_eq / lo_eq
        return distance

    def position_callback(self, data):
        """
        Callback for Odometry messages, updates the robot's position and yaw angle.
        """
        
        self.position = data.pose.pose.position

        if self.aux_initial:
            self.initial_position = copy.deepcopy(self.position)
            self.aux_initial = False

        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x,
                            orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

        self.goal_angle = math.atan2(
            self.goal_position.y - self.position.y, self.goal_position.x - self.position.x)

        self.heading = self.goal_angle - self.yaw

        self.error_angle = self.normalize(self.heading)
        print('error_angle', self.error_angle)

if __name__ == '__main__':
    rclpy.init(args=None)

    env = stage_navigation_node()

    count_time = 0 
    count_loop = 0

    print('foi', rclpy.ok())
    sleep(1)

    while rclpy.ok():
        rclpy.spin_once(env)
        rclpy.spin_once(env)
        distance_position_to_line = env.distance_to_line()

        print('state_wall', env.state_wall, 'state_point', env.state_go_to_point, 'sate', env.state)
        print(env.regions)
        print('distance position line', distance_position_to_line)
        print('distance goal', env.distance_goal())

        if env.state == 0:
            env.go_to_point()
            if env.regions['front'] > 0.01 and env.regions['front'] < 0.45:
                env.state = 1

        elif env.state == 1:
            env.follow_wall()
            if count_time > 10 and distance_position_to_line < 0.15:
                env.state = 0
                count_time = 0

        count_loop += 1
        if count_loop == 20:
            count_time += 1
            count_loop = 0