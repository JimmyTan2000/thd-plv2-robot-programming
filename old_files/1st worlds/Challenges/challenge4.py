import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
from transforms3d.euler import quat2euler
from math import pi 
from math import floor
from enum import Enum, auto

class State(Enum):
    GO = auto()
    STOPPED = auto()
    REACHED_FIRST_CELL = auto()
    FIRST_ROTATION_STOPPED = auto()
    SECOND_ROTATION_STOPPED = auto()
    END = auto()

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                1)          # history depth

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss

        self.odom_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,  # function to run upon message arrival
                1)  # allows packet loss

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.state = State.GO

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent
    
    def odom_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        angles = quat2euler([w,x,y,z])
        i = angles[0] * 180 / pi
        j = angles[1] * 180 / pi
        k = angles[2] * 180 / pi
        # rint(i, j, k)
        if self.state == State.GO:
            self.rotate(-10)
            print(round(abs(k)))
            if  round(abs(k)) == 0:
                self.rotate(0)
                self.state = State.FIRST_ROTATION_STOPPED
        if self.state == State.FIRST_ROTATION_STOPPED:
            if msg.pose.pose.position.x < 1.50:
                self.go(15)
            else:
                self.stop()
                self.state = State.REACHED_FIRST_CELL
        if self.state == State.REACHED_FIRST_CELL:
            self.rotate(10)
            if  round(abs(k)) == 90:
                self.rotate(0)
                self.state = State.SECOND_ROTATION_STOPPED



    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        print()
        print('Distances:')
        print('⬆️ :', msg.ranges[0])
        print('⬇️ :', msg.ranges[180])
        print('⬅️ :', msg.ranges[90])
        print('➡️ :', msg.ranges[-90])
        # print('Intensities: ', msg.intensities)
        
        if self.state == State.SECOND_ROTATION_STOPPED:
            self.collision_avoidance_sensor(msg, 0.3, State.STOPPED)

        
    def stop(self):
        self.vel(0)

    def go(self, val = 15):
        self.vel(val)

    def collision_avoidance_sensor(self, msg, min_distance, next_state = None):
        if msg.ranges[0] > min_distance:
            self.go(15)
        else:
            self.stop()
            print("it is stopped")
            self.state = next_state

    def rotate(self, ang_speed):
        self.vel(0, ang_speed)

def main(args=None):
    rclpy.init(args=args)
    rclpy.ok()
    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()