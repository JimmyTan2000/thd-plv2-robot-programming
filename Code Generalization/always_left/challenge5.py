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
    DRIVING = auto()
    BLOCKED = auto()
    AT_JUNCTION = auto()
    DEBUG = auto()
    ROTATING = auto()
    CLEARED = auto()
    REACHED_RED_WALL = auto()
    END = ()
    
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
        # self.state = State.DEBUG # GO for normal execution and DEBUG for debugging 
        self.state = State.DRIVING
        self.last_location_x = []
        self.last_location_y = []
        #self.at_junction = False
        #self.junction = [None, None, None] # left, x coord, y coord

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
        # print(i, j, k)

        # TODO premature rotation stopping observed
        if self.state == State.ROTATING:
            print(round(abs(k)))
            if round(abs(k)) == 0 or round(abs(k)) == 90 or round(abs(k)) == 180:
                self.rotate(0)
                self.state = State.DRIVING

        # TODO inspect this, stutter at junction turning consistently observed
        if self.state == State.AT_JUNCTION:
            print("AT JUNCTION")
            self.last_location_x.append(msg.pose.pose.position.x)
            self.last_location_y.append(msg.pose.pose.position.y)
            if abs(msg.pose.pose.position.x - self.last_location_x[0]) >= 0.2 or abs(msg.pose.pose.position.y - self.last_location_y[0]) >= 0.2:
                self.rotate(15)
                self.last_location_x = []
                self.last_location_y = []
                self.state = State.ROTATING

    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        # print()
        # print('Distances:')
        # print('⬆️ :', msg.ranges[0])
        # print('⬇️ :', msg.ranges[180])
        # print('⬅️ :', msg.ranges[90])
        # print('➡️ :', msg.ranges[-90])
        # print('Intensities: ', msg.intensities)
        if self.state == State.DRIVING:
            self.go()
    
        if self.state == State.DEBUG:
            print()
            print('⬆️ :', msg.ranges[0])
            print('⬇️ :', msg.ranges[180])
            print('⬅️ :', msg.ranges[90])
            print('➡️ :', msg.ranges[-90])

        if msg.ranges[0] < 0.6:
            self.state = State.BLOCKED
       
        if self.state == State.BLOCKED:
            self.stop()
            self.decide_rotate_direction(msg)

        if msg.ranges[0] > 1.2 and msg.ranges[90] > 1.2 and self.state == State.DRIVING and msg.ranges[180] > 1.2:
            self.state = State.AT_JUNCTION

        if msg.intensities[0] == 2.0:
            self.state = State.REACHED_RED_WALL

        if self.state == State.REACHED_RED_WALL:
            print("Red wall seen")
            self.touch_wall_and_stop(msg, 0.25, State.END)

    def decide_rotate_direction(self, msg):
        print("deciding")
        if msg.ranges [-90] > 1.1 and msg.ranges[90] > 1.1:
            self.rotate(15)
        elif msg.ranges[-90] > msg.ranges[90]:
            self.rotate(-15)
        else:
            self.rotate(15)
        # self.rotate(15)
        self.state = State.ROTATING

    def stop(self):
        self.vel(0)

    def go(self, val = 100):
        self.vel(val)

    def touch_wall_and_stop(self, msg, slow_distance, next_state = None):
        if msg.ranges[0] > slow_distance:
            self.go()
        else:
            self.go(1)
            if msg.ranges[0] <= 0.15:
                print("Red wall touched")
                self.stop()

    def collision_avoidance_sensor(self, msg, min_distance, next_state = None):
        if msg.ranges[0] > min_distance:
            self.go()
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