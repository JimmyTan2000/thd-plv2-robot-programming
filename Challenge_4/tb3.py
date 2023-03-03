import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
from transforms3d.euler import quat2euler
from math import pi 
from math import floor

state = "go"
# angle_1 = 0
# angle_2 = None
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
                qos_profile_sensor_data)  # allows packet loss
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0


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
        global state
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        print('x:', x)
        print('y:', y)
        print('z:', z)
        print('w:', w)
        angles = quat2euler([w,x,y,z])
        i = angles[0] * 180 / pi
        y = angles[1] * 180 / pi
        z = angles[2] * 180 / pi
        rotation_z = floor(i)
        print(i, y, z)
        # if state == "go":
        #     global angle_1
        #     angle_1 = abs (z)
        if state == "stopped":
            self.rotate(15)
            if  round(abs(z)) == 180:
                self.rotate(0)
                state = "first rotation stopped"
        #print(floor(abs(abs(z)-angle_1)))
    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        global state
        print()
        print('Distances:')
        print('⬆️ :', msg.ranges[0])
        print('⬇️ :', msg.ranges[180])
        print('⬅️ :', msg.ranges[90])
        print('➡️ :', msg.ranges[-90])
        print(state)
        
        # self.collision_avoidance_sensor(msg, 0.25)
        #self.loop_with_collision_avoidance(msg, 0.35)
        if state == "go":
            self.collision_avoidance_sensor(msg, 0.3, "stopped")

        if state == "first rotation stopped":
            self.collision_avoidance_sensor(msg, 0.3, "end")
        

    def stop(self):
        self.vel(0)

    def go(self, val = 15):
        self.vel(val)

    def collision_avoidance_sensor(self, msg, min_distance, next_state = None):
        if msg.ranges[0] > min_distance:
            self.go(20)
        else:
            global state
            self.stop()
            print("it is stopped")
            state = next_state

    def rotate(self, ang_speed):
        self.vel(0, ang_speed)

    # def loop_with_collision_avoidance(self, msg, min_distance,rotation_z):
    #     global state
    #     if state == "go":
    #         self.collision_avoidance_sensor(msg, 0.3, "stopped")
        
    #     if state == "stopped":
    #         self.rotate(15)
    #         if rotation_z == 90:
    #             self.rotate(0)
    #             state = "first rotation stopped"

    #     if state == "first rotation stopped":
    #         self.collision_avoidance_sensor(msg, 0.3, "first wall reached")

    #     # if state == "first wall reached":
    #     #     self.rotate(10)
    #     #     if msg.ranges[0] > min_distance + 0.02 and msg.ranges[-90] <= min_distance - 0.12:
    #     #         state = "go"

                



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