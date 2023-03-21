import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

state = "go"
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

    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        print()
        print('Distances:')
        print('⬆️ :', msg.ranges[0])
        print('⬇️ :', msg.ranges[180])
        print('⬅️ :', msg.ranges[90])
        print('➡️ :', msg.ranges[-90])
        print(state)
        
        # self.collision_avoidance_sensor(msg, 0.25)
        self.loop_with_collision_avoidance(msg)

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

    def loop_with_collision_avoidance(self, msg):
        global state
        if state == "go":
            self.collision_avoidance_sensor(msg, 0.3, "stopped")
        
        if state == "stopped":
            self.rotate(10)
            if msg.ranges[-90] <= 0.24:
                self.rotate(0)
                state = "first rotation stopped"

        if state == "first rotation stopped":
            self.collision_avoidance_sensor(msg, 0.3, "first wall reached")

        # if state == "first wall reached":
        #     self.rotate(10)
        #     if msg.ranges[0] > min_distance + 0.02 and msg.ranges[-90] <= min_distance - 0.12:
        #         state = "go"

                



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