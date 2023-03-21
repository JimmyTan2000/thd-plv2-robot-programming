# Challenges 
During the 8 days course of Robot Programming, Professor Gökçe Aydos has introduced a set of problems for us to solve. They will be addressed and explained one by one below.

## Challenge 1 
This is the first challenge to get our hands warm for the course. The goal of this challenge is to drive to the red wall as close as we can and stop without colliding into it. For this challenge, we need to run the bot in a simulated world of 1 x 1 as shown in the screenshot below: 

![This is a pic for 1 x 1 world](/Screenshots/world_1_1.png)

In order to solve this challenge, there are two questions that has to be solved. 
1. What is the minimum value of the laser distance sensor (LDS) that we should stop? 
2. How can we set the speed of the bot?

### Solution for Challenge 1 
In challenge 1 (as well as other following challenges), I am using tb3.py from Professor Gökçe Aydos as a starting point. In his code, These is a class for Tb3 Node, and inside this node there is a publisher and a subscriber. The publisher will publish to the 'cmd_vel' topic and the subscriber will get the data from the Laser Scanner. The original code for the class are shown below:   

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

As we can see from the code above, the function "scan_callback()" is making use of the subscriber to get the data from the simulated Laser Scanner from the bot. When the Tb3 node is active, the scan_callback() function will be run constantly and the laser data will be collected at the rate of 5 hertz. How do I know that?

While the node is active, we can see all active topics by running the following command in the shell:

    ros2 topic list -t 

The flag "-t" enables us to see the type of every topic. 
After running the above command, it returns the following: 

![Pic for topics](/Screenshots/topic_list.png)

The topic responsible for the laser scan is "/scan", as you can see from the type written behind in the square bracket. You can check its frequecy by the following command:

    ros2 topic hz /scan

And it returns the following: 

![Frequency for topic /scan](/Screenshots/scan_frequency.png)

#### What is the minimum value of the laser distance sensor (LDS) that we should stop?
Ideally, we should stop very near to the wall without crashing into it. Theoretically speaking, the closer the distance is to 0, the better. But sadly, there are simply too much noises in the Laser Scan (simulated LIDar) sensor, so we cannot set the distance too close to the wall as well. The sweet spot I found via trial and error is about 0.18.

#### How can we set the speed of the bot?
The percentage of angular and linear velocity, defined by "self.ang_vel_percent" and "self.lin_vel_percent" respectively, are set to 0 by default. We can use the setter function "vel()" to change both of the values. The vel() function takes two extra parameters, namely "lin_vel_percent" that is used to change the linear velocity, and "ang_vel_percent" that is used to change the angular velocity of the bot. The angular velocity, however, will be set to 0 by default if the value is not given into the function. 
Therefore, we can make the bot to move forward with for example 15% of its max linear velocity by writing the following code: 

    self.vel(15)

Combining both concepts, the solution looks as simple as below: 

     def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        if msg.ranges[0] > 0.18:
            self.vel(15)
        else:
            self.vel(0)

This function will be executed everytime a LaserScan msg is received, about 5 times per second. The value msg.ranges[0] is the distance between the direct front of the bot to the obstacle (normally walls in our labyrith). If the distance between them is more than 0.18, the bot will simply move forward with 15% of its maximum linear velocity, otherwise, it will stop completly. 

#### Below is the simulation for the solution: 

![Challenge 1 gif](/Screenshots/gifs/challenge1_showcase.gif)

## Challenge 2 
