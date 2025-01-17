#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



from evry_project_plugins.srv import DistanceToFlag


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 5.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear, min=-3, max=3)
        cmd_vel.angular.z = self.constraint(angular, min=-2, max=2)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

def rep(p1,p2,th):
    vec = p1-p2
    d = np.linalg.norm(vec,2)
    if  d > th:
        return 0
    else:
        return (vec/(d*d + 1e-4))



def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")
    robot1 =  Robot('robot_1')
    robot2 =  Robot('robot_2')
    robot3 =  Robot('robot_3')

    # Timing
    robot_id = int(robot_name[-1])

    katt = 0.1
    krep = 20
    th = 30
    kw = 0.75
    kv = 1

    flag1 = np.reshape(np.array([-21.21320344,21.21320344]),(-1,1))
    flag2 = np.reshape(np.array([21.21320344,21.21320344]),(-1,1)) 
    flag3 = np.reshape(np.array([0,-30]),(-1,1))

    while not rospy.is_shutdown():
        # Strategy
        p1 = np.reshape(np.array([robot1.x,robot1.y]),(-1,1))
        p2 = np.reshape(np.array([robot2.x,robot2.y]),(-1,1))
        p3 = np.reshape(np.array([robot3.x,robot3.y]),(-1,1))

        distance = float(robot.getDistanceToFlag())
        
        robot_id = int(robot_name[-1])

        if robot_id == 1:
            Frep = 0
            F = katt*(flag1 - p1)
        elif robot_id == 2:
            Frep = krep*rep(p2,p1,th)
            F = katt*(flag2 - p2) + Frep
        else:
            Frep = krep*rep(p3,p1,th) + krep*rep(p3,p2,th) 
            F = katt*(flag3 - p3) + 0.5*Frep

        angl_error = -robot.yaw + np.arctan2(F[1],F[0])
        velocity = kv*np.linalg.norm(F,2)*np.cos(angl_error)  
        angle = kw*(angl_error) 

        if distance<0.02:
            angle = 0
            
        print(f"{robot_name} Distance to Flag = ", distance)
        print(f"{robot_name} Repulsive Force = ", Frep)

        # Finishing by publishing the desired speed. 
        # DO NOT TOUCH.
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.5)
        

if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()


