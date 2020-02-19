import rospy
import rospy_template.msg
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
import pandas as pd
from tqdm import tqdm
import sys, select, termios, tty
import mutex
from math import cos, sin

class MoveException(Exception):
    pass


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Move:
    def __init__(self):
        #settings = termios.tcgetattr(sys.stdin)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.sub_robot_pose = rospy.Subscriber('orb_slam2_ros/pointcloud',
                                               PoseStamped, self.orbslam_pointcloud_callback)
        rospy.init_node('move_robot', anonymous=True)

        rate_node = 10
        self.rate = rospy.Rate(rate_node)

        self.robot_poses_mutex = mutex.mutex()

        self.initial_point = []
        self.robot_poses_orb = []
        self.last_pose = None
        rospy.wait_for_message("/drone/takeoff", Empty)
        nsec_2_wait = 5
        for _ in range(rate_node * nsec_2_wait):
            self.rate.sleep()
        print("working")
        self.run_elipses_with_orbslam()
        

    def run_elipses_with_orbslam(self):
        curr_pose = None
        radius = 10
        # if self.robot_poses_mutex.testandset():
        #     try:
        #         curr_pose = self.robot_poses_orb.pop(-1)
        #         print(last)
        #     except IndexError as e:
        #         pass
        # if curr_pose is not None:
        #     if self.last_pose is None:
        #         self.last_pose = curr_pose
        #         return
        for angle in range(360):
            twist = Twist()
            twist.linear.x = radius * cos(angle); twist.linear.y = 10 * sin(angle);
            self.pub_cmd_vel.publish(twist)
            self.rate.sleep()

    def orbslam_pointcloud_callback(self, point_cloud):
        print(point_cloud)


    def run_by_file(self):
        positions = self.read_teleops()
        for pos in positions:
            vel = pos['linear']
            spd = pos['angular']
            xv = vel['x']; yv = vel['y'];  zv = vel['z']
            xa = spd['x']; ya = spd['y'];  za = spd['z']
            twist = Twist()
            twist.linear.x = xv; twist.linear.y = yv; twist.linear.z = zv;
            twist.angular.x = xa; twist.angular.y = ya; twist.angular.z = za
            self.pub_cmd_vel.publish(twist)
            self.rate.sleep()
        land = rospy.Publisher('/drone/land', Empty, queue_size=1)
        land = land.publish(Empty())
        print("ending")
        rospy.signal_shutdown("finished ok")


    def read_teleops(self, path='/home/deadalus/CatkinEnvs/cmds2.txt'):
        lines = open(path).readlines()

        angular = False
        linear = False
        teleop = []
        linear_cur = {}
        angular_cur = {}
        linear_count = 0
        angular_count = 0
        for l in lines:
            if '---' in l:
                continue
    
            if 'angular:' in l:
                angular = True
                continue
            elif 'linear:' in l:
                linear = True
                continue
    
            if linear:         
                linear_cur[l[2:3]] = float(l[4:])
                linear_count += 1
                if linear_count > 2:
                    linear = False
                    linear_count = 0
            elif angular:         
                angular_cur[l[2:3]] = float(l[4:])
                angular_count += 1
                if angular_count > 2:
                    angular = False
                    angular_count = 0

            if linear_cur.keys() == ['y', 'x', 'z'] and angular_cur.keys() == ['y', 'x', 'z']:
                teleop.append(dict(linear=linear_cur, angular=angular_cur))
                linear_cur = {}
                angular_cur = {}

        return teleop
