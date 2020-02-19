import rospy
import rospy_template.msg
from geometry_msgs.msg import Pose
import pandas as pd

class TemplateException(Exception):
    pass


class Template:
    def __init__(self):
        rospy.Subscriber('/orb_slam2_mono/pose', Pose,
                         self.msg_pose_slam)
        rospy.Subscriber('/drone/gt_pose', Pose,
                         self.msg_pose_real)
        self.df = pd.DataFrame(columns=['real_pos', 'real_ori', 'slam_pos', 'slam_ori'])
        self.real_ori = []
        self.real_pos = []
        self.slam_ori = []
        self.slam_pos = []


    def save_df(self, path):
        min_len = int(min([len(self.real_pos),
                           len(self.real_ori),
                           len(self.slam_ori),
                           len(self.slam_pos)]))

        self.df.append(pd.DataFrame(data=dict(
            real_pos=self.real_pos[:min_len],
            real_ori=self.real_ori[:min_len],
            slam_pos=self.slam_pos[:min_len],
            slam_ori=self.slam_ori[:min_len]
        ))).to_csv(path)

    def msg_pose_real(self, pose):
        self.real_ori.append(pose.orientation)
        self.real_pos.append(pose.position)

    def msg_pose_slam(self, pose):
        print('slam')
        self.slam_ori.append(pose.orientation)
        self.slam_pos.append(pose.position)

