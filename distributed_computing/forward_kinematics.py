"""In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
"""

# add PYTHONPATH
import os
import sys

sys.path.append(
    os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "joint_control")
)
from numpy import identity
import numpy as np
from keyframes import *

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(
        self,
        simspark_ip="localhost",
        simspark_port=3100,
        teamname="DAInamite",
        player_id=0,
        sync_mode=True,
    ):
        super(ForwardKinematicsAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode
        )
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            "Head": [
                "HeadYaw",
                "HeadPitch",
            ],
            # YOUR CODE HERE
            "LArm": [
                "LShoulderPitch",
                "LShoulderRoll",
                "LElbowYaw",
                "LElbowRoll",
            ],
            "RArm": [
                "RShoulderPitch",
                "RShoulderRoll",
                "RElbowYaw",
                "RElbowRoll",
            ],
            "LLeg": [
                "LHipYawPitch",
                "LHipRoll",
                "LHipPitch",
                "LKneePitch",
                "LAnklePitch",
                "RAnkleRoll",
            ],
            "RLeg": [
                "RHipYawPitch",
                "RHipRoll",
                "RHipPitch",
                "RKneePitch",
                "RAnklePitch",
                "LAnkleRoll",
            ],
        }

        self.effectorPositions = {
            "Head": [
                [0.0, 0.0, 126.5],
                [0.0, 0.0, 0.0],
            ],
            "RArm": [
                [0.0, -98.0, 100.0],
                [0.0, 0.0, 0.0],
                [105.0, -15.0, 0.0],
                [0.0, 0.0, 0.0],
                [55.95, 0.0, 0.0],
            ],
            "LArm": [
                [0.0, 98.0, 100.0],
                [0.0, 0.0, 0.0],
                [105.0, 15.0, 0.0],
                [0.0, 0.0, 0.0],
                [55.95, 0.0, 0.0],
            ],
            "RLeg": [
                [0.0, -50.0, -85.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, -100.0],
                [0.0, 0.0, -102.9],
                [0.0, 0.0, 0.0],
            ],
            "LLeg": [
                [0.0, 50.0, -85.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, -100.0],
                [0.0, 0.0, -102.9],
                [0.0, 0.0, 0.0],
            ],
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        """calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        """
        T = identity(4)
        # YOUR CODE HERE

        # rotation matrix for x axis
        def rot_x_4d(theta):
            return np.matrix(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(theta), -np.sin(theta), 0],
                    [0, np.sin(theta), np.cos(theta), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rot_y_4d(theta):
            return np.matrix(
                [
                    [np.cos(theta), 0, np.sin(theta), 0],
                    [0, 1, 0, 0],
                    [-np.sin(theta), 0, np.cos(theta), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rot_z_4d(theta):
            return np.matrix(
                [
                    [np.cos(theta), -np.sin(theta), 0, 0],
                    [np.sin(theta), np.cos(theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )

        # print(joint_name)
        # print(0 if np.abs(joint_angle) == 0 else joint_angle)
        # print("is in z:", joint_name in z_rot_joints)
        # print("is in y:", joint_name in y_rot_joints)
        # print("is in x:", joint_name in x_rot_joints)
        if joint_name.endswith("Roll"):
            T = rot_x_4d(joint_angle)
        elif joint_name.endswith("Pitch"):
            T = rot_y_4d(joint_angle)
        elif joint_name.endswith("Yaw"):
            T = rot_z_4d(joint_angle)

        for chain in self.chains.keys():
            if joint_name in self.chains[chain]:
                joint_index = self.chains[chain].index(joint_name)
                # print(joint_index)
                # print(joint_name)
                # print(self.effectorPositions[chain][joint_index])
                T[0, 3] = self.effectorPositions[chain][joint_index][0]
                T[1, 3] = self.effectorPositions[chain][joint_index][1]
                T[2, 3] = self.effectorPositions[chain][joint_index][2]

        # print("T", T)
        return T

    def forward_kinematics(self, joints):
        """forward kinematics

        :param joints: {joint_name: joint_angle}
        """
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T, Tl)

                self.transforms[joint] = T


if __name__ == "__main__":
    agent = ForwardKinematicsAgent()
    agent.run()
