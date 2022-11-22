"""In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
"""


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, sin, cos, pi, matrix, random, linalg, asarray
from math import atan2
from scipy.linalg import pinv
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        """solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        """

        # def from_transformation(matrix):
        #     print(matrix)
        #     x, y, z = matrix[3, 0], matrix[3, 1], matrix[3, 2]
        #     tx, ty, tz = 0, 0, 0

        #     if matrix[2, 2] == 1:
        #         tz = atan2(matrix[1, 0], matrix[0, 0])
        #     elif matrix[1, 1] == 1:
        #         ty = atan2(matrix[0, 2], matrix[0, 0])
        #     elif matrix[0, 0] == 1:
        #         tx = atan2(matrix[2, 1], matrix[1, 1])

        #     return np.asarray([x, y, z, tx, ty, tz])

        # joint_angles = {}

        # for joint in self.chains[effector_name]:
        #     joint_angles[joint] = self.perception.joint[joint]
        # for joint in self.joint_names:
        #     if joint not in joint_angles:
        #         joint_angles[joint] = 0
        # print(joint_angles)
        # print(effector_name)
        # # constants
        # l = 1  # lambda
        # max_step = 0.1

        # last_effector = self.chains[effector_name][-1]
        # solution = from_transformation(transform).T

        # solved = False

        # while not solved:
        #     self.forward_kinematics(joint_angles)

        #     TS = [self.transforms[n] for n in self.chains[effector_name]]
        #     T = matrix(from_transformation(TS[-1])).T

        #     # calculate error
        #     error = solution - T

        #     error[error > max_step] = max_step
        #     error[error < -max_step] = -max_step

        #     T2 = matrix([from_transformation(i) for i in TS[1:-1]]).T

        #     J = solution - T2
        #     dt = solution - T2

        #     J[0, :] = -dt[1, :]
        #     J[1, :] = dt[0, :]
        #     J[-1, :] = 1
        #     JJT = J * J.T

        #     d_theta = l * np.dot(np.dot(J.T, np.linalg.pinv(JJT)), dt.T)

        #     for i, name in enumerate(self.chains[effector_name]):
        #         joint_angles[name] += np.asarray(d_theta.T)[0][i]

        #     if linalg.norm(error) < 0.001:
        #         solved = True

        # joint_angles = []
        # YOUR CODE HERE

        def from_trans(matrix):
            return [
                matrix[0, -1],
                matrix[1, -1],
                matrix[2, -1],
                atan2(matrix[1, 0], matrix[0, 0]),
            ]

        l = 0.1  # lambda
        max_step = 0.1  # gradient descent step size
        margin_error = 1e-4  # margin of error
        steps = 1000  # max number of steps 1000 is OK, 10.000 overkill

        joint_angles = {}

        # get current joint angles
        for joint in self.chains[effector_name]:
            joint_angles[joint] = self.perception.joint[joint]
        # set all other joints to 0
        for joint in self.joint_names:
            if joint not in joint_angles:
                joint_angles[joint] = 0

        target = from_trans(transform)
        for i in range(steps):
            # do fk
            self.forward_kinematics(joint_angles)

            # get values of the joints after fk
            Ts = list(self.transforms.values())
            Te = matrix([from_trans(Ts[-1])]).T
            # calculate error
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            # calculate jacobian
            T = matrix([from_trans(i) for i in Ts[1:-1]]).T
            J = Te - T
            dT = Te - T

            J[0, :] = -dT[1, :]
            J[1, :] = dT[0, :]
            J[-1, :] = 1

            d_theta = l * pinv(J) * e

            for i, joint in enumerate(self.chains[effector_name]):
                joint_angles[joint] += np.asarray(d_theta.T)[0][i]

            if linalg.norm(d_theta) < margin_error:
                break

        return [j for j in joint_angles.values()]

    def set_transforms(self, effector_name, transform):
        """solve the inverse kinematics and control joints use the results"""
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        names = self.chains[effector_name]
        times = [[1.0, 2.0]] * len(names)
        keys = [
            [
                [self.perception.joint[name], [3, 0, 0], [3, 0, 0]],
                [joint_angles[i], [3, 0, 0], [3, 0, 0]],
            ]
            for i, name in enumerate(names)
        ]

        self.keyframes = (names, times, keys)  # the result joint angles have to fill in


if __name__ == "__main__":
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)

    T[0,-1] = 1
    T[1,-1] = 1
    T[2,-1] = 1

    agent.set_transforms('LArm', T)

    agent.run()
