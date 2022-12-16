"""In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
"""

# add PYTHONPATH
import os
import sys
from threading import Thread

sys.path.append(
    os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "kinematics")
)

from inverse_kinematics import InverseKinematicsAgent

from xmlrpc.server import SimpleXMLRPCServer
import json


class ServerAgent(InverseKinematicsAgent):
    """ServerAgent provides RPC service"""

    # YOUR CODE HERE

    # simple rpc server
    def __init__(self):
        super(ServerAgent, self).__init__()

        self.server = SimpleXMLRPCServer(("localhost", 8001))

        self.server.register_function(self.get_angle, "rpc_get_angle")
        self.server.register_function(self.set_angle, "rpc_set_angle")
        self.server.register_function(self.get_posture, "rpc_get_posture")
        self.server.register_function(self.execute_keyframes, "rpc_execute_keyframes")
        self.server.register_function(self.get_transform, "rpc_get_transform")
        self.server.register_function(self.set_transform, "rpc_set_transform")

        t = Thread(target=self.server.serve_forever)
        t.start()
        print("Server is running on port 8001")

    def get_angle(self, joint_name):
        """get sensor value of given joint"""
        # YOUR CODE HERE
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        """set target angle of joint for PID controller"""
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        """return current posture of robot"""
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        """excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        """
        # YOUR CODE HERE
        self.begin = None

        self.keyframes = keyframes

        return True

    def get_transform(self, name):
        """get transform with given name"""
        print("get_transform: ", name)
        # print("transforms: ", self.transforms)
        print("transforms[name]: ", self.transforms[name])
        return json.dumps(self.transforms[name].tolist())

    def set_transform(self, effector_name, transform):
        """solve the inverse kinematics and control joints use the results"""
        # YOUR CODE HERE
        loadedTransform = json.loads(transform)
        self.target_joints = self.inverse_kinematics(effector_name, loadedTransform)
        return True


if __name__ == "__main__":
    agent = ServerAgent()
    agent.run()
