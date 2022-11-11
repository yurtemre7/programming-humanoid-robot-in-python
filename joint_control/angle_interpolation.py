"""In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
"""


from pid import PIDAgent
from keyframes import *

# formula of bezier interpolation
def bezier(p_0, p_1, p_2, p_3, t):
    return (
        (1 - t) ** 3 * p_0
        + 3 * (1 - t) ** 2 * t * p_1
        + 3 * (1 - t) * t**2 * p_2
        + t**3 * p_3
    )


class AngleInterpolationAgent(PIDAgent):
    def __init__(
        self,
        simspark_ip="localhost",
        simspark_port=3100,
        teamname="DAInamite",
        player_id=0,
        sync_mode=True,
    ):
        super(AngleInterpolationAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode
        )
        self.keyframes = ([], [], [])
        self.begin = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints["RHipYawPitch"] = target_joints.get(
            "LHipYawPitch", 0
        )  # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        # YOUR CODE HERE
        names, times, keys = keyframes

        if self.begin is None:
            self.begin = perception.time

        dt = perception.time - self.begin

        for i in range(len(names)):
            joint_name = names[i]

            if joint_name in self.joint_names:
                time = times[i]
                key = keys[i]

                timepoints = len(time)

                # go through all timepoints
                for tp in range(timepoints - 1):
                    if time[tp] < dt < time[tp + 1]:
                        p_0 = key[tp][0]
                        p_1 = p_0 + (key[tp][2][1] * key[tp][2][2])
                        p_3 = key[tp + 1][0]
                        p_2 = p_3 + (key[tp + 1][1][1] * key[tp + 1][1][2])
                        tdt = (dt - time[tp]) / (time[tp + 1] - time[tp])
                        target_joints[joint_name] = bezier(p_0, p_1, p_2, p_3, tdt)
                    # for the first keyframe
                    elif dt < time[0]:
                        p_0 = perception.joint[joint_name]
                        p_3 = key[0][0]
                        p_2 = p_3 + (key[tp][2][1] * key[tp][2][2])
                        tdt = dt / time[1]
                        target_joints[joint_name] = bezier(p_0, p_0, p_2, p_3, tdt)

        return target_joints


if __name__ == "__main__":
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
