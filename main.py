import pybullet
import pybullet_data
import numpy as np
import time
import os
import math
from utils.simenv import SimEnv, Manipulator


class quat:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return str(self.w) + " " + str(self.x) + " " + str(self.y) + " " + str(self.z)

    def __eq__(self, other):
        err_1 = abs(self.w - other.w) + abs(self.x - other.x) + abs(self.y - other.y) + abs(self.z - other.z)
        err_2 = abs(self.w + other.w) + abs(self.x + other.x) + abs(self.y + other.y) + abs(self.z + other.z)
        return err_1 < 0.01 or err_2 < 0.01


class Floor:
    def __init__(self, basePosition=[0, 0, 0], baseRPY=[0, 0, 0]):
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.id = pybullet.loadURDF('plane.urdf', basePosition=basePosition,
                                    baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))

    def changeFriction(self, lateralFriction=1.0, spinningFriction=1.0):
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, lateralFriction=lateralFriction,
                                spinningFriction=spinningFriction)
        print("Floor friction updated!")
        print("lateralFriction:", pybullet.getDynamicsInfo(self.id, -1)[1])
        print("spinningFriction:", pybullet.getDynamicsInfo(self.id, -1)[7])


class Cylinder:
    def __init__(self, basePosition=[0, 0, 1], baseRPY=[0, 0, 0], radius=0.1, length=1, mass=1,
                 rgbaColor=[0.8, 0.8, 0.8, 1], useFixedBase=False):
        mass = 0 if useFixedBase else mass
        collisionShapeIndex = pybullet.createCollisionShape(shapeType=pybullet.GEOM_CYLINDER, radius=radius, height=length)
        visualShapeIndex = pybullet.createVisualShape(shapeType=pybullet.GEOM_CYLINDER, radius=radius, length=length,
                                                      rgbaColor=rgbaColor)
        self.objectUniqueId = pybullet.createMultiBody(baseMass=mass,
                                                       baseCollisionShapeIndex=collisionShapeIndex,
                                                       baseVisualShapeIndex=visualShapeIndex,
                                                       basePosition=basePosition,
                                                       baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))

    def resetBasePosition(self, position, quaternion=[0, 0, 0, 1]):
        pybullet.resetBasePositionAndOrientation(self.objectUniqueId, position, quaternion)

    def remove(self):
        pybullet.removeBody(self.objectUniqueId)


class Box:
    def __init__(self):
        print('robot id:', self.id)
        self.changeFriction(lateralFriction=1.0, spinningFriction=1.0)
        self.enableFrictionAnchor()

    @classmethod
    def fromParam(cls, basePosition=[0, 0, 1], baseRPY=[0, 0, 0], size=[1, 1, 1], mass=1, rgbaColor=[0.8, 0.8, 0.8, 1],
                  useFixedBase=False):
        mass = 0 if useFixedBase else mass
        collisionShapeIndex = pybullet.createCollisionShape(shapeType=pybullet.GEOM_BOX, halfExtents=np.array(size) / 2)
        visualShapeIndex = pybullet.createVisualShape(shapeType=pybullet.GEOM_BOX, halfExtents=np.array(size) / 2,
                                                      rgbaColor=rgbaColor, specularColor=[0.0, 0.0, 0.0])
        cls.id = pybullet.createMultiBody(baseMass=mass,
                                          baseCollisionShapeIndex=collisionShapeIndex,
                                          baseVisualShapeIndex=visualShapeIndex,
                                          basePosition=basePosition,
                                          baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))
        return cls()

    @classmethod
    def fromID(cls, id):
        cls.id = id
        return cls()

    def getBasePosition(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[0]

    def getBaseOrientation(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[1]

    def resetBasePosition(self, position, quaternion=[0, 0, 0, 1]):
        pybullet.resetBasePositionAndOrientation(self.id, position, quaternion)

    def remove(self):
        pybullet.removeBody(self.id)

    def changeFriction(self, lateralFriction=1.0, spinningFriction=1.0):
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, lateralFriction=lateralFriction,
                                spinningFriction=spinningFriction)
        print("Box friction updated!")
        print("lateralFriction:", pybullet.getDynamicsInfo(self.id, -1)[1])
        print("spinningFriction:", pybullet.getDynamicsInfo(self.id, -1)[7])

    def enableFrictionAnchor(self):
        print("Box friction anchor enabled!")
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, frictionAnchor=True)


def goto_place(robot):
    # cur_pos = robot.get_current_arm_joint_position()
    # print(cur_pos)
    origin_pos = [0.1345, 0, 0.52]
    quat = [1, 0, 0, 0]

    robot.gotoCartesianTarget2(position=[0.1345, 0, 0.52], quaternion=[1, 0, 0, 0], sim_env=sim_env)


def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix."""
    q = np.empty((4, ), dtype=np.float64)
    M = np.identity(4, dtype=np.float64)
    for i in range(3):
        for j in range(3):
            M[i, j] = matrix[i][j]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return quat(q[3], q[0], q[1], q[2])

def sim(file, num):
    sim_env = SimEnv(sim_rate=100, real_time_sim=False, GUI=False)
    sim_env.resetGUIView(distance=0.8, yaw=90, pitch=-60, target_position=[0, 0, 0.0])
    sim_env.resetLightPosition(lightPosition=[1, 0, 1])
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SINGLE_STEP_RENDERING)

    # panda arm
    panda_urdf_path = os.path.join('..', 'robots/franka/urdf/panda_arm_hand_realsense.urdf')
    panda_config_path = os.path.join('robots/franka/config/panda_arm_hand.yaml')
    robot = Manipulator.loadFromURDF(urdf_path=panda_urdf_path, config_path=panda_config_path)
    # robot = pybullet.loadURDF(fileName=panda_urdf_path)
    print('robot id:{}, type:{}'.format(robot.id, type(robot.id)))

    # table, objects will be placed on top of it
    table = Box.fromParam(basePosition=[0, 0, 0.02],
                          baseRPY=[0, 0, 0],
                          size=[0.6, 1.2, 0.04],
                          useFixedBase=True)

    # cylinder to place the robot
    cylinder = Cylinder(basePosition=[-0.42, 0, 0.02], baseRPY=[0, 0, 0],
                        radius=0.1, length=0.04, rgbaColor=[0.3, 0.3, 0.3, 1],
                        useFixedBase=True)

    sim_env.reset_gravity(0)

    start_pos = [0.13, -0.02, 0.6]
    start_orie = pybullet.getQuaternionFromEuler([0, 0, 0])
    start_pos[2] -= 0.1

    object_name = file
    urdf = 'ocrtoc_materials/model_urdfs/' + object_name + '.urdf'
    pose_file = 'text_960/' + object_name + '.txt'

    pos = []
    axis_x = []
    axis_y = []
    axis_z = []

    with open(pose_file, 'r') as f:
        lines = f.readlines()
        pos = lines[num * 6 + 1].split(', ')
        axis_x = lines[num * 6 + 2].split(', ')
        axis_y = lines[num * 6 + 3].split(', ')
        axis_z = lines[num * 6 + 4].split(', ')

    for j, each in enumerate(pos):
        pos[j] = float(each)
    for j, each in enumerate(axis_x):
        axis_x[j] = -float(each)
    for j, each in enumerate(axis_y):
        axis_y[j] = -float(each)
    for j, each in enumerate(axis_z):
        axis_z[j] = float(each)

    x = np.array(axis_x)
    y = np.array(axis_y)
    z = np.array(axis_z)
    m = np.vstack([x, y, z])
    q = quaternion_from_matrix(m)
    print(q)
    for i in range(3):
        start_pos[i] += pos[i]
    print(start_pos)

    object = pybullet.loadURDF(urdf, start_pos, [q.x, q.y, q.z, q.w])
    # object = pybullet.loadURDF(urdf, start_pos, [0, 0, 0, 1])

    # robot.gotoArmJointConfig([0, 0, 0, 0, 0, 0, 0], T=1, sim_env=sim_env)

    # robot.goHome(2, sim_env=sim_env)
    # while 1:
    #     sim_env.step()
    print('goto_place')
    # robot.gotoArmJointConfig([0, 0, 0.4, -1.5707963267948966, 0, 1.5707963267948966, 0.7853981633974483], T=1, sim_env=sim_env)
    # robot.gotoCartesianTarget(position=[0, 0, 0], rpy=[0, np.pi, np.pi], sim_env=sim_env)
    # pos[2] = 0.47 <-> panda_joint7[2] = 0.72
    # robot.gotoCartesianTarget2(position=[0.1345, 0, 0.52], quaternion=[1, 0, 0, 0], sim_env=sim_env)
    quat = pybullet.getQuaternionFromEuler([0, np.pi * 2 / 2, 3.14])
    robot.gripperControl(0.08, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=robot.getToolPosition(), quaternion=quat, sim_env=sim_env)
    robot.goHome(1, sim_env=sim_env)
    print(robot.getToolPosition())
    # print(robot.getLinkState('panda_leftfinger'))
    # print(robot.getLinkState('panda_rightfinger'))
    robot.gripperControl(0, sim_env=sim_env)
    sim_env.reset_gravity(9.81)

    robot.gotoCartesianTarget_update(position=[0, 0, 0.3], quaternion=quat, sim_env=sim_env)

    # robot.gripperControl(0.08, sim_env=sim_env)
    # print(robot.getGripperOpeningLength())
    # sim_env.reset_gravity()
    robot.gotoCartesianTarget_update(position=[0, 0, 0.6], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.3], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.6], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0.3, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[-0.2, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0.3, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[-0.2, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, -0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, -0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.goHome(1, sim_env=sim_env)

    end_pos = np.array(pybullet.getBasePositionAndOrientation(object)[0])
    tool_pos = np.array(robot.getToolPosition())
    print(np.linalg.norm(end_pos - tool_pos))
    if np.linalg.norm(end_pos - tool_pos) < 0.01:
        print('True')
        return True
    return False


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=str)
    parser.add_argument('-n', '--num', type=int)
    args = parser.parse_args()

    sim_env = SimEnv(sim_rate=1000, GUI=False)
    sim_env.resetGUIView(distance=0.8, yaw=90, pitch=-60, target_position=[0, 0, 0.0])
    sim_env.resetLightPosition(lightPosition=[1, 0, 1])
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SINGLE_STEP_RENDERING)

    # panda arm
    panda_urdf_path = os.path.join('..', 'robots/franka/urdf/panda_arm_hand_realsense.urdf')
    panda_config_path = os.path.join('robots/franka/config/panda_arm_hand.yaml')
    robot = Manipulator.loadFromURDF(urdf_path=panda_urdf_path, config_path=panda_config_path)
    # robot = pybullet.loadURDF(fileName=panda_urdf_path)
    print('robot id:{}, type:{}'.format(robot.id, type(robot.id)))

    # table, objects will be placed on top of it
    table = Box.fromParam(basePosition=[0, 0, 0.02],
                          baseRPY=[0, 0, 0],
                          size=[0.6, 1.2, 0.04],
                          useFixedBase=True)

    # cylinder to place the robot
    cylinder = Cylinder(basePosition=[-0.42, 0, 0.02], baseRPY=[0, 0, 0],
                        radius=0.1, length=0.04, rgbaColor=[0.3, 0.3, 0.3, 1],
                        useFixedBase=True)

    sim_env.reset_gravity(0)

    start_pos = [0.13, -0.02, 0.6]
    start_orie = pybullet.getQuaternionFromEuler([0, 0, 0])
    start_pos[2] -= 0.1

    object_name = args.file
    urdf = 'ocrtoc_materials/model_urdfs/' + object_name + '.urdf'
    pose_file = 'text_960/' + object_name + '.txt'

    pos = []
    axis_x = []
    axis_y = []
    axis_z = []

    with open(pose_file, 'r') as f:
        lines = f.readlines()
        num = args.num
        pos = lines[num * 6 + 1].split(', ')
        axis_x = lines[num * 6 + 2].split(', ')
        axis_y = lines[num * 6 + 3].split(', ')
        axis_z = lines[num * 6 + 4].split(', ')

    for j, each in enumerate(pos):
        pos[j] = float(each)
    for j, each in enumerate(axis_x):
        axis_x[j] = -float(each)
    for j, each in enumerate(axis_y):
        axis_y[j] = -float(each)
    for j, each in enumerate(axis_z):
        axis_z[j] = float(each)

    x = np.array(axis_x)
    y = np.array(axis_y)
    z = np.array(axis_z)
    m = np.vstack([x, y, z])
    q = quaternion_from_matrix(m)
    print(q)
    for i in range(3):
        start_pos[i] += pos[i]
    print(start_pos)

    object = pybullet.loadURDF(urdf, start_pos, [q.x, q.y, q.z, q.w])
    # object = pybullet.loadURDF(urdf, start_pos, [0, 0, 0, 1])


    # robot.gotoArmJointConfig([0, 0, 0, 0, 0, 0, 0], T=1, sim_env=sim_env)

    # robot.goHome(2, sim_env=sim_env)
    # while 1:
    #     sim_env.step()
    print('goto_place')
    # robot.gotoArmJointConfig([0, 0, 0.4, -1.5707963267948966, 0, 1.5707963267948966, 0.7853981633974483], T=1, sim_env=sim_env)
    # robot.gotoCartesianTarget(position=[0, 0, 0], rpy=[0, np.pi, np.pi], sim_env=sim_env)
    # pos[2] = 0.47 <-> panda_joint7[2] = 0.72
    # robot.gotoCartesianTarget2(position=[0.1345, 0, 0.52], quaternion=[1, 0, 0, 0], sim_env=sim_env)
    quat = pybullet.getQuaternionFromEuler([0, np.pi * 2 / 2, 3.14])
    robot.gripperControl(0.08, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=robot.getToolPosition(), quaternion=quat, sim_env=sim_env)
    robot.goHome(2, sim_env=sim_env)
    print(robot.getToolPosition())
    # print(robot.getLinkState('panda_leftfinger'))
    # print(robot.getLinkState('panda_rightfinger'))
    robot.gripperControl(0, sim_env=sim_env)
    sim_env.reset_gravity(9.81)

    robot.gotoCartesianTarget_update(position=[0, 0, 0.3], quaternion=quat, sim_env=sim_env)

    # robot.gripperControl(0.08, sim_env=sim_env)
    # print(robot.getGripperOpeningLength())
    # sim_env.reset_gravity()
    robot.gotoCartesianTarget_update(position=[0, 0, 0.6], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.3], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.6], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0.3, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[-0.2, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0.3, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[-0.2, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, -0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, 0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.gotoCartesianTarget_update(position=[0, -0.4, 0.4], quaternion=quat, sim_env=sim_env)
    robot.goHome(2, sim_env=sim_env)

    end_pos = np.array(pybullet.getBasePositionAndOrientation(object)[0])
    tool_pos = np.array(robot.getToolPosition())
    print(np.linalg.norm(end_pos - tool_pos))
    if np.linalg.norm(end_pos - tool_pos) < 0.01:
        print('True')
    # robot.getToolPosition()

    # print(end_pos)
    # goto_place(robot)
    while 1:
        # sim_env.step()
        time.sleep(1)

