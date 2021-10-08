from env import MsnDiscrete, MaplessNaviEnv
from robot_utils import *
from robot_utils.log import msn_debug
from robot_utils.scene import *
from env import *
from collections import Counter

MAX_FORCE = 10.
TARGET_VELOCITY = 5.
MULTIPLY = 2.0

def keyboard_control():
    global MAX_FORCE
    global TARGET_VELOCITY
    global MULTIPLY

    cid = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 载入机器人和其他的物件
    _ = p.loadURDF("plane.urdf")
    urdf_path = os.path.join(os.path.dirname(__file__), "robot_utils/urdf/miniBox.urdf")
    robot_id = p.loadURDF(urdf_path, basePosition=[0., 0., 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi / 2.]))

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=[0, 1],
        controlMode=p.VELOCITY_CONTROL,
        forces=[0., 0.]
    )

    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while True:
        basePos, baseOrientation = p.getBasePositionAndOrientation(robot_id)
        baseEuler = p.getEulerFromQuaternion(baseOrientation)
        keyboard_control_miniBox(robot_id)

def u_MsnDiscrete():
    env = MsnDiscrete(render=True, laser_num=18)
    state = env.reset()
    done = False

    while not done:
        action = env.sample()
        state, reward, done, info = env.step(action)
        env.render()

# keyboard_control()
u_MsnDiscrete()