from typing import Any
import gym
from gym import spaces
import numpy as np
import pybullet as p
from robot_utils import RegisterScenes
from robot_utils import Robot, rayTest, checkCollision
from robot_utils.log import *
from robot_utils.utils import control_miniBox
import pybullet_data


class MsnDiscrete(gym.Env):
    meta_data = {'render.modes' : ['human']}
    def __init__(self, scene_name : str = "plane_static_obstacle-A", render : bool = False, evaluate : bool = False,
                 base_thickness=0.2, base_radius=0.5, wheel_thickness=0.1, left_wheel_joint_index=3, right_wheel_joint_index=2,
                 target_velocity=10., max_force=10., multiply=2., debug_text_color=(0., 0., 0.), debug_text_size=1.2,
                 miss_color=(0., 1., 0.), hit_color=(1., 0., 0.), ray_debug_line_width=2., max_distance=100., laser_length=18.,
                 laser_num=1024, use_max_coor=False, center_pos=(0., 0., 0.), internal_length=20, internal_width=20,
                 height=4, thickness=2, fence_mass=10000., fence_color=(1., 1., 1., 1.), depart_pos=(0., -9., 0.2), depart_euler=(0., 0., np.pi / 2), 
                 target_pos=(0., 9., 0.2), collision_reward=-8, reach_target_reward=1000000, distance_reward_coe=-10,
                 target_radius=2.5, distance_change_reward_coe=1000, time_reward_coe=-100, done_distance=1., done_collision=50,
                 done_step_num=72000) -> None:

        super().__init__()
        self.all_scene = ["plane_static_obstacle-A", "plane_static_obstacle-B", "plane_static_obstacle-C", "random"]
        self.scene_name = scene_name
        self._render = render
        self._evaluate = evaluate
        
        self.base_thickness=base_thickness
        self.base_radius=base_radius
        self.wheel_thickness=wheel_thickness
        self.left_wheel_joint_index=left_wheel_joint_index
        self.right_wheel_joint_index=right_wheel_joint_index
        self.target_velocity=target_velocity
        self.max_force=max_force
        self.multiply=multiply
        self.debug_text_color=debug_text_color
        self.debug_text_size=debug_text_size
        self.miss_color=miss_color
        self.hit_color=hit_color
        self.ray_debug_line_width=ray_debug_line_width
        self.max_distance=max_distance
        self.laser_length=laser_length
        self.laser_num=laser_num
        self.use_max_coor=use_max_coor
        self.center_pos=center_pos
        self.internal_length=internal_length
        self.internal_width=internal_width
        self.height=height
        self.thickness=thickness
        self.fence_mass=fence_mass
        self.fence_color=fence_color
        self.depart_pos=depart_pos
        self.depart_euler=depart_euler 
        self.target_pos=target_pos
        self.collision_reward=collision_reward
        self.reach_target_reward=reach_target_reward
        self.distance_reward_coe=distance_reward_coe
        self.target_radius=target_radius
        self.distance_change_reward_coe=distance_change_reward_coe
        self.time_reward_coe=time_reward_coe
        self.done_distance=done_distance
        self.done_collision=done_collision
        self.done_step_num=done_step_num

        # define action space and observation space
        self.action_space = spaces.Discrete(8)
        self.observation_space = spaces.Box(
            low =np.float32([0.] * self.laser_num + [0., 0.]),
            high=np.float32([self.laser_length + 1] * self.laser_num + [self.max_distance, np.pi])
        )

        # connect to engin
        self._physics_client_id = p.connect(p.GUI if render else p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self._register_scene = RegisterScenes()
        self.seed()
        self.reset()
    
    def reset(self) -> Any:
        p.resetSimulation(physicsClientId=self._physics_client_id)
        p.setGravity(0., 0., -9.8, physicsClientId=self._physics_client_id)
        p.setRealTimeSimulation(0)        

        self.step_num = 0

        self.collision_num = 0
        self.pre_dis = self.__distance(self.depart_pos, self.target_pos)                    # previous distance between robot and target
        self.depart_target_distance = self.__distance(self.depart_pos, self.target_pos)     # distance between depart pos and target pos

        self.robot = Robot(
            basePos=self.depart_pos,
            baseOri=p.getQuaternionFromEuler(self.depart_euler),
            physicsClientId=self._physics_client_id
        )

        if self.scene_name == "random":
            self.scene_name = np.random.choice(self.all_scene)
        self.scene = self._register_scene.construct(scene_name=self.scene_name)
        state = self.robot.get_observation(targetPos=self.target_pos)

        # add debug items to the target pos
        if self._evaluate:
            self.target_line = p.addUserDebugLine(
                lineFromXYZ=[self.target_pos[0], self.target_pos[1], 0.],
                lineToXYZ=[self.target_pos[0], self.target_pos[1], 5.],
                lineColorRGB=[1., 1., 0.2]
            )
            self.rayDebugLineIds = []
            froms, tos, results = rayTest(self.robot.robot, ray_length=self.laser_length, ray_num=self.laser_num)
            for index, result in enumerate(results):
                color = self.miss_color if result[0] == -1 else self.hit_color
                self.rayDebugLineIds.append(p.addUserDebugLine(froms[index], tos[index], color, self.ray_debug_line_width))

        return np.array(state)
    
    def render(self, mode='human'):
        pass
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    
    def close(self):
        if self._physics_client_id >= 0:
            p.disconnect()
        self._physics_client_id = -1

    def sample(self):
        return self.action_space.sample()
    
    def step(self, action):
        """
            first set, second step
            then calculate the reward
            return state, reward, done, info
        """
        if not hasattr(self, "robot"):
            raise RuntimeError("reset before step!!!")

        control_miniBox(self.robot.robot, instruction=action, target_velocity=self.target_velocity,
                        multiply=self.multiply, left_wheel_joint_index=self.left_wheel_joint_index,
                        right_wheel_joint_index=self.right_wheel_joint_index, max_force=self.max_force, 
                        physicsClientId=self._physics_client_id)
        
        p.stepSimulation(physicsClientId=self._physics_client_id)    
        self.step_num += 1
        state = self.robot.get_observation(self.target_pos)
        reward = self.__reward_func(state)
        if state[-2] < self.target_radius:
            done = True
        elif self.step_num > self.done_step_num:
            done = True
        else:
            done = False
        info = {"distance" : state[-2], "collision_num" : self.collision_num}

        # under evaluate mode, extra debug items need to be rendered
        if self._evaluate:
            froms, tos, results = rayTest(self.robot.robot, ray_length=self.laser_length, ray_num=self.laser_num)
            for index, result in enumerate(results):
                self.rayDebugLineIds[index] = p.addUserDebugLine(
                    lineFromXYZ=froms[index], 
                    lineToXYZ=tos[index] if result[0] == -1 else result[3], 
                    lineColorRGB=self.miss_color if result[0] == -1 else self.hit_color, 
                    lineWidth=self.ray_debug_line_width, 
                    replaceItemUniqueId=self.rayDebugLineIds[index]
                )

        return np.array(state), reward, done, info

    def __reward_func(self, state):
        if checkCollision(self.robot.robot, debug=False):
            self.collision_num += 1
            Rc = self.collision_reward
        else:
            Rc = 0
        
        cur_dis = self.__distance(self.robot.curPos(), self.target_pos)
        Rp = self.distance_change_reward_coe * (self.pre_dis - cur_dis)
        
        # TODO : quite important
        if self.step_num % 1000 == 0:
            # msn_debug("pre_dis is updated, increment:{}".format(self.pre_dis - cur_dis))
            self.pre_dis = cur_dis
        
        if state[-2] < self.target_radius:
            Rr = self.reach_target_reward
        else:
            # Rg = self.DISTANCE_REWARD_COE * cur_dis / self.depart_target_distance
            Rr = 0.

        Rt = self.time_reward_coe

        # msn_debug("Rc={}, Rp={}, Rr={}, Rt={}".format(Rc, Rp, Rr, Rt))
        # origin : Rc + Rp + Rr + Rt
        return Rc + Rp + Rr + Rt
    
    def __distance(self, v1, v2, _type="l2"):
        v1 = np.array(v1)
        v2 = np.array(v2)
        if _type == "l2":
            return np.linalg.norm(v1 - v2)
    