import gymnasium as gym
import mujoco
import numpy as np
import os
import mediapy as media

from gymnasium.envs.mujoco import MujocoEnv
from gymnasium import utils
from gymnasium import spaces


class Juggling_Env(MujocoEnv,utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 100,
    }
    
    def __init__(self,render_mode="human"):
        utils.EzPickle.__init__(self)
        FILE_PATH = os.getcwd() + '/robot/scene.xml'
        
        #agent observation space is the position of the cup
        #target observation will be the concatenated positions of the ball
        
        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Box(low=0, high=2, shape=(3,), dtype=np.float64),
                "target": spaces.Box(low=0, high=2, shape=(3,), dtype=np.float64)
            })
        
        # action space is the control values of our actuators
        self.action_space = spaces.Box(low=np.array([-2.9,-1.76,-3.07]), high=np.array([2.9,1.76,3.07]), shape=(3,), dtype=np.float64)

        MujocoEnv.__init__(self, FILE_PATH,5,observation_space=self.observation_space)
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self._initialize_simulation()
        
        
    def step(self,ctrl):
        #steo function has to return observation, reward, terminated, truncated, info
        self._step_mujoco_simulation(ctrl, 300)
        #TODO implement reward, termination, and truncation
        reward=1
        terminated=False
        truncated=False
        info={}
        return self._get_obs(), reward, terminated, truncated, info
    
    def reset(self,seed=None,options=""):
        self._reset_simulation()
        return self._get_obs(),{}
    
    def render(self):
        renderer = mujoco.Renderer(self.model)
        mujoco.mj_forward(self.model, self.data)
        renderer.update_scene(self.data)
        image=renderer.render()
        media.show_image(image)
        return image

    def _get_obs(self):
        return {"agent": self.get_body_com("Cone"), "target": self.get_body_com("Ball1")}