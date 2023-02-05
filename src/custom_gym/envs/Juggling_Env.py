import gymnasium as gym
import mujoco
import numpy as np
import os

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
    
    def __init__(self,render_mode=None):
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

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        MujocoEnv.__init__(self, FILE_PATH,5,observation_space=self.observation_space)

        self.renderer=mujoco.Renderer(self.model)
        self.data=mujoco.MjData(self.model)
        
    def step(self,a):
        self.data.ctrl=a
        mujoco.mj_step(self.model, self.data)

    
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)

    def _render_frame(self):
        self.renderer.update_scene(self.data)
        return self.renderer.render().copy()
    
    #TODO
    def render(self):
        return

    def _get_obs(self):
        return {"agent": self.get_body_com("Cone"), "target": self.get_body_com("Ball1")}