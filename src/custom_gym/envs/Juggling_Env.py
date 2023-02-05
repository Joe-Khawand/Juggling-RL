import gymnasium as gym
import mujoco
import numpy as np
import os

from gymnasium.envs.mujoco import MuJocoPyEnv
from gymnasium import utils
from gymnasium import spaces


class JugglingEnv(MuJocoPyEnv,utils.EzPickle):
    
    def __init__(self):
        
        utils.EzPickle.__init__(self)
        FILE_PATH = os.getcwd() + '/robot/scene.xml'

        mujoco_env.MujocoEnv.__init__(self, FILE_PATH, 5)
        # Observations
        
        #agent observation space is the position of the cup
        #target observation will be the concatenated positions of the two balls
        
        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Box(low=, high=, shape=(3,), dtype=np.float32),
                "target": spaces.Box(0, size - 1, shape=(6,), dtype=np.float32)
            })
        
        # action space is the control values of our actuators
        self.action_space = spaces.Box(low=np.array([-2.9,-1.76,-3.07]), high=np.array([2.9,1.76,3.07]), shape=(3,), dtype=np.float32)
        
    def step(self,a):
        return
    
    def reset(self):
        return

    def render(self):
        return

    def _get_obs(self):
        return {"agent": self._agent_location, "target": self._target_location}
    
    #TODO write reset() step() close() render() _render_frame()
    # https://gymnasium.farama.org/tutorials/gymnasium_basics/environment_creation/#sphx-glr-tutorials-gymnasium-basics-environment-creation-py
    