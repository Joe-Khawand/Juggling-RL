from gymnasium.envs.registration import register

register(
    id='Juggler',
    entry_point='custom_gym.envs:Juggling_Env',
    max_episode_steps=2000,
)