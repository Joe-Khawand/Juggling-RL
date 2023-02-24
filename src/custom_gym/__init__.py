from gymnasium.envs.registration import register
#Register the environment
register(
    id='Juggler',
    entry_point='custom_gym.envs:Juggling_Env',
)