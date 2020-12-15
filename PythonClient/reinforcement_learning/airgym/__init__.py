from gym.envs.registration import register

register(
    id="airsim-drone-trackpowerlines-v0", entry_point="airgym.envs:AirSimDroneEnv",
)

register(
    id="airsim-car-drive-v0", entry_point="airgym.envs:AirSimCarEnv",
)
