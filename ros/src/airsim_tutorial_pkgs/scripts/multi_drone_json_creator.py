# import airsim
import json
import numpy as np
import pdb

# todo expose airsimsettings.hpp via pybind11? this should be done for the full API *some*day
class Position():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Rotation():
    def __init__(self, yaw, pitch, roll):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

class Pose():
    def __init__(self, position, rotation):
        self.position = position
        self.rotation = rotation

class JSONSettingsCreator(object):
    def __init__(self, sim_mode="Multirotor"):
        # self.args = args
        self.sim_mode = sim_mode
        self.data = {}

    def add_minimal(self):
        self.data["SeeDocsAt"] = "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md"
        self.data["SettingsVersion"] = 1.2
        self.data["SimMode"] = self.sim_mode #"Multirotor"
        self.data["ClockSpeed"] = 1

    def set_pose(self, setting_key, pose):
        setting_key["X"] = pose.position.x
        setting_key["Y"] = pose.position.y
        setting_key["Z"] = pose.position.z
        setting_key["Pitch"] = pose.rotation.pitch
        setting_key["Roll"] = pose.rotation.roll
        setting_key["Yaw"] = pose.rotation.yaw

    def add_multirotor(self, vehicle_name, pose):
        assert(self.data["SimMode"] == "Multirotor")
        if "Vehicles" not in self.data.keys():
            self.data['Vehicles'] = {}

        self.data['Vehicles'][vehicle_name] = {}
        self.data['Vehicles'][vehicle_name]["VehicleType"] = "SimpleFlight"
        self.set_pose(self.data['Vehicles'][vehicle_name], pose)

def main():
    json_setting_helper = JSONSettingsCreator(sim_mode="Multirotor")
    json_setting_helper.add_minimal()

    # arrange drones in a rectangle. todo make classes for different swarm spawn shapes? 
    dist_between_drones = 2.0 
    num_drones_x = 5
    num_drones_y = 5
    grid_width = (num_drones_x-1) * dist_between_drones
    grid_length = (num_drones_y-1) * dist_between_drones

    x_vals = np.linspace(0, grid_width, num_drones_x)
    y_vals = np.linspace(0, grid_length, num_drones_y)
    x_grid, y_grid = np.meshgrid(x_vals, y_vals)

    # pdb.set_trace()

    for row_idx in range(x_grid.shape[0]):
        for col_idx in range(x_grid.shape[1]):
            json_setting_helper.add_multirotor("Drone"+str((row_idx * num_drones_x) + col_idx), 
                Pose(Position(x=y_grid[row_idx, col_idx], y=x_grid[row_idx, col_idx], z=0), Rotation(yaw=0, pitch=0, roll=0)))
    with open("bla.json", "w") as f:
        json.dump(json_setting_helper.data, f, indent=2, sort_keys=True)

if __name__ == "__main__":
    main()