# Adding AirSim to Custom Unity Projects
Before completing these steps, make sure you have properly [set up AirSim for Unity](Unity.md)
1. Open the Containing folder of your custom unity project
2. Copy and paste the following items from Unity demo into the main project folder of your custom environment:
```
Assets
ProjectSettings
```
[![Copy and paste video](images/unity_copy_and_paste.png)](https://youtu.be/5iplkEC88qw?start=5&end=12)

3. Open your custom environment in Unity
4. Drag your desired scene into the Scene Hierarchy panel
5. Drag CarDemo into the Scene Hierarchy panel
6. Copy the following items from CarDemo into your custom scene:
```
Main Camera
Directional Light
AirSimHUD
Car
```
7. After removing `CarDemo` from the Hierarchy panel, save your modified scene as `CarDemo`.
[![change scene](images/unity_change_scene.png)](https://youtu.be/5iplkEC88qw?start=45&end=78)
8. Repeat Steps 6 and 7 with `DroneDemo`. This time, save your custom scene as `DroneDemo`.

Your custom environment is now ready to interface with AirSim!