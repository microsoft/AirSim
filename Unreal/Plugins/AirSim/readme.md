**Creating new environments**

- Create black C++ Unreal project without starter content
- Add environment to project
- Project Settings > Maps - set the map
- Copy Plugins folder
- Add following in to uproject:

```
	"Plugins": [
		{
			"Name": "AirSim",
			"Enabled": true
		}
	]
```

- Regerate the project files
- Restart UE editor
- Enable "Custom Depth-Stencil Pass" setting (in Project Settings > Rendering > Postprocessing) 
- Set game mode to SimGameMode
