# Packaging a binary including the AirSim plugin 

In order to package a custom environment with the AirSim plugin, there are a few project settings that are necessary for ensuring all required assets needed for AirSim are included inside the package. Under `Edit -> Project Settings... -> Project -> Packaging`, please ensure the following settings are configured properly:

- `List of maps to include in a packaged build`: ensure one entry exists for `/AirSim/AirSimAssets` 
- `Additional Asset Directories to Cook`: ensure one entry exists for `/AirSim/HUDAssets`
