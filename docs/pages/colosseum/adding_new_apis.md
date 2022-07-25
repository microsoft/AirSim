# Adding New APIs to AirSim

Adding new APIs requires modifying the source code. Much of the changes are mechanical and required for various levels of abstractions that AirSim supports. The main files required to be modified are described below along with some commits and PRs for demonstration. Specific sections of the PRs or commits might be linked in some places, but it'll be helpful to have a look at the entire diff to get a better sense of the workflow. Also, don't hesitate in opening an issue or a draft PR also if unsure about how to go about making changes or to get feedback.

## Implementing the API

Before adding the wrapper code to call and handle the API, it needs to be implemented first. The exact files where this will occur varies depending on what it does. Few examples are given below which might help you in getting started.

### Vehicle-based APIs

`moveByVelocityBodyFrameAsync` API for velocity-based movement in the multirotor's X-Y frame.

The main implementation is done in [MultirotorBaseApi.cpp](https://github.com/microsoft/AirSim/pull/3169/files#diff-29ac01a05077b6e8e1f09221a113f779c952a80dc8823725eb451a9fc5d7de5f), where most of the multirotor APIs are implemented.

In some cases, additional structures might be needed for storing data, [`getRotorStates` API](https://github.com/microsoft/AirSim/pull/3242) is a good example for this, here the `RotorStates` struct is defined in 2 places for conversion from RPC to internal code. It also requires modifications in AirLib as well as Unreal/Plugins for the implementation.

### Environment-related APIs

These APIs need to interact with the simulation environment itself, hence it's likely that it'll be implemented inside the `Unreal/Plugins` folder.

- `simCreateVoxelGrid` API to generate and save a binvox-formatted grid of the environment - [WorldSimApi.cpp](https://github.com/microsoft/AirSim/pull/3209/files#diff-89d4ec9b62486b1322e5ba2dd9936b13962f9ed113ec5e35a0678846889c7e2d)

- `simAddVehicle` API to create vehicles at runtime - [SimMode*, WorldSimApi files](https://github.com/microsoft/AirSim/pull/2390/files#diff-fcc0aa1fbc74a924fccd12589295aceeea59074c94256eccba7df3ce85d3a26c)

### Physics-related APIs

`simSetWind` API shows an example of modifying the physics behaviour and adding an API + settings field for the same. See [the PR](https://github.com/microsoft/AirSim/pull/2867) for details about the code.

## RPC Wrappers

The APIs use [msgpack-rpc protocol](https://github.com/msgpack-rpc/msgpack-rpc) over TCP/IP through [rpclib](http://rpclib.net/) developed by [TamÃ¡s Szelei](https://github.com/sztomi) which allows you to use variety of programming languages including C++, C#, Python, Java etc. When AirSim starts, it opens port 41451 (this can be changed via [settings](settings.md)) and listens for incoming request. The Python or C++ client code connects to this port and sends RPC calls using [msgpack serialization format](https://msgpack.org).

To add the RPC code to call the new API, follow the steps below. Follow the implementation of other APIs defined in the files.

1. Add an RPC handler in the server which calls your implemented method in [RpcLibServerBase.cpp](https://github.com/microsoft/AirSim/blob/main/AirLib/src/api/RpcLibServerBase.cpp). Vehicle-specific APIs are in their respective vehicle subfolder.

2. Add the C++ client API method in [RpcClientBase.cpp](https://github.com/microsoft/AirSim/blob/main/AirLib/src/api/RpcLibClientBase.cpp)

3. Add the Python client API method in [client.py](https://github.com/microsoft/AirSim/blob/main/PythonClient/airsim/client.py). If needed, add or modify a structure definition in [types.py](https://github.com/microsoft/AirSim/blob/main/PythonClient/airsim/types.py)

## Testing

Testing is required to ensure that the API is working as expected. For this, as expected, you'll have to use the source-built AirSim and Blocks environment. Apart from this, if using the Python APIs, you'll have to use the `airsim` package from source rather than the PyPI package. Below are 2 ways described to go about using the package from source -

1. Use [setup_path.py](https://github.com/microsoft/AirSim/blob/main/PythonClient/multirotor/setup_path.py). It will setup the path such that the local airsim module is used instead of the pip installed package. This is the method used in many of the scripts since the user doesn't need to do anything other than run the script.
    Place your example script in one of the folders inside `PythonClient` like `multirotor`, `car`, etc. You can also create one to keep things separate, and copy the `setup_path.py` file from another folder.
    Add `import setup_path` before `import airsim` in your files. Now the latest main API (or any branch currently checked out) will be used.

2. Use a [local project pip install](https://pip.pypa.io/en/stable/cli/pip_install/#local-project-installs). Regular install would create a copy of the current source and use it, whereas Editable install (`pip install -e .` from inside the `PythonClient` folder) would change the package whenever the Python API files are changed. Editable install has the benefit when working on several branches or API is not finalized.

It is recommended to use a virtual environment for dealing with Python packaging so as to not break any existing setup.

When opening a PR, make sure to follow the [coding guidelines](coding_guidelines.md). Also add a docstring for the API in the Python files, and please include any example scripts and settings required in the script as well.
