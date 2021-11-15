# Using C++ APIs for AirSim

아직 준비가 되어있지 않은 분은 먼저  [API 문서](https://microsoft.github.io/AirSim/apis/)를 읽어보시기 바랍니다. 해당 문서에서는 C++의 예제 및 그 외의 C++의 세부정보를 설명합니다

## Quick Start

가장 간단한 방법으로 Visual Studio 2019에서 AirSim.sln을 여는 것 입니다. 솔루션에서 [Hello Car](https://github.com/Microsoft/AirSim/tree/master/HelloCar/) 와 [Hello Drone](https://github.com/Microsoft/AirSim/tree/master/HelloDrone/) 의 예제를 볼 수 있습니다. 해당 예제들로 VC++ 프로젝트에서 설정하는데 필요한 include 경로 및 lib 경로를 볼 수 있습니다. Linux를 사용하는 경우 [cmake file](https://github.com/Microsoft/AirSim/tree/master/cmake//HelloCar/CMakeLists.txt) 또는 컴파일러 명령줄에서 경로를 지정할 수 있습니다.

#### Include 와 Lib 폴더

- Include folders: `$(ProjectDir)..\AirLib\deps\rpclib\include;include;$(ProjectDir)..\AirLib\deps\eigen3;$(ProjectDir)..\AirLib\include`
- Dependencies: `rpc.lib`
- Lib folders: `$(ProjectDir)\..\AirLib\deps\MavLinkCom\lib\$(Platform)\$(Configuration);$(ProjectDir)\..\AirLib\deps\rpclib\lib\$(Platform)\$(Configuration);$(ProjectDir)\..\AirLib\lib\$(Platform)\$(Configuration)`
- References: 프로젝트 참조에 AirLib 및 MavLinkCom 을 참조합니다(프로젝트를 마우스 오른쪽 클릭을 한 뒤  `References`, `Add reference...`,  로 이동한 다음 AirLib 및 MavLinkCom을 선택합니다)

## Hello Car

C++을 사용하여 시뮬레이션된 자동차를 제어하는 AirSim API를 사용하는 방법은 다음과 같습니다. ( [파이썬예제 참조](https://microsoft.github.io/AirSim/apis/#hello_car))

```cpp
// 실행준비완료 예시: https://github.com/Microsoft/AirSim/blob/master/HelloCar/main.cpp

#include <iostream>
#include "vehicles/car/api/CarRpcLibClient.hpp"

int main() 
{
    msr::airlib::CarRpcLibClient client;
    client.enableApiControl(true); //수동제어 비활성화
    CarControllerBase::CarControls controls;

    std::cout << "Press enter to drive forward" << std::endl; std::cin.get();
    controls.throttle = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to activate handbrake" << std::endl; std::cin.get();
    controls.handbrake = true;
    client.setCarControls(controls);

    std::cout << "Press Enter to take turn and drive backward" << std::endl; std::cin.get();
    controls.handbrake = false;
    controls.throttle = -1;
    controls.steering = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to stop" << std::endl; std::cin.get();
    client.setCarControls(CarControllerBase::CarControls());

    return 0;
}
```

## Hello Drone

C++을 사용하여 시뮬레이션된 쿼드로터를 제어하는 AirSim API를 사용하는 방법은 다음과 같습니다 ( [파이썬예제참조](https://microsoft.github.io/AirSim/apis/#hello_drone))

```cpp
// 실행준비완료 예시: https://github.com/Microsoft/AirSim/blob/master/HelloDrone/main.cpp

#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int main() 
{
    using namespace std;
    msr::airlib::MultirotorRpcLibClient client;

    cout << "Press Enter to enable API control" << endl; cin.get();
    client.enableApiControl(true);

    cout << "Press Enter to arm the drone" << endl; cin.get();
    client.armDisarm(true);

    cout << "Press Enter to takeoff" << endl; cin.get();
    client.takeoffAsync(5)->waitOnLastTask();

    cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity" << endl; cin.get();  
    auto position = client.getMultirotorState().getPosition(); // 현재 위치에서location
    client.moveToPositionAsync(position.x() + 5, position.y(), position.z(), 1)->waitOnLastTask();

    cout << "Press Enter to land" << endl; cin.get();
    client.landAsync()->waitOnLastTask();

    return 0;
}
```

## See Also

- [예시](https://github.com/microsoft/AirSim/tree/master/Examples) AirSim의 내부 인프라를 다른 프로젝트에서 사용하는 방법에 대한 설명
- [드론셸](https://github.com/microsoft/AirSim/tree/master/DroneShell) C++ API를 사용하여 드론을 제어하는 간단한 인터페이스를 만드는 방법을 보여줍니다.
- [HelloSpawnedDrones](https://github.com/microsoft/AirSim/tree/master/HelloSpawnedDrones) 추가 차량을 신속하게 만드는 방법을 보여줍니다
- [Python APIs](https://microsoft.github.io/AirSim/apis/)

