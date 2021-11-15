# AirSim APIs 



### Introduction 

AirSim은 API를 공개했기 때문에 시뮬레이션 프로그래밍 방식으로 차량과 상호 작용할 수 있습니다. 이러한 API를 사용하여 이미지를 검색하고, 상태를 가져오고, 차량을 제어하는 등의 작업을 수행할 수 있습니다. 



### Python Quick Start

파이썬을 사용하여 AirSim API를 호출하려면 아나콘다를 파이썬 3.5 이상 버전과 함께 사용하는 것이 좋습니다. 그러나 일부 코드는 파이썬 2.7과 함께 사용할 수도 있습니다.

다음 명령어를 통해 패키지를 설치합니다.

```
pip install msgpack-rpc-python
```



Release에서 AirSim 바이너리를 가져오거나 Windows/Linux에서 컴파일할 수 있습니다. AirSim을 실행할 수 있으면 차량을 behicle로 선택하고 `PythonClient\car\` 폴더로 이동하여 다음을 실행합니다. 

```
python hello_car.py
```



### Installing AirSim Package

다음 명령어를 통해서도 AirSim 패키지를 설치할 수 있습니다.

```
pip install airsim
```

이 패키지에 대한 소스 코드와 샘플은 repo의 `PythonClient` 폴더에서 찾을 수 있습니다.

[참고] 

1. 이 예제 폴더에서 `setup_path.py` 파일을 확인할 수 있습니다. 이 파일에는 AirSim 패키지가 상위 폴더에 있는지 탐지할 수 있는 간단한 코드가 있습니다. 이 경우 우리는 pip 설치 패키지 대신 그것을 사용하므로 당신은 항상 최신 코드를 사용합니다. 

2. AirSim은 여전히 개발 중에 있습니다. 즉, 새로운 API를 사용하기 위해 패키지를 자주 업데이트해야 할 수도 있습니다.



### Hello Car

다음 코드는 Python으로 AirSim API를 사용해서 시뮬레이션된 자동차를 제어하는 방법이다.

```python
# hello_car를 실행하기 위해 필요한 모듈 import
import airsim
import time

# AirSim 시뮬레이터 연결하기
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

while True:
    # car상태 가져오기
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    # car제어 설정하기
    car_controls.throttle = 1
    car_controls.steering = 1
    client.setCarControls(car_controls)

    # 차 조금 움직이기
    time.sleep(1)

    # 차에서 카메라 이미지 가져오기
    responses = client.simGetImages([
        airsim.ImageRequest(0, airsim.ImageType.DepthVis),
        airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])
    print('Retrieved images: %d', len(responses))

    # 이미지로 여러가지 실행하기
	# write_pfm & write_file 실행 
    for response in responses:
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm('py1.pfm', airsim.get_pfm_array(response))
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file('py1.png', response.image_data_uint8)

```



### Hello Drone

다음 코드는 Python으로 AirSim API를 사용해서 시뮬레이션된 quadrotor(드론)를 제어하는 방법이다.

```python
# hello_drone를 실행하기 위해 필요한 모듈 import
import airsim
import os

# AirSim 시뮬레이터 연결하기
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods(비동기 메서드)는 Future를 반환한다. 
# 작업이 완료될 때까지 기다리기 위해 join()사용
client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -10, 5).join()

# 이미지 가져오기
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),
    airsim.ImageRequest("1", airsim.ImageType.DepthPlanar, True)])
print('Retrieved images: %d', len(responses))

# 이미지로 여러가지 실행하기
# write_pfm & write_file 실행 
for response in responses:
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath('/temp/py1.pfm'), airsim.get_pfm_array(response))
    else:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath('/temp/py1.png'), response.image_data_uint8)

```



### Common APIs

- `reset`: 차량이 원래 시동 상태로 재설정됩니다. 사용자는 `Reset` 호출 후 반드시 `enableApiControl` 과  `armDisarm` 을 다시 호출해야 합니다. 

-  `confirmConnection`: 1초마다 연결 상태를 확인하고 사용자가 연결 진행 상황을 볼 수 있도록 콘솔에 보고합니다. 

-  `enableApiControl`: 안전상의 이유로, 자율 주행 차량에 대한 API 제어의 기본값은 활성화되지 않고 인간 운영자는 (일반적으로 시뮬레이터의 RC 또는 조이스틱을 통해) 완전한 제어를 갖습니다. 클라이언트가 API를 통한 제어를 요청하려면 이 호출을 해야 합니다. 차량 운영자가 API 제어를 허용하지 않았을 수도 있으며, 이는 `enableApiControl` 이 아무런 영향을 미치지 않음을 의미합니다. 그것은 `isApiControlEnabled`을 통해 확인할 수 있습니다.

- `isApiControlEnabled`: API 제어가 설정된 경우 true를 반환합니다. false(기본값)일 경우 API 호출은 무시됩니다. `enableApiControl` 호출을 성공한 후에 `isApiControlEnabled` 는 true 값을 반환합니다.

- `ping`: 연결이 설정되면 이 호출이 true로 반환됩니다. 그렇지 않으면 시간이 초과될 때까지 차단됩니다.  

- `simPrintLogMessage`: 시뮬레이터의 창에 지정된 메시지를 인쇄합니다. message_param도 제공되면 메시지 옆에 인쇄됩니다. 이 API가 동일한 메시지 값으로 호출되지만 다시 다른 message_param을 호출하면 이전 행은 (API가 디스플레이에서 새로운 줄을 만드는 대신) 새로운 행으로 덮어쓰게 됩니다. 예를들어 `simPrintLogMessage("Iteration: ", to_string(i))` 는 API가 다른 i 값으로 호출될 때마다 동일한 줄을 계속 업데이트합니다.

  (여기서 param은 <param>태그를 말하는 것입니다. <param>태그는 삽입하려는 객체에 필요한 세부적인 부분을 지정합니다. 즉, 객체에 매개변수를 전당하는 역할을 합니다.)

- `simGetObjectPose`, `simSetObjectPose`: Unreal 환경 에서 지정된 개체의 상태를 가져오고 설정합니다. 여기서 객체는 Unreal 용어로 “actor”를 의미합니다. 이름뿐만 아니라 태그로도 검색됩니다. UE Editor에 표시된 이름은 실행 시 자동으로 생성되며 영구적이지 않다는 점을 주의하세요. 따라서 actor를 이름으로 지칭하려면 UE Editor에서 자동 생성 이름을 변경해야 합니다. 또는 Unreal Editor에서 해당 actor를 클릭한 다음 Tag 속성으로 이동한 다음 "+" 기호를 클릭하고 일부 문자열 값을 추가하여 수행할 수 있는 태그를 actor에 추가할 수 있습니다. 여러 actor에 동일한 태그가 있는 경우 첫 번째 일치 항목이 반환됩니다. 일치하는 항목이 없으면 NaN 상태가 반환됩니다. 반환된 상태는 Player Start에서 유래된 SI 단위로 NED 좌표입니다. `simSetObjectPose`는 지정한 actor의 이동성이 이동 가능으로 설정되어 있어야 합니다. 그렇지 않으면 정의되지 않은 동작이 발생합니다. `simSetObjectPose`에는 개체가 다른 개체를 통해 이동되고 그 이동이 성공하면 true를 반환하는 매개변수 teleport가 있다.

  

### Image / Computer Vision APIs

AirSim은 깊이, 격차, 표면 표준 및 시야를 포함한 실측 자료와 함께 여러 카메라에서 동기화된 이미지를 검색할 수 있는 포괄적인 이미지 API를 제공합니다. settings.json 에서 해상도, FOV, motion blur 등을 설정할 수 있습니다. 충돌 상태를 탐지하는 API도 있습니다. 카메라 평면에 대한 정규화, 시차 이미지 계산 및 pfm 형식으로 저장과 함께 지정된 수의 스테레오 이미지와 접지 진실 깊이를 생성하는 것은 전체 코드를 참조하십시오. 

(이미지 API 및 컴퓨터 비전 모드에 대한 추가 정보. 도메인 임의화를 통해 이점을 얻을 수 있는 비전 문제의 경우 지원되는 장면에서 사용할 수 있는 객체를 다시 텍스처링하는 API도 있습니다.)



### Pause and Continue APIs

AirSim은 `pause(is_paused)` API를 통해 시뮬레이션을 일시 중지하고 계속할 수 있습니다. 시뮬레이션을 일시 중지하려면 `pause(True)`를 호출하고 시뮬레이션을 계속하려면 `pause(False)`를 호출합니다. 특히 강화 학습을 사용하는 동안 지정된 시간 동안 시뮬레이션을 실행한 다음 자동으로 일시 중지하는 시나리오가 있을 수 있습니다. 시뮬레이션을 일시 중지하는 동안 값비싼 계산을 수행하고 새 명령을 보낸 다음 지정된 시간 동안 시뮬레이션을 다시 실행할 수 있습니다. 이 작업은 `continueForTime(seconds)` API을 통해 수행할 수 있습니다. 이 API는 지정된 시간(초) 동안 시뮬레이션을 실행한 다음 시뮬레이션을 일시 중지합니다. 이 API의 예시는 ‘pause_continue_car.py’ & ‘pause_continue_drone.py’에서 확인할 수 있습니다.



### Collision API

`simGetCollisionInfo` API를 사용하여 충돌 정보를 얻을 수 있습니다. 이 호출은 충돌 발생 여부뿐만 아니라 충돌 위치, 표면 정상, 관통 깊이 등의 정보를 가진 구조를 반환합니다.



### Time of Day API

AirSim은 'ADiectionLight actor'가 있는 환경에 `EngineSky/BP_Sky_Sphere` 클래스의 sky sphere가 있다고 가정합니다. 기본적으로 화면에서 태양의 위치는 시간에 따라 이동하지 않습니다. 설정을 사용하여 AirSim이 화면에서 태양의 위치를 계산하는 데 사용하는 위도, 경도, 날짜 및 시간을 설정할 수 있습니다.

또한 다음 API 호출을 사용하여 지정된 날짜 시간에 따라 태양 위치를 설정할 수 있습니다.

```python
simSetTimeOfDay(self, is_enabled, start_datetime = "", is_start_datetime_dst = False, celestial_clock_speed = 1, update_interval_secs = 60, move_sun = True)
```

시간 효과를 사용하려면 is_enabled 매개 변수가 True여야 합니다. False인 경우 태양 위치가 환경에서 원래 위치로 재설정됩니다.



### Line-of-sight and world extent APIs

차량에서 한 지점 또는 두 지점 사이의 시뮬레이션에서 시선을 테스트하려면 `simTestLineOfSightToPoint(point, vehicle_name)` 과 `simTestLineOfSightBetweenPoints(point1, point2)` 모두 확인해야합니다. Sim 세계에서의 범위는 두 GeoPoints의 벡터 형태로 `SimGetWorldExtents()`를 사용하여 검색할 수 있습니다.



### Vehicle Specific APIs

#### APIs for Car

차량에는 다음과 같은 API를 사용할 수 있습니다.

- `setCarControls`:  throttle, steering, 핸드 브레이크 및 자동 또는 수동 기어를 설정할 수 있습니다.

- `getCarState`: 속도, 전류 기어 및 위치, 방향, 선형 및 각속도, 선형 및 각가속도 등 6개의 운동학적 양을 포함한 상태 정보를 검색합니다. 차체 프레임에 있는 각속도와 가속도를 제외하고 모든 양은 세계 프레임의 SI 단위인 NED 좌표계에 있습니다.

  

#### APIs for Multirotor

Multirotor는 각도, 속도 벡터, 목적지 위치 또는 이들의 조합을 지정하여 제어할 수 있습니다. 이 목적에 해당하는 `move*` API가 있습니다. 위치 제어를 할 때, 우리는 약간의 경로를 따르는 알고리즘을 사용할 필요가 있습니다. 기본적으로 AirSim은 다음 알고리즘을 사용합니다. 높은 수준의 목표만 지정하면 되고 나머지는 firmware가 처리하므로 이를 "high level control"라고 합니다. 현재 AirSim에서 사용할 수 있는 가장 낮은 레벨 컨트롤은 `moveByAngleThrottleAsync` API입니다.



#### getMultirotorState

이 API는 한 번의 호출로 차량 상태를 반환합니다. 상태에는 충돌, 추정 운동학(즉, 센서를 융합하여 계산한 운동학) 및 타임스탬프가 포함되어 있습니다. 여기서 운동학은 위치, 방향, 선형 및 각속도, 선형 및 각가속도의 6개 분량을 의미합니다. simple_slight는 현재 simple_flight에 대해 추정 및 실제 운동학 값이 동일함을 의미하는 상태 추정기를 지원하지 않는다는 점에 유의해야 합니다. 그러나 추정 운동학은 각가속도를 제외하고 PX4에 사용할 수 있습니다. 차체 프레임에 있는 각속도와 가속도를 제외하고 모든 양은 세계 프레임의 SI 단위인 NED 좌표계에 있습니다.



### Using APIs on Real Vehicles

우리는 실제 차량에서와 동일한 코드를 시뮬레이션으로 실행할 수 있기를 원합니다. AirSim을 통해 시뮬레이터에서 코드를 테스트하고 실제 차량에 배포할 수 있습니다.

쉽게 말해서 API는 실제 차량에서는 수행할 수 없는 '실측 정보 파악' 같은 작업을 허용해서는 안 됩니다. 하지만 물론 시뮬레이터는 훨씬 더 많은 정보를 가지고 있고 실제 자동차에서 작동하는 것에 신경 쓰지 않는 어플리케이션에서 유용할 것입니다. 이러한 이유로, 우리는 'simGetGroundTruthKinematics'같은 sim 접두사를 추가하여 sim 전용 API를 명확하게 기술한다. 이렇게 하면 실제 차량에서 코드를 실행하는 데 관심이 있는 경우 이러한 시뮬레이션 전용 API를 사용하지 않을 수 있습니다.

AirLib은 기가바이트 barebone Mini PC 같은 오프보드 컴퓨팅 모듈에 설치할 수 있는 자체 내장 라이브러리입니다. 그런 다음 이 모듈은 정확히 동일한 코드와 비행 제어기 프로토콜을 사용하여 PX4와 같은 비행 제어기와 통신할 수 있습니다. 시뮬레이터에서 테스트하기 위해 작성한 코드는 변경되지 않습니다. 

