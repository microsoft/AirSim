# 새로운 소식

다음은 중요한 변경 사항을 요약 한 목록입니다. 여기에는 사소한 변경 사항이나 버그 수정 또는 설명서 업데이트가 포함되지 않습니다. 이 목록은 몇 달마다 업데이트되었습니다. 자세한 변경 내용은 [커밋 내역](https://github.com/Microsoft/AirSim/commits/master)을 검토하십시오.

### 2018년 11월
* 기상 효과와 [APIs](docs/apis.md#weather-apis) 추가
* [Time of Day API](docs/apis.md#time-of-day-api) 추가
* [AirSim on Unity](https://github.com/Microsoft/AirSim/tree/master/Unity)의 실험적 통합이 이제 사용할 수 있습니다. 자세한 것은 [Unity blog post](https://blogs.unity3d.com/2018/11/14/airsim-on-unity-experiment-with-autonomous-vehicle-simulation).
* [New environments](https://github.com/Microsoft/AirSim/releases/tag/v1.2.1): 숲, 평원 (풍차 농장), TalkingHeads (사람 머리 시뮬레이션), TrapCam (카메라를 통한 동물 탐지)
* 메인 스크린 렌더링을 껐을 때에 대한 [NoDisplay view mode](https://github.com/Microsoft/AirSim/blob/master/docs/settings.md#viewmode)의 높은 효율로 인해 이미지를 빠른 속도로 캡쳐 할 수 있습니다.
* 설정을 통한 [센서 활성화/비활성화](https://github.com/Microsoft/AirSim/pull/1479)
* [Lidar Sensor](docs/lidar.md)
* [Flysky FS-SM100 RC USB 어댑터 지원](https://github.com/Microsoft/AirSim/commit/474214364676b6631c01b3ed79d00c83ba5bccf5)
* 케이스 연구: [Formula Student Technion Driverless](https://github.com/Microsoft/AirSim/wiki/technion)
* [Multi-Vehicle Capability](docs/multi_vehicle.md)
* [Custom speed units](https://github.com/Microsoft/AirSim/pull/1181)
* [ROS publisher](https://github.com/Microsoft/AirSim/pull/1135)
* [simSetObjectPose API](https://github.com/Microsoft/AirSim/pull/1161)
* [Character Control APIs](https://github.com/Microsoft/AirSim/blob/master/PythonClient/airsim/client.py#L137) (works on TalkingHeads binaries in release)
* [Arducopter Solo 지원](https://github.com/Microsoft/AirSim/pull/1387)
* [sudo 액세스가 없는 Linux 설치](https://github.com/Microsoft/AirSim/pull/1434)
* [Kinect같은 ROS 퍼블리셔](https://github.com/Microsoft/AirSim/pull/1298)


### 2018년 6월
* 개발 워크 플로우 문서
* 더 나은 Python 2 호환성
* OSX setup 수정
* API를 거의 완전히 새로운 쓰레딩 모델로 다시 작성, 기존 API 병합, 몇 가지 새로운 API 추가

### 2018년 4월
* 언리얼 엔진 4.18 및 Visual Studio 2017로 업그레이드
* world-level의 API를 지원하기 위한 API 프레임 워크 리팩토링
* 최신 PX4 펌웨어 지원
* 자세한 정보를 포함한 CarState
* ThrustMaster 휠 지원
* 드론과 차에 대한 pause 와 continueForTime APIs
* 성능 저하없이 드론 시뮬레이션을 더 높은 클럭 속도로 실행
* 드론을 위한 완전한 기능의 Forward-only 모드 (중심을 보면서 궤도를 돎)
* 드론의 흔들림을 줄이기 위한 향상된 PID 튜닝
* 드론 및 자동차에 대한 임의의 차량 블루프린트를 설정하는 기능
* 설정을 통한 짐벌 안정화
* skinned 및 skeletal 메시를 이름별로 분류하는 기능
* moveByAngleThrottle API 추가
* 더 나은 기동성을 위한 자동차 물리 튜닝
* 설정을 통한 추가 카메라 구성
* 지리적으로 태양 위치를 계산한 하루 시간
* 키보드를 통한 더 나은 자동차 조종
* segmentation 세팅에 MeshNamingMethod를 추가
* gimbal API 추가
* getCameraParameters API 추가
* GPU 리소스를 절약하기 위해 메인 렌더링을 끄는 기능
* 캡처 설정을 위한 프로젝션 모드
* getRCData, setRCData APIs 추가
* negative ID를 사용하여 segmentation을 해제하는 기능
* OSX 빌드 개선
* Initial ID가 있는 매우 큰 환경에서 작동하는 segmentation
* Segmentation ID를 위한 더 우수하고 확장 가능한 해시 계산
* 커스텀 통합 메소드를 위한 확장 가능한 PID 컨트롤러
* 센서 구조는 이제 ray casting과 같은 렌더러 특정 기능을 가능하게 합니다.
* 레이저 고도계 센서


### 2018년 1월
* 구성 시스템 재 작성, 향후 타겟팅 할 유연한 구성 가능
* Multi-Vehicle 지원 1단계, 핵심 인프라 변경
* MacOS 지원
* 적외선 뷰
* 카메라에 대한 5가지 유형의 노이즈 및 간섭
* 카메라의 WYSIWIG 캡처 설정, 메인 뷰에서 녹화 설정 미리보기
* Azure 지원 1단계, headless 모드에 대한 인스턴스 구성 가능
* API를 통한 포즈, 선형 및 각속도 + 가속을 얻는 기능을 가진 전체 kinematics API 추가
* 여러 대의 카메라에서 다중 이미지 기록
* 정규식을 통한 검색, 객체 ID 구성 설정 기능을 가진 새로운 segmentation API 추가
* 환경에서 동물과 같은 물체의 자세를 얻는 기능을 가진 새로운 object pose API 추가
* 카메라 인프라 향상. 단 몇 줄로 IR과 같은 새로운 이미지 유형 추가 기능 탑재
* 속도 요소를 0 < x < 무한 으로 설정해 시뮬레이션을 실행할 수 있는 드론 및 자동차 용 클럭 속도 API 추가
* Logitech G920 휠 지원
* 자동차의 물리적 속성 조율. 자동차가 전복되지 않고, 더 나은 커브로 조종에 반응하여 가스 패들 동작을 보다 사실적으로 풀어줆
* API 디버깅
* 24 시간 이상 연속 작동에 대한 강한 테스트
* Landscape 와 하늘 segmentation에 대한 지원
* CV 모드에서 가속 컨트롤을 통한 수동 탐색으로 사용자가 훨씬 쉽게 환경을 탐색 할 수 있습니다.
* 충돌 API 추가
* 녹화 향상. 지상의 사실성, 여러 이미지, 제어 상태를 포함한 여러 새로운 데이터 포인트를 기록
* Planner 와 Perspective Depth 뷰
* Disparity 뷰
* 새로운 이미지 API는 float, png 또는 numpy 형식을 지원합니다
* 이미지 캡처를 위한 6가지 구성 세팅. 자동 노출, 모션 블러, 감마 등 설정 기능
* Sub-windows, 녹화, API 등을 포함한 완벽한 멀티 카메라 지원
* 한 번에 모든 환경을 구축하는 커맨드 라인 스크립트
* 하위 모듈 제거, rpclib를 다운로드로 사용

### 2017년 11월
* 이제 [car model](docs/using_car.md)이 있습니다.
* 코드를 빌드할 필요가 없습니다. [바이너리](https://github.com/Microsoft/AirSim/releases)를 다운로드하기만 하면 됩니다.
* AirSim에서의 [강화 학습 예제](docs/reinforcement_learning.md)
* 추가 설정없이 "작동"하는 새로운 내장 비행 컨트롤러 [simple_flight](docs/simple_flight.md). 또한 현재 *default*입니다.
* AirSim은 이제 카메라 계획에 있는 [깊이 및 시차 이미지](docs/image_apis.md)도 생성합니다.
* 이제 공식적인 리눅스 빌드 또한 있습니다!

## 2017년 9월
- [car model](docs/using_car.md)을 추가했습니다!

## 2017년 8월
- [simple_flight](docs/simple_flight.md)는 이제 드론의 기본 비행 컨트롤러입니다. PX4를 사용하려면 [PX4 setup doc](docs/px4_setup.md)에 따라 settings.json을 수정해야합니다.
- Linux 빌드는 공식이며 현재 다양한 버그 수정으로 인해 Unreal 4.17을 사용합니다
- ImageType 열거 형에는 몇 가지 새로운 추가 기능이 추가되고 기존 항목이 명확해졌습니다.
- SubWindows는 이제 settings.json에서 구성 할 수 있습니다
- PythonClient는 이제 완성되었으며 C ++ API와 동등합니다. 이들 중 일부는 주요 변경 사항이 있습니다.

## 2017년 2월
- 첫 릴리즈!