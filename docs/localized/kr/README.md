# AirSim에 온 것을 환영합니다

AirSim은 [Unreal Engine](https://www.unrealengine.com/)으로 빌드 된 드론, 차량 등을 위한 시뮬레이터 입니다 (이제 실험적인 [Unity](https://unity3d.com/) 릴리즈 또한 포함합니다). AirSim은 오픈소스이고, 크로스 플랫폼입니다. 그리고 물리적, 시각적으로 현실같은 시뮬레이션을 위한 PX4와 같은 인기있는 비행 컨트롤러들에 대해서 hardware-in-loop을 지원합니다. AirSim은 간단하게 Unreal 환경에 빠질 수 있는 Unreal 플러그인으로 개발되었습니다. 유사하게, 우리는 Unity 플러그인에 대한 실험적 릴리즈가 있습니다.

우리의 목표는 AirSim을 딥 러닝, 컴퓨터 비전과 자율 주행 차량의 학습 알고리즘 강화를 포함한 AI 연구를 위한 플랫폼으로써 개발하는 것입니다. 이를 위해서 AirSim은 또한 데이터를 검색하고 플랫폼 독립적인 방법으로 차량을 제어하기 위해서 API들을 제공합니다.

**빠른 1.5분 데모 확인**

AirSim에서의 드론

[![AirSim Drone Demo Video](docs/images/demo_video.png)](https://youtu.be/-WfTr1-OBGQ)

AirSim에서의 차량

[![AirSim Car Demo Video](docs/images/car_demo_video.png)](https://youtu.be/gnz1X3UNM5Y)

## 새로운 소식
* 멀티로터용 ROS 래퍼를 사용할 수 있습니다. ROS API에 대해서는 [airsim_ros_pkgs](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_ros_pkgs)을 보십시오, 그리고 튜토리얼에 대해서는 [airsim_tutorial_pkgs](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_tutorial_pkgs)을 보십시오. 
* [기압계, IMU, GPS, 자력계, 거리 센서에 대한 센서 API 추가](https://microsoft.github.io/AirSim/docs/sensors.md) 
* [docker in ubuntu](https://microsoft.github.io/AirSim/docs/docker_ubuntu)에 대한 지원 추가
* 기상 효과와 [APIs](https://microsoft.github.io/AirSim/docs/apis#weather-apis) 추가
* [Time of Day API](https://microsoft.github.io/AirSim/docs/apis#time-of-day-api) 추가
* [AirSim on Unity](https://github.com/Microsoft/AirSim/tree/master/Unity)의 실험적 통합이 이제 사용할 수 있습니다. 자세한 것은 [Unity blog post](https://blogs.unity3d.com/2018/11/14/airsim-on-unity-experiment-with-autonomous-vehicle-simulation).
* [New environments](https://github.com/Microsoft/AirSim/releases/tag/v1.2.1): 숲, 평원 (풍차 농장), TalkingHeads (사람 머리 시뮬레이션), TrapCam (카메라를 통한 동물 탐지)
* 메인 스크린 렌더링을 껐을 때에 대한 [NoDisplay view mode](https://microsoft.github.io/AirSim/docs/settings#viewmode)의 높은 효율로 인해 이미지를 빠른 속도로 캡쳐 할 수 있습니다.
* [Lidar Sensor](https://microsoft.github.io/AirSim/docs/lidar)
* 케이스 연구: [Formula Student Technion Driverless](https://github.com/Microsoft/AirSim/wiki/technion)
* [Multi-Vehicle Capability](https://microsoft.github.io/AirSim/docs/multi_vehicle)
* [ROS publisher](https://github.com/Microsoft/AirSim/pull/1135)

변경 사항 전체 목록에 대해서는, [Changelog](CHANGELOG.md)를 보십시오

## 얻는 방법

### Windows
* [Download binaries](https://microsoft.github.io/AirSim/docs/use_precompiled)
* [Build it](https://microsoft.github.io/AirSim/docs/build_windows)

### Linux
* [Build it](https://microsoft.github.io/AirSim/docs/build_linux)

[![Build Status](https://travis-ci.org/Microsoft/AirSim.svg?branch=master)](https://travis-ci.org/Microsoft/AirSim)

## 사용 방법

### 문서

AirSim에 대한 모든 방면에 대해서 [detailed documentation](https://microsoft.github.io/AirSim/)을 보십시오.

### 수동 운전

만약 아래 그림과 같이 원격 컨트롤러 (RC)를 갖고 있다면, 시뮬레이터에서 수동으로 드론을 조종할 수 있습니다. 차에 대해서는, 방향키를 사용해서 수동으로 조종할 수 있습니다.

[More details](https://microsoft.github.io/AirSim/docs/remote_control/)

![record screenshot](docs/images/AirSimDroneManual.gif)

![record screenshot](docs/images/AirSimCarManual.gif)


### 프로그래밍 제어

AirSim은 API들을 제공하므로 차량들과 시뮬레이션에서 프로그래밍 방식으로 상호작용할 수 있습니다. 당신은 이 API를 이미지를 검색하고, 상태를 얻고, 차량을 조종하는 등을 하는데 쓸 수 있습니다. API는 RPC를 통해서 제공됩니다. 그리고 C++, Python, C#, Java를 포함한 다양한 언어를 통해서 접근 가능합니다.

이 API는 또한 분리, 독립적인 크로스 플랫폼 라이브러리의 일부분으로써 이용가능합니다. 그래서 당신은 차량의 컴패니언 컴퓨터에 배포할 수 있습니다. 이 방법으로 시뮬레이터에서 코드를 작성하고 테스트 한 후 나중에 실제 차량에서 실행할 수 있습니다. 전이 학습 및 관련 연구는 우리의 중점 분야 중 하나입니다.

AirSim을 시작할 때마다 메시지가 표시되지 않도록 기본 차량이나 새 [ComputerVision mode](https://microsoft.github.io/AirSim/docs/image_apis#computer-vision-mode-1)을 지정하기 위해서 [SimMode setting](https://microsoft.github.io/AirSim/docs/settings#simmode)을 쓸 수 있습니다.

[More details](https://microsoft.github.io/AirSim/docs/apis/)

### 학습 데이터 수집

딥 러닝을 위해 AirSim에서 교육 데이터를 생성 할 수있는 두 가지 방법이 있습니다. 가장 쉬운 방법은 우측 하단에 있는 기록 버튼을 누르는 것입니다. 각 프레임마다 포즈와 이미지 기록이 시작됩니다. 데이터 로깅 코드는 매우 간단하며 당신의 생각에 맞게 수정할 수 있습니다.

![record screenshot](docs/images/record_data.png)

원하는 방식으로 교육 데이터를 생성하는 더 좋은 방법은 API에 액세스하는 것입니다. 이것은 당신이 기록하려는 데이터를 어떻게, 무엇을, 어디서, 언제 완전히 통제 할 수 있도록 합니다.

### 컴퓨터 비전 모드

AirSim을 사용하는 또 다른 방법은 "컴퓨터 비전" 모드라고 부르는 것입니다. 이 모드에서는, 차량이나 물리가 없습니다. 키보드를 사용하여 씬 주위를 이동하거나, API를 사용하여 임의의 포즈로 사용 가능한 카메라를 배치하고, 깊이, 시차, 표면 법선 또는 객체 분할과 같은 이미지를 수집 할 수 있습니다.

[More details](https://microsoft.github.io/AirSim/docs/image_apis/)

### 날씨 효과

날씨 효과에 사용할 수 있는 다양한 옵션을 보려면 F10을 누르십시오. [APIs](https://microsoft.github.io/AirSim/docs/apis#weather-apis)를 사용하여 날씨를 제어 할 수도 있습니다. 사용 가능한 다른 옵션을 보려면 F1을 누르십시오.

![record screenshot](docs/images/weather_menu.png)

## 튜토리얼

- [Video - AirSim과 Pixhawk 설정 튜토리얼](https://youtu.be/1oY8Qu5maQQ) by Chris Lovett
- [Video - AirSim과 Pixhawk 사용 튜토리얼](https://youtu.be/HNWdYrtw3f0) by Chris Lovett
- [Video - AirSim에서 자체 환경 사용](https://www.youtube.com/watch?v=y09VbdQWvQY) by Jim Piavis
- [AirSim에서 강화 학습](https://microsoft.github.io/AirSim/docs/reinforcement_learning) by Ashish Kapoor
- [자율 주행 설명서](https://aka.ms/AutonomousDrivingCookbook) by Microsoft Deep Learning and Robotics Garage Chapter
- [간단한 충돌 방지를 위한 TensorFlow 사용](https://github.com/simondlevy/AirSimTensorFlow) by Simon Levy and WLU team

## 참여

### 논문

자세한 기술 정보는 [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065)에 있습니다. 이것을 인용하십시오:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

### 기여

기여할 분야를 찾고 있다면 [open issues](https://github.com/microsoft/airsim/issues)를 살펴보십시오.

* [AirSim 디자인에 대한 자세한 사항](https://microsoft.github.io/AirSim/docs/design)
* [코드 구조에 대한 자세한 사항](https://microsoft.github.io/AirSim/docs/code_structure)
* [기여 가이드라인](CONTRIBUTING.md)
* [Trello Board](https://trello.com/b/1t2qCeaA/wishlist-by-community-for-community)

### AirSim을 사용하는 사람은 누구입니까?

우리는 우리가 알고있는 몇 가지 프로젝트, 사람 및 그룹의 [목록](https://microsoft.github.io/AirSim/docs/who_is_using)을 유지하고 있습니다. 이 목록에 포함 되길 원하시면 [여기서 요청하세요](https://github.com/microsoft/airsim/issues).

## 연락

[Facebook](https://www.facebook.com/groups/1225832467530667/)에서 AirSim 그룹에 가입하여 최신 정보를 받거나 질문을 하십시오.

## FAQ

문제가 발생하면 [FAQ](https://microsoft.github.io/AirSim/docs/faq)를 확인하고 [AirSim](https://github.com/Microsoft/AirSim/issues) 리포지토리에 자유롭게 문제를 게시하십시오.

## 라이선스

이 프로젝트는 MIT 라이센스에 따라 배포됩니다. 자세한 내용은 [라이센스 파일](LICENSE)을 확인하십시오.
