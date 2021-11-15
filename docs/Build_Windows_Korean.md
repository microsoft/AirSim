# 윈도우에서 AirSim 빌드 방법

## Unreal Engine 설치

1. 먼저 Epic Games Launcher를 [설치](https://www.unrealengine.com/download)합니다. Unreal Engine은 오픈소스이며 Epic Games 회원가입만 하면 무료로 다운로드 받을 수 있습니다.
2. Epic Games Launcher를 실행하고, 좌측의 `Unreal Engine`  탭을 누릅니다.
   오른쪽 상단의 `Install` 버튼을 클릭하면 **Unreal Engine >= 4.25** 라는 설치 옵션이 표시됩니다. 아래 이미지와 같이 Unreal Engine을 설치할 위치를 선택합니다. 만약 여러 개 버전의 Unreal Engine이 설치되어 있을 경우 버튼 옆에 있는 아래 화살표를 클릭하여 **사용중인 버전이 `current`로 설정되어 있는지 확인합니다**.

   **참고**: 만약 UE 4.16 버전 이전의 프로젝트가 있을 경우, [upgrade guide](https://microsoft.github.io/AirSim/unreal_upgrade/)를 참고하여 프로젝트를 업그레이드 하기 바랍니다.

![Unreal Engine Tab UI Screenshot](D:\강의자료\21년~ 자료\3-2\인공지능\팀플\ue_install.png)

![Unreal Engine Install Location UI Screenshot](D:\강의자료\21년~ 자료\3-2\인공지능\팀플\ue_install_location.png)

## AirSim 빌드
* Visual Studio 2019 설치합니다.
VS 2019를 설치하는 동안 **반드시** **Desktop Development with C++** and **Windows 10 SDK 10.0.18362**를 선택하고(default로 선택되어 있음) '개별 구성 요소' 탭에서 latest .NET Framework SDK를 선택해야 합니다.
* `Developer Command Prompt for VS 2019` 실행합니다.
* 리포지토리: `git clone https://github.com/Microsoft/AirSim.git`을 clone 하고, `cd AirSim` 명령어를 통해 AirSim 디렉토리로 이동합니다.

    **참고:** C드라이브에 AirSim을 설치하는 것을 권장하지 않습니다. 이로 인해 스크립트 실행이 실패할 수도 있으며, VS를 관리자 모드로 실행해야하는 문제점이 있습니다. 대신 D 또는 E와 같은 다른 드라이브에 설치할 것을 권장합니다.

* 명령 입력창에서 `build.cmd` 을 실행합니다. 실행하면 Unreal\Plugins 폴더의 플러그인 비트를 생성할 준비가 끝납니다.

## Unreal 프로젝트 빌드

마지막으로 차량의 환경을 제공하는 Unreal 프로젝트가 필요합니다. 첫번째 환경을 구축하기 전에 Unreal Engine과 Epic Games Launcher를 닫았다가 다시 실행합니다. Epic Games Launcher를 재실행한 이후에 Unreal Engine의 프로젝트 파일과 연결하라는 메시지가 나타난다면 '지금 수정'을 눌러 수정합니다. AirSim에 내장된 "Blocks Environment"을 사용하거나 자기만의 환경을 구축할 수 있습니다. 자세한 설정 방법은 [setting up Unreal Environment](https://microsoft.github.io/AirSim/unreal_proj/)를 참조하세요.

## 원격 제어 설정 (Multirotor 한정)

수동으로 조작할 경우 원격 제어 설정이 필요합니다. 자세한 사항은 [remote control setup](https://microsoft.github.io/AirSim/remote_control/)을 참조하세요.

다른 방법으로는 [APIs](https://microsoft.github.io/AirSim/apis/)를 이용하여 프로그램 제어를 활용하거나 [Computer Vision mode](https://microsoft.github.io/AirSim/image_apis/)를 사용하여 키보드로 조작할 수 있습니다.

## AirSim 사용 방법

위 단계를 거쳐 AirSim 설정이 완료되었다면, 다음을 수행합니다:

1. 프로젝트의 .sln 파일을 더블 클릭하여 `Unreal\Environments\Blocks` 디렉토리 안에 있는 Blocks 프로젝트를 불러옵니다.(또는 당신의 [커스텀](https://microsoft.github.io/AirSim/unreal_custenv/) Unreal 프로젝트의 .sln 파일). 만약 .sln 파일이 보이지 않는다면 위의 Unreal Project 빌드 단계를 완료하지 않은 것일 수 있습니다.
2. 당신의 Unreal 프로젝트를 시작 프로젝트(예: Blocks 프로젝트)로 선택하고 빌드 구성이 "Develop Editor"와 x64로 설정되어 있는지 확인합니다.
3. Unreal Editor를 불러온 뒤 재생 버튼을 누릅니다. 

**Tip**
    '편집->편집기 기본 설정' 메뉴에서 '검색' 상자에 'CPU'를 입력합니다. 그 뒤 '백그라운드에서 CPU를 적게 사용' 옵션이 선택되어 있지 않은지 확인합니다.

다른 사용 가능한 옵션들을 확인하려면 [Using APIs](https://microsoft.github.io/AirSim/apis/)와 [settings.json](https://microsoft.github.io/AirSim/settings/) 문서를 참고하십시오.

# Unity에서 AirSim 사용(시험 단계)
[Unity](https://unity3d.com/)는 또 다른 훌륭한 게임 엔진 플랫폼으로, Unity를 통한 AirSim 사용 가이드가 [AirSim with Unity](https://microsoft.github.io/AirSim/Unity/)에 준비되어 있습니다. 단, 아직 완성 작업이 진행중이며 모든 기능이 아직 완전히 작동하지 않을 수 있습니다.

