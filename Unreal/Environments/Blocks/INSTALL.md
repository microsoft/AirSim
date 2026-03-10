# AirSim Blocks 项目迁移指南

本文档提供将 AirSim Blocks 电磁干扰仿真项目迁移到新电脑的完整步骤。

## 目录
- [系统要求](#系统要求)
- [软件安装](#软件安装)
- [项目克隆与构建](#项目克隆与构建)
- [AirSim settings.json 配置](#airsim-settingsjson-配置)
- [验证安装](#验证安装)
- [后端（Python）安装与迁移](#后端python安装与迁移)
- [后端快速自检与联通](#后端快速自检与联通)
- [固定轨迹演示（模拟已训练效果）](#固定轨迹演示模拟已训练效果)
- [单元测试与日志](#单元测试与日志)
- [常见问题](#常见问题)

---

## 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Windows 10/11 64位 |
| 内存 | 16 GB 以上（推荐 32 GB） |
| 硬盘空间 | 150 GB 以上（SSD 推荐） |
| 显卡 | 支持 DirectX 11，4GB 显存以上 |

---

## 软件安装

### 1. 安装 Visual Studio 2022

1. 下载地址：https://visualstudio.microsoft.com/downloads/
2. 选择 **Community** 版本（免费）
3. 运行安装程序，勾选以下工作负载：
   - **使用 C++ 的桌面开发**
   - **使用 C++ 的游戏开发**
4. 在「单个组件」中确保勾选：
   - Windows 10/11 SDK（最新版本）
   - MSVC v143 C++ 生成工具
   - C++ CMake 工具
5. 点击安装，等待完成
6. **重启电脑**

### 2. 安装 Git

1. 下载地址：https://git-scm.com/download/win
2. 运行安装程序，使用默认选项一路 Next
3. 验证安装：
   ```cmd
   git --version
   ```
   应显示类似 `git version 2.x.x`

### 3. 安装 CMake

1. 下载地址：https://cmake.org/download/
2. 选择 **Windows x64 Installer**
3. 安装时**务必勾选**「Add CMake to the system PATH」
4. 验证安装：
   ```cmd
   cmake --version
   ```
   应显示类似 `cmake version 3.x.x`

### 4. 安装 Miniconda/Anaconda（可选，用于 AirSim Python 客户端）

1. 下载地址（推荐 Miniconda）：https://docs.conda.io/miniconda.html
2. 完成安装后，打开新的 cmd/PowerShell，验证：
   ```cmd
   conda --version
   ```
3. 创建并激活独立环境（示例名称：airsim-client，Python 3.10）：
   ```cmd
   conda create -n airsim-client python=3.10 -y
   conda activate airsim-client
   ```
4. 安装 AirSim Python 客户端依赖（在已激活的环境中）：
   ```cmd
   pip install msgpack-rpc-python airsim numpy opencv-python
   ```

### 5. 安装 Unreal Engine 4.27

1. 下载 Epic Games Launcher：https://www.unrealengine.com/download
2. 安装并运行 Epic Games Launcher
3. 注册/登录 Epic 账号
4. 在 Launcher 中点击左侧「Unreal Engine」
5. 点击「Library」标签
6. 点击「+」按钮，选择版本 **4.27.2**
7. 选择安装路径（推荐 `C:\Program Files\Epic Games\UE_4.27`）
8. 等待下载完成（约 30-50 GB，需要较长时间）

### 6. 安装 Oceanology 插件（可选）

项目启用了 Oceanology 海洋插件，如需使用：
1. 在 Epic Games Launcher 的「商城」中搜索「Oceanology」
2. 购买/获取后，添加到 UE 4.27

---

## 项目克隆与构建

### 1. 克隆仓库

打开 cmd 或 PowerShell，执行：

```cmd
cd D:\
git clone -b dev https://github.com/555-zlq/AirSim.git
```

> 注意：使用 `-b dev` 指定克隆 dev 分支

### 2. 构建 AirSim

```cmd
cd D:\AirSim
build.cmd
```

此命令会自动：
- 下载第三方依赖（rpclib、Eigen3 等）
- 编译 AirLib 核心库
- 编译 MavLinkCom 通信库
- 将编译产物复制到 Unreal 插件目录

**预计耗时**：10-30 分钟，取决于电脑性能

**成功标志**：命令行显示 `Build completed successfully`

### 3. 生成 Visual Studio 项目文件

方式一（推荐）：
- 在文件资源管理器中找到 `D:\AirSim\Unreal\Environments\Blocks\Blocks.uproject`
- 右键点击，选择「Generate Visual Studio project files」

方式二（命令行）：
```cmd
"C:\Program Files\Epic Games\UE_4.27\Engine\Binaries\DotNET\UnrealBuildTool.exe" -projectfiles -project="D:\AirSim\Unreal\Environments\Blocks\Blocks.uproject" -game -engine
```

### 4. 打开项目

双击 `D:\AirSim\Unreal\Environments\Blocks\Blocks.uproject`

首次打开时，Unreal Editor 会自动编译 C++ 模块，请耐心等待。

---

## AirSim settings.json 配置

AirSim 通过 `settings.json` 文件配置仿真参数，该文件位于用户目录下。

### 配置文件位置

| 操作系统 | 路径 |
|----------|------|
| Windows | `C:\Users\<用户名>\Documents\AirSim\settings.json` |
| Linux | `~/Documents/AirSim/settings.json` |

> 首次运行 AirSim 时会自动创建默认配置，但需要手动修改以适配本项目。

### 创建配置文件

1. 创建目录（如不存在）：
   ```cmd
   mkdir %USERPROFILE%\Documents\AirSim
   ```

2. 创建 `settings.json` 文件，内容如下：

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "LocalHostIp": "0.0.0.0",
  "Vehicles": {
    "Drone1":  { "VehicleType": "SimpleFlight", "X": 0,   "Y": 0,    "Z": -2 },
    "Drone2":  { "VehicleType": "SimpleFlight", "X": -5,  "Y": 5,    "Z": -4 },
    "Drone3":  { "VehicleType": "SimpleFlight", "X": 5,   "Y": 5,    "Z": -4 }
  }
}
```

### 配置项说明

| 字段 | 说明 |
|------|------|
| `SettingsVersion` | 配置版本，使用 `1.2` |
| `SimMode` | 仿真模式，本项目使用 `Multirotor`（多旋翼无人机） |
| `LocalHostIp` | API 监听地址，设为 `0.0.0.0` 允许外部连接（WSL/远程） |
| `Vehicles` | 定义无人机列表，每个无人机有独立名称和初始位置 |

### 多无人机配置

本项目使用 3 架无人机进行多智能体仿真：

| 无人机名称 | 初始位置 (X, Y, Z) | 说明 |
|------------|-------------------|------|
| `Drone1` | (0, 0, -2) | 主无人机，位于原点上方 2m |
| `Drone2` | (-5, 5, -4) | 左后方，高度 4m |
| `Drone3` | (5, 5, -4) | 右后方，高度 4m |

> 注意：Z 轴为负表示向上（NED 坐标系）

### 关键配置：LocalHostIp

如果后端运行在 **WSL/Linux** 而 UE 运行在 **Windows**，必须设置：

```json
"LocalHostIp": "0.0.0.0"
```

这允许 AirSim RPC 服务接受来自 WSL 的连接。

### 高级配置示例

如需更多功能，可添加以下配置：

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "LocalHostIp": "0.0.0.0",
  "ClockSpeed": 1.0,
  "ViewMode": "FlyWithMe",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": -2,
      "DefaultVehicleState": "Armed",
      "EnableCollisionPassthrogh": false,
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      "Cameras": {
        "front_center": {
          "CaptureSettings": [{ "ImageType": 0, "Width": 640, "Height": 480 }]
        }
      }
    },
    "Drone2": { "VehicleType": "SimpleFlight", "X": -5, "Y": 5, "Z": -4 },
    "Drone3": { "VehicleType": "SimpleFlight", "X": 5, "Y": 5, "Z": -4 }
  }
}
```

| 高级字段 | 说明 |
|----------|------|
| `ClockSpeed` | 仿真时间倍率，1.0 为实时 |
| `ViewMode` | 视角模式：`FlyWithMe`（跟随）、`GroundObserver`（地面）等 |
| `DefaultVehicleState` | 初始状态：`Armed`（已解锁）或 `Disarmed` |
| `Cameras` | 相机配置，可设置分辨率、图像类型等 |

### 配置生效

修改 `settings.json` 后，需要**重启 Unreal Editor** 才能生效。

---

## 验证安装

完成以上步骤后，进行以下验证：

### 验证清单

- [ ] Unreal Editor 成功打开项目
- [ ] 无编译错误（查看 Output Log）
- [ ] 点击「Play」能正常运行仿真
- [ ] HTTP API 可访问（浏览器打开 http://localhost:18080/ping）
- [ ] Python 客户端能连接：
  ```python
  import airsim
  client = airsim.MultirotorClient()
  client.confirmConnection()
  ```

---
---

## 后端（Python）安装与迁移

> 本节用于在新电脑（Linux/WSL/Windows 皆可，推荐 Linux/WSL）部署当前后端系统：AirSim×UE 多智能体强化学习并行环境（PettingZoo）。

### 1. 克隆后端仓库

```bash
cd /home/<你的用户名>/python
git clone https://github.com/555-zlq/Airsim-Simulation.git
cd Airsim-Simulation
```

### 2. 准备 Python 环境

```bash
# 如未安装，先安装 Miniconda：https://docs.conda.io/miniconda.html
conda create -n airsim-rl python=3.10 -y
conda activate airsim-rl

# 安装项目依赖（requirements.txt）
pip install -r requirements.txt

# 可选：测试工具
pip install pytest pyyaml
```

依赖清单见仓库根目录的 `requirements.txt`。核心依赖包括：airsim、gymnasium、pettingzoo、torch、PyYAML、requests。

### 3. 后端目录结构（关键路径）

```
src/airsim_multi_rl/
  config/default.yaml        # 后端唯一配置来源（IP/端口/参数）
  envs/                      # 适配层、观测、奖励、终止、并行环境
  runners/                   # rollout 与轨迹播放入口
  scripts/                   # 自检工具（HTTP 拉取等）
  utils/                     # 工具（日志等）
tests/                       # 基础单测
```

### 4. 关键配置项

编辑配置文件：[default.yaml](file:///home/carton/python/Airsim-Simulation/src/airsim_multi_rl/config/default.yaml)：

- AirSim 连接（通常 UE/Blocks 在 Windows 上，后端在 WSL/Linux）：
  - `ip`/`port`：AirSim RPC（MultirotorClient）连接参数
- UE HTTP 干扰服务：
  - `jammer_penalty_mode: "power"` 建议开启功率模式
  - `ue_rpc.http_base: "http://<Windows主机可达IP>:18081"` 将 IP 替换为 Windows 主机在 WSL 可达的地址（示例：`211.83.105.240`）
  - `ue_rpc.timeout` 和 `query_every_n_steps` 可按网络状况调整
- 轨迹演示（可模拟“已训练”效果）：
  - `trajectory.enabled: true`，按需调整 `speed/yaw_align/waypoints`

Windows 主机可达 IP 获取方法（在 Windows `cmd` 执行 `ipconfig`，使用以太网或 WLAN 的 IPv4 地址；确保防火墙放行端口 18081 或按需修改）。


## 常见问题
## 后端快速自检与联通

> 目标：验证 Python 后端能在新机器上正确运行，并与 UE HTTP 与 AirSim RPC 互通。

### 1. 后端 Smoke Test

- 在线模式（需要 UE/AirSim 运行）：
  ```bash
  PYTHONPATH=src python -m airsim_multi_rl.scripts.smoke_test
  ```
- 离线模式（无 AirSim，仅验证环境逻辑与输出）：
  ```bash
  SMOKE_OFFLINE=1 PYTHONPATH=src python -m airsim_multi_rl.scripts.smoke_test
  ```

预期输出包含 agents 名称与每步奖励和，验证基本连通与形状正确性。

### 2. UE HTTP 拉取式联通检查（WSL → Windows）

确认 UE 的 Jammer HTTP 服务端口（默认 18081，或按你在 UE 中设置的端口），然后运行：

```bash
# 指定 Windows 主机 IP 与端口（推荐）
PYTHONPATH=src python -m airsim_multi_rl.scripts.http_pull_check \
  --base http://<WIN_HOST_IP>:18081 --names BP_JammerActor,BP_JammerActor2 --x 10 --y 0 --z 0

# 或自动探测（需要 WSL 正常路由）
PYTHONPATH=src python -m airsim_multi_rl.scripts.http_pull_check --port 18081
```

成功判定：
- `/ping` 返回 200 和 JSON
- `/jammers` 返回非空列表（含 Jammer 名称与位置字段）
- `/jammer_power` 至少一种方式（传米或传厘米）返回有效 `power` 数值

若失败，请检查：
- Windows 防火墙是否放行端口（默认 18081）
- UE HTTP 服务是否绑定到 0.0.0.0 或 Windows 主机可达 IP（不是仅 127.0.0.1）
- 名称大小写/下划线差异（脚本内置规范化与回退）

---

## 固定轨迹演示（模拟已训练效果）

> 用后端在 UE 中复现“已训练策略”的演示效果：通过固定路径驱动三架无人机穿越干扰区，记录关键指标。

1) 配置轨迹（示例已在 [default.yaml](file:///home/carton/python/Airsim-Simulation/src/airsim_multi_rl/config/default.yaml) 提供）：
   - `trajectory.enabled: true`
   - `trajectory.waypoints`：为 `Drone1/2/3` 设置三维路径点（米，NED）
   - `trajectory.csv_enabled: true` 与 `csv_path`：导出每步指标（reward、dist_to_goal、jammer_power 等）

2) 运行轨迹播放：
```bash
PYTHONPATH=src python -m airsim_multi_rl.runners.trajectory_playback
# 离线演示（DummyClient）：仅验证管线与 CSV 写入
TRAJECTORY_OFFLINE=1 PYTHONPATH=src python -m airsim_multi_rl.runners.trajectory_playback
```

3) 输出与验证：
- CSV 路径见 `trajectory.csv_path`（默认 `logs/trajectory_demo.csv`）
- 日志文件见 `logs/train.log`（可在 `logging.file_path` 调整）

---

## 单元测试与日志

### 1. 运行单测

```bash
PYTHONPATH=. python -m pytest -q tests/test_env_shapes.py
PYTHONPATH=. python -m pytest -q tests/test_trajectory_playback.py
```

单测通过说明：环境空间形状与轨迹演示最小功能正常（离线 DummyClient）。

### 2. 训练与干扰强度日志

- 日志配置位于 [default.yaml](file:///home/carton/python/Airsim-Simulation/src/airsim_multi_rl/config/default.yaml) 的 `logging` 段
- 结构化日志采用异步队列写入，包含 episode/step 与干扰强度（power 或 distance）
- 可使用脚本进行日志性能自检：
  ```bash
  PYTHONPATH=src python -m airsim_multi_rl.scripts.log_perf_check --n 10000 --mode both
  ```


### Q1: build.cmd 报错找不到 Visual Studio

**解决方案**：
- 确保安装了「使用 C++ 的桌面开发」工作负载

**解决方案**：
- 确保安装了「使用 C++ 的桌面开发」工作负载
- 尝试以管理员身份运行 cmd

### Q2: 找不到 AirSim 插件

**解决方案**：
- 确保 `build.cmd` 执行成功
- 检查 `D:\AirSim\Unreal\Plugins\AirSim` 目录是否存在

### Q3: Unreal Editor 打开时编译失败

**解决方案**：
1. 关闭 Unreal Editor
2. 删除以下目录：
   - `D:\AirSim\Unreal\Environments\Blocks\Binaries`
   - `D:\AirSim\Unreal\Environments\Blocks\Intermediate`
3. 重新运行 `build.cmd`
4. 重新打开项目

### Q4: UE4 版本不匹配

**解决方案**：
- 本项目要求 Unreal Engine **4.27**
- 在 Epic Games Launcher 中确认安装的版本

### Q5: HTTP API 无法访问

**解决方案**：
- 确认仿真正在运行
- 检查防火墙是否阻止了 18080 端口
- 查看 `Config/DefaultEngine.ini` 中的 `DefaultBindAddress` 配置

### Q6（后端）: 后端无法查询 UE /jammers 或 /jammer_power

**解决方案**：
- 确认 `config/default.yaml` 中 `ue_rpc.http_base` 指向 Windows 主机可达的 IP 与端口（如 `http://<IP>:18081`）
- 使用脚本联通自检（见「后端快速自检与联通」）
- 若 UE 仅绑定 127.0.0.1，请将其改为 0.0.0.0 或使用端口代理

### Q7（后端）: 无人机不动或动作无效

**解决方案**：
- 确认顺序：`enableApiControl → armDisarm → takeoff` 后再发送速度控制
- 确保 `moveByVelocityAsync(...).join()` 已等待（后端已封装）
- 检查 `v_max/yaw_rate_max_deg` 是否过小

---

## 项目结构

```
D:\AirSim\
├── AirLib\                      # AirSim 核心库
├── build.cmd                    # 构建脚本
├── PythonClient\                # Python 客户端
├── Unreal\
│   ├── Plugins\AirSim\          # AirSim Unreal 插件
│   └── Environments\
│       └── Blocks\              # 本项目
│           ├── Blocks.uproject  # 项目文件
│           ├── Config\          # 配置文件
│           ├── Content\         # 资源文件
│           └── Source\Blocks\   # C++ 源代码
│               ├── JammerActor.*        # 干扰器基类
│               ├── AntennaJammer.*      # 天线干扰器
│               ├── DroneJammer.*        # 无人机干扰器
│               └── JammerHttpService.*  # HTTP API 服务
```

---

## 联系方式

如有问题，请在 GitHub 仓库提交 Issue：
https://github.com/555-zlq/Airsim-Simulation/issues
