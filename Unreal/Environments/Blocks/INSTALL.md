# AirSim Blocks 项目迁移指南

本文档提供将 AirSim Blocks 电磁干扰仿真项目迁移到新电脑的完整步骤。

## 目录

- [系统要求](#系统要求)
- [软件安装](#软件安装)
- [项目克隆与构建](#项目克隆与构建)
- [验证安装](#验证安装)
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

### 4. 安装 Python（可选，用于 AirSim Python 客户端）

1. 下载地址：https://www.python.org/downloads/
2. 推荐版本：Python 3.10
3. 安装时**务必勾选**「Add Python to PATH」
4. 验证安装：
   ```cmd
   python --version
   pip --version
   ```
5. 安装 AirSim Python 依赖：
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

## 常见问题

### Q1: build.cmd 报错找不到 Visual Studio

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
https://github.com/555-zlq/AirSim/issues
