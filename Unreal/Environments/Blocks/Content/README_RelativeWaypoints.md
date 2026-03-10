# CruisingJammerActor 相对坐标使用指南

## 概述

`ACruisingJammerActor` 使用相对坐标系统来定义巡航路径点。所有路径点都相对于 Actor 的初始位置，这让路径配置更加直观和可移植。

## 相对坐标系统

- **参考点**：Actor 放置时的初始位置
- **相对坐标**：相对于初始位置的偏移量
- **单位**：厘米（与 UE 一致）

```cpp
// 相对坐标示例
RelativeWaypoints.Add(FVector(0, 0, 0));     // 原点（Actor初始位置）
RelativeWaypoints.Add(FVector(1000, 0, 0));  // 向东移动 10m
RelativeWaypoints.Add(FVector(1000, 1000, 0)); // 向东南移动 10m
RelativeWaypoints.Add(FVector(0, 1000, 0));   // 向南移动 10m
```

## 使用方法

### 1. 在编辑器中配置

在 Details 面板的 "Cruising" 分组中：

```
RelativeWaypoints: Array of FVector[]
[0]: (X: 0, Y: 0, Z: 0)
[1]: (X: 2000, Y: 0, Z: 0)
[2]: (X: 2000, Y: 1000, Z: 0)
[3]: (X: 0, Y: 1000, Z: 0)
```

### 2. 在蓝图/代码中使用

```cpp
// 在 BeginPlay 或适当位置配置
void AMyActor::SetupCruisePath()
{
    // 方式1：添加单个相对坐标点
    AddRelativeWaypoint(FVector(1000.f, 0.f, 0.f));  // 向前10米
    AddRelativeWaypoint(FVector(1000.f, 1000.f, 0.f)); // 向右前10米
    AddRelativeWaypoint(FVector(-1000.f, 0.f, 0.f));  // 向后10米
    AddRelativeWaypoint(FVector(-1000.f, -1000.f, 0.f)); // 向左后10米

    // 方式2：使用世界坐标（会自动转换为相对坐标）
    FVector WorldPosition = FVector(5000, 5000, 100);
    AddWorldWaypoint(WorldPosition);

    // 开始巡航
    StartCruising();
}

// 获取巡航信息
FVector CurrentTarget = GetCurrentWaypointLocation();
TArray<FVector> AllWaypoints = GetAllWaypointLocations();
```

## 常见巡航路径示例

### 示例1：矩形巡逻

```
(0, 0)      (2000, 0)
  •———•———•
  |       |
  |       |
  •———•———•
(0, 1000)  (2000, 1000)
```

```cpp
// 设置矩形巡逻路径
RelativeWaypoints.Add(FVector(0, 0, 0));
RelativeWaypoints.Add(FVector(2000, 0, 0));
RelativeWaypoints.Add(FVector(2000, 1000, 0));
RelativeWaypoints.Add(FVector(0, 1000, 0));
```

### 示例2：圆形巡逻

```
•               •
 \             /
  •———•———•
  /     \
•         •
```

```cpp
// 设置圆形巡逻路径（使用6个点近似圆形）
for (int32 i = 0; i < 6; ++i)
{
    float Angle = (float)i * 360.0f / 6.0f;
    float AngleRad = FMath::DegreesToRadians(Angle);
    float Radius = 1500.0f; // 15米半径

    float X = Radius * FMath::Cos(AngleRad);
    float Y = Radius * FMath::Sin(AngleRad);

    RelativeWaypoints.Add(FVector(X, Y, 0));
}
```

### 示例3：八字形巡逻

```cpp
// 设置八字形巡逻路径
float Radius = 1000.0f;
RelativeWaypoints.Add(FVector(0, -Radius, 0)); // 上
RelativeWaypoints.Add(FVector(Radius, 0, 0));   // 右
RelativeWaypoints.Add(FVector(0, Radius, 0));  // 下
RelativeWaypoints.Add(FVector(-Radius, 0, 0)); // 左
RelativeWaypoints.Add(FVector(0, -Radius, 0)); // 上（形成闭合）
```

### 示例4：高度变化的立体路径

```cpp
// 设置高度变化的巡航路径
RelativeWaypoints.Add(FVector(0, 0, 0));      // 起点地面
RelativeWaypoints.Add(FVector(1000, 0, 500)); // 向上升高5米
RelativeWaypoints.Add(FVector(2000, 0, 0));   // 回到地面
RelativeWaypoints.Add(FVector(1000, 1000, -500)); // 向下进入水中5米
RelativeWaypoints.Add(FVector(0, 0, 0));     // 回到起点
```

## 调试功能

启用以下参数来查看路径：
- `bShowWaypoints = true`：显示航路点和路径
- `bShowDirection = true`：显示当前移动方向

## 注意事项

1. **相对坐标**：所有路径点都会相对于 Actor 的初始位置计算
2. **Actor 移动**：如果 Actor 被移动，相对坐标保持不变
3. **起点检测**：开始巡航时，系统会自动选择最近的航路点作为起点
4. **最小要求**：至少需要 2 个航路点才能开始巡航

## 实际应用示例

### 创建一个海上巡逻舰

```cpp
// 海上巡洋舰巡逻模式
void AWarship::SetupPatrolPattern()
{
    // 设置巡航速度（km/h转换为cm/s）
    float SpeedKmh = 30.0f; // 30 km/h
    float SpeedCmPerSec = SpeedKmh * 100000.0f / 3600.0f; // 833 cm/s
    CruiseSpeed = SpeedCmPerSec;

    // 设置巡逻区域（2km x 2km 正方形海域）
    float PatrolRadius = 1000.0f; // 1km
    RelativeWaypoints.Add(FVector(0, 0, 0));
    RelativeWaypoints.Add(FVector(PatrolRadius, 0, 0));
    RelativeWaypoints.Add(FVector(PatrolRadius, PatrolRadius, 0));
    RelativeWaypoints.Add(FVector(0, PatrolRadius, 0));

    // 设置干扰器参数
    JammerPower = 2.0f;
    bIsJamming = true;

    // 启用调试显示
    bShowWaypoints = true;
    bShowDirection = true;
}
```

这样配置后，舰船会在以初始位置为中心的 2km x 2km 海域内进行矩形巡逻，并持续干扰范围内的目标。