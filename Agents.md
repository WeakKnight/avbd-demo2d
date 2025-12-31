# AVBD-Demo2D 项目开发指南

## 项目概述

这是一个 **Augmented Vertex Block Descent (AVBD)** 算法的 2D 实现演示项目。AVBD 是一种用于物理模拟的约束求解算法，适用于刚体动力学、关节系统、碰撞检测等场景。

- **项目主页**: https://graphics.cs.utah.edu/research/projects/avbd/
- **作者**: Chris Giles
- **许可证**: 自由使用许可

---

## 技术栈

| 技术 | 版本/说明 |
|------|----------|
| C++ | C++17 标准 |
| CMake | 3.13+ |
| SDL2 | 窗口管理、输入处理 |
| OpenGL | 图形渲染 (Legacy OpenGL) |
| ImGui | 即时模式 GUI |
| Emscripten | Web 平台支持 (WebGL2) |

---

## 目录结构

```
avbd-demo2d/
├── CMakeLists.txt          # CMake 构建配置
├── README.md               # 项目说明
├── LICENSE                 # 许可证
├── Agents.md               # 开发指南 (本文件)
├── source/                 # 源代码目录
│   ├── main.cpp            # 应用程序入口
│   ├── solver.h            # 求解器头文件 (核心类定义)
│   ├── solver.cpp          # 求解器实现
│   ├── maths.h             # 数学库 (向量、矩阵、运算)
│   ├── scenes.h            # 预定义场景
│   ├── rigid.cpp           # 刚体实现
│   ├── force.cpp           # 力/约束基类
│   ├── joint.cpp           # 关节约束
│   ├── spring.cpp          # 弹簧约束
│   ├── motor.cpp           # 电机约束
│   ├── manifold.cpp        # 碰撞流形
│   ├── collide.cpp         # 碰撞检测
│   └── shell.html          # Web 版本 HTML 模板
└── external/               # 外部依赖
    ├── SDL/                # SDL2 库
    └── imgui/              # Dear ImGui 库
```

---

## 核心架构

### 1. 求解器 (Solver)

`Solver` 是模拟的核心类，负责：
- 管理所有刚体 (`Rigid*`) 链表
- 管理所有约束/力 (`Force*`) 链表
- 执行物理模拟步进 (`step()`)
- 碰撞检测与处理

**关键参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| `dt` | float | 时间步长 |
| `gravity` | float | 重力加速度 |
| `iterations` | int | 求解器迭代次数 |
| `alpha` | float | 稳定化参数 |
| `beta` | float | 惩罚递增参数 |
| `gamma` | float | 暖启动衰减参数 |
| `postStabilize` | bool | 是否启用后稳定化 |

### 2. 刚体 (Rigid)

表示 2D 刚体，支持：
- 位置、速度、旋转
- 质量、转动惯量
- 摩擦系数
- 尺寸 (矩形)

### 3. 约束系统 (Force 继承体系)

```
Force (基类)
├── Joint          # 旋转关节 + 角度约束
├── Spring         # 弹簧力
├── Motor          # 电机 (角速度驱动)
├── Manifold       # 碰撞接触流形
└── IgnoreCollision # 忽略碰撞标记
```

每个约束必须实现：
- `rows()` - 约束维度 (最大 4)
- `initialize()` - 初始化约束
- `computeConstraint(alpha)` - 计算约束值
- `computeDerivatives(body)` - 计算雅可比矩阵/海森矩阵
- `draw()` - 调试绘制 (可选)

### 4. 数学库 (maths.h)

自定义数学类型：
- `float2` - 2D 向量
- `float3` - 3D 向量 (x, y, θ 用于 2D 刚体状态)
- `float2x2` - 2x2 矩阵
- `float3x3` - 3x3 矩阵

常用函数：
- 向量运算: `dot()`, `cross()`, `length()`, `lengthSq()`
- 矩阵运算: `rotation()`, `transform()`, `outer()`, `solve()`

---

## 场景系统

在 `scenes.h` 中定义了 19 个预设场景：

| 场景名 | 说明 |
|--------|------|
| Empty | 空场景 |
| Ground | 仅地面 |
| Dynamic Friction | 动摩擦演示 |
| Static Friction | 静摩擦演示 |
| Pyramid | 金字塔堆叠 |
| Cards | 纸牌屋 |
| Rope | 绳索链 |
| Heavy Rope | 重物绳索 |
| Hanging Rope | 悬挂绳索 |
| Spring | 弹簧演示 |
| Spring Ratio | 弹簧刚度比 |
| Stack | 方块堆叠 |
| Stack Ratio | 不同大小堆叠 |
| Rod | 刚性杆 |
| Soft Body | 软体 |
| Joint Grid | 关节网格 |
| Net | 网状结构 |
| Motor | 电机驱动 |
| Fracture | 断裂演示 |

**添加新场景**:
1. 在 `scenes.h` 中定义场景函数
2. 将函数添加到 `scenes[]` 数组
3. 将场景名称添加到 `sceneNames[]` 数组
4. 更新 `sceneCount`

---

## 构建指南

### Windows Native

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
# 运行: Release/avbd_demo2d.exe
```

### Web (Emscripten)

```powershell
# 安装 Emscripten 和 Ninja
winget install Ninja-build.Ninja

mkdir build-web
cd build-web
emcmake cmake ..
ninja
# 打开 avbd_demo2d.html
```

---

## 用户交互

### 桌面控制
| 操作 | 快捷键 |
|------|--------|
| 移动相机 | W/A/S/D 或 中键拖动 |
| 缩放相机 | Q/E 或 滚轮 |
| 创建方块 | 右键点击 |
| 拖动物体 | 左键拖动 |
| 全屏切换 | Alt + Enter |
| 退出 | Escape |

### 触屏控制
| 操作 | 手势 |
|------|------|
| 移动相机 | 双指拖动 |
| 缩放相机 | 双指捏合 |
| 创建方块 | 双击 |
| 拖动物体 | 点击并拖动 |

---

## 开发规范

### 代码风格
- 使用 C++17 特性
- 头文件使用 `#pragma once`
- 类成员变量不使用前缀
- 使用自定义数学类型 (float2, float3 等)

### 内存管理
- 刚体和约束通过 `new` 创建，自动注册到 Solver
- 析构时自动从 Solver 链表中移除
- `solver->clear()` 清除所有对象

### 添加新约束类型
1. 在 `solver.h` 中声明新类，继承自 `Force`
2. 实现所有纯虚函数
3. 创建对应的 `.cpp` 文件

示例：
```cpp
struct MyConstraint : Force
{
    MyConstraint(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    
    int rows() const override { return 1; }
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    void draw() const override;
};
```

### 常量定义
```cpp
#define MAX_ROWS 4                // 单个约束最大行数
#define PENALTY_MIN 1.0f          // 最小惩罚参数
#define PENALTY_MAX 1000000000.0f // 最大惩罚参数
#define COLLISION_MARGIN 0.0005f  // 碰撞边界余量
#define STICK_THRESH 0.01f        // 静摩擦阈值
#define SHOW_CONTACTS true        // 显示接触点
```

---

## 常见开发任务

### 1. 修改物理参数
编辑 `main.cpp` 中的全局变量或通过 ImGui 界面调整

### 2. 添加新形状
当前仅支持矩形，添加新形状需要：
- 修改 `Rigid` 结构体
- 修改 `collide.cpp` 中的碰撞检测
- 修改 `rigid.cpp` 中的绘制逻辑

### 3. 优化性能
- 增加空间哈希/四叉树进行宽相碰撞
- 使用 SIMD 优化数学运算
- 并行化约束求解

### 4. 导出/保存场景
当前不支持，需要添加序列化功能

---

## 文件详细说明

| 文件 | 行数约 | 功能 |
|------|--------|------|
| `main.cpp` | 420 | 入口、SDL/OpenGL/ImGui 初始化、主循环、UI |
| `solver.h` | 230 | 核心类型定义 (Solver, Rigid, Force 及其派生类) |
| `solver.cpp` | - | 求解器实现、AVBD 算法核心 |
| `maths.h` | 350 | 向量/矩阵类型及运算 |
| `scenes.h` | 360 | 19 个预定义场景 |
| `rigid.cpp` | - | 刚体实现 |
| `force.cpp` | - | 约束基类 |
| `joint.cpp` | - | 关节约束 |
| `spring.cpp` | - | 弹簧约束 |
| `motor.cpp` | - | 电机约束 |
| `manifold.cpp` | - | 碰撞流形处理 |
| `collide.cpp` | - | OBB-OBB 碰撞检测 |

---

## 调试技巧

1. **启用接触点显示**: `SHOW_CONTACTS` 设为 `true`
2. **暂停模拟**: UI 中勾选 "Pause"，使用 "Step" 单步执行
3. **调整求解器参数**: 通过 UI 滑块实时调整 `alpha`, `beta`, `gamma`
4. **查看物体状态**: 使用 ImGui 添加调试窗口

---

## 扩展建议

- [ ] 添加圆形/多边形碰撞支持
- [ ] 实现场景保存/加载
- [ ] 添加性能统计面板
- [ ] 支持鼠标多选
- [ ] 添加物体属性编辑器
- [ ] 实现撤销/重做功能
- [ ] 添加更多约束类型 (滑轮、距离约束等)

---

## 参考资源

- [AVBD 论文与项目主页](https://graphics.cs.utah.edu/research/projects/avbd/)
- [SDL2 文档](https://wiki.libsdl.org/)
- [Dear ImGui 文档](https://github.com/ocornut/imgui)
- [Erin Catto - 物理引擎资料](https://box2d.org/publications/)

