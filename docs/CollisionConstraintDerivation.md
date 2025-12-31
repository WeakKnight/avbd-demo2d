# 碰撞约束推导详解

本文档详细解释 AVBD (Augmented Vertex Block Descent) 中碰撞接触约束的数学推导过程，对应代码文件 `source/manifold.cpp`。

> 参考论文：Augmented VBD (SIGGRAPH 2025)

---

## 目录

1. [问题定义](#1-问题定义)
2. [几何表示](#2-几何表示)
3. [约束函数定义](#3-约束函数定义)
4. [雅可比矩阵推导](#4-雅可比矩阵推导)
5. [泰勒展开线性化](#5-泰勒展开线性化)
6. [摩擦模型](#6-摩擦模型)
7. [代码实现对照](#7-代码实现对照)
8. [数值稳定性考虑](#8-数值稳定性考虑)

---

## 1. 问题定义

### 1.1 碰撞响应的目标

当两个刚体发生碰撞时，我们需要：
1. **防止穿透**：物体不能相互穿过
2. **处理摩擦**：接触面有切向阻力

### 1.2 约束求解框架

在 AVBD 框架中，碰撞被建模为 **不等式约束**：

$$C_n(q) \geq 0 \quad \text{（法向：不穿透）}$$

$$|C_t(q)| \leq \mu \cdot \lambda_n \quad \text{（切向：库仑摩擦）}$$

其中：
- $q$ 是刚体状态向量
- $C_n$ 是法向约束（穿透深度）
- $C_t$ 是切向约束（滑动量）
- $\mu$ 是摩擦系数
- $\lambda_n$ 是法向约束力

---

## 2. 几何表示

### 2.1 刚体状态

2D 刚体的配置空间是 $SE(2)$，用 3 个自由度描述：

$$q = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix}$$

其中 $(x, y)$ 是质心位置，$\theta$ 是旋转角度。

### 2.2 旋转矩阵

2D 旋转矩阵：

$$R(\theta) = \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix}$$

其导数：

$$\frac{dR}{d\theta} = \begin{pmatrix} -\sin\theta & -\cos\theta \\ \cos\theta & -\sin\theta \end{pmatrix} = R(\theta + \frac{\pi}{2})$$

### 2.3 接触点的世界坐标

设接触点在刚体局部坐标系中的位置为 $r$，则其世界坐标为：

$$p^{world} = \begin{pmatrix} x \\ y \end{pmatrix} + R(\theta) \cdot r$$

对于两个接触的刚体 A 和 B：

$$p_A = q_A^{xy} + R(\theta_A) \cdot r_A$$
$$p_B = q_B^{xy} + R(\theta_B) \cdot r_B$$

---

## 3. 约束函数定义

### 3.1 接触坐标系

在接触点建立局部坐标系：
- **法向 $n$**：从 B 指向 A 的单位向量
- **切向 $t$**：垂直于 $n$

在 2D 中，切向可以简单地通过旋转 90° 得到：

$$t = \begin{pmatrix} n_y \\ -n_x \end{pmatrix}$$

### 3.2 基底矩阵

将接触坐标系表示为矩阵：

$$\text{basis} = \begin{pmatrix} n_x & n_y \\ t_x & t_y \end{pmatrix}$$

这个矩阵将世界坐标中的向量转换到接触坐标系。

### 3.3 约束函数

接触约束定义为两个接触点在接触坐标系中的相对位置：

$$C(q) = \text{basis} \cdot (p_A - p_B)$$

展开：

$$C(q) = \text{basis} \cdot \left[ (q_A^{xy} + R_A \cdot r_A) - (q_B^{xy} + R_B \cdot r_B) \right]$$

结果是一个 2D 向量：

$$C = \begin{pmatrix} C_n \\ C_t \end{pmatrix}$$

其中：
- $C_n$：法向分量，表示穿透深度（负值表示穿透）
- $C_t$：切向分量，表示相对滑动量

---

## 4. 雅可比矩阵推导

### 4.1 为什么需要雅可比矩阵

AVBD 使用迭代求解器，需要约束的梯度信息。雅可比矩阵 $J = \frac{\partial C}{\partial q}$ 描述了约束对状态变化的敏感度。

### 4.2 对刚体 A 的导数

状态向量 $q_A = (x_A, y_A, \theta_A)$。

首先计算 $p_A$ 对 $q_A$ 的导数：

$$\frac{\partial p_A}{\partial x_A} = \begin{pmatrix} 1 \\ 0 \end{pmatrix}$$

$$\frac{\partial p_A}{\partial y_A} = \begin{pmatrix} 0 \\ 1 \end{pmatrix}$$

$$\frac{\partial p_A}{\partial \theta_A} = \frac{dR}{d\theta} \cdot r_A = \begin{pmatrix} -r_{Aw,y} \\ r_{Aw,x} \end{pmatrix}$$

其中 $r_{Aw} = R(\theta_A) \cdot r_A$ 是接触点相对质心的世界坐标偏移。

### 4.3 2D 叉积形式

注意到 $\frac{\partial p_A}{\partial \theta_A}$ 可以用 2D 叉积表示：

$$\frac{\partial p_A}{\partial \theta_A} = \begin{pmatrix} -r_{Aw,y} \\ r_{Aw,x} \end{pmatrix}$$

对于任意方向 $d$，有：

$$d \cdot \frac{\partial p_A}{\partial \theta_A} = d_x \cdot (-r_{Aw,y}) + d_y \cdot r_{Aw,x} = r_{Aw} \times d$$

其中 2D 叉积定义为：

$$a \times b = a_x b_y - a_y b_x$$

### 4.4 法向雅可比

对于法向约束 $C_n = n \cdot (p_A - p_B)$：

$$J_A^n = \frac{\partial C_n}{\partial q_A} = \begin{pmatrix} n_x \\ n_y \\ r_{Aw} \times n \end{pmatrix}^T$$

写成行向量形式（与代码中的 float3 对应）：

$$J_A^n = (n_x, n_y, r_{Aw} \times n)$$

### 4.5 切向雅可比

类似地，对于切向约束：

$$J_A^t = (t_x, t_y, r_{Aw} \times t)$$

### 4.6 对刚体 B 的导数

由于约束是 $p_A - p_B$，对 B 的导数符号相反：

$$J_B^n = (-n_x, -n_y, -r_{Bw} \times n)$$
$$J_B^t = (-t_x, -t_y, -r_{Bw} \times t)$$

### 4.7 完整雅可比矩阵

对于单个接触点，完整的雅可比矩阵是 $2 \times 6$ 矩阵（2 个约束，6 个自由度）：

$$J = \begin{pmatrix} J_A^n & J_B^n \\ J_A^t & J_B^t \end{pmatrix}$$

---

## 5. 泰勒展开线性化

### 5.1 非线性问题

约束函数 $C(q)$ 是非线性的，因为：
- 旋转矩阵 $R(\theta)$ 包含三角函数
- 接触点位置依赖于刚体姿态

### 5.2 一阶泰勒展开

在初始状态 $q^-$（时间步开始时的状态）处展开：

$$C(q) \approx C(q^-) + \frac{\partial C}{\partial q}\bigg|_{q^-} \cdot (q - q^-)$$

简记为：

$$C(q) \approx C_0 + J \cdot \Delta q$$

其中：
- $C_0 = C(q^-)$：初始约束值
- $J$：在 $q^-$ 处计算的雅可比矩阵
- $\Delta q = q - q^-$：位置增量

### 5.3 为什么忽略二阶项

对于碰撞接触，AVBD 使用 **截断的泰勒级数**（论文 Section 4）：
1. 接触持续时间短，位移小
2. 二阶项（海森矩阵）计算代价高
3. 多次迭代可以补偿近似误差

### 5.4 Baumgarte 稳定化

为了控制穿透校正速度，引入参数 $\alpha$：

$$C(q) \approx C_0 \cdot (1 - \alpha) + J \cdot \Delta q$$

- $\alpha = 0$：完全校正初始穿透
- $\alpha = 1$：忽略初始穿透，只阻止进一步穿透
- $0 < \alpha < 1$：逐渐校正穿透

---

## 6. 摩擦模型

### 6.1 库仑摩擦

库仑摩擦模型定义摩擦力边界：

$$|f_t| \leq \mu \cdot |f_n|$$

其中：
- $f_t$：切向力（摩擦力）
- $f_n$：法向力
- $\mu$：摩擦系数

### 6.2 组合摩擦系数

两个物体接触时，使用几何平均：

$$\mu = \sqrt{\mu_A \cdot \mu_B}$$

### 6.3 约束边界

在 AVBD 中，摩擦被建模为有界约束：

**法向约束**（单向，只能推不能拉）：
$$-\infty \leq \lambda_n \leq 0$$

**切向约束**（受法向力限制）：
$$-\mu |\lambda_n| \leq \lambda_t \leq \mu |\lambda_n|$$

### 6.4 静摩擦与动摩擦

代码中通过 `stick` 标志区分：

```cpp
contacts[i].stick = abs(lambda[i * 2 + 1]) < frictionBound 
                  && abs(contacts[i].C0.y) < STICK_THRESH;
```

- **静摩擦**：切向力未达到边界，接触点"粘住"
- **动摩擦**：切向力达到边界，接触点滑动

---

## 7. 代码实现对照

### 7.1 初始化阶段 (`initialize`)

```cpp
// 构建接触坐标系基底
float2 normal = contacts[i].normal;
float2 tangent = { normal.y, -normal.x };  // t = R(90°) * n
float2x2 basis = {
    normal.x, normal.y,    // 第一行：法向
    tangent.x, tangent.y   // 第二行：切向
};

// 将局部接触点旋转到世界坐标
float2 rAW = rotate(bodyA->position.z, contacts[i].rA);
float2 rBW = rotate(bodyB->position.z, contacts[i].rB);

// 预计算雅可比矩阵
contacts[i].JAn = { basis[0][0], basis[0][1], cross(rAW, normal) };
contacts[i].JBn = { -basis[0][0], -basis[0][1], -cross(rBW, normal) };
contacts[i].JAt = { basis[1][0], basis[1][1], cross(rAW, tangent) };
contacts[i].JBt = { -basis[1][0], -basis[1][1], -cross(rBW, tangent) };

// 计算初始约束值 C0
contacts[i].C0 = basis * (bodyA->position.xy() + rAW 
                        - bodyB->position.xy() - rBW) 
               + float2{ COLLISION_MARGIN, 0 };
```

**对应公式**：

| 代码 | 数学公式 |
|------|----------|
| `tangent = {n.y, -n.x}` | $t = (n_y, -n_x)$ |
| `rAW = rotate(θ, rA)` | $r_{Aw} = R(\theta_A) \cdot r_A$ |
| `cross(rAW, normal)` | $r_{Aw} \times n$ |
| `JAn = {nx, ny, cross}` | $J_A^n = (n_x, n_y, r_{Aw} \times n)$ |
| `C0 = basis * (pA - pB)` | $C_0 = \text{basis} \cdot (p_A - p_B)$ |

### 7.2 约束计算阶段 (`computeConstraint`)

```cpp
void Manifold::computeConstraint(float alpha)
{
    for (int i = 0; i < numContacts; i++)
    {
        // 计算位置增量
        float3 dpA = bodyA->position - bodyA->initial;
        float3 dpB = bodyB->position - bodyB->initial;
        
        // 线性化约束：C ≈ C0(1-α) + J·Δq
        C[i * 2 + 0] = contacts[i].C0.x * (1 - alpha) 
                     + dot(contacts[i].JAn, dpA) 
                     + dot(contacts[i].JBn, dpB);
        C[i * 2 + 1] = contacts[i].C0.y * (1 - alpha) 
                     + dot(contacts[i].JAt, dpA) 
                     + dot(contacts[i].JBt, dpB);

        // 更新摩擦边界
        float frictionBound = abs(lambda[i * 2 + 0]) * friction;
        fmax[i * 2 + 1] = frictionBound;
        fmin[i * 2 + 1] = -frictionBound;
    }
}
```

**对应公式**：

| 代码 | 数学公式 |
|------|----------|
| `dpA = position - initial` | $\Delta q_A = q_A - q_A^-$ |
| `C0.x * (1-alpha) + dot(JAn, dpA) + dot(JBn, dpB)` | $C_n = C_0^n(1-\alpha) + J_A^n \cdot \Delta q_A + J_B^n \cdot \Delta q_B$ |
| `frictionBound = \|λn\| * μ` | $f_{max}^t = \mu \cdot \|\lambda_n\|$ |

### 7.3 导数计算阶段 (`computeDerivatives`)

```cpp
void Manifold::computeDerivatives(Rigid* body)
{
    for (int i = 0; i < numContacts; i++)
    {
        if (body == bodyA)
        {
            J[i * 2 + 0] = contacts[i].JAn;
            J[i * 2 + 1] = contacts[i].JAt;
        }
        else
        {
            J[i * 2 + 0] = contacts[i].JBn;
            J[i * 2 + 1] = contacts[i].JBt;
        }
    }
}
```

这里只是将预计算的雅可比矩阵复制到 `Force::J` 数组中，供求解器使用。

---

## 8. 数值稳定性考虑

### 8.1 碰撞边界 (COLLISION_MARGIN)

```cpp
contacts[i].C0 = ... + float2{ COLLISION_MARGIN, 0 };
```

添加微小边界（0.0005）防止接触点在边界上抖动。

### 8.2 静摩擦阈值 (STICK_THRESH)

```cpp
contacts[i].stick = ... && abs(contacts[i].C0.y) < STICK_THRESH;
```

当切向滑动小于阈值（0.01）时，认为是静摩擦。

### 8.3 接触持久化

通过 `FeaturePair` 跨帧追踪相同的接触点：

```cpp
if (contacts[i].feature.value == oldContacts[j].feature.value)
{
    // 保留旧的 penalty 和 lambda（暖启动）
    penalty[i * 2 + 0] = oldPenalty[j * 2 + 0];
    lambda[i * 2 + 0] = oldLambda[j * 2 + 0];
}
```

这实现了 **暖启动 (Warm Starting)**，用上一帧的解作为初始猜测，加速收敛。

---

## 总结

碰撞约束的推导流程：

```
1. 几何定义
   └─> 接触点世界坐标 pA, pB

2. 约束函数
   └─> C = basis · (pA - pB)

3. 雅可比矩阵
   └─> J = ∂C/∂q = (n, r×n) 形式

4. 线性化
   └─> C(q) ≈ C0(1-α) + J·Δq

5. 摩擦边界
   └─> |λt| ≤ μ|λn|
```

这套数学框架将非线性碰撞问题转化为迭代可求解的线性约束问题，是 AVBD 算法的核心组成部分。

---

## 参考

- 论文：Augmented Vertex Block Descent (SIGGRAPH 2025)
- 代码：`source/manifold.cpp`, `source/collide.cpp`
- Erin Catto, "Iterative Dynamics with Temporal Coherence", GDC 2005

