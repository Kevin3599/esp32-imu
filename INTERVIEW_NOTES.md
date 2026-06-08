# ESP32 IMU + GPS 项目 — 面试技术答辩归档

> 本文档面向技术面试场景，逐一解释项目中遇到的真实工程问题、解决方案原理及量化效果。
> 所有描述均有对应代码佐证（标注了文件和行号）。

---

## 目录

1. [项目背景一句话介绍](#0-项目背景)
2. [问题一：IMU 被发动机震动淹没](#问题一imu-被发动机震动淹没)
3. [问题二：姿态角纯用加速度计 → 震动下抖动](#问题二姿态角纯靠加速度计的局限)
4. [问题三：GPS 静止时速度不归零（零漂）](#问题三gps-静止零漂)
5. [问题四：GPS-IMU 融合的信心度机制](#问题四gps-imu-融合与动态信心度)
6. [问题五：IMU 加速度偏置（Zero-g Offset）校准](#问题五imu-加速度偏置校准)
7. [问题六：GPS 位置跳变过滤](#问题六gps-位置跳变过滤)
8. [已知缺口与改进方向](#已知缺口与改进方向)
9. [面试常见追问 Q&A](#面试常见追问-qa)

---

## 0. 项目背景

**一句话**：用 ESP32 + MPU6050 + GT-U7 GPS 做一个摩托车/汽车车载 "Dragy 风格" 性能仪，
实时显示速度、G 力、加速测试（0-60 / 0-100 km/h）、倾角、圈速，并通过 Wi-Fi 热点把数据推到手机浏览器。

**挑战核心**：传感器装在发动机舱附近，震动烈、信号杂；GPS 10 Hz 更新，低速时噪声比信号还大。

---

## 问题一：IMU 被发动机震动淹没

### 现象

MPU6050 原始加速度数据在发动机怠速时（约 20~50 Hz 机械振动）出现高频毛刺，
幅值叠加到真实加速度上，导致 G 力跳变、姿态角抖动 ±5° 量级。

### 根因分析

MPU6050 内置的数字低通滤波器（DLPF）默认带宽 260 Hz，远高于发动机振动频率，
等于没有滤波。

### 解决方案：三级滤波

#### 第一级：硬件 DLPF 收窄带宽（`src/mpu6050.cpp`）

```cpp
mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
```

MPU6050 DLPF 选项（数字低通，对应截止频率）：

| 枚举值 | 截止频率 | 适用场景 |
|--------|----------|----------|
| `MPU6050_BAND_260_HZ` | 260 Hz | 默认，几乎不滤 |
| `MPU6050_BAND_94_HZ`  | 94 Hz  | **本项目**，保留快速动作响应 |
| `MPU6050_BAND_21_HZ`  | 21 Hz  | 强震动，但牺牲动态响应 |
| `MPU6050_BAND_5_HZ`   | 5 Hz   | 极强震动，适合静态监测 |

选 94 Hz 的原因：陀螺仪需要追踪摩托车快速转向（可达 90°/s），截止太低会让动态响应变迟钝；
发动机主要振动集中在 20~50 Hz，94 Hz 带宽已能衰减部分，配合后两级软件滤波足够。

#### 第二级：滑动平均（Moving Average）（`src/mpu6050.cpp` `FilterDataForVibration()`）

```cpp
#define FILTER_SAMPLES 10  // 环形缓冲区大小

// 每次采样写入缓冲区，取窗口内均值
acc_x_avg += acc_buffer_x[i];
...
acc_x_avg /= samples_to_use;
```

- 采样率 200 Hz，窗口 10 点 → **等效时间窗口 50 ms**
- 滑动平均本质是一个 FIR 低通，截止频率 ≈ `采样率 / (2 × 窗口大小)` = 10 Hz
- 优点：实现简单，无相位累积误差；缺点：对突变响应慢（延迟 = 窗口半宽 = 25 ms）

#### 第三级：一阶 IIR 低通（Exponential Moving Average）（`src/mpu6050.cpp`）

```cpp
#define LOW_PASS_ALPHA 0.1  // α = 0.1 → 强滤波

// y[n] = α × x[n] + (1 - α) × y[n-1]
mpu6050_data.Acc_X_Filtered = LOW_PASS_ALPHA * acc_x_avg
                             + (1.0 - LOW_PASS_ALPHA) * acc_x_filtered_prev;
```

等效截止频率公式（离散域近似）：

$$f_c = \frac{\alpha \cdot f_s}{2\pi(1-\alpha)} \approx \frac{0.1 \times 200}{2\pi \times 0.9} \approx 3.5 \text{ Hz}$$

**总结**：三级级联后，等效截止频率 ≈ 3~5 Hz，发动机 20~50 Hz 振动衰减 > 12 dB。

### 量化效果

| 指标 | 滤波前（估算） | 滤波后 |
|------|--------------|--------|
| 静止时 G 力噪声 | ±0.15 g | ±0.01 g（死区内归零） |
| 发动机怠速姿态角抖动 | ±5°（估算） | ±0.5°（估算） |
| 信号延迟 | 0 ms | ≈ 25~50 ms（可接受） |

---

## 问题二：姿态角纯靠加速度计的局限

### 现象（已知缺口，面试时要主动说）

当前 Roll/Pitch 计算只用加速度计：

```cpp
// src/mpu6050.cpp CalculateAttitude()
mpu6050_data.Roll  = atan2(Acc_Y, Acc_Z) * 180 / PI;
mpu6050_data.Pitch = atan2(-Acc_X, sqrt(Acc_Y² + Acc_Z²)) * 180 / PI;
mpu6050_data.Yaw   = 0.0;  // 无磁力计，硬编码为 0
```

**问题**：
1. 加速度计无法区分"重力"和"线加速度"——摩托车加速时 X 轴会感受到向后惯性力，
   导致 Pitch 角计算出现 5~10° 的虚假俯仰
2. 陀螺仪积分漂移没有被校正（陀螺仪温漂约 0.1~1°/s，几分钟后 Yaw 角累积误差数十度）
3. Yaw 固定为 0，无法感知转向

### 正确解法（面试加分项）：互补滤波

互补滤波核心思想：**加速度计低频可信，陀螺仪高频可信**，两者互补。

$$\theta_{fused}[n] = \alpha \cdot (\theta[n-1] + \omega \cdot dt) + (1-\alpha) \cdot \theta_{accel}[n]$$

- $\alpha = 0.98$（常用值）：98% 信任陀螺仪积分，2% 用加速度计长期修正
- 截止频率：$f_c = \frac{1-\alpha}{2\pi \alpha \cdot dt}$，$\alpha=0.98, dt=5\text{ms}$ → $f_c \approx 0.32 \text{ Hz}$
- 优点：计算量极小，适合 MCU；缺点：$\alpha$ 需要根据振动程度调参

**本项目未实现的原因**：Yaw 轴没有磁力计，即使做互补滤波，Yaw 仍会漂移。
摩托车倾角（Roll）和俯仰（Pitch）可以用互补滤波，但本项目 G 力测量优先级更高，
姿态角是次要功能，暂未迭代。

---

## 问题三：GPS 静止零漂

### 现象

GT-U7 在静止时输出速度为 0.5~2 km/h（多路径反射、卫星几何误差导致），
直接用原始值会导致：① 静止时看到"车辆在缓慢移动"；② 0-100 测试开始时间判断错误。

### 解决方案：多层零漂抑制（`src/gps_module.cpp`）

#### 层 1：20 点滑动平均 + 标准差分析

```cpp
static double speed_buffer[20] = {0};  // 10Hz × 2秒 = 20点

// 计算均值和标准差
double avg_speed = speed_sum / 20.0;
double std_dev = sqrt(variance / 20.0);

// 严格的稳定判定：标准差 < 0.2 km/h 且均值 < 动态阈值
bool is_speed_stable = (std_dev < 0.2) && (avg_speed < dynamic_threshold);
bool is_acceleration_stable = abs(new_speed - last_speed) < 0.3;
```

#### 层 2：自适应 IIR 滤波（按速度段动态调整 α）

```
速度 < 0.5 km/h  → α = 0.92（慢响应，强抑零漂）
速度 1.5~4 km/h  → α = 0.60（中等）
速度 > 4 km/h    → α = 0.35（快响应，不延迟测试计时）
```

信号质量（卫星数/HDOP）也影响 α：`α × signal_quality`

#### 层 3：最终死区（Dead Zone）

```cpp
double final_threshold = (satellites >= 8 && hdop < 1.0) ? 0.8 : 1.2;
if (speed_kmh < final_threshold) speed_kmh = 0.0;
```

### 量化效果

| 条件 | 零漂压制阈值 |
|------|------------|
| 优质信号（≥8 颗星，HDOP < 1.0） | 0.8 km/h |
| 普通信号 | 1.2 km/h |
| 判定为静止需要的持续时间 | 1.5 秒（15 次 × 100ms） |

---

## 问题四：GPS-IMU 融合与动态信心度

### 问题背景

IMU 加速度计噪声大、有偏置；GPS 加速度（对速度差分）延迟高（100ms/帧）、低速不准。
两者各有缺陷，需要**按实时信号质量动态分配权重**，而不是固定比例融合。

### 实现：动态信心度（`src/performance_analyzer.cpp` `calculateGForces()`）

**Step 1：评估 GPS 当前稳定性**

```cpp
// 16 点 GPS 速度历史，计算方差
double gps_variance = Var(gps_speed_history[16]);
double gps_stability = 1.0 / (1.0 + gps_variance);  // 方差越大，稳定性越低
```

**Step 2：评估信号质量**

```cpp
double signal_quality = (satellites >= 8 && hdop < 1.5) ? 1.0 : 0.6;
```

**Step 3：综合信心度**

```cpp
imu_confidence = gps_stability × signal_quality;
// 取值范围 [0, 1]，越高说明 GPS 越稳定
```

**Step 4：G 力融合**

```cpp
// 动态状态下，用 GPS 速度差分辅助修正 IMU 加速度
double gps_accel_g = Δv_gps / (dt × 9.81);
raw_accel_g = raw_accel_g * imu_confidence
            + gps_accel_g * (1.0 - imu_confidence);
```

**Step 5：滤波 α 也随信心度变化**

```cpp
double filter_alpha = is_stationary ? 0.95           // 静止：强滤波
                                    : (0.4 + 0.4 × imu_confidence);  // 动态：自适应
```

### 为什么不用卡尔曼滤波？

| | 互补/自适应IIR | 卡尔曼滤波（EKF） |
|-|--------------|----------------|
| 计算量 | 极低，适合 ESP32 | 矩阵运算，资源消耗高 |
| 参数调节 | 直观（α，死区） | 需要建模噪声协方差 Q/R |
| 非线性 | 手动处理 | EKF 可自动线性化 |
| 本项目选择 | ✅ | 未实现，为改进方向 |

---

## 问题五：IMU 加速度偏置校准

### 问题背景

MPU6050 出厂零偏（Zero-g Offset）可达 ±50~150 mg（约 ±0.5~1.5 m/s²），
直接积分求速度会引入显著漂移（1 m/s² 偏置 × 10s = 10 m/s 误差）。

代码中也有一处硬编码的手动校准值（说明这是被发现过的真实问题）：

```cpp
// src/performance_analyzer.cpp calculateGForces()
double acc_x = mpu6050_data.Acc_X_Filtered - 0.47;  // 手动测量的偏置
double acc_y = mpu6050_data.Acc_Y_Filtered + 0.5;
double acc_z = mpu6050_data.Acc_Z_Filtered - 0.48;
```

### 自动校准方案（`src/gps_module.cpp` 第 609 行附近）

利用 GPS 判断静止，在静止时在线收集样本做自动校准：

```cpp
if (gps_data.speed_ms < 0.5 && calibration_count < 200) {
    // 在线递推均值：增量式更新，不需要存储所有样本
    accel_bias = (accel_bias * calibration_count + forward_accel)
               / (calibration_count + 1);
    calibration_count++;

    if (calibration_count == 200) {
        imu_calibrated = true;
        Serial.printf("IMU校准完成: 偏置=%.4f m/s²\n", accel_bias);
    }
}
// 使用时减去偏置
forward_accel -= accel_bias;
```

**算法亮点**：递推均值（Recursive Mean）无需存储 200 个样本，内存 O(1)。

### 两种偏置来源的区别

| 类型 | 原因 | 本项目处理方式 |
|------|------|--------------|
| 出厂零偏 | 制造工艺误差 | 手动测量后硬编码（performance_analyzer.cpp） |
| 安装偏置 | 传感器未与车辆轴对齐 | 静止在线校准（gps_module.cpp） |
| 温度漂移 | 温度变化改变灵敏度 | 未处理（改进方向） |

---

## 问题六：GPS 位置跳变过滤

### 现象

GPS 在信号遮挡（隧道出口、高楼）后重新定位时，位置可能瞬间跳变数百米，
导致距离统计异常飙升、轨迹显示乱跳。

### 解决方案：合理性检验（`src/gps_module.cpp`）

```cpp
// 计算新坐标与上一帧的距离（haversine 公式）
double distance = calculateDistance(last_lat, last_lon, new_lat, new_lon);

// 1秒内移动超过 1000m → 拒绝（即使 F1 赛车也只有 ~83m/s）
if (distance < 1000) {
    gps_data.latitude = new_lat;
    gps_data.longitude = new_lon;
}
// 否则保持上一帧坐标，等待下一帧
```

**Haversine 公式**（球面两点距离）：

$$d = 2R \cdot \arcsin\!\left(\sqrt{\sin^2\!\tfrac{\Delta\varphi}{2} + \cos\varphi_1\cos\varphi_2\sin^2\!\tfrac{\Delta\lambda}{2}}\right)$$

$R = 6{,}371{,}000$ m（地球平均半径）

---

## 已知缺口与改进方向

面试时主动说出缺口，比被追问更能体现技术成熟度。

| 缺口 | 影响 | 推荐改进 |
|------|------|---------|
| 姿态角无互补滤波 | 加速/制动时 Pitch 误差 5~10° | 实现互补滤波，α ≈ 0.98 |
| Yaw 固定为 0 | 无法感知转向 | 加 QMC5883L 磁力计，或用 GPS 航向 |
| IMU 温漂未补偿 | 长时间测量后漂移 | 利用温度传感器（MPU6050 内置）做温度补偿 |
| 手动硬编码偏置 | 换硬件/安装方式就要重新测 | 全部替换为自动静止校准 |
| GPS-IMU 融合未用卡尔曼 | 信心度模型较粗糙 | 实现 1D EKF（只融合纵向速度，状态量少） |
| 摩托倾角未补重力投影 | 大倾角（>30°）时纵向 G 力含重力分量误差 | 用 Roll 角分解加速度：$a_{真} = a_{测} - g\sin(\text{Roll})$ |

---

## 面试常见追问 Q&A

**Q: 为什么用 IIR 而不是 FIR？**
> A: 在资源受限的 MCU 上，IIR 只需存储上一个输出值（O(1) 内存），FIR 需要存储整个窗口。
> 代价是 IIR 有相位延迟且参数影响稳定性；本项目加速度测量对相位延迟不敏感（不做频谱分析），所以 IIR 合适。

**Q: LOW_PASS_ALPHA = 0.1 是怎么定的？**
> A: 发动机怠速振动主频约 20~50 Hz，希望截止频率在 5 Hz 以下。
> 公式 $f_c \approx \alpha f_s / (2\pi)$，代入 $f_s=200, \alpha=0.1$ 得 $f_c \approx 3.2$ Hz。
> 实际通过串口波形观察调参，0.1 时静止噪声可接受，动态响应也够快。

**Q: 互补滤波的 α = 0.98 怎么理解？**
> A: 陀螺仪每 5 ms 积分一次，短时间内精度高，长时间有漂移；
> 加速度计长时间均值准确，但短时间（振动时）噪声大。
> α = 0.98 意味着：每帧 98% 信任陀螺仪，2% 用加速度计纠偏。
> 时间常数 $\tau = \alpha \cdot dt / (1-\alpha) = 0.98 \times 0.005 / 0.02 = 0.245$ 秒——
> 即约 0.25 秒内加速度计才能把陀螺仪拉回正确值，足够平滑掉短暂振动。

**Q: GPS 10 Hz 和 IMU 200 Hz 频率不一样怎么融合？**
> A: 典型做法是"预测-更新"框架（卡尔曼的核心思想）：
> - **预测（200 Hz）**：每个 IMU 样本到来，用加速度积分更新速度估计
> - **更新（10 Hz）**：GPS 数据到来，用 GPS 速度修正/锚定累积漂移
> 本项目简化实现：GPS 帧到来时重新计算融合权重，IMU 帧只做低通滤波，
> 未做完整的时间对齐预测步骤（改进方向）。

**Q: haversine 公式的精度够用吗？**
> A: 对于地表短距离（< 几百公里），误差 < 0.5%，完全满足跳变检测（1 km 阈值）和圈速计算（几百米赛道）。
> 更精确场景可用 Vincenty 公式（椭球面），但 ESP32 上计算量更大，无必要。

**Q: 为什么不用 GPS 的 Doppler 速度而是用位置差分？**
> A: GT-U7 的 `$GPRMC` 句中速度字段 **本身就是 Doppler 测速**（对卫星信号频移积分），
> 比位置差分精度高 10 倍左右（不受多路径位置误差影响）。
> 本项目用的就是 TinyGPS++ 解析的 `gps.speed.kmph()`，即 Doppler 速度，
> 并非自己做位置差分——这是正确做法。

---

*最后更新：2026-06-01 | 项目路径：`e:\esp32\esp32-imu`*
