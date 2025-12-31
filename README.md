# ESP32 双I²C总线 IMU+OLED 项目

本项目演示如何在ESP32-WROOM-32上使用两个独立的I²C控制器，分别连接MPU6050 IMU传感器和SSD1306 OLED显示屏。

## 🔌 硬件连接

### ESP32引脚分配
```
MPU6050 (I²C总线0)：
├── VCC  → 3.3V
├── GND  → GND  
├── SDA  → GPIO21
└── SCL  → GPIO19

OLED显示屏 (I²C总线1)：
├── VCC  → 3.3V
├── GND  → GND
├── SDA  → GPIO25  
└── SCL  → GPIO26
```

### 接线图说明
```
ESP32-WROOM-32
     ┌─────────────┐
     │             │
     │  GPIO19 ────┼──── SCL (MPU6050)
     │  GPIO21 ────┼──── SDA (MPU6050)  
     │             │
     │  GPIO26 ────┼──── SCL (OLED)
     │  GPIO25 ────┼──── SDA (OLED)
     │             │
     │   3.3V ─────┼──── VCC (两个设备)
     │   GND ─────┼──── GND (两个设备)
     └─────────────┘
```

## 📋 物料清单

| 组件 | 型号/规格 | 数量 | 备注 |
|------|-----------|------|------|
| 微控制器 | ESP32-WROOM-32 | 1 | 主控制器 |
| IMU传感器 | MPU6050 | 1 | 6轴惯性传感器 |
| 显示屏 | SSD1306 OLED 128×64 | 1 | I²C接口 |
| 电阻 | 4.7KΩ | 4 | I²C上拉电阻（可选） |
| 跳线 | 杜邦线 | 若干 | 连接线 |

## 🛠️ 软件环境

### 开发环境
- **IDE**: Visual Studio Code + PlatformIO
- **框架**: Arduino Framework
- **平台**: Espressif 32

### 库依赖
```ini
lib_deps = 
    adafruit/Adafruit SSD1306@^2.5.7
    adafruit/Adafruit GFX Library@^1.11.8
    electroniccats/MPU6050@^1.0.2
    wire
```



## 📈 扩展建议

### 1. 姿态解算
```cpp
// 添加四元数或欧拉角计算
float pitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180/PI;
float roll = atan2(accelY, accelZ) * 180/PI;
```

### 2. 数据滤波
```cpp
// 添加低通滤波或卡尔曼滤波
class LowPassFilter {
private:
    float alpha;
    float filteredValue;
public:
    LowPassFilter(float cutoff) : alpha(cutoff), filteredValue(0) {}
    float update(float input) {
        filteredValue = alpha * input + (1 - alpha) * filteredValue;
        return filteredValue;
    }
};
```



## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 🤝 贡献

欢迎提交 Pull Request 和 Issue！

## 📧 联系方式

如有问题，请提交 GitHub Issue 或发送邮件。

---

**项目状态**: ✅ 稳定版本  
**最后更新**: 2024-12-23  
**兼容性**: ESP32-WROOM-32, Arduino Framework
