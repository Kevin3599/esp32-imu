# ESP32 双I²C总线 IMU+OLED 项目

本项目演示如何在ESP32-WROOM-32上使用两个独立的I²C控制器，分别连接MPU6050 IMU传感器和SSD1306 OLED显示屏。

## 🎯 项目特点

- ✅ **硬件I²C**：使用ESP32的两个独立硬件I²C控制器，性能稳定
- ✅ **双总线设计**：避免I²C地址冲突，提高系统可靠性  
- ✅ **实时显示**：OLED屏幕实时显示IMU数据
- ✅ **结构化代码**：清晰的代码结构，便于扩展和维护
- ✅ **完整数据处理**：包含加速度、角速度和温度数据读取

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

## 🚀 快速开始

### 1. 环境准备
```bash
# 安装PlatformIO Core
pip install platformio

# 或使用VS Code扩展
# 在VS Code中搜索并安装"PlatformIO IDE"扩展
```

### 2. 项目编译
```bash
# 进入项目目录
cd esp32-imu

# 编译项目
pio run

# 上传到ESP32
pio run --target upload

# 监视串口输出
pio device monitor
```

### 3. 预期输出

**串口输出示例：**
```
ESP32双I²C总线IMU+OLED项目启动...
初始化I²C总线0 (MPU6050)...
初始化I²C总线1 (OLED)...  
初始化MPU6050传感器...
✓ MPU6050连接成功
✓ MPU6050配置完成
初始化OLED显示屏...
✓ OLED初始化成功
系统初始化完成
开始数据采集和显示...

=================== IMU数据 ===================
加速度 (g):   X=0.012, Y=-0.034, Z=0.987
角速度 (°/s): X=0.45, Y=-0.23, Z=0.12
温度: 28.5°C
合成加速度: 0.988g
===============================================
```

**OLED显示内容：**
```
ESP32 IMU 数据
加速度 (g):
X:0.01 Y:-0.03
Z:0.99
角速度 (°/s):
X:0.5 Y:-0.2 Z:0.1
```

## 🔧 关键设计选择

### 为什么使用双I²C总线？

1. **避免地址冲突**
   - MPU6050默认地址：0x68
   - SSD1306默认地址：0x3C
   - 虽然地址不同，但使用双总线可以提高系统稳定性

2. **提高通信效率**
   - 两个设备可以并行通信
   - 减少总线竞争和延迟

3. **增强系统可靠性**
   - 单个设备故障不会影响另一个总线
   - 更好的电气隔离

4. **便于扩展**
   - 每个总线可以独立添加更多设备
   - 更灵活的系统架构

### 代码架构特点

- **模块化设计**：各功能模块独立，便于维护
- **错误检测**：包含设备连接检测和错误处理
- **实时性能**：100ms更新间隔，保证实时性
- **数据处理**：包含物理单位转换和滤波

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

### 3. 数据记录
```cpp
// 添加SD卡或Flash存储
#include <SD.h>
#include <SPI.h>

void logDataToSD() {
    File dataFile = SD.open("imu_data.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                       millis(), accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
        dataFile.close();
    }
}
```

### 4. 无线传输
```cpp
// 添加WiFi或蓝牙数据传输
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

void handleIMUData() {
    String json = "{\"accel\":{\"x\":" + String(accelX) + 
                  ",\"y\":" + String(accelY) + 
                  ",\"z\":" + String(accelZ) + "}}";
    server.send(200, "application/json", json);
}
```

## 🐛 故障排除

### 常见问题

1. **MPU6050连接失败**
   - 检查I²C总线0接线（GPIO19, GPIO21）
   - 确认MPU6050供电正常（3.3V）
   - 检查是否需要上拉电阻

2. **OLED显示异常**
   - 检查I²C总线1接线（GPIO25, GPIO26）
   - 确认OLED地址是否为0x3C
   - 检查显示屏型号是否为SSD1306

3. **数据读取异常**
   - 检查I²C时钟频率设置（400kHz）
   - 确认设备初始化顺序正确
   - 检查电源稳定性

### 调试技巧

```cpp
// I²C设备扫描代码
void scanI2CDevices(TwoWire &wire) {
    Serial.println("扫描I²C设备...");
    for (byte address = 1; address < 127; address++) {
        wire.beginTransmission(address);
        if (wire.endTransmission() == 0) {
            Serial.printf("发现设备地址: 0x%02X\n", address);
        }
    }
}
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