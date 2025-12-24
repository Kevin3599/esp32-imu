# 项目编译和使用指南

## 📋 环境要求

### 1. 安装VS Code和PlatformIO扩展

1. 下载并安装 [Visual Studio Code](https://code.visualstudio.com/)
2. 在VS Code中安装PlatformIO扩展：
   - 打开VS Code
   - 按Ctrl+Shift+X打开扩展面板
   - 搜索"PlatformIO IDE"
   - 点击安装

### 2. 打开项目

1. 在VS Code中选择 **File → Open Folder**
2. 选择项目目录：`esp32 imu`
3. PlatformIO会自动识别项目并下载依赖

## 🔧 编译步骤

### 方法1：使用VS Code界面（推荐）

1. 打开项目后，在底部状态栏会显示PlatformIO图标
2. 点击底部的"✓"图标进行编译
3. 或者使用快捷键：Ctrl+Alt+B

### 方法2：使用PlatformIO命令面板

1. 按Ctrl+Shift+P打开命令面板
2. 输入"PlatformIO: Build"
3. 选择并执行

### 方法3：使用PlatformIO终端

1. 在VS Code中打开终端（Ctrl+`）
2. 执行命令：
```bash
# 如果已安装PlatformIO CLI
pio run

# 或者使用完整命令
platformio run
```

## 📤 上传到ESP32

### 准备工作

1. **连接ESP32**：使用USB数据线连接ESP32到电脑
2. **确认端口**：在设备管理器中查看COM端口号
3. **驱动安装**：确保ESP32驱动已正确安装

### 上传步骤

1. **VS Code界面方式**：
   - 点击底部状态栏的"→"(上传)图标
   - 或使用快捷键：Ctrl+Alt+U

2. **命令行方式**：
```bash
# 编译并上传
pio run --target upload

# 指定端口上传（如果检测不到）
pio run --target upload --upload-port COM3
```

## 🖥️ 串口监控

### 查看输出数据

1. **VS Code方式**：
   - 点击底部状态栏的"🔌"(串口监视器)图标
   - 波特率会自动设置为115200

2. **命令行方式**：
```bash
# 监视串口输出
pio device monitor

# 指定端口和波特率
pio device monitor --port COM3 --baud 115200
```

### 预期输出示例

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

## 🔍 故障排除

### 编译错误

1. **库依赖问题**：
   - 确保`platformio.ini`中的库版本正确
   - 清除缓存：PlatformIO → Miscellaneous → Clean

2. **版本冲突**：
   - 删除`.pio`文件夹
   - 重新编译项目

### 上传失败

1. **端口问题**：
   - 检查USB连接
   - 确认端口号正确
   - 尝试不同的USB线

2. **ESP32进入下载模式**：
   - 按住BOOT按键，然后按RESET按键
   - 松开RESET，再松开BOOT
   - 重试上传

### 运行异常

1. **设备连接检查**：
   - 验证所有接线正确
   - 检查电源供应（3.3V）
   - 测试I²C设备地址

2. **代码调试**：
   - 检查串口输出中的错误信息
   - 使用I²C扫描功能查找设备

## 🛠️ 开发技巧

### 1. 实时调试

在`main.cpp`中添加调试代码：
```cpp
// I²C设备扫描
void scanI2CDevices(TwoWire &wire, const char* busName) {
    Serial.printf("扫描%s总线设备...\n", busName);
    int deviceCount = 0;
    for (byte address = 1; address < 127; address++) {
        wire.beginTransmission(address);
        if (wire.endTransmission() == 0) {
            Serial.printf("发现设备地址: 0x%02X\n", address);
            deviceCount++;
        }
    }
    Serial.printf("总共发现 %d 个设备\n", deviceCount);
}

// 在setup()中调用
scanI2CDevices(I2C0, "I²C0");
scanI2CDevices(I2C1, "I²C1");
```

### 2. 性能监控

添加性能监控代码：
```cpp
void loop() {
    unsigned long startTime = micros();
    
    // 原有代码...
    readIMUData();
    updateDisplay();
    
    unsigned long endTime = micros();
    Serial.printf("循环执行时间: %lu微秒\n", endTime - startTime);
}
```

### 3. 错误恢复

添加自动重连机制：
```cpp
bool reconnectMPU6050() {
    Serial.println("尝试重新连接MPU6050...");
    mpu.initialize(&I2C0);
    if (mpu.testConnection()) {
        Serial.println("MPU6050重连成功");
        return true;
    }
    Serial.println("MPU6050重连失败");
    return false;
}
```

## 📚 进阶配置

### 自定义编译选项

在`platformio.ini`中添加：
```ini
build_flags = 
    -D CORE_DEBUG_LEVEL=3      # 调试级别
    -D CONFIG_ARDUHAL_LOG_COLORS=1  # 彩色日志
    -D BOARD_HAS_PSRAM         # 如果使用PSRAM
    
monitor_filters = 
    esp32_exception_decoder    # 异常解码
    time                       # 添加时间戳
```

### 优化设置

```ini
board_build.partitions = huge_app.csv  # 更大的应用分区
board_build.flash_mode = qio           # 更快的Flash模式
board_build.f_cpu = 240000000L         # CPU频率240MHz
board_build.f_flash = 80000000L        # Flash频率80MHz
```

---

**注意**：如果遇到任何问题，请首先检查硬件连接，然后查看串口输出中的错误信息。大部分问题都可以通过仔细检查接线和配置来解决。