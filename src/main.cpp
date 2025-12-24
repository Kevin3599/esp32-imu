/**
 * ESP32 I²C总线项目 - 使用Adafruit MPU6050库
 * 功能：使用Adafruit MPU6050库连接MPU6050和OLED显示屏
 * 
 * 硬件连接：
 * I²C总线 (共享):
 *   - SDA: GPIO21
 *   - SCL: GPIO22
 *   - MPU6050和OLED共享同一条I²C总线
 * 
 * 设备地址：
 *   - MPU6050: 0x68
 *   - OLED SSD1306: 0x3C
 * 
 * 作者：ESP32项目
 * 日期：2024
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "mpu6050.h"

// I²C引脚定义 (共享总线)
#define I2C_SDA 21     // 共享I²C总线的SDA引脚
#define I2C_SCL 22     // 共享I²C总线的SCL引脚

// OLED显示屏配置
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// 创建显示器对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 定时变量
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // 100ms更新间隔

// 串口打印配置
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;  // 200ms打印间隔(每秒5次)
const unsigned long DETAILED_PRINT_INTERVAL = 2000; // 2秒详细打印间隔
unsigned long lastDetailedPrintTime = 0;

// 打印模式选择 (0=简要模式, 1=详细模式, 2=混合模式)
int printMode = 2;

// 函数声明
void updateDisplay();
void printDataToSerial();
void handleSerialCommands();
void scanI2CDevices();

/**
 * 更新OLED显示
 */
void updateDisplay() {
  display.clearDisplay();
  
  // 标题
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 IMU 数据");
  
  // 加速度数据
  display.setCursor(0, 12);
  display.println("加速度 (m/s²):");
  display.setCursor(0, 22);
  display.printf("X:%.2f Y:%.2f", mpu6050_data.Acc_X, mpu6050_data.Acc_Y);
  display.setCursor(0, 32);
  display.printf("Z:%.2f", mpu6050_data.Acc_Z);
  
  // 姿态角数据
  display.setCursor(0, 44);
  display.println("姿态角 (°):");
  display.setCursor(0, 54);
  display.printf("R:%.1f P:%.1f", mpu6050_data.Roll, mpu6050_data.Pitch);
  
  display.display();
}

/**
 * 向串口输出数据 - 增强版
 */
void printDataToSerial() {
  unsigned long currentTime = millis();
  
  // 按照模式选择打印方式
  switch(printMode) {
    case 0: // 简要模式 - 高频率简洁输出
      if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
        PrintMPU6050DataBrief();
        lastPrintTime = currentTime;
      }
      break;
      
    case 1: // 详细模式 - 低频率详细输出
      if (currentTime - lastDetailedPrintTime >= DETAILED_PRINT_INTERVAL) {
        PrintMPU6050Data();
        lastDetailedPrintTime = currentTime;
      }
      break;
      
    case 2: // 混合模式 - 简要+定期详细
    default:
      // 每200ms简要输出
      if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
        PrintMPU6050DataBrief();
        lastPrintTime = currentTime;
      }
      // 每2秒详细输出
      if (currentTime - lastDetailedPrintTime >= DETAILED_PRINT_INTERVAL) {
        PrintMPU6050Data();
        lastDetailedPrintTime = currentTime;
      }
      break;
  }
}

/**
 * 处理串口命令
 */
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "0" || command.equalsIgnoreCase("brief")) {
      printMode = 0;
      Serial.println("📡 已切换到简要模式 - 高频率简洁输出");
    }
    else if (command == "1" || command.equalsIgnoreCase("detailed")) {
      printMode = 1;
      Serial.println("📊 已切换到详细模式 - 低频率详细输出");
    }
    else if (command == "2" || command.equalsIgnoreCase("mixed")) {
      printMode = 2;
      Serial.println("🔄 已切换到混合模式 - 简要+定期详细");
    }
    else if (command.equalsIgnoreCase("help") || command == "?") {
      Serial.println("");
      Serial.println("⚙️  === MPU6050 串口打印模式选择 ===");
      Serial.println("📡 0 或 'brief'    - 简要模式 (高频率简洁输出)");
      Serial.println("📊 1 或 'detailed' - 详细模式 (低频率详细输出)");
      Serial.println("🔄 2 或 'mixed'    - 混合模式 (简要+定期详细)");
      Serial.println("🌡️ 'temp'           - 单次温度查询");
      Serial.println("❓ 'help' 或 '?'    - 显示此帮助");
      Serial.println("⚙️  ===============================");
      Serial.printf("🔄 当前模式: %d\n", printMode);
      Serial.println("");
    }
    else if (command.equalsIgnoreCase("temp")) {
      Serial.printf("🌡️ 当前温度: %.2f°C\n", mpu6050_data.Temperature);
    }
    else {
      Serial.println("❌ 未知命令，输入 'help' 查看帮助");
    }
  }
}

/**
 * 扫描I2C总线上的设备
 */
void scanI2CDevices() {
  Serial.println("正在扫描I2C设备...");
  int deviceCount = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("找到I2C设备，地址: 0x%02X", address);
      if (address == 0x68) {
        Serial.print(" (MPU6050)");
      } else if (address == 0x3C) {
        Serial.print(" (OLED SSD1306)");
      }
      Serial.println();
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("❌ 未找到任何I2C设备！");
    Serial.println("请检查：");
    Serial.println("  - SDA接线 (GPIO21)");
    Serial.println("  - SCL接线 (GPIO22)");
    Serial.println("  - 设备供电");
    Serial.println("  - 接线是否松动");
  } else {
    Serial.printf("✅ 总共找到 %d 个I2C设备\n", deviceCount);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 IMU+OLED项目启动...");
  
  // 初始化I²C总线
  Serial.println("初始化I²C总线...");
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // 等待I2C稳定
  delay(100);
  
  // 扫描I2C设备
  scanI2CDevices();
  
  // 初始化MPU6050传感器 (使用Adafruit库)
  Serial.println("初始化MPU6050传感器...");
  Init_mpu6050();
  
  // 初始化OLED显示屏
  Serial.println("初始化OLED显示屏...");
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("✓ OLED初始化成功");
    
    // 显示启动画面
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP32 IMU项目");
    display.println("Adafruit MPU6050库");
    display.println("");
    display.println("MPU6050: 已连接");
    display.println("OLED:    已连接");
    display.println("");
    display.println("初始化完成!");
    display.display();
    
    delay(2000);
  } else {
    Serial.println("✗ OLED初始化失败！");
    Serial.println("请检查I²C接线");
  }
  
  Serial.println("系统初始化完成");
  Serial.println("开始数据采集和显示...");
  Serial.println("");
  Serial.println("📡 输入 'help' 或 '?' 查看串口打印模式选择");
  Serial.println("🔄 当前模式: 混合模式 (简要+定期详细)");
  Serial.println("");
}

void loop() {
  unsigned long currentTime = millis();
  
  // 处理串口命令
  handleSerialCommands();
  
  // 定时更新数据和显示
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    ReadMPU6050();  // 使用Adafruit库读取MPU6050数据
    updateDisplay();
    printDataToSerial();
    
    lastUpdateTime = currentTime;
  }
}