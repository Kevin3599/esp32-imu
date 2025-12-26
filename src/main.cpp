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

// I²C双路总线引脚定义
// I2C0 - MPU6050传感器
#define I2C0_SDA 21    // MPU6050 SDA引脚
#define I2C0_SCL 22    // MPU6050 SCL引脚

// I2C1 - OLED显示屏
#define I2C1_SDA 18    // OLED SDA引脚  
#define I2C1_SCL 19    // OLED SCL引脚

// OLED显示屏配置
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// 双路I²C总线设计
TwoWire I2C_MPU = TwoWire(0);  // I2C0 - MPU6050传感器
TwoWire I2C_OLED = TwoWire(1); // I2C1 - OLED显示屏

// 创建显示器对象（使用I2C1）
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

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
 * 更新OLED显示 - 图形化六轴显示
 */
void updateDisplay() {
  display.clearDisplay();
  
  // 屏幕中心点
  int centerX = SCREEN_WIDTH / 2;   // 64
  int centerY = SCREEN_HEIGHT / 2;  // 32
  
  // 绘制坐标轴参考线
  display.drawPixel(centerX, centerY, SSD1306_WHITE); // 中心点
  
  // 绘制水平和垂直参考线
  display.drawLine(centerX - 30, centerY, centerX + 30, centerY, SSD1306_WHITE); // 水平线
  display.drawLine(centerX, centerY - 20, centerX, centerY + 20, SSD1306_WHITE); // 垂直线
  
  // Roll箭头 (左侧，绕X轴旋转)
  float rollRad = mpu6050_data.Roll * PI / 180.0;
  int rollArrowLen = 25;
  int rollX = centerX - 50;
  int rollY = centerY;
  
  int rollEndX = rollX + rollArrowLen * cos(rollRad - PI/2);
  int rollEndY = rollY + rollArrowLen * sin(rollRad - PI/2);
  
  // 绘制Roll箭头
  display.drawLine(rollX, rollY, rollEndX, rollEndY, SSD1306_WHITE);
  // 箭头头部
  display.drawLine(rollEndX, rollEndY, 
                   rollEndX - 3 * cos(rollRad - PI/2 - 0.5), 
                   rollEndY - 3 * sin(rollRad - PI/2 - 0.5), SSD1306_WHITE);
  display.drawLine(rollEndX, rollEndY, 
                   rollEndX - 3 * cos(rollRad - PI/2 + 0.5), 
                   rollEndY - 3 * sin(rollRad - PI/2 + 0.5), SSD1306_WHITE);
  
  // Roll标签和数值
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(rollX - 20, rollY + 15);
  display.print("R:");
  display.print((int)mpu6050_data.Roll);
  
  // Pitch箭头 (右侧，绕Y轴旋转)
  float pitchRad = mpu6050_data.Pitch * PI / 180.0;
  int pitchArrowLen = 25;
  int pitchX = centerX + 50;
  int pitchY = centerY;
  
  int pitchEndX = pitchX + pitchArrowLen * cos(pitchRad - PI/2);
  int pitchEndY = pitchY + pitchArrowLen * sin(pitchRad - PI/2);
  
  // 绘制Pitch箭头
  display.drawLine(pitchX, pitchY, pitchEndX, pitchEndY, SSD1306_WHITE);
  // 箭头头部
  display.drawLine(pitchEndX, pitchEndY, 
                   pitchEndX - 3 * cos(pitchRad - PI/2 - 0.5), 
                   pitchEndY - 3 * sin(pitchRad - PI/2 - 0.5), SSD1306_WHITE);
  display.drawLine(pitchEndX, pitchEndY, 
                   pitchEndX - 3 * cos(pitchRad - PI/2 + 0.5), 
                   pitchEndY - 3 * sin(pitchRad - PI/2 + 0.5), SSD1306_WHITE);
  
  // Pitch标签和数值
  display.setCursor(pitchX - 10, pitchY + 15);
  display.print("P:");
  display.print((int)mpu6050_data.Pitch);
  
  // 加速度合成矢量 (中心箭头)
  float totalAccel = sqrt(mpu6050_data.Acc_X * mpu6050_data.Acc_X + 
                         mpu6050_data.Acc_Y * mpu6050_data.Acc_Y + 
                         mpu6050_data.Acc_Z * mpu6050_data.Acc_Z);
  
  // 基于加速度计算方向 (简化显示)
  float accelAngle = atan2(mpu6050_data.Acc_Y, mpu6050_data.Acc_X);
  int accelLen = map(constrain(totalAccel, 8, 12), 8, 12, 5, 15); // 映射到箭头长度
  
  int accelEndX = centerX + accelLen * cos(accelAngle);
  int accelEndY = centerY + accelLen * sin(accelAngle);
  
  // 绘制加速度矢量箭头
  display.drawLine(centerX, centerY, accelEndX, accelEndY, SSD1306_WHITE);
  display.drawLine(accelEndX, accelEndY, 
                   accelEndX - 2 * cos(accelAngle - 0.5), 
                   accelEndY - 2 * sin(accelAngle - 0.5), SSD1306_WHITE);
  display.drawLine(accelEndX, accelEndY, 
                   accelEndX - 2 * cos(accelAngle + 0.5), 
                   accelEndY - 2 * sin(accelAngle + 0.5), SSD1306_WHITE);
  
  // 角速度指示器 (顶部三个小条)
  int gyroBarY = 5;
  int gyroBarWidth = 2;
  int gyroBarMaxHeight = 10;
  
  // X轴角速度条
  int gyroXHeight = map(constrain(abs(mpu6050_data.Angle_Velocity_R * 180/PI), 0, 100), 0, 100, 1, gyroBarMaxHeight);
  display.fillRect(centerX - 20, gyroBarY + gyroBarMaxHeight - gyroXHeight, gyroBarWidth, gyroXHeight, SSD1306_WHITE);
  display.setCursor(centerX - 22, gyroBarY + gyroBarMaxHeight + 2);
  display.print("X");
  
  // Y轴角速度条
  int gyroYHeight = map(constrain(abs(mpu6050_data.Angle_Velocity_P * 180/PI), 0, 100), 0, 100, 1, gyroBarMaxHeight);
  display.fillRect(centerX - 1, gyroBarY + gyroBarMaxHeight - gyroYHeight, gyroBarWidth, gyroYHeight, SSD1306_WHITE);
  display.setCursor(centerX - 3, gyroBarY + gyroBarMaxHeight + 2);
  display.print("Y");
  
  // Z轴角速度条
  int gyroZHeight = map(constrain(abs(mpu6050_data.Angle_Velocity_Y * 180/PI), 0, 100), 0, 100, 1, gyroBarMaxHeight);
  display.fillRect(centerX + 18, gyroBarY + gyroBarMaxHeight - gyroZHeight, gyroBarWidth, gyroZHeight, SSD1306_WHITE);
  display.setCursor(centerX + 16, gyroBarY + gyroBarMaxHeight + 2);
  display.print("Z");
  
  // 温度显示 (右上角)
  display.setCursor(90, 0);
  display.print((int)mpu6050_data.Temperature);
  display.print("C");
  
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
  
  // 扫描I2C0总线 (MPU6050)
  Serial.println("I2C0总线 (GPIO21/22) - MPU6050:");
  int deviceCount0 = 0;
  for (byte address = 1; address < 127; address++) {
    I2C_MPU.beginTransmission(address);
    byte error = I2C_MPU.endTransmission();
    
    if (error == 0) {
      Serial.printf("  找到I2C设备，地址: 0x%02X", address);
      if (address == 0x68) {
        Serial.print(" (MPU6050)");
      }
      Serial.println();
      deviceCount0++;
    }
  }
  
  // 扫描I2C1总线 (OLED)
  Serial.println("I2C1总线 (GPIO18/19) - OLED:");
  int deviceCount1 = 0;
  for (byte address = 1; address < 127; address++) {
    I2C_OLED.beginTransmission(address);
    byte error = I2C_OLED.endTransmission();
    
    if (error == 0) {
      Serial.printf("  找到I2C设备，地址: 0x%02X", address);
      if (address == 0x3C) {
        Serial.print(" (OLED SSD1306)");
      }
      Serial.println();
      deviceCount1++;
    }
  }
  
  if (deviceCount0 == 0 && deviceCount1 == 0) {
    Serial.println("❌ 未找到任何I2C设备！");
    Serial.println("请检查：");
    Serial.println("  MPU6050: SDA(GPIO21), SCL(GPIO22)");
    Serial.println("  OLED:    SDA(GPIO18), SCL(GPIO19)");
    Serial.println("  设备供电和接线");
  } else {
    Serial.printf("✅ I2C0总线找到 %d 个设备，I2C1总线找到 %d 个设备\n", deviceCount0, deviceCount1);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 IMU+OLED项目启动...");
  Serial.println("使用双路I2C配置：");
  Serial.println("  I2C0 (GPIO21/22) - MPU6050传感器");
  Serial.println("  I2C1 (GPIO18/19) - OLED显示屏");
  
  // 初始化双路I²C总线
  Serial.println("初始化双路I²C总线...");
  I2C_MPU.begin(I2C0_SDA, I2C0_SCL, 400000);   // I2C0 - MPU6050
  I2C_OLED.begin(I2C1_SDA, I2C1_SCL, 400000);  // I2C1 - OLED
  
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