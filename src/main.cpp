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
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // 根据实际数据分析，传感器垂直安装时X轴承受主要重力
  // X=10.25, Y=-0.5, Z=0.5 说明X轴垂直向下
  // 重新校准：假设静止时X轴应该是-9.8 (向下)
  float acc_x_corrected = mpu6050_data.Acc_X_Filtered - 0.47;  // 校准偏移 (10.25-9.8)
  float acc_y_corrected = mpu6050_data.Acc_Y_Filtered + 0.5;   // 校准Y轴偏移
  float acc_z_corrected = mpu6050_data.Acc_Z_Filtered - 0.48;  // 微调Z轴偏移，消除Roll的-1度偏移
  
  // 计算校正后的合成加速度
  float corrected_total_accel = sqrt(acc_x_corrected * acc_x_corrected + 
                                    acc_y_corrected * acc_y_corrected + 
                                    acc_z_corrected * acc_z_corrected);
  
  // 顶部显示基本数据 (温度和校正后的加速度)
  display.setCursor(0, 0);
  display.print("T:");
  display.print((int)mpu6050_data.Temperature);
  display.setCursor(35, 0);
  display.print("G:");
  display.print(corrected_total_accel, 1);
  
  // 3D立方体显示区域 (使用屏幕中心区域 32x32像素)
  int centerX = 64;   // 屏幕中心X (128/2)
  int centerY = 35;   // 屏幕中心Y (稍微下移)
  
  // 基于校正后的加速度计算倾斜角度
  // 垂直安装：X轴向下为重力方向
  float roll = -atan2(acc_z_corrected, acc_x_corrected) * 180.0 / PI + 1.0;   // 绕Y轴转动(前后倾斜) + 补偿1度偏移
  float pitch = atan2(acc_y_corrected, acc_x_corrected) * 180.0 / PI;   // 绕Z轴转动(左右倾斜)
  float yaw = (mpu6050_data.Gyro_X_Filtered * 180/PI) * 0.1;            // X轴角速度积分
  
  // 角度限制和平滑处理
  roll = constrain(roll, -90, 90);
  pitch = constrain(pitch, -90, 90);
  
  // 转换为弧度
  float rollRad = roll * PI / 180.0;
  float pitchRad = pitch * PI / 180.0;
  float yawRad = yaw * PI / 180.0;
  
  // 3D立方体的8个顶点 (相对坐标，边长14像素，稍小一点)
  float vertices[8][3] = {
    {-7, -7, -7}, {7, -7, -7}, {7, 7, -7}, {-7, 7, -7},  // 后面4个点
    {-7, -7, 7},  {7, -7, 7},  {7, 7, 7},  {-7, 7, 7}   // 前面4个点
  };
  
  // 旋转后的2D投影点
  int projected[8][2];
  
  // 对每个顶点进行3D旋转和2D投影
  for(int i = 0; i < 8; i++) {
    float x = vertices[i][0];
    float y = vertices[i][1]; 
    float z = vertices[i][2];
    
    // 绕X轴旋转 (Roll) - 前后倾斜
    float y1 = y * cos(rollRad) - z * sin(rollRad);
    float z1 = y * sin(rollRad) + z * cos(rollRad);
    
    // 绕Y轴旋转 (Pitch) - 左右倾斜
    float x2 = x * cos(pitchRad) + z1 * sin(pitchRad);
    float z2 = -x * sin(pitchRad) + z1 * cos(pitchRad);
    
    // 绕Z轴旋转 (Yaw) - 水平转动
    float x3 = x2 * cos(yawRad) - y1 * sin(yawRad);
    float y3 = x2 * sin(yawRad) + y1 * cos(yawRad);
    
    // 简单透视投影
    float distance = 40 + z2;  // 减小基础距离
    if (distance < 10) distance = 10; // 防止除零
    projected[i][0] = centerX + (x3 * 35) / distance;
    projected[i][1] = centerY + (y3 * 35) / distance;
  }
  
  // 绘制立方体的12条边
  // 后面正方形的4条边 (0-1-2-3-0)
  display.drawLine(projected[0][0], projected[0][1], projected[1][0], projected[1][1], SSD1306_WHITE);
  display.drawLine(projected[1][0], projected[1][1], projected[2][0], projected[2][1], SSD1306_WHITE);
  display.drawLine(projected[2][0], projected[2][1], projected[3][0], projected[3][1], SSD1306_WHITE);
  display.drawLine(projected[3][0], projected[3][1], projected[0][0], projected[0][1], SSD1306_WHITE);
  
  // 前面正方形的4条边 (4-5-6-7-4)
  display.drawLine(projected[4][0], projected[4][1], projected[5][0], projected[5][1], SSD1306_WHITE);
  display.drawLine(projected[5][0], projected[5][1], projected[6][0], projected[6][1], SSD1306_WHITE);
  display.drawLine(projected[6][0], projected[6][1], projected[7][0], projected[7][1], SSD1306_WHITE);
  display.drawLine(projected[7][0], projected[7][1], projected[4][0], projected[4][1], SSD1306_WHITE);
  
  // 连接前后两个正方形的4条边
  display.drawLine(projected[0][0], projected[0][1], projected[4][0], projected[4][1], SSD1306_WHITE);
  display.drawLine(projected[1][0], projected[1][1], projected[5][0], projected[5][1], SSD1306_WHITE);
  display.drawLine(projected[2][0], projected[2][1], projected[6][0], projected[6][1], SSD1306_WHITE);
  display.drawLine(projected[3][0], projected[3][1], projected[7][0], projected[7][1], SSD1306_WHITE);
  
  // 底部显示校正后的角度数值
  display.setCursor(0, 56);
  display.print("R:");
  display.print((int)roll);
  display.setCursor(35, 56);
  display.print("P:");
  display.print((int)pitch);
  display.setCursor(70, 56);
  display.print("Y:");
  display.print((int)yaw);
  
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
    
    // 显示启动画面 - 使用纯ASCII字符
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP32 IMU");
    display.println("MPU6050 + OLED");
    display.println("");
    display.println("MPU6050: OK");
    display.println("OLED:    OK");
    display.println("");
    display.println("Ready!");
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