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
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include "mpu6050.h"
#include "gps_module.h"
#include "performance_analyzer.h"
#include "audio_recorder.h"

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

// WiFi热点配置
const char* ap_ssid = "ESP32-IMU";
const char* ap_password = "12345678";

// Web服务器
WebServer server(80);

// 显示模式：0=IMU数据, 1=二维码, 2=GPS数据, 3=性能数据, 4=摩托车模式
int display_mode = 1;

// ===== Dragy模式定时配置 =====
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 50;      // 50ms显示更新(20Hz)

unsigned long lastIMUTime = 0;
const unsigned long IMU_INTERVAL = 5;          // 5ms IMU采样(200Hz) - 用于GPS-IMU融合

unsigned long lastGPSTime = 0;
const unsigned long GPS_INTERVAL = 100;        // 100ms GPS处理(10Hz)

// 串口打印配置
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;  // 200ms打印间隔(每秒5次)
const unsigned long DETAILED_PRINT_INTERVAL = 2000; // 2秒详细打印间隔
unsigned long lastDetailedPrintTime = 0;

// 打印模式选择 (0=简要模式, 1=详细模式, 2=混合模式)
int printMode = 2;

// 函数声明
void updateDisplay();
void displayGPSData();
void displayPerformanceData();
void displayMotorcycleData();
void handleMotoData();
void handleAudioData();
void handleAudioControl();
void handleAudioDownload();
void printDataToSerial();
void handleSerialCommands();
void scanI2CDevices();

/**
 * 更新OLED显示 - 图形化六轴显示
 */
// Web服务器处理函数
void handleRoot() {
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
        server.send(404, "text/plain", "HTML file not found! Please upload SPIFFS files first.");
        return;
    }
    
    // 添加HTTP缓存头和keep-alive
    server.sendHeader("Cache-Control", "public, max-age=300"); // 5分钟缓存
    server.sendHeader("Connection", "keep-alive");
    server.sendHeader("Content-Encoding", "identity");
    
    server.streamFile(file, "text/html");
    file.close();
}

void handleData() {
    // 简化数据处理 - 减少计算负载
    float acc_x_corrected = mpu6050_data.Acc_X_Filtered - 0.47;
    float acc_y_corrected = mpu6050_data.Acc_Y_Filtered + 0.5;
    float acc_z_corrected = mpu6050_data.Acc_Z_Filtered - 0.48;
    
    // 简化角度计算
    float roll = atan2(acc_z_corrected, acc_x_corrected) * 57.2958;  // 57.2958 = 180/PI
    float pitch = atan2(acc_y_corrected, acc_x_corrected) * 57.2958;
    
    // 简单滤波 - 只在必要时使用
    static float prev_roll = 0.0, prev_pitch = 0.0;
    static bool first_run = true;
    
    if (!first_run) {
        // 简单的均值滤波
        roll = (prev_roll + roll) * 0.5;
        pitch = (prev_pitch + pitch) * 0.5;
    }
    first_run = false;
    prev_roll = roll;
    prev_pitch = pitch;
    
    // 构建高精度JSON响应 - 最大化性能
    char json_buffer[750];
    
    // 高精度GPS数据验证
    double safe_lat = (gps_data.valid_location && gps_data.satellites >= 6) ? gps_data.latitude : 0.0;
    double safe_lon = (gps_data.valid_location && gps_data.satellites >= 6) ? gps_data.longitude : 0.0;
    double safe_speed = (gps_data.valid_speed && gps_data.satellites >= 6) ? gps_data.speed_kmh : 0.0;
    
    // 高精度数据输出
    sprintf(json_buffer, 
        "{"
        "\"temp\":%d,"
        "\"gravity\":%.3f,"
        "\"roll\":%d,"
        "\"pitch\":%d,"
        "\"gps\":{"
            "\"speed\":%.2f,"
            "\"rawGPSSpeed\":%.2f,"
            "\"imuSpeed\":%.2f,"
            "\"speedConfidence\":%.2f,"
            "\"maxSpeed\":%.2f,"
            "\"satellites\":%d,"
            "\"valid\":%s,"
            "\"latitude\":%.8f,"
            "\"longitude\":%.8f,"
            "\"altitude\":%.2f,"
            "\"hdop\":%.3f,"
            "\"course\":%.1f,"
            "\"accuracy\":%.2f"
        "},"
        "\"performance\":{"
            "\"accelG\":%.4f,"
            "\"lateralG\":%.4f,"
            "\"accel0to100\":%.2f,"
            "\"brake100to0\":%.2f,"
            "\"maxAccelG\":%.4f,"
            "\"maxBrakeG\":%.4f,"
            "\"maxLateralG\":%.4f,"
            "\"power\":%.2f,"
            "\"confidence\":%.2f"
        "}"
        "}",
        (int)mpu6050_data.Temperature,
        sqrt(acc_x_corrected*acc_x_corrected + acc_y_corrected*acc_y_corrected + acc_z_corrected*acc_z_corrected),
        (int)roll,
        (int)pitch,
        safe_speed,                 // 融合后的最终速度
        gps_data.speed_kmh,         // 原GPS速度
        0.0,                        // IMU积分速度(暂未实现)
        100.0,                      // 速度可信度(默认100%)
        gps_data.max_speed_session,
        gps_data.satellites,
        (gps_data.valid_location && gps_data.satellites >= 6) ? "true" : "false",
        safe_lat,
        safe_lon,
        gps_data.altitude,
        gps_data.hdop,
        gps_data.course,
        gps_data.position_accuracy,
        fused_data.current_accel_g,
        fused_data.current_lateral_g,
        performance_data.accel_0_100_time,
        performance_data.brake_100_0_time,
        fused_data.session_max_accel_g,
        fused_data.session_max_brake_g,
        fused_data.session_max_lateral_g,
        fused_data.instantaneous_power_estimate,
        (gps_data.satellites >= 8 && gps_data.hdop < 1.5) ? 1.0 : 0.6
    );
    
    // 优化的HTTP头
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Connection", "keep-alive");
    
    server.send(200, "application/json", json_buffer);
}

/**
 * 处理摩托车数据API请求
 */
void handleMotoData() {
    // 更新摩托车数据
    updateMotorcycleData();
    
    char json_buffer[600];
    
    sprintf(json_buffer,
        "{"
        "\"lean\":{\"current\":%.2f,\"filtered\":%.2f,\"maxL\":%.1f,\"maxR\":%.1f},"
        "\"g\":{\"fwd\":%.3f,\"lat\":%.3f,\"vert\":%.3f,\"combined\":%.3f},"
        "\"max\":{\"fwdG\":%.2f,\"brakeG\":%.2f,\"latG\":%.2f,\"combG\":%.2f},"
        "\"state\":{\"accel\":%s,\"brake\":%s,\"lean\":%s,\"wheelie\":%s,\"stoppie\":%s,\"corner\":%s},"
        "\"corner\":{\"count\":%d,\"speed\":%.1f,\"g\":%.2f,\"radius\":%.0f},"
        "\"wheelie\":{\"angle\":%.1f,\"max\":%.1f,\"count\":%d},"
        "\"speed\":%.1f,"
        "\"topSpeed\":%.1f,"
        "\"gyro\":{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f},"
        "\"sats\":%d"
        "}",
        moto_data.lean_angle,
        moto_data.lean_angle_filtered,
        moto_data.max_lean_left,
        moto_data.max_lean_right,
        moto_data.forward_g,
        moto_data.lateral_g,
        moto_data.vertical_g,
        moto_data.combined_g,
        moto_data.max_forward_g,
        moto_data.max_brake_g,
        moto_data.max_lateral_g,
        moto_data.max_combined_g,
        moto_data.is_accelerating ? "true" : "false",
        moto_data.is_braking ? "true" : "false",
        moto_data.is_leaning ? "true" : "false",
        moto_data.is_wheelie ? "true" : "false",
        moto_data.is_stoppie ? "true" : "false",
        moto_data.in_corner ? "true" : "false",
        moto_data.corner_count,
        moto_data.corner_speed,
        moto_data.corner_g,
        moto_data.corner_radius_estimate,
        moto_data.wheelie_angle,
        moto_data.max_wheelie_angle,
        moto_data.wheelie_count,
        gps_data.valid_speed ? gps_data.speed_kmh : 0.0,
        moto_data.top_speed,
        moto_data.roll_rate,
        moto_data.pitch_rate,
        moto_data.yaw_rate,
        gps_data.satellites
    );
    
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Connection", "keep-alive");
    server.send(200, "application/json", json_buffer);
}

/**
 * 处理录音状态API请求
 */
void handleAudioData() {
    char json_buffer[400];
    
    const char* stateStr = "idle";
    switch (audio_data.state) {
        case RECORDING_IDLE: stateStr = "idle"; break;
        case RECORDING_WAITING: stateStr = "waiting"; break;
        case RECORDING_ACTIVE: stateStr = "recording"; break;
        case RECORDING_PAUSED: stateStr = "paused"; break;
        case RECORDING_STOPPED: stateStr = "stopped"; break;
    }
    
    sprintf(json_buffer,
        "{"
        "\"state\":\"%s\","
        "\"filename\":\"%s\","
        "\"duration\":%.1f,"
        "\"fileSize\":%lu,"
        "\"volume\":%d,"
        "\"peakVolume\":%d,"
        "\"autoRecord\":%s,"
        "\"storage\":{"
            "\"total\":%lu,"
            "\"used\":%lu,"
            "\"free\":%lu"
        "},"
        "\"error\":%s,"
        "\"errorMsg\":\"%s\""
        "}",
        stateStr,
        audio_data.filename,
        audio_data.duration_ms / 1000.0,
        audio_data.file_size,
        audio_data.current_volume,
        audio_data.peak_volume,
        audio_data.auto_record_on_ride ? "true" : "false",
        audio_data.spiffs_total,
        audio_data.spiffs_used,
        audio_data.spiffs_free,
        audio_data.error ? "true" : "false",
        audio_data.error_msg
    );
    
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json_buffer);
}

/**
 * 处理录音控制API请求
 * 参数: action = start | stop | pause | resume | delete | toggle_auto
 */
void handleAudioControl() {
    String action = server.arg("action");
    String response = "{\"success\":true,\"message\":\"";
    
    if (action == "start") {
        if (startRecording()) {
            response += "录音已开始\"}";
        } else {
            response = "{\"success\":false,\"message\":\"";
            response += audio_data.error_msg;
            response += "\"}";
        }
    } else if (action == "stop") {
        stopRecording();
        response += "录音已停止\"}";
    } else if (action == "pause") {
        pauseRecording();
        response += "录音已暂停\"}";
    } else if (action == "resume") {
        resumeRecording();
        response += "录音已恢复\"}";
    } else if (action == "delete") {
        deleteAllRecordings();
        response += "所有录音已删除\"}";
    } else if (action == "toggle_auto") {
        audio_data.auto_record_on_ride = !audio_data.auto_record_on_ride;
        response += audio_data.auto_record_on_ride ? "自动录音已开启\"}" : "自动录音已关闭\"}";
    } else if (action == "list") {
        // 返回录音列表
        String list = getRecordingsList();
        server.send(200, "application/json", list);
        return;
    } else {
        response = "{\"success\":false,\"message\":\"未知操作\"}";
    }
    
    server.send(200, "application/json", response);
}

/**
 * 处理录音文件下载
 * 参数: file = 文件名
 */
void handleAudioDownload() {
    String filename = server.arg("file");
    if (filename.length() == 0) {
        server.send(400, "text/plain", "Missing file parameter");
        return;
    }
    
    // 确保文件名以/开头
    if (!filename.startsWith("/")) {
        filename = "/" + filename;
    }
    
    if (!SPIFFS.exists(filename)) {
        server.send(404, "text/plain", "File not found");
        return;
    }
    
    File file = SPIFFS.open(filename, "r");
    if (!file) {
        server.send(500, "text/plain", "Cannot open file");
        return;
    }
    
    // 设置下载头
    server.sendHeader("Content-Disposition", "attachment; filename=" + filename.substring(1));
    server.streamFile(file, "audio/wav");
    file.close();
}

void displayQRCode() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // 显示WiFi信息
    display.setCursor(0, 0);
    display.println("WiFi: ESP32-IMU");
    display.println("Pass: 12345678");
    display.println("");
    display.println("Scan QR or visit:");
    display.println("http://192.168.4.1");
    display.println("");
    display.println("Press BOOT to switch");
    
    // 简单的二维码替代（文字版）
    display.setCursor(0, 56);
    display.print("QR: 192.168.4.1");
    
    display.display();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // 根据实际数据分析，传感器垂直安装时X轴承受主要重力
  // X=10.25, Y=-0.5, Z=0.5 说明X轴垂直向下
  // 重新校准：假设静止时X轴应该是-9.8 (向下)
  // 垂直安装校准参数
  float acc_x_corrected = mpu6050_data.Acc_X_Filtered - 0.47;  // 校准偏移 (10.25-9.8)
  float acc_y_corrected = mpu6050_data.Acc_Y_Filtered + 0.5;   // 校准Y轴偏移
  float acc_z_corrected = mpu6050_data.Acc_Z_Filtered - 0.48;  // 微调Z轴偏移
  
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
  float roll = atan2(acc_z_corrected, acc_x_corrected) * 180.0 / PI + 2.0;    // 补偿+2度偏移（消除-3度）
  float pitch = atan2(acc_y_corrected, acc_x_corrected) * 180.0 / PI;         // 绕Z轴转动（左右倾斜）
  float yaw = (mpu6050_data.Gyro_X_Filtered * 180/PI) * 0.1;                  // X轴角速度积分
  
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
        printGPSData();
        printFusedData();
        lastPrintTime = currentTime;
      }
      break;
      
    case 1: // 详细模式 - 低频率详细输出
      if (currentTime - lastDetailedPrintTime >= DETAILED_PRINT_INTERVAL) {
        PrintMPU6050Data();
        printGPSData();
        printFusedData();
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
        printGPSData();
        printFusedData();
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
  
  // 配置BOOT按钮
  pinMode(0, INPUT_PULLUP);
  
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
  
  // 初始化GPS模块
  Serial.println("初始化GPS模块...");
  initGPS();
  
  // 初始化性能分析器
  Serial.println("初始化性能分析器...");
  initPerformanceAnalyzer();
  
  // 初始化摩托车模式
  Serial.println("初始化摩托车模式...");
  initMotorcycleMode();
  
  // 初始化录音模块
  Serial.println("初始化录音模块...");
  initAudioRecorder();
  
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
  
  // 初始化SPIFFS文件系统
  Serial.println("正在初始化SPIFFS文件系统...");
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS初始化失败!");
  } else {
    Serial.println("✅ SPIFFS初始化成功");
    // 列出文件系统中的文件
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file) {
      Serial.printf("  文件: %s (%d bytes)\n", file.name(), file.size());
      file = root.openNextFile();
    }
  }
  
  // 初始化WiFi热点
  Serial.println("正在创建WiFi热点...");
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("热点已创建! IP地址: ");
  Serial.println(ip);
  Serial.println("热点名称: ESP32-IMU");
  Serial.println("密码: 12345678");
  Serial.println("网页地址: http://192.168.4.1");
  
  // 配置Web服务器路由
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/moto", handleMotoData);  // 摩托车数据端点
  server.on("/audio", handleAudioData);  // 录音状态端点
  server.on("/audio/control", handleAudioControl);  // 录音控制端点
  server.on("/audio/download", handleAudioDownload);  // 录音下载端点
  server.begin();
  Serial.println("Web服务器已启动!");
  Serial.println("");
  
  // 初始显示二维码模式
  display_mode = 1;
  displayQRCode();
}

void loop() {
  unsigned long currentTime = millis();
  
  // 处理Web服务器请求
  server.handleClient();
  
  // 检查BOOT按钮切换显示模式（GPIO0通常是BOOT按钮）
  static bool lastBootState = HIGH;
  bool currentBootState = digitalRead(0);
  if (lastBootState == HIGH && currentBootState == LOW) {
    display_mode = (display_mode + 1) % 5; // 切换显示模式: 0=IMU, 1=二维码, 2=GPS, 3=性能, 4=摩托车
    delay(200); // 防抖
  }
  lastBootState = currentBootState;
  
  // 处理串口命令
  handleSerialCommands();
  
  // ===== 高频IMU采样 (200Hz) - Dragy模式关键 =====
  if (currentTime - lastIMUTime >= IMU_INTERVAL) {
    ReadMPU6050();  // 200Hz读取IMU数据
    
    // GPS-IMU速度融合 (每次IMU采样都进行)
    double dt = IMU_INTERVAL / 1000.0; // 转换为秒
    fuseIMUSpeed(mpu6050_data.Acc_X, mpu6050_data.Acc_Y, dt);
    
    lastIMUTime = currentTime;
  }
  
  // ===== GPS数据处理 (10Hz) =====
  if (currentTime - lastGPSTime >= GPS_INTERVAL) {
    updateGPSData(); // 10Hz更新GPS数据
    lastGPSTime = currentTime;
  }
  
  // ===== 录音模块更新 (持续) =====
  updateAudioRecorder();  // 处理音频采样和自动录音触发
  
  // ===== 显示和融合更新 (20Hz) =====
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    updateFusedData(); // 更新融合性能数据
    
    // 根据显示模式选择显示内容
    if (display_mode == 0) {
      updateDisplay();  // 显示IMU数据
    } else if (display_mode == 1) {
      displayQRCode();  // 显示二维码
    } else if (display_mode == 2) {
      displayGPSData(); // 显示GPS数据
    } else if (display_mode == 3) {
      displayPerformanceData(); // 显示性能数据
    } else if (display_mode == 4) {
      displayMotorcycleData(); // 显示摩托车数据
    }
    
    printDataToSerial();
    
    lastUpdateTime = currentTime;
  }
}

/**
 * 显示GPS数据
 */
void displayGPSData() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("=== GPS STATUS ===");
  
  // 显示卫星和信号质量
  display.printf("Sats: %d HDOP: %.1f\n", gps_data.satellites, gps_data.hdop);
  
  // 显示速度信息
  if (gps_data.valid_speed) {
    display.printf("Speed: %.1f km/h\n", gps_data.speed_kmh);
    display.printf("Max: %.1f km/h\n", gps_data.max_speed_session);
  } else {
    display.println("Speed: No Fix");
  }
  
  // 显示位置信息
  if (gps_data.valid_location) {
    display.printf("Lat: %.4f\n", gps_data.latitude);
    display.printf("Lon: %.4f\n", gps_data.longitude);
  } else {
    display.println("Location: No Fix");
  }
  
  // 显示时间
  if (gps_data.valid_time) {
    display.printf("Time: %02d:%02d:%02d\n", 
                   gps_data.hour, gps_data.minute, gps_data.second);
  }
  
  display.display();
}

/**
 * 显示性能数据
 */
void displayPerformanceData() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("=== PERFORMANCE ===");
  
  // 显示当前状态
  display.printf("Speed: %.1f km/h\n", fused_data.current_speed_kmh);
  display.printf("Accel: %.2fG ", fused_data.current_accel_g);
  display.printf("Lat: %.2fG\n", fused_data.current_lateral_g);
  
  // 显示状态指示
  display.print("Status: ");
  if (fused_data.is_accelerating) display.print("ACC ");
  if (fused_data.is_braking) display.print("BRK ");
  if (fused_data.is_cornering) display.print("COR");
  display.println();
  
  // 显示测试结果
  if (performance_data.accel_0_100_time > 0) {
    display.printf("0-100: %.2fs\n", performance_data.accel_0_100_time);
  }
  
  if (performance_data.brake_100_0_time > 0) {
    display.printf("100-0: %.2fs\n", performance_data.brake_100_0_time);
  }
  
  // 显示峰值数据
  display.printf("Max G: +%.1f/%.1f/%.1f\n", 
                fused_data.session_max_accel_g,
                fused_data.session_max_brake_g,
                fused_data.session_max_lateral_g);
  
  display.display();
}

/**
 * 显示摩托车数据 (display_mode = 4)
 */
void displayMotorcycleData() {
  // 更新摩托车数据
  updateMotorcycleData();
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // 标题栏 - 包含录音状态
  display.setCursor(0, 0);
  display.print("MOTO ");
  
  // 录音状态指示
  if (audio_data.state == RECORDING_ACTIVE) {
    display.print("[REC]");  // 正在录音
  } else if (audio_data.auto_record_on_ride) {
    display.print("[AUTO]"); // 自动录音待命
  }
  
  // 显示状态图标
  display.setCursor(70, 0);
  if (moto_data.is_wheelie) display.print("W!");
  else if (moto_data.is_stoppie) display.print("S!");
  else if (moto_data.is_leaning) display.print(">");
  else if (moto_data.is_braking) display.print("B");
  else if (moto_data.is_accelerating) display.print("A");
  
  // GPS状态
  display.setCursor(100, 0);
  display.printf("%dS", gps_data.satellites);
  
  // === 大字体显示倾角 ===
  display.setTextSize(2);
  display.setCursor(0, 10);
  
  // 倾角显示
  float lean = moto_data.lean_angle_filtered;
  if (lean >= 0) {
    display.printf("R%4.1f", lean);  // 右倾
  } else {
    display.printf("L%4.1f", -lean); // 左倾
  }
  display.setTextSize(1);
  display.setCursor(80, 10);
  display.print("deg");
  
  // 速度显示
  display.setCursor(80, 18);
  display.printf("%.0fkm/h", gps_data.speed_kmh);
  
  // === 第三行: G力数据 ===
  display.setCursor(0, 28);
  display.printf("Fwd:%.2fG Lat:%.2fG", moto_data.forward_g, moto_data.lateral_g);
  
  // === 第四行: 弯道信息 ===
  display.setCursor(0, 38);
  if (moto_data.in_corner) {
    display.printf("Corner! G:%.2f R:%.0fm", moto_data.corner_g, moto_data.corner_radius_estimate);
  } else {
    display.printf("Corners:%d Wheelie:%d", moto_data.corner_count, moto_data.wheelie_count);
  }
  
  // === 第五行: 录音信息或峰值记录 ===
  display.setCursor(0, 48);
  if (audio_data.state == RECORDING_ACTIVE) {
    // 显示录音信息
    display.printf("REC %.0fs Vol:%d", 
                  audio_data.duration_ms / 1000.0,
                  audio_data.current_volume);
  } else {
    display.printf("MaxL:%.0f/%.0f MaxG:%.1f", 
                  -moto_data.max_lean_left, 
                  moto_data.max_lean_right, 
                  moto_data.max_combined_g);
  }
  
  // === 第六行: 最高速度 ===
  display.setCursor(0, 56);
  display.printf("Top:%.1fkm/h W:%.0fdeg", 
                moto_data.top_speed,
                moto_data.max_wheelie_angle);
  
  display.display();
}