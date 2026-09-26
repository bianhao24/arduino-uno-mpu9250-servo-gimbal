#include <Wire.h>           // 包含I2C通信库，用于MPU9250数据读取
#include <Servo.h>          // 包含舵机控制库，用于驱动舵机

#define MPU_ADDR 0x68       // MPU9250的I2C地址（AD0引脚接地时为0x68）
#define GYRO_SENS 131.0     // 陀螺仪灵敏度系数，±250°/s量程下为131 LSB/(°/s)

Servo servoPitch, servoRoll; // 创建两个舵机对象，分别控制俯仰和横滚

float pitch = 0, roll = 0;   // 俯仰角和横滚角，通过陀螺仪积分得到
unsigned long lastTime = 0;  // 记录上次循环的时间，用于计算时间间隔dt

void setup() {
  Wire.begin();              // 初始化I2C总线，SDA=A4, SCL=A5
  
  writeReg(0x6B, 0x00);      // 向电源管理寄存器写入0x00，唤醒MPU9250（解除睡眠模式）
  writeReg(0x1B, 0x00);      // 向陀螺仪配置寄存器写入0x00，设置量程为±250°/s（最灵敏）
  
  servoPitch.attach(9);      // 俯仰舵机信号线连接到数字引脚D9
  servoRoll.attach(10);      // 横滚舵机信号线连接到数字引脚D10
  
  delay(100);                // 延时100毫秒，等待模块稳定
}

void loop() {
  unsigned long now = micros();                  // 获取当前微秒时间（1秒=1000000微秒）
  float dt = (now - lastTime) / 1000000.0;       // 计算时间间隔，单位转换为秒
  lastTime = now;                                // 更新上次时间为当前时间
  
  Wire.beginTransmission(MPU_ADDR);              // 开始I2C传输，指定MPU9250地址
  Wire.write(0x43);                              // 指定寄存器地址0x43（陀螺仪X轴高字节起始位）
  Wire.endTransmission(false);                   // 发送重复起始条件，保持总线连接
  
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)1); // 请求读取6字节数据（X/Y/Z三轴，每轴2字节）
  
  float gx = ((Wire.read() << 8 | Wire.read()) / GYRO_SENS);   // 读取X轴高字节和低字节，组合成16位有符号整数，除以灵敏度得到角速度（°/s）
  float gy = ((Wire.read() << 8 | Wire.read()) / GYRO_SENS);   // 读取Y轴高字节和低字节，同上计算Y轴角速度
  float gz = ((Wire.read() << 8 | Wire.read()) / GYRO_SENS);   // 读取Z轴高字节和低字节，同上计算Z轴角速度
  
  pitch += gx * dt;                              // 对X轴角速度积分，得到俯仰角（角度=角速度×时间）
  roll += gy * dt;                               // 对Y轴角速度积分，得到横滚角
  
  servoPitch.write(constrain(90 - (int)(pitch * 3), 0, 180));  // 俯仰角映射到舵机：90度为中位，乘以3倍放大系数，限幅0-180度
  servoRoll.write(constrain(90 + (int)(roll * 3), 0, 180));    // 横滚角映射到舵机：90度为中位，乘以3倍放大系数，限幅0-180度
}

void writeReg(uint8_t reg, uint8_t val) {         // 自定义函数：向MPU9250指定寄存器写入数据
  Wire.beginTransmission(MPU_ADDR);              // 开始I2C传输
  Wire.write(reg);                               // 发送目标寄存器地址
  Wire.write(val);                               // 发送要写入的值
  Wire.endTransmission();                        // 结束传输，释放总线
}