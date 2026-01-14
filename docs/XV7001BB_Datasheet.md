# XV7001BB 陀螺仪传感器技术文档

## 芯片概述

**XV7001BB** 是 Epson 生产的高精度单轴角速率陀螺仪传感器，采用压电振动陀螺技术。

### 主要特性
- **测量范围**: ±300°/s
- **灵敏度**: 13.2 mV/(°/s) (典型值)
- **零偏稳定性**: ±2°/s (25°C)
- **工作电压**: 3.0V - 5.25V
- **接口**: SPI (最高 1MHz)
- **工作温度**: -40°C ~ +85°C
- **封装**: 24-pin QFN (5mm × 5mm × 1.6mm)

---

## 引脚定义

| 引脚号 | 名称 | 类型 | 功能描述 |
|--------|------|------|----------|
| 1 | AVDD | 电源 | 模拟电源 (3.0V - 5.25V) |
| 2 | AGND | 地 | 模拟地 |
| 3 | VOUT | 输出 | 模拟电压输出 |
| 4 | VREF | 输出 | 参考电压输出 (1.35V ±0.1V) |
| 5 | ST | 输入 | 自检引脚 (高电平启用) |
| 6-11 | NC | - | 未连接 |
| 12 | DVDD | 电源 | 数字电源 (3.0V - 5.25V) |
| 13 | DGND | 地 | 数字地 |
| 14 | SCLK | 输入 | SPI 时钟 (最高 1MHz) |
| 15 | MISO | 输出 | SPI 主设备输入/从设备输出 |
| 16 | MOSI | 输入 | SPI 主设备输出/从设备输入 |
| 17 | CS | 输入 | SPI 片选 (低电平有效) |
| 18-23 | NC | - | 未连接 |
| 24 | PAD | 地 | 散热焊盘 (接地) |

---

## SPI 通信协议

### 时序参数
- **SPI 模式**: Mode 3 (CPOL=1, CPHA=1)
- **时钟频率**: 最高 1MHz
- **数据位宽**: 8-bit
- **字节顺序**: MSB First

### 寄存器访问
#### 读操作
1. CS 拉低
2. 发送地址字节: `0x80 | reg_addr` (最高位为 1 表示读)
3. 读取数据字节
4. CS 拉高

#### 写操作
1. CS 拉低
2. 发送地址字节: `reg_addr` (最高位为 0 表示写)
3. 发送数据字节
4. CS 拉高

---

## 寄存器映射

### 控制寄存器

#### 寄存器 0x00 - WHO_AM_I
- **读/写**: 只读
- **默认值**: 0x03
- **说明**: 器件 ID

#### 寄存器 0x02 - CTRL
- **读/写**: 读/写
- **默认值**: 0x00
- **位定义**:
  | 位 | 名称 | 说明 |
  |---|------|------|
  | [7:6] | RESERVED | 保留 (写 0) |
  | [5:4] | FILTER_ORDER | 滤波器阶数: 00=2阶, 01=3阶, 10=4阶, 11=4阶 |
  | [3:0] | FILTER_FC | 截止频率选择 (见滤波器配置表) |

**滤波器配置表**:
| FILTER_FC[3:0] | 截止频率 (-3dB) |
|----------------|-----------------|
| 0000 | 5 Hz |
| 0001 | 10 Hz |
| 0010 | 25 Hz |
| 0011 | 50 Hz |
| 0100 | 100 Hz |
| 0101 | 200 Hz |
| 0110 | 500 Hz |

#### 寄存器 0x04 - MODE_CTRL
- **读/写**: 读/写
- **默认值**: 0x00
- **位定义**:
  | 位 | 名称 | 说明 |
  |---|------|------|
  | [7:1] | RESERVED | 保留 |
  | [0] | POWER_DOWN | 0=正常, 1=掉电模式 |

#### 寄存器 0x06 - STATUS
- **读/写**: 只读
- **位定义**:
  | 位 | 名称 | 说明 |
  |---|------|------|
  | [7:2] | RESERVED | 保留 |
  | [1] | ST_RESULT | 自检结果: 0=通过, 1=失败 |
  | [0] | DATA_RDY | 数据就绪: 0=未就绪, 1=就绪 |

---

### 数据寄存器

#### 寄存器 0x08-0x09 - GYRO_OUT (16-bit)
- **地址**: 0x08 (低字节), 0x09 (高字节)
- **读/写**: 只读
- **格式**: 16-bit 二进制补码 (LSB first)
- **范围**: -32768 ~ 32767
- **计算公式**: 
  ```
  角速率 (°/s) = GYRO_OUT × 0.0125
  ```

#### 寄存器 0x0A-0x0B - TEMP_OUT (16-bit)
- **地址**: 0x0A (低字节), 0x0B (高字节)
- **读/写**: 只读
- **格式**: 16-bit 二进制补码
- **计算公式**:
  ```
  温度 (°C) = TEMP_OUT / 131 + 25
  ```

---

## 初始化流程

### 1. 上电时序
```
1. 施加 AVDD/DVDD 电源
2. 等待 ≥ 50ms (上电稳定时间)
3. 开始 SPI 通信
```

### 2. 配置步骤
```c
// 1. 验证器件 ID
uint8_t id = ReadRegister(0x00);
if (id != 0x03) {
    // 通信错误
}

// 2. 配置滤波器 (4阶 50Hz)
WriteRegister(0x02, 0x23);  // FILTER_ORDER=10, FILTER_FC=0011

// 3. 退出掉电模式
WriteRegister(0x04, 0x00);  // POWER_DOWN=0

// 4. 等待数据稳定
Delay(100ms);
```

### 3. STM32 代码示例 (基于项目)
```c
#include "xv7001bb.h"

void XV7001BB_Init(void) {
    uint8_t id;
    
    // 初始化 SPI 和 CS 引脚
    SPI_Init();
    CS_GPIO_Init();
    
    // 上电延迟
    HAL_Delay(50);
    
    // 读取 ID
    id = XV7001BB_ReadReg(XV7001BB_REG_WHO_AM_I);
    if (id != XV7001BB_DEVICE_ID) {
        Error_Handler();
    }
    
    // 配置 4 阶 50Hz 低通滤波器
    XV7001BB_WriteReg(XV7001BB_REG_CTRL, 0x23);
    
    // 退出掉电模式
    XV7001BB_WriteReg(XV7001BB_REG_MODE_CTRL, 0x00);
    
    // 稳定延迟
    HAL_Delay(100);
}
```

---

## 数据读取

### 读取角速率
```c
int16_t XV7001BB_ReadGyro(void) {
    uint8_t data[2];
    int16_t raw;
    
    // 读取 16-bit 数据 (LSB first)
    data[0] = XV7001BB_ReadReg(0x08);  // 低字节
    data[1] = XV7001BB_ReadReg(0x09);  // 高字节
    
    // 组合为 16-bit
    raw = (int16_t)((data[1] << 8) | data[0]);
    
    return raw;
}

float XV7001BB_GetAngularRate(void) {
    int16_t raw = XV7001BB_ReadGyro();
    
    // 转换为角速率 (°/s)
    return (float)raw * 0.0125f;
}
```

### 读取温度
```c
float XV7001BB_GetTemperature(void) {
    uint8_t data[2];
    int16_t raw;
    
    data[0] = XV7001BB_ReadReg(0x0A);
    data[1] = XV7001BB_ReadReg(0x0B);
    raw = (int16_t)((data[1] << 8) | data[0]);
    
    return (float)raw / 131.0f + 25.0f;
}
```

---

## 角度积分 (基于项目实现)

### 零偏校准
```c
#define CALIBRATION_SAMPLES 1000
#define SAMPLE_RATE_HZ 500  // 2ms 间隔

float calibrated_bias = 0.0f;

void XV7001BB_Calibrate(void) {
    float sum = 0.0f;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += XV7001BB_GetAngularRate();
        HAL_Delay(2);  // 2ms 采样间隔
    }
    
    calibrated_bias = sum / CALIBRATION_SAMPLES;
}
```

### 角度积分 (梯形法)
```c
float integrated_angle = 0.0f;
float last_rate = 0.0f;

void XV7001BB_UpdateAngle(void) {
    float current_rate = XV7001BB_GetAngularRate() - calibrated_bias;
    float dt = 0.002f;  // 2ms = 0.002s
    
    // 梯形积分
    integrated_angle += (last_rate + current_rate) * 0.5f * dt;
    last_rate = current_rate;
}
```

---

## 硬件设计建议

### 电源
- AVDD 和 DVDD 可共用同一 3.3V 电源
- 每个电源引脚添加 100nF 陶瓷电容 (靠近芯片)
- 额外添加 10µF 钽电容

### PCB 布局
- 将陀螺仪放置在 PCB 中心
- 避免高频信号线路经过陀螺仪下方
- 散热焊盘必须接地并良好散热

### SPI 信号
- SCLK、MOSI、MISO 添加 10-33Ω 串联电阻
- 保持走线短且等长
- 避免与高速信号并行走线

---

## 性能参数

### 灵敏度
- 典型值: 13.2 mV/(°/s)
- 范围: 11.88 ~ 14.52 mV/(°/s)

### 零偏
- 25°C: ±2°/s (典型)
- -40°C ~ +85°C: ±10°/s (最大)

### 噪声
- 角度随机游走: 0.03°/√Hz (典型)
- 零偏不稳定性: 0.5°/s (1σ, Allan variance)

### 带宽
- 3dB 带宽: 取决于滤波器配置 (5Hz ~ 500Hz)

---

## 故障排查

### 问题: 读取 ID 失败
**可能原因**:
- SPI 配置错误 (检查模式、时钟极性)
- CS 引脚未正确控制
- 电源未稳定

**解决方法**:
```c
// 验证 SPI 模式为 Mode 3
SPI_InitStruct.CLKPolarity = SPI_POLARITY_HIGH;  // CPOL=1
SPI_InitStruct.CLKPhase = SPI_PHASE_2EDGE;       // CPHA=1
```

### 问题: 数据抖动严重
**可能原因**:
- 未配置滤波器
- 采样频率过高
- 零偏未校准

**解决方法**:
```c
// 降低滤波器截止频率
XV7001BB_WriteReg(XV7001BB_REG_CTRL, 0x21);  // 25Hz

// 进行零偏校准
XV7001BB_Calibrate();
```

---

## 参考代码文件

项目中相关实现文件：
- [xv7001bb.c](../src/xv7001bb.c) - 驱动实现
- [xv7001bb.h](../include/xv7001bb.h) - 头文件定义
- [tim.c](../src/tim.c) - 定时器中断读取与积分

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-01-12 | 基于 XV7001BB_AE_Ver100 PDF 整理 |

---

**注意**: 本文档基于 Epson XV7001BB 官方数据手册整理，用于 STM32F103C8 项目开发参考。详细规格参数请查阅原始 PDF 文档。
