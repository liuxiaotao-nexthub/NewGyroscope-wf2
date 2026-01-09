# STM32F1 Gyroscope-Based AGV Control System

## Project Overview
Embedded system controlling a two-wheeled AGV (Automated Guided Vehicle) using **STM32F103C8** microcontroller, **XV7001BB gyroscope**, and **FreeRTOS**. System integrates gyroscope-based rotation control with CAN bus motor commands for precise navigation.

## Architecture

### Core Components
- **XV7001BB Gyroscope** ([xv7001bb.c](xv7001bb.c)): SPI-based angular rate sensor, configured with 4th-order 50Hz low-pass filter
- **CAN Bus Motor Control** ([car.c](car.c), [can.c](can.c)): Communicates with two wheel motors via standardized CAN protocol
- **PID Controller** ([PID.c](PID.c)): Angle error correction during linear motion (Kp=1.0, Ki/Kd=0)
- **Timer Integration** ([tim.c](tim.c)): TIM2 interrupt every 2ms reads gyroscope, integrates angle via FreeRTOS queue

### Task Architecture (FreeRTOS CMSIS-RTOS)
1. **CAN_SendTask**: 10ms periodic - publishes gyro status/temp/rate/angle to CAN (IDs 0x001-0x004)
2. **CAN_ControlTask**: Receives CAN frames, routes motor feedback (0x312, 0x409) to `CarCanQueueHandle`
3. **Gyro_ProcessTask**: Consumes gyro queue, detects zero-bias stability, integrates angle
4. **Car_LockTask** ([car_task.c](car_task.c)): State machine for motor binding/init/calibration/enable sequence
5. **Car_TestTask**: User test routines (forward/backward/turn with PID angle correction)
6. **Main_Task**: Blinks PB9 LED (100ms heartbeat)

### Data Flow
```
TIM2 IRQ (2ms) → Read XV7001BB SPI → gyroQueueHandle (float) 
→ Gyro_ProcessTask → Zero-bias detection → Angle integration
→ CAN_SendTask broadcasts angle → Car motion commands use angle for PID correction
```

## Build System
- **VisualGDB**: Primary IDE integration (Visual Studio + GDB)
- **Toolchain**: ARM GCC 14.2.1 (`arm-none-eabi-gcc`)
- **Target**: STM32F103C8 (Cortex-M3, 72MHz, 64KB flash, 20KB RAM)
- **Linker**: `STM32F103C8_flash.lds` with nano.specs/nosys.specs
- **HAL**: STM32F1xx HAL Driver (configured in [stm32f1xx_hal_conf.h](stm32f1xx_hal_conf.h))
- **Build artifacts**: [VisualGDB/Debug/](VisualGDB/Debug/) contains `.dep`, `.gcc.rsp`, `.link.rsp` files

### FreeRTOS Configuration
- Heap: `heap_4.c` (deterministic allocation)
- Tick rate: 1000 Hz (1ms)
- Stack sizes: 64-512 words per task
- Queue discipline: FIFO message queues (`osMessageQ`)

## Critical Patterns

### Motor Control Protocol
- **Device IDs**: 0x01=left wheel, 0x02=right wheel (calibrated by checking displacement direction in `Car_LockTask`)
- **CAN IDs**: 0x312=SN report, 0x313=bind, 0x316=init params, 0x408=position read, 0x409=position feedback, 0x413=power enable, 0x60D=lock/query
- **Position encoding**: 8-byte CAN frame → 32-bit signed integer (mm) via `Car_ParseMotorPosition()`
- **Wheel geometry**: 77.25mm diameter (effective), 764mm wheelbase (for 180° = 1200mm per wheel)

### Gyroscope Integration Strategy
1. **Zero-bias calibration** ([tim.c](tim.c)): 1000-sample sliding window, detects stability when 10 consecutive 100-sample segments have variance < 1.0
2. **Bias update**: Running average of stable samples (σ² < 0.35 threshold)
3. **Integration**: Trapezoidal method, dt=2ms, accumulates only when `bias_initialized==1`
4. **Fixed-point CAN**: Temp×100, Rate×10000, Angle×10000 (sign byte + big-endian magnitude)

### PID Angle Correction
During `Car_MoveForward/Backward()`, gyro angle drift is corrected via `PID_Calculate()`:
- **Error**: target_angle - actual_angle
- **Output**: Differential wheel displacement (mm), clamped to ±10mm
- **Application**: Right wheel gets `-output`, left wheel gets `+output` (corrects clockwise drift)
- Example: 0.6° drift → ~4mm correction per control loop

## Development Workflow

### Debugging
- **GDB variables**: Modify `g_linear_velocity`, `g_rotation_velocity`, `g_acceleration`, `g_pid_kp` at runtime
- **Angle monitoring**: Watch `g_angle_error`, `g_pid_output`, `integrator.angle`, `bias`
- **CAN traffic**: Check `CAN_RxFlag`, `CAN_RxStdId`, `CAN_RxData[]` globals in [can.c](can.c)

### Code Organization
- **Header guards**: Use `#ifndef __MODULE_H` pattern
- **Chinese comments**: Mixed Chinese/English (陀螺仪=gyroscope, 小车=car/AGV)
- **Task naming**: `Module_FunctionTask()` for FreeRTOS entry points
- **Hardware abstraction**: SPI/CAN/TIM in separate modules, HAL wrappers (`SPI_CS_LOW()`, `CAN_SendData()`)

### Common Operations
- **Add CAN message**: Update `CAN_ControlTask()` routing, add handler in [car_task.c](car_task.c) state machine
- **Tune PID**: Adjust `g_pid_kp/ki/kd` in [PID.c](PID.c), monitor `g_pid_output` via GDB
- **Change gyro filter**: Modify reg 0x02 write in `XV7001BB_Init()` (bits[5:4]=order, bits[3:0]=cutoff)
- **Adjust integration rate**: Change TIM2 period (currently 2000 ticks @ 72MHz/72 = 2ms)

## File Conventions
- **Peripheral drivers**: `module.c` + `module.h` (can, spi, tim, xv7001bb)
- **Application logic**: `module_task.c` + `module_task.h` (can_task, car_task)
- **Config files**: `FreeRTOSConfig.h`, `stm32f1xx_hal_conf.h`, `stm32.props`
- **Entry point**: [NewGyroscope.c](NewGyroscope.c) - system init, clock config, task creation

## External Dependencies
- **BSP_ROOT**: VisualGDB BSP with FreeRTOS source, STM32F1 HAL, CMSIS, startup files
- **Include paths**: FreeRTOS headers via `<../CMSIS_RTOS/cmsis_os.h>` (unusual upward relative path)
- **No external libraries**: Pure HAL + FreeRTOS, no middleware
