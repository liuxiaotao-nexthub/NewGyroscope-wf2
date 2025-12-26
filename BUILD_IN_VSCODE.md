# 在VS Code中编译此项目

本项目原为VisualGDB项目，现已配置可在VS Code中直接编译。

## 方法一：使用Makefile编译（推荐）

### 前置要求
1. **安装ARM GCC工具链**
   - 下载：[ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
   - 或使用VisualGDB自带的：`C:\SysGCC\arm-eabi`
   
2. **配置工具链路径**
   编辑 `Makefile` 第4行，修改为您的工具链路径：
   ```makefile
   TOOLCHAIN_ROOT = C:/SysGCC/arm-eabi
   ```

3. **安装Make工具**
   - Windows: 使用MinGW/MSYS2的make，或安装GnuWin32 Make
   - 确保`make`命令在PATH中

### 编译步骤
1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 "Tasks: Run Build Task"
3. 选择 **"Build STM32 (Makefile)"**

或直接在终端运行：
```bash
make all -j4
```

### 输出文件
编译成功后在 `VisualGDB/Debug/` 目录生成：
- `NewGyroscope.elf` - 可执行文件（用于调试）
- `NewGyroscope.bin` - 二进制文件（用于烧录）
- `NewGyroscope.hex` - Hex文件（用于烧录）
- `NewGyroscope.map` - 内存映射文件

### 清理
```bash
make clean
```

## 方法二：使用MSBuild（需要VS2022）

如果您安装了Visual Studio 2022和VisualGDB，可以使用MSBuild：

1. 确保VS2022在PATH中
2. 运行任务：**"Build via MSBuild (VisualGDB)"**

或在终端运行：
```bash
msbuild NewGyroscope.sln /p:Configuration=Debug /p:Platform=VisualGDB
```

## 常见问题

### 找不到工具链
错误：`arm-none-eabi-gcc: command not found`

**解决**：
1. 检查Makefile中的TOOLCHAIN_ROOT路径
2. 或将工具链bin目录添加到系统PATH

### 找不到BSP文件
错误：头文件找不到

**解决**：
确保VisualGDB的BSP已安装在：
```
C:\Users\liuxi\AppData\Local\VisualGDB\EmbeddedBSPs\arm-eabi\com.sysprogs.arm.stm32
```

### Make命令不存在
**解决**：
安装Make工具：
- MSYS2: `pacman -S make`
- 或下载 [GnuWin32 Make](http://gnuwin32.sourceforge.net/packages/make.htm)

## 烧录固件

编译完成后使用以下工具烧录：
- **ST-Link Utility** (推荐)
- **OpenOCD**
- **VisualGDB** 的 Program and Start 功能

烧录文件：`VisualGDB/Debug/NewGyroscope.bin`
