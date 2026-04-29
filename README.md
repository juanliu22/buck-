# Buck 恒压恒流控制系统

基于 STM32F103 的 Buck 降压电路控制工程。项目使用 STM32 HAL 库实现 PWM 输出、INA226 电压电流采样、串口调参和恒压/恒流自动切换控制。

## 功能特点

- STM32F103xB 主控，工程由 STM32CubeMX / Makefile 生成
- TIM1 CH1 输出 PWM 控制 Buck 占空比
- INA226 通过 I2C 采集输出电压和电流
- 增量式 PID 控制，占空比限制在 5% 到 95%
- 支持恒压 CV / 恒流 CC 自动切换
- USART1 串口输出实时状态，并支持在线设置目标电压和限流值

## 主要参数

| 参数 | 默认值 | 范围 |
| --- | --- | --- |
| 目标电压 | 5.0 V | 5.0 V 到 12.0 V |
| 电流限制 | 2.0 A | 0.5 A 到 2.0 A |
| 串口波特率 | 115200 | 8N1 |
| 控制周期 | 约 10 ms | - |

## 目录结构

```text
.
├── project/                 # STM32 工程源码
│   ├── Core/                # 用户代码和中断代码
│   ├── Drivers/             # CMSIS 与 STM32 HAL 驱动
│   ├── Makefile             # arm-none-eabi-gcc 构建脚本
│   ├── project.ioc          # STM32CubeMX 配置文件
│   └── STM32F103XX_FLASH.ld # 链接脚本
├── .eide/                   # EIDE 工程配置
├── .pack/                   # Keil STM32F1 芯片包
├── .vscode/                 # VS Code 任务配置
└── RTE_Components.h
```

## 编译方法

进入 `project` 目录后执行：

```bash
make
```

编译产物会生成在：

```text
project/build/
```

常用输出文件包括：

- `project.elf`
- `project.hex`
- `project.bin`

如果 `arm-none-eabi-gcc` 没有加入系统 PATH，可以在编译时指定工具链路径：

```bash
make GCC_PATH=/path/to/gcc-arm-none-eabi/bin
```

## 串口调参

USART1 配置为 `115200 8N1`。程序每秒输出一次当前工作状态，包括模式、电压、电流、限流值和占空比。

发送目标电压数值并以非数字字符结束，可修改恒压目标：

```text
5.0
9.0
12.0
```

发送 `I` 或 `i` 开头的数值，可修改限流值：

```text
I1.0
I1.5
I2.0
```

有效范围：

- 电压：`5.0` 到 `12.0`
- 限流：`0.5` 到 `2.0`

## 控制逻辑

主循环每 10 ms 执行一次 `Control_Loop()`：

1. 读取 INA226 输出电压和电流。
2. 当输出电流达到限流值时进入恒流模式。
3. 否则进入恒压模式。
4. 通过增量式 PID 调整 TIM1 PWM 占空比。
5. 每秒通过 USART1 输出一次运行状态。

## 注意事项

- `build/` 已加入 `.gitignore`，编译产物不会提交到仓库。
- INA226 电流换算系数与采样电阻、硬件校准有关，移植到其他板子时需要重新校准。
- 上电调试 Buck 电源时建议先使用限流电源，并从低占空比、低负载条件开始验证。
