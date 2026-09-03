# car-mspm0 — 循迹小车主控 (MSPM0G3507)

2026 电赛 H 题小车底盘控制端，TI CCS 工程。

## 硬件

- 主控：MSPM0G3507（TI LaunchPad）
- 电机：MG513 霍尔编码器减速电机 ×2（DRV8701E 驱动板）
- 循迹：亚博八路灰度传感器（4051 多路复用，GPIO 扫描）
- 显示：OLED（软件 I2C，显示任务号与计时）
- 按键：PA9 发车 / PA15 待机下循环切换任务
- 调参：UART0 接 VOFA+（波形监控 + 在线改参）
- 联动：UART3 接 STM32 摆杆控制端

## 代码结构

| 文件 | 职责 |
|---|---|
| `empty.c` | 主循环：按键扫描、任务状态机、回 A 停车判定、速度剖面 |
| `APP/app.c/h` | 双轮速度环 PI、循迹位置环 PD、数字量质心计算 |
| `APP/vofa.c/h` | VOFA 串口命令解析（在线调参） |
| `BSP/car.c/h` | DRV8701 电机驱动（含板级反相/左右交换适配） |
| `BSP/grayscale_sensor.c/h` | 八路灰度 4051 扫描 |
| `BSP/usart.c/h` | UART0(DMA 环形缓冲) + UART3(STM32 联动协议) |
| `BSP/encoder.c/h` | 编码器四路 GPIO 双边沿计数 |
| `SYSTEM/pid.c/h` | 通用 PID（积分分离/前馈/死区） |
| `config.h` | 任务参数：速度档、匀加速时长、运行时长 |

## 任务与赛题对应

| 任务 | 赛题要求 | 实现 |
|---|---|---|
| TASK2 | 要求2：一圈回 A 停车计时 ≤20s | 29 档高速，三路冗余停车线识别，实测约 15.6s |
| TASK3 | 要求3：滚球 | 小车不动，PA9 发 `q3` 给 STM32 |
| TASK4 | 要求4：A→B ≤8s | 4s 匀加速→25 档，8s 定时停车，发车发 `q4` 稳 O 点 |
| TASK5 | 要求5：一圈 ≤30s 球稳 O | 4s 匀加速→18 档巡航，30s 定时停车 |
| TASK6 | 要求6：一圈球稳任意点 | 剖面同 TASK5，发 `q6:x.x` 指定停球位置（VOFA 可在线改） |

## 使用

1. CCS 导入工程，SysConfig 由 `empty.syscfg` 重新生成 `ti_msp_dl_config`
2. 上电默认 TASK2，PA15 切换任务，OLED 显示当前任务号
3. PA9 发车/启动；切任务与发车时自动通过 UART3 通知 STM32 联动

## 联动协议（MSP → STM32，ASCII 行 + \r\n）

`q3` 滚球 / `q4` 稳 O 点 / `q6:x.x` 稳任意点 / `stop` 全停回平。
STM32 回复 `qX_started` / `qX_start_rejected`，OLED 回显 LNK:OK/REJ。
