# can_normal

基于 STM32F407 的四麦克纳姆轮底盘控制程序。

数据流：**DR16 遥控器 → 逆运动学分配 → 3508 速度闭环 → CAN 电流输出**

## 硬件

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F407IGH6（UFBGA176，168 MHz） |
| 电机 | 4 × M3508 + C620 电调 |
| CAN | CAN1，1 Mbps |
| 遥控 | DJI DR16，USART3，100000 8N1，DMA 双缓冲 + IDLE 中断 |
| 调试串口 | USART6，115200 8N1，可接 SerialPlot 看波形 |
| 板载 LED | PH10 / PH11 / PH12 |

## 代码结构

```
can_normal/
├── Core/            CubeMX 生成的外设初始化、中断入口、main
├── Drivers/         STM32 HAL 库
├── MDK-ARM/         Keil 工程文件
├── user/            RoboMaster 参考代码（多数已不参与链接）
└── user1/           本项目自研模块
```

### user1/ 各模块职责

| 文件 | 职责 |
| --- | --- |
| `dr16.c/h` | DR16 接收：DMA 双缓冲 + IDLE 中断，18 字节帧解析，掉线检测与自动重连 |
| `can_assist.c/h` | 分配层：读摇杆 → 麦克纳姆逆运动学 → 四个轮子的目标转速 |
| `speed_ctrl.c/h` | 伺服层：每个轮子一个 PID，目标转速 → CAN 电流 |
| `uart_assist.c/h` | USART6 调试输出：串口指令切换「正弦波 / 摇杆数据」两种波形 |
| `bsp_led.c/h` | 三个板载 LED 的开关封装 |

### user/ 各模块职责

| 文件 | 状态 |
| --- | --- |
| `bsp_can.c/h` | ✅ 在用：CAN 滤波器配置 + 启动 |
| `CAN_receive.c/h` | ✅ 部分在用：底盘收发（云台部分未链接） |
| `pid.c/h` | ✅ 在用：DJI 通用 PID |
| `struct_typedef.h` | ✅ 在用：`fp32` / `fp64` |
| `bsp_rc.c/h` | ❌ 死代码，已被 `user1/dr16.c` 取代 |
| `remote_control.c/h` | ❌ 死代码，已被 `user1/dr16.c` 取代 |

## 控制流程

```
DR16 (USART3)  ──18字节──▶  dr16_parse()  ──▶  ch[2] / ch[3]
                                                    │
                                          Dr16_Get_Channel_Norm()
                                                    │  −1.0 ~ +1.0
                                                    ▼
                              can_assist.c  逆运动学分配
                              mix[4] = forward·dir[] + strafe·strafe_dir[]
                                                    │  ±1000
                                                    ▼
                              speed_ctrl.c  四路速度闭环
                              PID(反馈 = speed_rpm, 目标 = mix)
                                                    │  ±1000 counts (≈ ±1.22 A)
                                                    ▼
                              CAN_cmd_chassis()  ──0x200──▶  四个 C620
```

麦克纳姆轮方向系数：

```c
can_assist_dir[4]        = { 1,  1, -1, -1};   // 前后
can_assist_strafe_dir[4] = { 1, -1, -1,  1};   // 平移
```

## 关键参数

| 宏 | 位置 | 当前值 | 含义 |
| --- | --- | --- | --- |
| `CAN_ASSIST_CH_FORWARD` | `can_assist.c:9` | 3 | 左摇杆水平 → 前后 |
| `CAN_ASSIST_CH_STRAFE` | `can_assist.c:10` | 2 | 左摇杆垂直 → 平移 |
| `CAN_ASSIST_CURRENT_MAX` | `can_assist.c:11` | 1000 | 摇杆满量程对应的 `mix` 值（命名有误，实际等于目标 rpm） |
| `SPEED_CTRL_MAX_RPM` | `speed_ctrl.c:6` | 1000 | 摇杆推到底的目标转速 |
| `SPEED_CTRL_CURRENT_MAX` | `speed_ctrl.c:7` | 1000 | PID 输出电流上限，约 1.22 A |
| `SPEED_CTRL_IOUT_MAX` | `speed_ctrl.c:8` | 500 | 积分项上限，约 0.61 A |
| PID 参数 | `speed_ctrl.c:10` | Kp 10 / Ki 0.2 / Kd 0 | 位置式，无 dt |

C620 电流换算：`±16384 counts ↔ ±20 A`，即 **1 A ≈ 819.2 counts**。

## 编译

用 Keil MDK 打开 `MDK-ARM/can_normal.uvprojx`，直接 Build（F7）。

命令行方式：

```sh
UV4.exe -b MDK-ARM/can_normal.uvprojx -j0 -o build_check.log
```

## 注意事项

- **`user/` 目录只读**，新的模块代码一律放在 `user1/`。
- `user/bsp_rc.h` 声明了 `RC_unable()` 和 `RC_restart()`，但两者**从未定义**。当前没人调用所以不报错，一旦引用会链接失败。
- 逆运动学缺少**归一化**：对角线方向（`forward` 与 `strafe` 同时非零）时，每个轮子会被单独夹到 ±1000，导致运动方向被扭曲。正确做法是按 `|forward| + |strafe|` 等比缩放。
- `CAN_ASSIST_CURRENT_MAX`、`SPEED_CTRL_MAX_RPM`、`SPEED_CTRL_CURRENT_MAX` 三个常量都是 1000，但含义各不相同，改动前先确认自己改的是哪一个。
- `speed_ctrl.c:56-63` 的目标转速钳位在当前参数下不会生效（上游已夹在 ±1000）。
