# eurobot_odom_stm

Eurobot 機器人的 odometry 韌體，跑在 **STM32 NUCLEO-F446RE** 上。

STM32 讀 4 個 odometry pod 的 encoder 和一顆 BNO085 IMU，透過 **micro-ROS** 發布到 ROS 2。
位姿 (x, y, θ) 由上位機計算。

```
odometry pod ×4 ──encoder──▶ STM32F446RE ──USART2 1 Mbaud (micro-ROS)──▶ 上位機 micro-ROS agent ──▶ ROS 2
BNO085 ─────────I2C1─────────────▲
```

## 發布的資料

| 項目 | 內容 |
|------|------|
| Node | `odom_stm32` |
| Topic | `/wheel_counts` |
| Type | `std_msgs/Int32MultiArray`，QoS best effort |
| 週期 | 10 ms（`robot_config.h` 的 `ODOM_SAMPLE_PERIOD_MS`） |
| `data[0]` | stamp：STM32 開機後的毫秒數（讀 encoder 的時間點） |
| `data[1..4]` | pod1~4 的累計 count |

- count 是累計值，算位移時用「這一筆 − 上一筆」，掉幾筆也不會影響位置。
- stamp 是 STM32 自己的時鐘，不是 ROS 時間，用來算 dt。

| 項目 | 內容 |
|------|------|
| Topic | `/imu` |
| Type | `sensor_msgs/Imu`，QoS best effort，`frame_id` = `imu_link` |
| 週期 | 約 10 ms（`robot_config.h` 的 `IMU_REPORT_PERIOD_US`），有新資料才發；沒接 IMU 就不發 |
| `header.stamp` | 姿態取樣時間，STM32 開機時間（和 `/wheel_counts` 的 `data[0]` 同一個時鐘） |
| `orientation` | Game Rotation Vector（陀螺儀 + 加速度計，**不用磁力計**），yaw 會慢慢漂 |
| `angular_velocity` | Gyroscope Calibrated（rad/s） |
| `linear_acceleration` | Accelerometer（m/s²，含重力） |
| 共變異數 | 只填對角線，估計值（`robot_config.h` 的 `IMU_*_VAR`） |

- IMU 不一定裝得跟車身同方向，上位機要發 `base_link → imu_link` 的 static TF。

## 硬體

| Pod  | Timer | 腳位 (A / B) | 計數器 |
|------|-------|--------------|--------|
| pod1 | TIM2  | PA0 / PA1    | 32-bit |
| pod2 | TIM3  | PA6 / PA7    | 16-bit |
| pod3 | TIM1  | PA8 / PA9    | 16-bit |
| pod4 | TIM4  | PB6 / PB7    | 16-bit |

**IMU（Adafruit BNO085，I2C 位址 0x4A）**

| BNO085 | Nucleo 排針 | STM32 | 用途 |
|--------|-------------|-------|------|
| VIN    | 3V3         | —     | 電源 |
| GND    | GND         | —     | |
| SCL    | D15         | PB8   | I2C1_SCL（400 kHz，上拉在 Adafruit 板上） |
| SDA    | D14         | PB9   | I2C1_SDA |
| INT    | D5          | PB4   | 有資料時拉低（EXTI4） |
| RST    | D4          | PB5   | 拉低 = 重置 |

P0 / P1 / DI 不接（預設就是 I2C 模式、位址 0x4A）。

- **UART**：USART2（Nucleo 的 USB 虛擬 COM port），1,000,000 baud（42 MHz 剛好整除，誤差 0%），RX/TX 都用 DMA。
  上位機的 agent 也要用同樣的速度：`micro-ros-agent serial --dev /dev/ttyACM0 -b 1000000`。
- **LED2**（綠燈）：熄滅 = 等待 agent，亮起 = 已連線。agent 晚開或斷線會自動重連。
- 腳位在 [`eurobot_odom_stm.ioc`](eurobot_odom_stm.ioc) 設定，接線表在 [`odometry.c`](Core/Src/odometry.c)，兩邊要一致。

## 專案結構

```
Core/Inc/robot_config.h    可調參數：pod 方向、週期

Core/Src/app.c             進入點：app_init() / app_task()

Core/Src/odometry.c        init encoder、處理溢位、累計 count
Core/Src/imu.c             BNO085 task：初始化、開報告、存快照
Core/Src/imu_hal.c         sh2 函式庫的硬體介面（I2C1、RST、INT、微秒時鐘）

Core/Src/microros_app.c    micro-ROS 連線與發布
Core/Src/microros/         micro-ROS 官方的 transport / allocator（不用改）

Core/Src/main.c 等         CubeMX 產生

Middlewares/               FreeRTOS（CubeMX 產生）
Middlewares/Third_Party/sh2/   CEVA 的 BNO08x 驅動（SH-2 協定）
micro_ros_stm32cubemx_utils/   micro-ROS 函式庫（已編好）

tests/host/                電腦上跑的單元測試
tools/odom_host.py         從 count 算 (x, y, θ) 
```

`main.c` 裡我們只加了三行（`#include "app.h"`、`app_init();`、`app_task();`），都在 `USER CODE` 區塊內。

## 編譯與燒錄

1. **STM32CubeIDE**：`File → Import → Existing Projects into Workspace`，選這個資料夾。
2. `Project → Build Project`（micro-ROS 函式庫已經在 repo 裡，不用裝 Docker）。
3. 用 `eurobot_odom_stm.launch` 燒錄。

## 改硬體設定（CubeMX）

要另外裝 **STM32CubeMX 6.x**（CubeIDE 2.x 不內建，CubeMX2 不支援 F4），打開 `.ioc` 改完按 Generate Code。

- 同一時間只讓一個人改 `.ioc`。
- 自己的程式要寫在 `USER CODE BEGIN / END` 之間，否則重新產生時會消失。
- 這些是 micro-ROS 需要的，不要改：FreeRTOS（CMSIS_V2）、defaultTask stack 3000 words、
  `TOTAL_HEAP_SIZE` 30000、`USE_NEWLIB_REENTRANT`、Timebase = TIM6、USART2 DMA（RX Circular）。
- `TOTAL_HEAP_SIZE` 不要加大：micro-ROS 會再開一塊一樣大的 heap，RAM（128 KB）會不夠。

## 重新編譯 micro-ROS 函式庫（通常不需要）

只有換 ROS 2 版本或加自訂訊息時才要。需要 Docker：

```bash
# 先在 CubeIDE Build 一次（腳本從 Debug/ 的 .mk 讀編譯旗標）
rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
docker run --rm -v "<專案完整路徑>:/project" \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide \
  microros/micro_ros_static_library_builder:humble
```

`Middlewares/Third_Party/sh2/` 來自 [ceva-dsp/sh2](https://github.com/ceva-dsp/sh2)
（commit `b514b1e`，Apache-2.0），沒有修改。

`micro_ros_stm32cubemx_utils/` 來自官方 [repo](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
的 `humble` 分支（commit `bd531b2`），只改了 `.gitignore`。

## 單元測試

不需要板子，在專案根目錄執行：

```bash
gcc -std=gnu11 -Wall -Wextra -Wshadow -Werror \
    -Itests/host/fake_hal -ICore/Inc \
    tests/host/test_odometry.c Core/Src/odometry.c \
    -o test_odometry && ./test_odometry
```
