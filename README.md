# eurobot_odom_stm

Eurobot 機器人的 odometry 韌體，跑在 **STM32 NUCLEO-F446RE** 上。

STM32 用 Timer 的 Encoder mode 讀取 4 個 odometry pod（被動輪 + encoder）的計數，
透過 USART2（Nucleo 的 USB 虛擬 COM port）送給上位機。機器人的位姿 (x, y, θ) 由上位機計算。

輸出有兩種模式，在 [`Core/Inc/robot_config.h`](Core/Inc/robot_config.h) 用 `APP_OUTPUT_MICROROS` 切換：

| 模式 | 設定 | 上位機要跑什麼 | 用途 |
|------|------|----------------|------|
| **micro-ROS**（預設） | `1` | micro-ROS agent（ROS 2 Humble） | 正式使用，資料直接進 ROS 2 |
| **CSV** | `0` | 任何序列埠工具，或 `tools/odom_host.py` | 除錯，人眼可讀 |

```
odometry pod ×4 ──encoder──▶ STM32F446RE ──USART2 460800──▶ 上位機 (PC / 樹莓派)
                              FreeRTOS                        micro-ROS agent 或 CSV
                              讀計數、處理溢位                 算 x, y, θ
```

> ⚠️ 目前 micro-ROS 模式還是「環境測試」版本：連上 agent 後只在 `/stm32_test` 發布一個遞增的
> `std_msgs/Int32`，還沒有發布 odometry 資料。

## 硬體

| Pod  | Timer | 腳位 (A / B) | 計數器 |
|------|-------|--------------|--------|
| pod1 | TIM2  | PA0 / PA1    | 32-bit |
| pod2 | TIM3  | PA6 / PA7    | 16-bit |
| pod3 | TIM1  | PA8 / PA9    | 16-bit |
| pod4 | TIM4  | PB6 / PB7    | 16-bit |

- **UART**：USART2，PA2 (TX) / PA3 (RX)，**460800 baud**，8N1，RX/TX 都用 DMA。插 USB 就會出現虛擬 COM port。
- **LED2**（板上綠燈，micro-ROS 模式）：熄滅 = 等待 agent，亮起 = 已連線。
- **時脈**：HSI 16 MHz → PLL → 84 MHz。HAL 時基用 TIM6（SysTick 給 FreeRTOS）。
- 腳位設定在 [`eurobot_odom_stm.ioc`](eurobot_odom_stm.ioc)，接線表在 [`Core/Src/odometry.c`](Core/Src/odometry.c)，兩邊要一致。

## 專案結構

```
Core/
  Inc/robot_config.h      機器人參數（輪徑、每圈 count 數、各 pod 方向、取樣週期、輸出模式）
  Src/app.c               主流程：app_init()（開機一次）/ app_task()（FreeRTOS task，依模式分流）
  Src/odometry.c          讀 encoder 計數、處理溢位、累計總量
  Src/microros_app.c      micro-ROS：連線狀態機（等 agent → 連線 → 斷線自動重連）與發布
  Src/telemetry.c         CSV 模式：把資料格式化成一行文字從 UART 送出
  Src/microros/           micro-ROS 官方提供的 transport / allocator / time（從 micro_ros_stm32cubemx_utils 複製）
  Src/main.c              CubeMX 產生：硬體初始化、啟動 FreeRTOS，再呼叫 app_init / app_task
  Src/freertos.c, ...     CubeMX 產生
Middlewares/              FreeRTOS（CubeMX 產生）
micro_ros_stm32cubemx_utils/
  microros_static_library_ide/libmicroros/   編好的 micro-ROS 函式庫（libmicroros.a + include）
tests/host/               在電腦上跑的單元測試（不需要板子）
tools/odom_host.py        上位機參考實作（CSV 模式）：讀 CSV，算出機器人位姿
```

自己寫的程式都在 `app.c` / `odometry.c` / `microros_app.c` / `telemetry.c`。
`main.c` 裡我們只加了 `#include "app.h"`、`app_init();`、`app_task();` 三行，都在 `USER CODE` 區塊內，
CubeMX 重新產生程式碼時不會被蓋掉。

## 編譯與燒錄

1. 用 **STM32CubeIDE** 開啟：`File → Import → Existing Projects into Workspace`，選這個資料夾。
2. `Project → Build Project`。micro-ROS 函式庫已經編好放在 repo 裡，不需要另外裝 Docker。
3. 接上 Nucleo，用 `eurobot_odom_stm.launch` 執行 Run / Debug 燒錄。

### 修改硬體設定（CubeMX）

STM32CubeIDE 2.x 已經不內建 CubeMX，要另外安裝 **STM32CubeMX 6.x**（不是 CubeMX2，它不支援 F4）。
用它打開 `eurobot_odom_stm.ioc`，改完按 Generate Code。

- 同一時間只讓一個人改 `.ioc`，改完的 `.ioc` 和產生的程式碼放在同一個 commit。
- 自己的程式一定要寫在 `/* USER CODE BEGIN */ ... /* USER CODE END */` 之間，區塊外的內容重新產生時會消失。
- micro-ROS 需要的設定不要動：FreeRTOS（CMSIS_V2）、defaultTask stack 3000 words、
  `TOTAL_HEAP_SIZE` 30000、`USE_NEWLIB_REENTRANT` Enabled、Timebase = TIM6、
  USART2 DMA（RX Circular / TX Normal）與 USART2 global interrupt。
- `TOTAL_HEAP_SIZE` 不能隨便加大：micro-ROS 的 `custom_memory_manager.c` 會再開一塊同樣大小的 heap，
  實際 RAM 用量是兩倍（F446 只有 128 KB）。

### 重新編譯 micro-ROS 函式庫（通常不需要）

只有在要換 ROS 2 版本、加自訂訊息、或改編譯旗標時才需要。需要 Docker Desktop：

```bash
# 先在 CubeIDE Build 一次專案（腳本要從 Debug/ 的 .mk 檔讀編譯旗標）
rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
docker run --rm -v "<專案完整路徑>:/project" \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide \
  microros/micro_ros_static_library_builder:humble
```

`micro_ros_stm32cubemx_utils/` 是官方 [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
`humble` 分支（commit `bd531b2`）的複本，只改了它的 `.gitignore`，讓編好的函式庫可以進版控。

## 參數設定

校正或換零件時只需要改 [`Core/Inc/robot_config.h`](Core/Inc/robot_config.h)：

| 參數 | 說明 |
|------|------|
| `APP_OUTPUT_MICROROS` | `1` = micro-ROS，`0` = CSV |
| `ODOM_COUNTS_PER_REV` | 輪子轉一圈的 count 數（已含 4 倍頻），預設 2000 |
| `ODOM_WHEEL_DIAMETER_MM` | 輪徑，預設 32 mm，實測校正後更新 |
| `ODOM_PODx_DIRECTION` | 往前推時數值變負，就把 `+1` 改成 `-1` |
| `ODOM_SAMPLE_PERIOD_MS` | CSV 模式的取樣與傳輸週期，預設 10 ms |

## CSV 模式的輸出格式

每行一筆 CSV，以 `\r\n` 結尾，共 14 個欄位：

```
seq, ts_ms, total1..4, delta1..4, mm1..4
```

| 欄位 | 說明 |
|------|------|
| `seq` | 0~255 循環，上位機可以用來偵測掉行 |
| `ts_ms` | 取樣時間（`HAL_GetTick()`，ms）。算速度請用 `ts_ms` 的差值 |
| `total1..4` | 開機以來的累計 count |
| `delta1..4` | 和上一筆相比的變化量 |
| `mm1..4` | 累計距離（mm，小數 2 位） |

格式定義在 [`Core/Inc/telemetry.h`](Core/Inc/telemetry.h)。改格式時要同步更新上位機的解析程式。

上位機參考實作：

```bash
pip install pyserial
python tools/odom_host.py COM5            # Windows（COM 編號看裝置管理員）
python tools/odom_host.py /dev/ttyACM0    # Linux / 樹莓派
```

使用前先把 `odom_host.py` 開頭的 `PODS` 改成實際的安裝位置與量測方向。
至少要 3 個 pod，而且不能全部平行，才能同時算出 x、y、θ。
座標約定和 ROS 相同：x 朝前、y 朝左、θ 逆時針為正；開機時的位置是原點。

## 單元測試

不需要板子，在專案根目錄用 gcc 編譯執行：

```bash
gcc -std=gnu11 -Wall -Wextra -Wshadow -Werror \
    -Itests/host/fake_hal -ICore/Inc \
    tests/host/test_odometry.c Core/Src/odometry.c Core/Src/telemetry.c \
    -o test_odometry && ./test_odometry
```

`tests/host/fake_hal/main.h` 用假的 HAL 取代硬體，測試計數器溢位處理、初始化和 CSV 輸出格式。
