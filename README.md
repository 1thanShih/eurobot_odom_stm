# eurobot_odom_stm

Eurobot 機器人的 odometry 韌體，跑在 **STM32 NUCLEO-F446RE** 上。

STM32 用 Timer 的 Encoder mode 讀取 4 個 odometry pod（被動輪 + encoder）的計數，
每 10 ms 透過 USART2（Nucleo 的 USB 虛擬 COM port）送一行 CSV 給上位機。
機器人的位姿 (x, y, θ) 由上位機計算，`tools/odom_host.py` 是一份參考實作。

```
odometry pod ×4 ──encoder──▶ STM32F446RE ──UART CSV (115200)──▶ 上位機 (PC / 樹莓派)
                              讀計數、處理溢位                    算 x, y, θ
```

## 硬體接線

| Pod  | Timer | 腳位 (A / B) | 計數器 |
|------|-------|--------------|--------|
| pod1 | TIM2  | PA0 / PA1    | 32-bit |
| pod2 | TIM3  | PA6 / PA7    | 16-bit |
| pod3 | TIM1  | PA8 / PA9    | 16-bit |
| pod4 | TIM4  | PB6 / PB7    | 16-bit |

- UART：USART2，PA2 (TX) / PA3 (RX)，115200 baud，8N1。插 USB 就會出現虛擬 COM port。
- 腳位設定在 [`Eurobot odometry.ioc`](Eurobot%20odometry.ioc)，接線表在 [`Core/Src/odometry.c`](Core/Src/odometry.c)，兩邊要一致。

## 專案結構

```
Core/
  Inc/robot_config.h   機器人參數（輪徑、每圈 count 數、各 pod 方向、取樣週期）
  Src/app.c            主流程：app_init() / app_loop()，相當於 Arduino 的 setup / loop
  Src/odometry.c       讀 encoder 計數、處理溢位、累計總量
  Src/telemetry.c      把資料格式化成 CSV 從 UART 送出
  Src/main.c           CubeMX 產生，只做硬體初始化後呼叫 app_init / app_loop
tests/host/            在電腦上跑的單元測試（不需要板子）
tools/odom_host.py     上位機：讀 CSV，算出機器人位姿
```

自己寫的程式都在 `app.c` / `odometry.c` / `telemetry.c`，新增功能時不需要改 `main.c`，
CubeMX 重新產生程式碼也不會被蓋掉。

## 編譯與燒錄

1. 用 **STM32CubeIDE** 開啟：`File → Import → Existing Projects into Workspace`，選這個資料夾。
2. `Project → Build Project`。
3. 接上 Nucleo，用 `Eurobot odometry.launch` 執行 Run / Debug 燒錄。

## 參數設定

校正或換零件時只需要改 [`Core/Inc/robot_config.h`](Core/Inc/robot_config.h)：

| 參數 | 說明 |
|------|------|
| `ODOM_COUNTS_PER_REV` | 輪子轉一圈的 count 數（已含 4 倍頻），預設 2000 |
| `ODOM_WHEEL_DIAMETER_MM` | 輪徑，預設 32 mm，實測校正後更新 |
| `ODOM_PODx_DIRECTION` | 往前推時數值變負，就把 `+1` 改成 `-1` |
| `ODOM_SAMPLE_PERIOD_MS` | 取樣與傳輸週期，預設 10 ms |

一行 CSV 在 115200 baud 下大約要 6~7 ms，所以取樣週期不能再縮短。
要提高取樣率的話，CubeMX 的 USART2 baud rate 和上位機要一起改。

## 輸出格式

每行一筆 CSV，以 `\r\n` 結尾，共 14 個欄位：

```
seq, ts_ms, total1..4, delta1..4, mm1..4
```

| 欄位 | 說明 |
|------|------|
| `seq` | 0~255 循環，上位機可以用來偵測掉行 |
| `ts_ms` | 取樣時間（`HAL_GetTick()`，ms）。週期不保證完全固定，算速度請用 `ts_ms` 的差值 |
| `total1..4` | 開機以來的累計 count |
| `delta1..4` | 和上一筆相比的變化量 |
| `mm1..4` | 累計距離（mm，小數 2 位） |

格式定義在 [`Core/Inc/telemetry.h`](Core/Inc/telemetry.h)。改格式時要同步更新上位機的解析程式。

## 上位機

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
