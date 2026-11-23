# eurobot_odom_stm

2027/Eurobot主機器人的/Odom and Imu韌體，主板使用 **WeAct Black Pill（STM32F411CEU6）** 。

***內容：***

用 STM32 讀 4 個 odometry pod 的和一顆 **BNO055** IMU，透過 **micro-ROS** 發布到 ROS 2。
下位機只送原始資料（累計 count、IMU 讀值）

```
odometry pod ×4 and imu(BNO055) -> STM32F411CE Type-C USB (micro-ROS) -> micro-ROS agent -> ROS 2
```

---

## 1. Quick Start

1. Type-C 接上電腦（同時供電）。Linux 會出現 `/dev/ttyACM0`，Windows 會出現 `COMx`。
2. 開 micro-ROS agent（鮑率沒有作用，隨便填）：
   ```bash
   # 已安裝 micro_ros_agent
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   # 或用 Docker
   docker run -d --name uros_agent --device=/dev/ttyACM0 \
     microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
   ```
3. 板上藍色 LED 亮起 = 已連上 agent。
4. 看資料（**一定要設 `ROS_DOMAIN_ID=4`**）：
   ```bash
   export ROS_DOMAIN_ID=4
   ros2 topic list                                   # 應該有 /wheel_counts、/imu/data_raw
   ros2 topic hz /wheel_counts                       # 約 100 Hz
   ros2 topic echo /imu/data_raw --qos-reliability best_effort
   ```
---

## 2. 發布的資料

| 項目 | 值 |
|------|----|
| Node | `odom_stm32` |
| ROS_DOMAIN_ID | **4** |
| QoS | **best effort w/ best effort / `qos_profile_sensor_data`**  用 reliable 收不到 |
| Freq | 100 Hz |

### `/wheel_counts`（`std_msgs/Int32MultiArray`）

| 欄位 | 內容 |
|------|------|
| `data[0]` | stamp：讀 encoder 那一刻的 STM32 開機時間（ms） |
| `data[1..4]` | pod1~4 開機以來的累計 count（int32，已乘上方向 ±1） |

- count：位移 = 這一筆 − 上一筆。中間掉幾筆也不會讓位置算錯，只是那一段解析度變差。
- 4 個 count 是同一時間讀的，硬體 4 倍頻計數（A、B 相的上升和下降緣都算）。

### IMU：原始值，姿態由上位機融合

BNO055 跑在 **ACCGYRO 模式**：只開加速度計和陀螺儀（磁力計關閉），**不做融合、沒有姿態**。
topic 名稱和格式照 [`imu_tools`](https://github.com/CCNYRoboticsLab/imu_tools) 的慣例，可以直接接 `imu_filter_madgwick`（見 [4.5](#45-建議的上位機架構)）。

#### `/imu/data_raw`（`sensor_msgs/Imu`，`frame_id` = `imu_link`）

| 欄位 | 內容 |
|------|------|
| `header.stamp` | **和同一週期 `/wheel_counts` 的 `data[0]` 完全相同**（sec = ms / 1000） |
| `orientation` | **沒有**：全 0，`orientation_covariance[0] = -1`（ROS 慣例，表示沒有姿態） |
| `angular_velocity` | 角速度（rad/s） |
| `linear_acceleration` | 加速度（m/s²，**含重力**，靜止平放時 z ≈ 9.8） |
| 共變異數 | 只填對角線，估計值（`robot_config.h` 的 `IMU_*_VAR`） |

感測器自己的更新率（BNO055 預設值）：**加速度 125 Hz、角速度 100 Hz**。
我們每 10 ms 讀一次，所以角速度、加速度偶爾會和上一筆一樣，是正常的。

### 時間戳要注意

- **stamp 是 STM32 開機時間，不是 ROS 時間。** 例如 `header.stamp.sec = 36` 代表開機後 36 秒。
  拿來互相配對、算 dt 沒問題；要交給 `robot_localization` 等套件前必須先換成 ROS 時間（見 [4.4](#44-換成-ros-時間)）。
- IMU 和 `/wheel_counts` 的 stamp 一樣，但 **IMU 的數值本身會比 encoder 舊一點**：
  感測器有自己的更新率，和我們讀取的時間不同步，角速度最多舊約 10 ms、加速度最多舊約 8 ms，下位機無法消除。
  靜止或直線影響很小；高速旋轉時（例如 3 rad/s）約等於 1° 以內的瞬間誤差。
- 某一筆的間隔偶爾會到約 20 ms（下位機每秒 ping 一次 agent），**dt 一律用 stamp 相減**，不要假設固定 10 ms。

---

## 3. 下位機的狀態與可能狀況

### LED（PC13，藍燈）

| LED | 意思 |
|-----|------|
| 熄滅 | 還沒連上 agent（每 0.5 s 找一次） |
| 亮 | 已連上，正在發布 |

### Odometry

| 狀況 | 下位機行為 | 上位機要做的事 |
|------|-----------|---------------|
| agent 沒開 / 斷線 | **照常讀 encoder**（不會漏算），只是不發布；每 1 s 確認 agent 一次，斷線後自動重連 | 重連後 count 接續原本的值，直接算差值即可（中間那段的 dt 會很大） |
| STM32 重開機（斷電、按 NRST、重燒） | count 和 stamp 都從 0 開始 | **偵測 `data[0]` 變小**，把上一筆清掉重新開始，不要拿重開機前後相減 |
| 某個 pod 往前推時 count 變負 | — | 改 `robot_config.h` 的 `ODOM_PODx_DIRECTION` 重燒，或在上位機乘 −1 |
| 按到板上 KEY 鍵 | PA0（pod1 的 A 相）被拉到地，pod1 會亂跳 | 接了 pod1 之後不要按 KEY |
| 16-bit 計數器繞回 | 下位機每 10 ms 處理一次，上位機看到的 int32 不會繞回（要 2^31 counts 才會） | 不用處理 |

### IMU

| 狀況 | 下位機行為 | 上位機看到的 |
|------|-----------|-------------|
| 正常 | 每週期讀一筆 | `/imu/data_raw` 100 Hz |
| 沒接 / 沒電 / SDA、SCL 不通 | 每 1 s 重試一次（重置 + 在 0x28、0x29 找晶片） | **只有 `/wheel_counts`，沒有 IMU topic**（odometry 不受影響） |
| 連續 10 次 I2C 讀取失敗，或 BNO055 自己重開機 | 重置 BNO055 並重新初始化（約 0.7 s） | IMU topic 中斷約 0.7 s 後恢復 |
| 角速度靜止時不是 0 | ACCGYRO 模式給的是原始值，下位機**不做陀螺儀零點校正** | 上位機要處理零點偏移（madgwick 有 `zeta` 參數可以慢慢修正，或開機時靜止幾秒取平均扣掉） |

> **上位機注意：** IMU topic 中斷再恢復時，下位機只是重新初始化 BNO055，原始值的意義不變；
> 但如果上位機的融合濾波器在中斷期間沒有資料，記得檢查它恢復後是否正常。

### 用 ST-LINK 查下位機狀態（除錯用）

接 ST-LINK，在 CubeIDE Debug 的 Live Expressions 加這些變數：

| 變數 | 意思 |
|------|------|
| `imu_status` | 0 = 啟動中，1 = 找不到 IMU，2 = 正常 |
| `imu_i2c_addr7` | 找到的 I2C 位址（0x28 / 0x29），0 = 沒找到 |
| `imu_sys_err` | BNO055 錯誤碼，0 = 沒有錯 |
| `imu_restart_count` | IMU 重新初始化過幾次 |

---

## 4. 上位機怎麼用這些資料

### 4.1 訂閱設定

```python
from rclpy.qos import qos_profile_sensor_data   # best effort
self.create_subscription(Int32MultiArray, '/wheel_counts', self.on_wheel, qos_profile_sensor_data)
self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
```
執行前要 `export ROS_DOMAIN_ID=4`。

### 4.2 從 count 算位姿

- 每筆算 4 個 pod 的 Δcount（這一筆 − 上一筆），乘上 mm/count 得到每個輪子的位移。
- 用 pod 的安裝位置和方向，以最小平方法解出 Δx、Δy、Δθ，再累加成場地座標的位姿。
- 數學已經寫在 [`tools/odom_host.py`](tools/odom_host.py) 的 `OdomEstimator`（`PODS` 填各 pod 的位置和方向、`MM_PER_COUNT` 填輪徑和解析度），
  可以直接搬進 ROS 2 node。檔案裡讀序列埠 CSV 的部分是舊版的，已經不能用。

### 4.3 配對 wheel 和 IMU

`/wheel_counts` 沒有 header，`message_filters` 不能用，請**自己用 stamp 配對**：

- 下位機每週期依序發 `/wheel_counts`、`/imu/data_raw`，所以通常 wheel 會先到。
- 在 `on_wheel` 裡把 `(stamp → counts)` 存進一個小字典（留最近幾十筆）；`on_imu` 收到時用 `header.stamp` 換回 ms 去找。
- IMU 沒接時不會有 IMU topic，odometry 計算不要等 IMU。

```python
def on_wheel(self, msg):
    stamp, counts = msg.data[0], list(msg.data[1:5])
    if self.prev is not None and stamp < self.prev[0]:    # STM32 重開機
        self.prev, self.offset_ns = None, None
        self.wheel_by_stamp.clear()
    if self.prev is not None:
        dt = (stamp - self.prev[0]) / 1000.0
        d = [c - p for c, p in zip(counts, self.prev[1])]
        # ... 用 OdomEstimator 的數學更新位姿
    self.prev = (stamp, counts)
    self.wheel_by_stamp[stamp] = counts
    if len(self.wheel_by_stamp) > 50:
        self.wheel_by_stamp.pop(next(iter(self.wheel_by_stamp)))

def on_imu(self, msg):
    stamp = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000
    counts = self.wheel_by_stamp.get(stamp)   # 同一週期的 wheel，找不到就是那筆 wheel 掉了
```

### 4.4 換成 ROS 時間

要把資料交給 `robot_localization`、TF、rosbag 對齊等，需要 ROS 時間。做法是估「ROS 時間 − STM32 時間」的 offset：

```python
def to_ros_ns(self, stamp_ms):
    off = self.get_clock().now().nanoseconds - stamp_ms * 1_000_000
    if self.offset_ns is None or off < self.offset_ns:
        self.offset_ns = off              # 取最小值 = 傳輸延遲最短的那筆
    self.offset_ns += 1000                # 每筆放寬 1 µs，追得上兩邊石英的頻率差
    return stamp_ms * 1_000_000 + self.offset_ns
```
- STM32 重開機時要把 `offset_ns` 清掉重估（上面 4.3 的範例已經有）。
- 只要求簡單的話，也可以直接用「收到訊息的時間」，代價是多了 USB/agent 的延遲和抖動（幾 ms）。

### 4.5 建議的上位機架構

寫一個 bridge node，把下位機的原始資料轉成標準訊息，其他節點只吃標準訊息：

| 輸入 | 輸出 | 內容 |
|------|------|------|
| `/wheel_counts` | `/odom/wheel`（`nav_msgs/Odometry`）+ TF `odom → base_link` | 4.2 算出的位姿與速度，stamp 換成 ROS 時間 |
| `/imu/data_raw` | 同名 topic（例如 `/stm/imu/data_raw`） | 同樣的內容，stamp 換成 ROS 時間 |
| — | static TF `base_link → imu_link` | IMU 的實際安裝位置與方向 |

姿態用 `imu_filter_madgwick` 算（`sudo apt install ros-humble-imu-filter-madgwick`）：
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  -r imu/data_raw:=/stm/imu/data_raw \
  -p use_mag:=false -p world_frame:=enu -p publish_tf:=false
# 輸出 /imu/data（有 orientation 的 sensor_msgs/Imu）
```
- 下位機不送磁場，一定要 `use_mag: false`：只用陀螺儀 + 加速度計，yaw 會慢慢漂，但不怕馬達干擾。

如果再用 `robot_localization` 融合 odometry 和 IMU：
- 用 madgwick 輸出的 `/imu/data`。不用磁力計時 yaw 會漂，建議**只融合 yaw rate（`angular_velocity` 的 z）**，或用 `imu0_relative: true`。
- `IMU_*_VAR` 是估計值，可以在 bridge 裡直接覆寫共變異數來調，不用重燒下位機。

### 4.6 IMU 安裝方向

BNO055 的軸以 Adafruit 板子上印的 X/Y 箭頭為準。IMU 沒有和車身對齊時，用 static TF 修正，不要改下位機：
```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.1 \
  --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id imu_link
```

---

## 5. 參數設定

### `Core/Inc/robot_config.h`（常調的都在這）

| 參數 | 目前值 | 說明 |
|------|--------|------|
| `ODOM_POD1~4_DIRECTION` | +1 | pod 往前推 count 變負就改 −1 |
| `ODOM_SAMPLE_PERIOD_MS` | 10 | 讀 encoder + IMU 並發布的週期。BNO055 角速度更新率 100 Hz，設比 10 小對 IMU 沒用 |
| `IMU_GYRO_VAR` | 0.0004 (rad/s)² | 角速度共變異數 |
| `IMU_ACCEL_VAR` | 0.04 (m/s²)² | 加速度共變異數 |

### 寫在程式裡的固定設定

| 位置 | 設定 | 值 |
|------|------|----|
| `microros_app.c` | Node / topic 名稱 | `odom_stm32`、`wheel_counts`、`imu/data_raw`，`frame_id` = `imu_link` |
| `microros_app.c` | `ROS_DOMAIN_ID` | 4 |
| `microros_app.c` | 等待時找 agent 的間隔 / 連線中確認的間隔 | 500 ms / 1000 ms（確認失敗 3 次 × 50 ms 才算斷線） |
| `imu.c` | BNO055 模式 | ACCGYRO（0x05，原始值不融合，磁力計關閉），角速度單位 rad/s，用外部 32.768 kHz 石英；感測器設定用 BNO055 預設值 |
| `imu.c` | 找不到 IMU 的重試間隔 / 失敗幾次重新初始化 / 檢查狀態間隔 | 1000 ms / 連續 10 次 / 每 100 筆（約 1 s） |
| `imu_hal.c` | I2C 位址 | 先試 0x28，再試 0x29 |
| `usb_cdc_transport.c` | USB 送出 | 非阻塞：先放進 2 KB 的 TX buffer 就返回；buffer 滿了最多等 10 ms，還是不夠就丟掉那一筆（電腦沒在讀時不會卡住） |

---

## 6. 硬體與接線

### 6.1 Odometry pod

| Pod  | Timer | 腳位 (A / B) | 計數器 |
|------|-------|--------------|--------|
| pod1 | TIM2  | PA0 / PA1    | 32-bit |
| pod2 | TIM3  | PA6 / PA7    | 16-bit |
| pod3 | TIM1  | PA8 / PA9    | 16-bit |
| pod4 | TIM4  | PB6 / PB7    | 16-bit |

PA0 也接著板上的 KEY 按鈕（按下 = 接地），接了 pod1 之後不要按 KEY。
腳位在 [`eurobot_odom_stm.ioc`](eurobot_odom_stm.ioc) 設定，接線表在 [`odometry.c`](Core/Src/odometry.c)，兩邊要一致。

### 6.2 IMU（Adafruit BNO055）

| BNO055 | Black Pill 排針 | STM32 | 用途 |
|--------|-----------------|-------|------|
| Vin    | 3V3             | —     | 電源 |
| GND    | G               | —     | |
| SCL    | B8              | PB8   | I2C1_SCL（400 kHz，上拉在 Adafruit 板上） |
| SDA    | B9              | PB9   | I2C1_SDA |
| RST    | B5              | PB5   | 拉低 = 重置 |
| INT    | B4（可不接）    | PB4   | BNO055 用不到 |

3vo（板子自己穩壓出來的 3.3V 輸出）、PS0 / PS1（不接 = I2C 模式）、ADR（不接 = 0x28，接 3.3V = 0x29，韌體兩個都會找）都不接。

### 6.3 上位機連線

- 板上的 Type-C 是 USB CDC 虛擬 COM port（PA11 / PA12，VID:PID `0483:5740`），同時供電給板子。
- Linux：`/dev/ttyACM0`。多個 USB 序列裝置時，用 `/dev/serial/by-id/` 底下的名字比較穩定。
- **Windows + Docker Desktop**（agent 跑在容器裡）：
  ```powershell
  # 第一次（系統管理員）：把板子分享給 WSL，BUSID 用 usbipd list 查
  usbipd bind --busid <BUSID>
  # 每次開機：Docker Desktop 要先開
  usbipd attach --wsl --busid <BUSID> --auto-attach        # 重燒、重插後會自動接回
  wsl -d docker-desktop -e /sbin/modprobe cdc_acm          # 沒有這步不會出現 /dev/ttyACM0
  docker run -d --name uros_agent --device=/dev/ttyACM0 microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
  # 看資料
  docker exec -it -e ROS_DOMAIN_ID=4 uros_agent bash
  source /opt/ros/humble/setup.bash
  ```

### 6.4 其他

- **時脈**：板上 25 MHz 石英（HSE）→ SYSCLK 96 MHz，USB 剛好 48 MHz。
- **燒錄線（ST-LINK V2）**：照外殼上印的字接，**不要接 SWIM、RST、5V**。

| ST-LINK V2 | Black Pill 尾端排針 |
|------------|---------------------|
| SWDIO      | DIO（PA13） |
| SWCLK      | SCK（PA14） |
| GND        | GND        |
| 3.3V       | 3V3（只在沒插 Type-C 時接，兩邊同時供電可能互相灌電） |

---

## 7. 專案結構

```
Core/Inc/robot_config.h    可調參數：pod 方向、週期、IMU 共變異數

Core/Src/app.c             進入點：app_init() / app_task()

Core/Src/odometry.c        init encoder、處理溢位、累計 count
Core/Src/imu.c             BNO055 task：初始化、收到觸發就讀一筆、存快照
Core/Src/imu_hal.c         BNO055 的硬體介面（I2C1 讀寫暫存器、RST）

Core/Src/microros_app.c    micro-ROS 連線與發布；每週期讀 encoder → 觸發 IMU → 發布
Core/Src/microros/         micro-ROS 官方的 allocator / 時間（不用改）和 USB CDC transport（改寫過：修 bug、非阻塞送出）

Core/Src/main.c 等         CubeMX 產生

USB_DEVICE/                USB CDC 設定（CubeMX 產生）
Middlewares/               FreeRTOS、ST USB Device 函式庫（CubeMX 產生）
micro_ros_stm32cubemx_utils/   micro-ROS 函式庫（已編好）

tests/host/                電腦上跑的單元測試
tools/odom_host.py         從 count 算 (x, y, θ) 的數學（參考用）
```

`main.c` 裡我們只加了三行（`#include "app.h"`、`app_init();`、`app_task();`），都在 `USER CODE` 區塊內。

每個週期（10 ms）在下位機的順序：

1. micro-ROS task 讀 4 個 encoder，記下 stamp。
2. 通知 IMU task（優先權較高，會馬上執行）讀 BNO055，用同一個 stamp 存起來（約 1 ms）。
3. micro-ROS task 依序發布 `/wheel_counts`、`/imu/data_raw`（放進 USB 的 TX buffer 就返回，不等電腦收完）。

---

## 8. 編譯與燒錄

1. **STM32CubeIDE**：`File → Import → Existing Projects into Workspace`，選這個資料夾。
2. `Project → Build Project`（micro-ROS 函式庫已經在 repo 裡，不用裝 Docker）。
3. 接 ST-LINK（見 [6.4](#64-其他)），用 `eurobot_odom_stm.launch` 燒錄。

命令列燒錄（CubeIDE 附的 CubeProgrammer）：
```bash
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w Debug/eurobot_odom_stm.elf -v -rst
```
杜邦線較長時用 `freq=100`（100 kHz），太快可能出現 `failed to erase memory`。

也可以不用 ST-LINK、直接用 Type-C 燒（USB DFU）：要先裝 STM32CubeProgrammer（附 DFU 驅動），
按住 BOOT0 → 按一下 NRST → 放開 BOOT0，再用 `-c port=usb1` 燒錄，燒完按 NRST。這個方法不能設中斷點除錯。

---

## 9. 改硬體設定（CubeMX）

要另外裝 **STM32CubeMX 6.x**（CubeIDE 2.x 不內建，CubeMX2 不支援 F4），打開 `.ioc` 改完按 Generate Code。

- 同一時間只讓一個人改 `.ioc`。
- 自己的程式要寫在 `USER CODE BEGIN / END` 之間，否則重新產生時會消失。
- 這些是 micro-ROS 需要的，不要改：FreeRTOS（CMSIS_V2）、defaultTask stack 3000 words、
  `TOTAL_HEAP_SIZE` 30000、`USE_NEWLIB_REENTRANT`、Timebase = TIM11、
  USB_OTG_FS Device_Only + USB_DEVICE CDC、HSE 25 MHz / PLLQ 讓 USB = 48 MHz。
- `TOTAL_HEAP_SIZE` 不要加大：micro-ROS 會再開一塊一樣大的 heap，RAM（128 KB）會不夠。

---

## 10. 重新編譯 micro-ROS 函式庫（通常不需要）

只有換 ROS 2 版本或加自訂訊息時才要。需要 Docker：

```bash
# 先在 CubeIDE Build 一次（腳本從 Debug/ 的 .mk 讀編譯旗標）
rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
docker run --rm -v "<專案完整路徑>:/project" \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide \
  microros/micro_ros_static_library_builder:humble
```

`micro_ros_stm32cubemx_utils/` 來自官方 [repo](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
的 `humble` 分支（commit `bd531b2`），只改了 `.gitignore`。

---

## 11. 單元測試

不需要板子，在專案根目錄執行：

```bash
gcc -std=gnu11 -Wall -Wextra -Wshadow -Werror \
    -Itests/host/fake_hal -ICore/Inc \
    tests/host/test_odometry.c Core/Src/odometry.c \
    -o test_odometry && ./test_odometry
```
