"""
上位機 odometry 參考實作
========================

⚠️ 只當參考：韌體已經改成用 micro-ROS 發布（/wheel_counts），不再送 CSV，
所以下面「開序列埠讀 CSV」的部分（parse_line、main）已經不能用。
OdomEstimator 裡從 4 個 count 算出 (x, y, θ) 的數學仍然適用，
之後寫上位機的 ROS 2 node 時可以直接搬過去。

原本的用途：讀取 STM32 透過 UART 送出的 CSV，計算機器人在場地上的位姿 (x, y, θ)。

用法：
    pip install pyserial
    python tools/odom_host.py COM5            # Windows（COM 編號看裝置管理員）
    python tools/odom_host.py /dev/ttyACM0    # Linux / 樹莓派

座標約定（與 ROS 相同）：
    機器人座標：x 朝前、y 朝左、θ 逆時針為正
    場地座標：開機時機器人所在位置為原點、車頭朝 +X
"""
import math
import sys
from dataclasses import dataclass

BAUD = 460800   # 要和 CubeMX 的 USART2 設定一致

# ============================================================================
# 1. 機構參數：依實際安裝修改，並實測校正
# ============================================================================

MM_PER_COUNT = math.pi * 32.0 / 2000.0   # 輪徑 32 mm、每圈 2000 counts

# 每個 pod 的安裝方式：(CSV 中的 pod 編號 0~3, x_mm, y_mm, 量測方向 φ 度數)
#   x, y：pod 相對機器人中心的位置（x 朝前、y 朝左）
#   φ   ：pod 輪子往「正」方向滾時，指向哪裡。0 = 朝前，90 = 朝左
# 至少要 3 個 pod（而且不能全部平行）才能同時算出 x、y、θ。
# 下面是「左右各一個量前後 + 後方一個量左右」的範例，請改成你們的實際數值。
PODS = [
    (0,    0.0,  125.0,  0.0),   # pod1：左側，量前後
    (1,    0.0, -125.0,  0.0),   # pod2：右側，量前後
    (2, -100.0,    0.0, 90.0),   # pod3：後方，量左右
    # (3, 100.0,   0.0, 90.0),   # pod4：前方，量左右（接上後取消註解）
]

# ============================================================================
# 2. 解析 CSV
# ============================================================================

FIELD_COUNT = 14   # seq, ts_ms, total×4, delta×4, mm×4


@dataclass
class Sample:
    seq: int          # 0~255 循環
    ts_ms: int        # STM32 的 HAL_GetTick()
    total: list       # 4 個 pod 的累計 count（int32）


def parse_line(line):
    """把一行 CSV 轉成 Sample。格式不對就回傳 None（直接丟掉這行）。"""
    parts = line.strip().split(",")
    if len(parts) != FIELD_COUNT:
        return None
    try:
        seq = int(parts[0])
        ts_ms = int(parts[1])
        total = [int(x) for x in parts[2:6]]
    except ValueError:
        return None
    return Sample(seq, ts_ms, total)


def wrap_diff(cur, prev, bits):
    """兩個 bits 位元計數值的差，就算中間繞回（溢位）也正確。
    例：bits=8，prev=250、cur=3 → 9（不是 -247）"""
    half = 1 << (bits - 1)
    return ((cur - prev + half) % (1 << bits)) - half


# ============================================================================
# 3. 位姿計算
# ============================================================================

def _solve_3x3(m):
    """3x3 矩陣反矩陣（餘因子法）。"""
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-9:
        raise ValueError("pod 配置無法同時算出 x、y、θ：至少需要 3 個 pod，且不能全部平行。"
                         "（只有 2 個 pod 時，θ 要改用 IMU 提供）")
    return [
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det],
    ]


class OdomEstimator:
    def __init__(self, pods=PODS, mm_per_count=MM_PER_COUNT):
        self.pods = pods
        self.mm_per_count = mm_per_count

        # 每個 pod 一列：d_i = Δx·cosφ + Δy·sinφ + Δθ·(x·sinφ − y·cosφ)
        self.A = []
        for _, x, y, phi_deg in pods:
            p = math.radians(phi_deg)
            self.A.append([math.cos(p), math.sin(p), x * math.sin(p) - y * math.cos(p)])

        # 最小平方法：[Δx, Δy, Δθ] = (AᵀA)⁻¹ Aᵀ d，(AᵀA)⁻¹Aᵀ 只需要算一次
        ata = [[sum(r[i] * r[j] for r in self.A) for j in range(3)] for i in range(3)]
        inv = _solve_3x3(ata)
        self.A_pinv = [[sum(inv[i][k] * r[k] for k in range(3)) for r in self.A] for i in range(3)]

        self.x = 0.0        # mm，場地座標
        self.y = 0.0        # mm
        self.theta = 0.0    # rad，連續累加（不會跳回 0）
        self.vx = 0.0       # mm/s，機器人座標
        self.vy = 0.0
        self.omega = 0.0    # rad/s
        self.prev = None
        self.dropped = 0    # 掉了幾行

    def update(self, s):
        """收到一筆新的 Sample 時呼叫。"""
        if self.prev is None:                  # 第一筆只記錄，當作起點
            self.prev = s
            return

        # (a) 偵測掉行：seq 應該每次 +1（255 之後回到 0）
        gap = wrap_diff(s.seq, self.prev.seq, 8)
        if gap <= 0:                           # 重複或亂序的行，丟掉
            return
        self.dropped += gap - 1

        # (b) 每個 pod 的位移（mm）：用 total 相減，掉行也不會漏算
        d = [wrap_diff(s.total[idx], self.prev.total[idx], 32) * self.mm_per_count
             for idx, _, _, _ in self.pods]

        # (c) 機器人座標下的移動
        dx, dy, dth = (sum(self.A_pinv[i][k] * d[k] for k in range(len(d))) for i in range(3))

        # (d) 轉到場地座標（用這段時間的平均車頭方向）
        th_mid = self.theta + dth / 2.0
        self.x += dx * math.cos(th_mid) - dy * math.sin(th_mid)
        self.y += dx * math.sin(th_mid) + dy * math.cos(th_mid)
        self.theta += dth

        # (e) 速度：用 STM32 的 ts_ms 算 dt，不要假設固定 10 ms
        dt = wrap_diff(s.ts_ms, self.prev.ts_ms, 32) / 1000.0
        if dt > 0:
            self.vx, self.vy, self.omega = dx / dt, dy / dt, dth / dt

        self.prev = s

    @property
    def theta_deg(self):
        """顯示用：-180° ~ 180°"""
        return math.degrees(math.atan2(math.sin(self.theta), math.cos(self.theta)))


# ============================================================================
# 4. 主程式：開 COM port，一直讀
# ============================================================================

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    import serial   # pip install pyserial

    odom = OdomEstimator()
    with serial.Serial(sys.argv[1], BAUD, timeout=1) as ser:
        ser.reset_input_buffer()               # 丟掉開 port 前累積的舊資料
        n = 0
        while True:
            raw = ser.readline()
            s = parse_line(raw.decode("ascii", errors="ignore"))
            if s is None:
                continue
            odom.update(s)
            n += 1
            if n % 10 == 0:                    # 每 10 筆（約 0.1 秒）印一次
                print(f"x={odom.x:8.1f} mm  y={odom.y:8.1f} mm  θ={odom.theta_deg:7.2f}°  "
                      f"v=({odom.vx:6.0f},{odom.vy:6.0f}) mm/s  dropped={odom.dropped}")


if __name__ == "__main__":
    main()
