/**
 * @file    telemetry.h
 * @brief   透過 USART2（Nucleo 的 USB 虛擬 COM port）把資料送給上位機。
 *
 * 輸出格式：每行一筆 CSV，以 "\r\n" 結尾，共 14 個欄位：
 *
 *   seq, ts_ms,
 *   total1, total2, total3, total4,     累計 count
 *   delta1, delta2, delta3, delta4,     本次變化量 count
 *   mm1, mm2, mm3, mm4                  累計距離 mm（小數 2 位）
 *
 *   seq   : 0~255 循環，上位機可用來偵測掉行
 *   ts_ms : 取樣時間（ms）。取樣週期不保證完全固定，算速度請用 ts_ms 的差值。
 *
 * 修改格式時，務必同步更新上位機的解析程式和這段說明。
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "odometry.h"

/** 將一筆 odometry 資料以 CSV 格式送出（阻塞式，約 6~7 ms）。 */
void telemetry_send_odom(const OdomSnapshot *snap);

#endif /* TELEMETRY_H */
