# STM32-F767ZI × PiR × PiT 整合系統運作流程（最終確認版）

> 版本：v1.0｜日期：2026-10-19  
> 本文件彙整已確認之系統需求與建議實作方案。尚未確定的硬體腳位、訊號長度及逾時值以「待確認」標示。

## 1. 系統組成與分工

| 裝置 | 功能 |
|---|---|
| STM32-F767ZI（NUCLEO-F767ZI） | DS3231 定時、ADC/DMA/FFT 偵測 30/34 kHz、模式狀態機、2-CH 繼電器獨立控制、安全關機協調 |
| PiR（Receiver） | 透過 Ethernet 與 AD7606B 通訊、錄製 30 秒、輸出 WAV、執行辨識、透過 TCP 傳送結果、協調每次任務 |
| PiT（Transmitter） | TCP 接收辨識結果、選擇輸出設定、透過 AudioInjector Octo 輸出聲學訊號、回報輸出完成 |
| DS3231 | 提供 RTC 時間及五分鐘排程基準 |
| 2-CH Relay | CH1 獨立控制 PiR 電源；CH2 獨立控制 PiT 電源（不是共用單一繼電器） |

### 1.1 已確認設計決策

1. 正常模式每五分鐘啟動一輪任務；每輪 PiR 錄製 30 秒訊號。
2. PiR **每次完成辨識均傳送結果**，即使與上次辨識結果相同。
3. PiR 與 PiT 透過 TCP 長連線交換辨識結果與完成訊息。
4. 警戒模式中採**序列化循環**：PiT 完成輸出後，PiR 才開始下一次錄製。
5. STM32 偵測有效 30 kHz 進入警戒模式；偵測有效 34 kHz 進入退出程序，**完成目前已接受的任務後才關機**。
6. PiR 與 PiT 使用同一塊 2-CH Relay 模組，但各自由不同通道供電，可獨立安全關機與斷電。
7. PiT 聲學輸出時間**尚未固定**，必須以 `TX_DONE` 事件判斷完成，不得用寫死的延遲取代。

## 2. 硬體與通訊拓樸

```text
                    DS3231 (I2C)
                         │
水下麥克風 → 類比前端 → STM32-F767ZI ← ADC/DMA/FFT
                         │
                  2-CH Relay Module
                   │             │
                 CH1             CH2
                   │             │
                  PiR ←── TCP ──→ PiT
                   │               │
           Ethernet / AD7606B   AudioInjector Octo
                   │               │
              30 s WAV        聲學訊號輸出
                   │
                 Decoder
```

- PiR 的原生 Ethernet 若已連 AD7606B，可用第二個 USB-Ethernet 介面連 PiT，或使用 Switch；建議分離網段以避免路由衝突。
- STM32 與兩臺 Pi 的 GPIO 需確認 3.3 V 相容、共地或隔離、上/下拉及避免關機反向供電。
- 繼電器須由合適驅動電路控制，並驗證觸點容量、浪湧電流及電源穩定性。

## 3. 正常模式 NORMAL（每五分鐘一次）

1. STM32 由 DS3231 取得時間，於固定五分鐘時間槽（例如 00、05、10 分）觸發。
2. STM32 開啟 Relay CH1、CH2，分別供電給 PiR 與 PiT。
3. 等待兩臺 Pi 開機及服務初始化完成（`PIR_READY`、`PIT_READY`）。
4. PiR 透過 AD7606B 擷取**30 秒**資料，檢查取樣完整性、遺失封包及取樣率。
5. PiR 轉換／儲存 WAV，並執行目標辨識；辨識失敗應回報錯誤，不能當作 `NONE`。
6. PiR 確認 TCP 連線與 PiT `READY`，傳送帶有唯一 Task ID 的 `RESULT`。
7. PiT 保存任務並回傳 `RESULT_ACK`；若未收到 ACK，PiR 依相同 Task ID 查詢／重試。
8. PiT 依辨識結果選擇聲學輸出 Profile，透過 AudioInjector Octo 輸出；可變長度。
9. PiT 回傳 `TX_STARTED`（選用）與 `TX_DONE`；PiR 回傳 `TX_DONE_ACK`。**RESULT_ACK 不等於 TX_DONE**。
10. PiR 確認本輪完成，並檢查 STM32 最新模式。若仍為 NORMAL 且無待處理任務，進入關機協調。
11. PiR、PiT 分別安全關機；STM32 各自確認實際可斷電條件，依序或獨立關閉 CH1、CH2。
12. STM32 繼續監測 30/34 kHz 並等待下一個固定時間槽。

> 若上一輪尚未結束而到達下一個五分鐘時間槽，建議跳過該時間槽，不重複啟動任務。若正常模式運行期間收到有效 30 kHz，切換 ALERT 並保持供電。

## 4. 警戒模式 ALERT

### 4.1 進入條件

- STM32 連續監測水下麥克風訊號；確認有效 **30 kHz** 單頻波後進入 `ALERT`。
- 若兩臺 Pi 尚未上電，開啟 CH1/CH2 並等待 Ready；若已運行則不中斷當前任務。

### 4.2 連續序列化任務

```text
ALERT → PiR 錄製 30 秒 → WAV → Decoder → RESULT
                                             ↓
                                      PiT RESULT_ACK
                                             ↓
                                      PiT 聲學輸出
                                             ↓
                                           TX_DONE
                                             ↓
                               查詢 STM32 模式 / STOP_REQUEST
                                      │              │
                                   ALERT         EXIT_PENDING
                                      │              │
                               下一輪錄製       安全關機
```

- PiT **完成本次聲學輸出後**，PiR 才可開始下一輪錄製；不得在 PiT 輸出期間錄製下一輪。
- 循環時間：`30 s + 解碼時間 + TCP/協調時間 + PiT實際輸出時間 + 其他開銷`。
- PiT 輸出長度可依 Profile 或參數調整，應設定合理、可配置的最大執行逾時。

### 4.3 退出條件

- STM32 確認有效 **34 kHz** 單頻波後，切換 `EXIT_PENDING`，送出 `STOP_REQUEST`。
- PiR 不再接受下一輪錄製；已接受／執行中的任務必須完成（含 PiT 的 TX_DONE）。
- 任務完成後兩臺 Pi 分別安全關機；STM32 確認後關閉對應 Relay 通道。
- 回到 NORMAL，沿用 DS3231 固定五分鐘時間槽。
- **建議優先權**：若 EXIT_PENDING 期間再次確認有效 30 kHz，取消退出並恢復 ALERT；需於韌體中一致實作。

## 5. 系統狀態機

### 5.1 STM32 模式狀態

| 狀態 | 說明 | 主要轉換 |
|---|---|---|
| `NORMAL` | 定時啟動單次任務 | 30 kHz → ALERT；五分鐘時間槽 → 啟動 |
| `ALERT` | 持續供電、序列化任務循環 | 34 kHz → EXIT_PENDING |
| `EXIT_PENDING` | 禁止新任務，等待當前任務完成 | 完成並關機 → NORMAL；30 kHz → ALERT（建議） |

### 5.2 STM32 電源狀態（PiR/PiT 各自追蹤）

`OFF → POWERING_ON → BOOTING → RUNNING → SHUTDOWN_REQUESTED → WAIT_POWER_SAFE → OFF`

任何階段若逾時或發生異常，進入 `FAULT`，由 STM32 依預先定義的復原策略處理。因為是兩個 Relay 通道，PiR/PiT 的電源狀態必須分別保存，不能共用一個 `is_power_on` 旗標。

### 5.3 PiR 任務狀態

`IDLE → RECORDING → WAV_READY → DECODING → RESULT_PENDING → WAIT_RESULT_ACK → WAIT_TX_DONE → TASK_COMPLETE`

失敗時進入 `TASK_ERROR`，保留錯誤碼、Task ID 與必要紀錄；在 ALERT 中，只有 `TASK_COMPLETE` 且未收到停止要求才開始下一輪。

### 5.4 PiT 任務狀態

`IDLE → RESULT_RECEIVED → READY_TO_TX → TRANSMITTING → COMPLETED`

異常狀態包括 `TX_FAILED`、`TX_UNCERTAIN`。PiT 必須保存 Task ID 與狀態，避免重傳 RESULT 導致重複聲學輸出。

## 6. STM32 ↔ PiR/PiT GPIO 訊號（邏輯定義）

| 訊號 | 方向 | 功能 |
|---|---|---|
| `RELAY_PIR_EN` | STM → Relay CH1 | PiR 獨立電源控制 |
| `RELAY_PIT_EN` | STM → Relay CH2 | PiT 獨立電源控制 |
| `ALERT_ACTIVE` | STM → PiR | 警戒模式狀態 |
| `STOP_REQUEST` | STM → PiR | 停止啟動新任務；完成當前任務後關機 |
| `PIR_READY` | PiR → STM | PiR 應用程式就緒 |
| `PIT_READY` | PiT → STM | PiT 應用程式就緒 |
| `TASK_BUSY` | PiR → STM | 當前任務尚未完成 |
| `PIR_POWER_SAFE` | PiR/電源管理電路 → STM | PiR 真正可安全斷電 |
| `PIT_POWER_SAFE` | PiT/電源管理電路 → STM | PiT 真正可安全斷電 |

> `POWER_SAFE` **不可僅由 Python 在呼叫 shutdown 前拉高**；應透過可驗證的關機完成硬體機制，或經測試的專用電源管理方案。STM32 不得把 `TASK_DONE` 直接視為安全斷電許可。

## 7. PiR ↔ PiT TCP 協定

### 7.1 角色與連線

- PiT：TCP Server；PiR：TCP Client。
- 兩臺 Pi 啟動後建立長連線，交換 `HELLO` / `READY`；連線中斷自動重連。
- TCP 是 byte stream，接收端需依封包長度完整讀取，處理黏包／拆包。

### 7.2 訊息類型

| 訊息 | 方向 | 意義 |
|---|---|---|
| `HELLO` | PiR → PiT | 應用層握手 |
| `READY` | PiT → PiR | 接收端就緒 |
| `RESULT` | PiR → PiT | 每次辨識完成均傳送一筆結果 |
| `RESULT_ACK` | PiT → PiR | 結果已接收並保存 |
| `TX_STARTED` | PiT → PiR | 已開始聲學輸出（建議） |
| `TX_DONE` | PiT → PiR | 實際聲學輸出完成 |
| `TX_DONE_ACK` | PiR → PiT | 確認收到輸出完成事件 |
| `PING` / `PONG` | 雙向 | 閒置連線健康檢查 |
| `ERROR` / `STATUS` | 雙向 | 錯誤及任務狀態查詢 |

### 7.3 建議封包欄位

`Magic(2B) | Version(1B) | Type(1B) | SessionID(4B) | TaskID(4B) | PayloadLength(2B) | Payload(NB) | CRC16(2B)`

- `RESULT` Payload 可包含 `TargetID`、`Confidence`、`DetectionTimestamp`、`TXProfileID`。
- 初始 Target ID：`0=NONE`、`1=AUV`、`2=DIVER`；其餘保留擴充。
- 使用固定的位元組序與 CRC 規格；Task ID 需跨重試保持不變。
- PiT 應先持久化任務再回傳 `RESULT_ACK`。
- `TX_DONE` 僅在輸出裝置實際完成輸出且狀態可確認後送出；不應以固定 sleep 時間代替。

### 7.4 ACK、重傳及去重

1. PiR 傳送 RESULT 並等待對應 Task ID 的 RESULT_ACK。
2. 逾時時先查詢狀態，或以**同一 Task ID** 重傳；不可生成新的任務編號。
3. PiT 收到重複 Task ID 時，回覆已保存狀態，不重複啟動聲學輸出。
4. PiT 已開始或完成輸出但回覆遺失時，PiR 查詢 PiT 狀態；不得盲目重新觸發輸出。
5. 若 PiT 當機時輸出狀態不明，標記 `TX_UNCERTAIN`，按安全策略處置。
6. 建議 SQLite 保存 PiR 待確認佇列及 PiT 任務紀錄，以支援重啟復原。

## 8. 異常復原與保護

| 異常 | 建議處置 |
|---|---|
| PiT 未 Ready / TCP 無法連接 | PiR 週期性重連；超過啟動逾時則回報 STM32 |
| RESULT_ACK 遺失 | 同 Task ID 查詢或重傳，PiT 去重 |
| TX_DONE 逾時 | 查詢 PiT 任務狀態，不直接重播 |
| PiT 音訊輸出失敗 | 記錄 `TX_FAILED`，依政策決定是否重試 |
| PiR AD7606B 取樣失敗 | 重試取樣介面；保留失敗資訊，不能當作 NONE |
| WAV 或 Decoder 失敗 | 記錄 `RECORD_ERROR` / `DECODE_ERROR` |
| PiR/PiT 應用程式無回應 | STM32 進入 FAULT；避免無條件立即斷電 |
| 任一 Pi 關機逾時 | 該通道保持供電或依經驗證的故障斷電策略處理 |
| ALERT 中收到 34 kHz | STOP_REQUEST；完成當前已接受任務再關機 |
| NORMAL 中收到 30 kHz | 切換 ALERT，不中斷當前任務 |
| 到達五分鐘時間槽但任務尚未完成 | 跳過本次排程，不重複啟動 |

建議初始參數（須實測調整）：TCP 重連間隔 2 秒、閒置心跳 2 秒、RESULT_ACK 等待 1 秒；PiT 的 TX_DONE 逾時依實際 Profile 時長與設備緩衝設定，不可固定為短時間。

## 9. STM32 FFT 偵測建議

- STM32 透過 ADC + DMA 連續擷取水下麥克風訊號，使用 Hann Window + FFT。
- 初始評估參數：96 kHz 取樣率、4096 點 FFT、50% overlap；頻率解析度約 23.44 Hz。
- 30 kHz 與 34 kHz 應使用頻率容差、頻帶能量／雜訊比、幅度門檻及多個連續視窗確認。
- ADC 前端需具備適當偏壓、放大、保護與抗混疊濾波；驗證 STM32F767 實際 ADC 取樣能力。
- STM32 的 FFT 偵測與 RTC/繼電器控制應採非阻塞設計，Pi 關機期間仍持續偵測。
- 需評估 PiT 自身發送訊號是否會被 STM32 偵測為 30/34 kHz 指令，必要時採頻率保護、時序避讓或更明確的控制訊號編碼。

## 10. 軟體模組建議

```text
STM32/
  rtc_scheduler.c
  acoustic_detector.c
  system_fsm.c
  power_manager.c
  gpio_protocol.c

PiR/
  main.py
  recorder.py
  wav_writer.py
  decoder.py
  task_controller.py
  tcp_client.py
  stm_interface.py
  pending_results.db

PiT/
  main.py
  tcp_server.py
  audio_transmitter.py
  task_manager.py
  stm_interface.py
  received_results.db
```

PiR/PiT 主服務建議以 systemd 開機自動啟動；網路服務、錄音、辨識與控制狀態需記錄可追溯的日誌。

## 11. 建議驗證順序

1. **TCP 通訊**：HELLO、READY、RESULT、ACK、TX_DONE、重傳與去重。
2. **電源管理**：2-CH Relay 獨立上電、兩臺 Pi 安全關機、斷電確認。
3. **正常排程**：DS3231 每五分鐘喚醒、30 秒錄製、辨識與 PiT 發送。
4. **FFT 警戒**：使用已知 30/34 kHz 測試訊號驗證模式切換。
5. **整合壓力測試**：警戒循環、網路斷線、任務逾時、電源故障、34 kHz 退出期間再觸發 30 kHz。

## 12. 尚待確認事項

- STM32 各 GPIO 實際腳位、Relay 高／低有效邏輯、電源額定與安全關機硬體實現。
- PiR 第二網路介面或 Switch 配置、IP 網段與 TCP Port。
- PiT 聲學 Profile、輸出波形、實際輸出時長與最大逾時。
- `NONE` 對應 PiT 應發送何種訊號（或是否不發送）；即使不發聲也須回報任務完成。
- PiT 播放中當機後，`TX_UNCERTAIN` 任務應人工確認、跳過或有條件重試。
- STM32 30/34 kHz 的幅度門檻、持續時間、誤觸發保護及優先權。

---

**總體控制原則：** STM32 決定系統模式及 PiR/PiT 各自供電；PiR 控制「錄製 → 辨識 → 傳送」任務；PiT 控制可變長度聲學輸出並以 `TX_DONE` 宣告完成。警戒模式只有在當前任務完成且 STM32 仍允許繼續時，才進入下一輪錄製。
