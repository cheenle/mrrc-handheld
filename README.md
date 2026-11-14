# MRRC Handheld — ESP32-S3 firmware for the `coolkit-xz-02` box

用现成的小智整机（= `nologo/xingzhi-cube-1.54tft-wifi` 硬件）做 MRRC Modern 手持终端。
不引入 LVGL，不依赖小智协议栈；板级引脚全部来自原厂固件取证。

* 主控 ESP32-S3（8 MB Octal PSRAM / 16 MB Flash）
* 屏 ST7789 240×240（SPI，需 invert_color）
* 音频：I2S 数字麦（GPIO 4/5/6）+ I2S 功放（GPIO 15/16/7），无 codec 芯片
* 按键：BOOT(0) / VOL+(40) / VOL−(39)，电池 + 充电检测（GPIO 38 / ADC2_CH6）

---

## 1. 操作模型（本次需求实现）

| 操作 | FREQ 模式 | VOL 模式 |
|---|---|---|
| VOL+ / VOL− 单击 | 频率 **+1 kHz / −1 kHz** | **本机喇叭音量 ±5%**（存 NVS，不改电台 AF） |
| VOL+ / VOL− 长按 | 自动连发（450 ms 后每 120 ms 一步） | 同左 |
| **BOOT 长按**（≥600 ms） | **PTT 发射**（松手即停） | 同左 |
| **BOOT 连续短按两次** | 切换 **FREQ ⇄ VOL** | 同左 |
| BOOT 单击 | 无动作（避免误发射） | — |

状态栏中间的 `FREQ`（琥珀）/ `VOL`（青色）徽标就是当前档位，切换时会有 toast。

**PTT 安全链**（照 MRRC 设计稿 §5 实现）：

```
按下并越过 600 ms ──► ptt:true ──► txhb ──► 等 200 ms ──► 开始推 Opus 帧
松手 ─────────────► 停帧 ──► "s:" ──► ptt:false ──► 200 ms 后回 IDLE
按住 180 s ───────► 强制 ptt:false + 屏上 "TX TIMEOUT"
发射中每 1.5 s ────► txhb（服务端活性闸门）
```

短按不会发射：键控只在越过长按阈值之后才发生，所以"点两下切音量"绝不会漏出载波。

**心跳为什么必须独立成任务**（这是实测踩出来的）：服务器有 TX 活性闸门——键控会话
超过 `MRRC_REMOTE_SESSION_TX_HEARTBEAT_S`（云端 5 s）没有 `txhb` 就会强制收键并回
`Link lost while transmitting`。最初心跳写在应用主循环里，而 TX 时 Opus 编码任务
（prio 7, core 0）把 core 0 打满，主循环（prio 1）被饿死数秒 → 心跳断 → 发射 5.6 s
必被服务器收键。现在心跳是独立 prio 10 任务（`tx_hb_task`），并且服务端回
`PTT released` / `FT8 正在发射` / `CQ 进行中` 时会同步本地状态机收键。

---

## 2. 屏幕布局（240×240，手绘 framebuffer + 抗锯齿字模）

版式参照 ICOM 手持机。`docs/ui-preview-*.png` 是主机渲染（同一份 `ui.c`），
`docs/ui-device-capture.png` 是从设备 dump 下来的真实 framebuffer：

```
y   0.. 21  顶栏   : [LSB] 徽标            VFO A   87%  ▂▄▆█
y  40.. 80  频率   : "7.215.00"  30 px 抗锯齿数字，白字黑底（发射时转红）
y  80.. 92  信息行 : S9+10   FIL 2.4K   AF 128（VOL 档显示 SPK 70%）
y  95..107  频谱标 : SPECTRUM SCOPE                100K
y 108..164  频谱   : 底噪扣除后的填充 + 亮迹线 + 中心虚线标记
y 166..206  瀑布   : 40 行历史
y 208..219  刻度   : 中心频率 ±span/2 的三个标签
y 222..238  状态片 : [FREQ]  [RX]  [LINK]
```

要点：

* 字模由 `tools/gen_font.py` 从 DejaVu 烘焙成 8-bit alpha（`main/ui_fonts.c`），
  不是手点阵，所以数字平滑。
* 频谱必须**扣噪声基底**：FT-710 的 bins 底噪在 120-150/255，只按峰值归一化会让整块
  变成实心色块。现在做 `(v - floor) / (peak - floor)`，floor/peak 都是慢速跟踪。
* 服务端 `/WSspectrum` 的 850 bins 每帧按最大值降到 240 列；声明 `mid` 档后
  帧长 851 B、约 15 Hz。

**调试图**：开机 15 s 自动把 framebuffer 以 hex-PPM 打到串口一次；也可同时长按
VOL+ 与 VOL- 1.5 s 触发。抓下来即可还原屏幕：

```bash
cd ~/esp && ./console.py 30 > /tmp/dump.log
# 取 ===FBDUMP-P6-240-240-255=== … ===FBDUMP-END=== 之间的 hex，每行 1440 字符
```

主机端预览（不用烧板子）：

```bash
gcc -O1 -DCONFIG_MRRC_UI_FPS=12 -I tools/host -I main/include -I main \
    -o /tmp/render tools/host/render.c main/ui_font.c main/ui_fonts.c -lm
/tmp/render [tx] [vol]      # 输出 /tmp/ui.ppm
```

---

## 3. 协议接线（已对着本机 `server.py` 核过，并用 PC 探针实测）

| 通道 | 方向 | 格式 |
|---|---|---|
| `/WSradio` | 双向 | JSON：`fullState` / `stateUpdate` / `value` / `error`；发 `{"type":"set","field":"freq","value":14074000}`、`af_gain`、`ptt`、`txhb` |
| `/WSspectrum` | 收 | `0x01 + 850 bins`（先发 `{"type":"spectrumCaps","profile":"mid"}`） |
| `/WSaudioRX` | 收 | 1 字节 tag（`0x00` PCM / `0x01` Opus 48 kHz 单声道）+ 载荷 |
| `/WSaudioTX` | 发 | 同上；`"s:"` 文本 = 停止采集 |
| 认证 | — | `POST /api/auth/login {"password":...}` → `token`，之后每个 WS 握手带 `Authorization: Bearer` |

实现里的几个坑（都是实测撞出来的）：

* `/WSradio` 连接后**第一帧是 `onlineUsers`**，不是 `fullState`——必须忽略未知 `type`。
* `s_meter` 以 ~20 Hz 推送，整屏重绘会打爆 SPI：已按 M_MAIN / M_METER 分桶，表头单独 ≤16 Hz 刷新。
* 服务端 TX 上行**不接受 16 kHz PCM 直推**（不做 16k→48k 重采样），所以 Opus 是唯一正确路线；
  Opus 解码器固定 48 kHz 输出，16 kHz 编码过去也能正常播出。
* 不能在 WebSocket 事件回调里发消息（会和自己抢锁）：连接后的 `txhb` / `spectrumCaps`
  由监督任务补发。

---

## 4. 构建

```bash
cd ~/esp/mrrc-handheld
source ~/esp-idf/export.sh
idf.py build            # app 1.26 MB / 分区 6 MB（80% 余量）
```

依赖（`main/idf_component.yml`，构建时自动拉取）：

* `esphome/micro-opus` — 标准 `opus.h` API（编码 + 解码），带 Xtensa DSP 优化
* `espressif/esp_websocket_client` — IDF 5.4 已移出核心，需显式声明

## 5. 烧录 / 回滚

```bash
cd ~/esp/mrrc-handheld
./flash.sh              # 或 ./flash.sh monitor 看串口日志
```

回滚到原厂小智固件（16 MB 全片镜像，`ota_1` 是空片所以这是唯一退路）：

```bash
~/esp/bin/esptool.py -p /dev/ttyACM0 -b 921600 write_flash 0x0 \
  ~/esp-backup-20261114-0151/esp32s3_flash_16MB.bin
```

分区表刻意保持 `nvs@0x9000` / `phy_init@0xf000` 与原厂一致，因此全片恢复不会错位。
本固件只写 `factory@0x10000`，不碰 `model`(0x10000 之前的唤醒词分区之外) 区域。

## 5.5 两个致命的 Opus 坑（都是实测踩出来的）

1. **编解码必须串行在同一个任务里**。ESP32-S3 的 GCC 不支持 `_Thread_local`（不生成
   `.tbss`，`global_stack`/`scratch_ptr` 只是普通全局变量），micro-opus 号称的
   "per-thread pseudostack" 实际是全局的。两个任务同时进 libopus 会互相踩栈，
   症状是 `Cache disabled but cached memory region accessed`。
2. **必须用定点（fixed-point），并且 pseudostack 要 240 KB**。float 版 SILK
   （`silk_energy_FLP`）的 scratch 需求超过 micro-opus 申请的 pseudostack，
   第一次 `opus_encode` 就越界写坏堆，表现为按键瞬间
   `Guru Meditation (LoadProhibited)`。
   对应配置：`# CONFIG_OPUS_FLOATING_POINT is not set` + `CONFIG_OPUS_PSEUDOSTACK_SIZE=240000`。

## 6. 已经验证 / 还没验证

**已验证（PC 只读探针 `~/esp/probe_mrrc_ws.py`，打真服务端 `ba4eg.mrrc.vlsc.net:443`）**

```
login: 200 {"ok":true,"role":"admin"}
/WSradio 首帧 onlineUsers，随后 fullState；s_meter 约 20 Hz
/WSspectrum: 首帧 1701 B，声明 mid 后 851 B / 850 bins
/WSaudioRX: tag 0x01 (Opus 48k) x20
```

**编译已验证**：`idf.py build` 通过，1.26 MB，无警告即错误项。

**PTT 全链路实测（25 秒键控自检，`CONFIG_MRRC_DEV_SELFTEST`）**

```
I (45566) mrrc: PTT ON
I (50572) hb: ... s=0 ... audio(rx=1656 tx=236  txB=7016)  drops(wifi=0 ws=0)
I (60578) hb: ... s=0 ... audio(rx=1656 tx=725  txB=21952) drops(wifi=0 ws=0)
W (70547) SELFTEST: PTT release
I (80602) hb: ... s=73 ... audio(rx=2086 tx=1214 txB=36984) drops(wifi=0 ws=0)
```

≈48 帧/秒稳定上行、心跳不断、服务器不回 `Link lost`、松手后 RX 立即恢复。

**启动时会打印上一次的复位原因**（`reset reason: PANIC (4)` 之类），这是排查
"发射掉线"最有效的一行——四条 socket 同时断开 + 6 秒后重新登录 = 设备重启，不是网络问题。

**待上机验证（按风险排序）**

1. **麦克风采样率**：默认 `CONFIG_MRRC_MIC_RATE=16000`。若想走 48 kHz 原始 PCM
   （`CONFIG_MRRC_AUDIO_PCM`）必须先确认这颗 I2S MEMS 麦支持 48 k。
   16 k + Opus 是安全默认值，且 SSB 音频带宽本来只有 3 kHz。
2. **TLS 内存**：4 条 wss + mbedtls。已把 `SSL_IN_CONTENT_LEN` 压到 4096、
   `SSL_OUT_CONTENT_LEN` 压到 2048 并开动态缓冲，仍建议开机看 `free heap`。
   若吃紧：`CONFIG_MRRC_SPECTRUM_PROFILE="low"` 或先不连频谱。
3. **功放增益/音量**：软件音量 `CONFIG_MRRC_SPK_VOLUME`、麦增益 `CONFIG_MRRC_MIC_GAIN`
   需要按实测调；AF 增益是服务端侧（`af_gain`），两者串联。
4. **电量 ADC**：ESP32-S3 的 ADC2 与 Wi-Fi 共用硬件，读数可能失败——实现里已做 3 次重试，
   失败则显示 `--`（不影响其它功能）。
5. **天线**：板载 PCB 天线，手持遮挡会掉包，瀑布用 `low` 档最稳。

## 7. 目录

```
mrrc-handheld/
├── main/
│   ├── app_main.c       应用壳：按键分发、PTT 状态机、180 s 兜底
│   ├── board_display.c  ST7789 + SPI + LEDC 背光 + PSRAM framebuffer
│   ├── board_audio.c    双 I2S（32-bit mono left，与出厂固件一致）
│   ├── board_input.c    按键手势（长按/双击/连发）+ 电池/充电
│   ├── mrrc_net.c       Wi-Fi、登录、4 路 WS、RX 环形缓冲、Opus 编解码
│   ├── ui.c / ui_font.c 7 段频率 + S 表 + 频谱/瀑布 + toast
│   └── Kconfig.projbuild
├── partitions.csv       nvs / phy_init / factory(6 MB)
├── sdkconfig.defaults   16MB flash、Octal PSRAM 80M、TLS 内存压缩
└── flash.sh
```

## 8. 已写死的参数（本次要求）

| 项 | 值 |
|---|---|
| Wi-Fi SSID / 密码 | 本地配置（不提交） |
| MRRC 服务器 | `ba4eg.mrrc.vlsc.net:443`（wss，Let's Encrypt 证书走内置 bundle） |
| Web 密码 | 本地配置（不提交） |

改法：`idf.py menuconfig` → `MRRC Handheld`（值只写进本地 `sdkconfig`，不入库）；`main/Kconfig.projbuild` 的 default 已清空。
