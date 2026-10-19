# 2026-10-19 北斗 UM980 Kinematic 分析器新增与字段审计

## 一、会话目标
1. 在 `beidou_data/` 中新增两份 UM980 实测数据，基于 `beidou_manual/` 中的和芯星通 N4 手册实现北斗/UM980 专用分析器。
2. 保持北云、华测原有分析器源码不修改，仅新增北斗分析器和 HMI 注册。
3. 对照北云/华测既有测试逻辑与字段口径，逐项核查北斗字段映射。
4. 生成 HTML/Markdown 报告、图表和 OBSVM 解码表。
5. 同步 GitHub。

## 二、手册依据
文件：
- `beidou_manual/Unicore Reference Commands Manual For N4 High Precision Products_V2_CH_R1.15.pdf`

关键表：
- `BESTNAV`：7.3.19 表 7-70
- `GGA`：7.1.1.3 表 7-4
- `GRS`：7.1.1.6 表 7-7
- `GSA`：7.1.1.7 表 7-8
- `GST`：7.1.1.8 表 7-9
- `GSV`：7.1.1.9
- `BESTSAT`：7.3.22 表 7-76~7-80
- `OBSVM`：7.3.65 表 7-137 / 7-138 / 7-140
- `RTKSTATUS`：7.3.85 表 7-169
- `SATSINFO`：7.3.89 表 7-174~7-177
- 二进制 Header：表 7-50；ASCII Header：表 7-51

## 三、新增文件
- `gps_kinematic_analyzer_beidou.py`

主要功能：
1. 混合流解析：
   - ASCII：`#BESTNAVA`、`$GNGGA`、`$GNGSA`、`$GNGST`、`$GNGRS`、`#SATSINFOA`、`#BESTSATA`、`#RTKSTATUSA`
   - 二进制：`OBSVMB`
2. 二进制恢复：
   - 同步头 `AA 44 B5`
   - 解析 Message ID / MessageLength
   - 使用 CRC-32 校验，只保留合法完整 OBSVMB 块
3. ASCII 恢复：
   - `#` 报文使用 Unicore CRC-32
   - `$` 报文使用 NMEA XOR
4. 完整性检查与周期核对
5. BESTNAVA 主定位、速度、σ、基站ID、差分龄期、跟踪/解算卫星数
6. Sol Status 与 Pos Type 分开统计
7. GSA 组合 DOP
8. GST 伪距残差 RMS
9. GRS 逐星残差
10. RTKSTATUS 状态
11. BESTSAT 参与解算卫星
12. OBSVM C/N0、locktime、失锁/重捕获事件
13. ENU 轨迹
14. HTML / Markdown 报告
15. `OBSVMB_decoded.csv`

## 四、三产品字段对应审计

### 主定位 / σ / RTK / 卫星数
| 项 | 北云 | 华测 | 北斗 UM980 |
|---|---|---|---|
| 主定位报文 | `BESTGNSSPOSA` | `<BESTP` / `#BESTPA` | `BESTNAVA` |
| lat/lon/hgt σ | 数据 `[7]/[8]/[9]` | 数据 `[8]/[9]/[10]` | 数据 `[7]/[8]/[9]` |
| 基站ID | `[10]` | 末字段基站ID | `[10]` |
| Diff age | `[11]` | `[11]` | `[11]` |
| Sol age | `[12]` | `[12]` | `[12]` |
| 跟踪卫星 | `[13] #SVs` | `[13] #SVs` | `[13] #SVs` |
| 参与解算卫星 | `[14] #solnSVs` | `[14] #solnSVs` | `[14] #solnSVs` |

### 速度
- 北云：`INSPVAXA [6]/[7]/[8]` 直接给 N/E/U。
- 华测：`BESTVA/RTKV` 使用水平速度 + 真北航迹角 + 垂直速度，再分解 E/N/U。
- 北斗：`BESTNAVA [21] Sol Status`、`[22] Vel Type`、`[23] 水平速度`、`[24] 真北航迹角`、`[25] 垂直速度`。
  - `velocity_east = hor_spd * sin(track)`
  - `velocity_north = hor_spd * cos(track)`
  - `velocity_up = vert_spd`

### 伪距残差
- 北云：`$GPGST` field 3。
- 华测：`$GNGST` field 3。
- 北斗：`$GNGST` field 3。
- `GNGRS` 单独作为逐星残差，不混同为 GST 整体 RMS。

### 原始观测
- 北云：`TRACKSTATA`
- 华测：`RANGEA`
- 北斗：`OBSVMB`
- OBSVM：
  - C/N0 原始值除以 100，单位 dB-Hz
  - locktime 原始值除以 32，单位 s
  - 通道状态解析系统、信号、伪距有效、载波有效

## 五、重要修正：GSA 组合 DOP 分组

初版只按“PRN数量回落”分组，但 UM980 实测 GSA 组内顺序并不总是简单递增再回落，导致组合条被高估：

- `1019beidou.log` 初版 463 条，正确应为 331 条
- `1019beidou2.log` 初版 999 条，正确应为 535 条

修正后：
- 使用 GSA 尾部 `SysID` 识别组内星座条
- 每组取 PRN 槽数最多的组合条
- 不平均星座 DOP
- 实测组合条通常为 SysID 4，即 BDS 组合条
- 修正后组合 DOP 条数与 GGA 唯一 UTC 历元数一致

## 六、SATSINFOA 处理决策
- 当前仅做存在性和周期检查。
- 原因：ASCII 变长频率段在手册表格中不足以无歧义切分。
- 不生成天空分布图。
- 不输出 SATSINFOA SNR 统计。
- 这是保守处理，避免字段猜测。

## 七、实测结果

### `1019beidou.log`
- BESTNAVA：34帧，实际 1 Hz
- OBSVMB：33个合法二进制历元，CRC失败 0
- 解类型：`NARROW_INT` 34/34
- 固定解率：100.0%
- 基站ID：1793
- Diff age 最大：1.000 s
- OBSVM：3293个频点观测
- 平均 C/N0：41.83 dB-Hz
- 低 C/N0 `<35` 占比：13.9%
- locktime归零/消失事件：12
- 重捕获事件：7

### `1019beidou2.log`
- BESTNAVA：54帧，实际 1 Hz
- OBSVMB：54个合法二进制历元，CRC失败 0
- 解类型：`SINGLE` 45、`NARROW_INT` 8、`WIDE_INT` 1
- 固定解率：16.7%
- 基站ID：1793
- Diff age 最大：1.000 s
- OBSVM：4759个频点观测
- 平均 C/N0：42.87 dB-Hz
- locktime归零/消失事件：10
- 重捕获事件：47

## 八、观测矛盾与证据边界
1. `BESTNAVA` 显示 RTK 解、基站ID 1793、Diff age 1 s；但 `RTKSTATUSA` 的 Calculate status 为 `0 = 无差分源数据输入`。两份文件均如此。
   - 当前报告保留两边原始状态，不强行解释。
   - 后续需供应商确认 RTKSTATUS 状态更新条件和差分源判断口径。
2. Kinematic 数据无外业真值：
   - 不计算绝对定位误差
   - σ / RMS / GRS 均为接收机自估质量指标，不是真值误差
3. UM980 无 IMU、单天线：
   - 不生成 INS 分析
   - 不生成双天线航向分析

## 九、配置与完整性结论
推荐配置基准来自 `beidou_manual/UM980_北云指令对照表.md`：
- BESTNAVA 10 Hz
- GGA 10 Hz
- GSA 10 Hz
- GRS 10 Hz
- GSV 1 Hz
- GST 1 Hz
- SATSINFOA 1 Hz
- BESTSATA 1 Hz
- RTKSTATUSA 1 Hz
- OBSVMB 1 Hz

实测：
- BESTNAVA 为 1 Hz，与推荐 10 Hz 不符，报告标黄
- GGA/GRS 为 10 Hz
- GST/SATSINFOA/BESTSATA/RTKSTATUSA/OBSVMB 为 1 Hz

## 十、HMI 与打包
- `gnss_kinematic_hmi.py` 新增产品：
  - key: `beidou`
  - display: `北斗星通 UM980（和芯星通 N4）`
  - script: `gps_kinematic_analyzer_beidou.py`
- `GNSS_Kinematic_HMI.spec` 的 datas 已加入北斗脚本
- README 已加入北斗说明、使用方法、目录和依据
- 未修改：
  - `gps_kinematic_analyzer_beiyun.py`
  - `gps_kinematic_analyzer_huace.py`

## 十一、验证
完整执行：
```powershell
python gps_kinematic_analyzer_beidou.py beidou_data\1019beidou.log
python gps_kinematic_analyzer_beidou.py beidou_data\1019beidou2.log
```

语法验证：
```powershell
python -m py_compile gnss_kinematic_hmi.py gps_kinematic_analyzer_beidou.py gps_kinematic_analyzer_beiyun.py gps_kinematic_analyzer_huace.py kinematic_core\criteria.py
```

## 十二、GitHub
- 仓库：`https://github.com/mingzhewei/GNSS_KINEMATIC_TEST.git`
- 分支：`main`
- 提交：`d770d60`
- 信息：`Add UM980 Beidou kinematic analyzer`
- 推送成功
- `HEAD` 与 `origin/main` 均为：
  `d770d609abb8002ecbf2a417dcc4bbcab488e437`

## 十三、后续待办
1. 向供应商确认 `RTKSTATUSA Calculate status=0` 与 `BESTNAVA` RTK解/基站ID/差分龄期同时存在的口径差异。
2. 确认 SATSINFOA ASCII 变长频率段的准确切分规则后再实现天空分布和 SNR 统计。
3. 如需10 Hz BESTNAVA，检查现场配置为什么实际为1 Hz。
4. Build/dist 目录尚未重新打包，后续如需发布新版 HMI，应重新执行 PyInstaller 并验证北斗 worker。
