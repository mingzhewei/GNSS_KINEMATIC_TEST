# GPS/RTK Kinematic 数据分析工具（北云 + 华测）

## 项目简介

针对北云 M21 组合惯导模组 与 华测 M720 纯GNSS模组的 Kinematic（动态）定位数据分析工具。
分别解析两家专有报文，生成同口径的定位质量分析报告，用于两模组性能对比。

## 脚本

| 脚本 | 适用模组 | 数据源 | 特点 |
|------|---------|--------|------|
| `gps_kinematic_analyzer_beiyun.py` | 北云 M21（组合惯导） | `BESTGNSSPOSA`(GNSS) / `INSPVAXA`(惯导) | 含 GNSS+INS 联合分析、惯导轨迹 |
| `gps_kinematic_analyzer_huace.py` | 华测 M720（纯GNSS） | `#BESTPA`(10Hz主源) / `#BESTVA`(速度) / `<RTKV`(旧格式) / `#BESTDOPSA` | 纯GNSS/RTK分析，无惯导 |

> 华测 M720 惯导需外接 IMU 才启用，当前未外接，故华测版**已彻底移除全部惯导处理**。
> 速度分析：新采集用 `#BESTVA`，旧采集回退 `<RTKV`（RTK速度报文）。

## 使用方法

```
python gps_kinematic_analyzer_beiyun.py  <北云数据.dat>
python gps_kinematic_analyzer_huace.py   <华测数据.log>
```

不带参数运行会弹出 GUI 文件选择框。分析结果保存在与数据同名的子目录中。

## 核心功能：Kinematic 完整性检查与周期核对

两版均在报告开头插入「完整性检查与报文周期核对」区块，按以下策略执行（以实测数据为准）：

1. **完整性检查**：统计每条报文的条数 / 原始行数 / 乱码残片行数。
2. **周期核对**：提取各报文实际周期（带时间戳的以时间戳为准；无时间戳按报文插入规律推断），
   与 setup 图设定的命令周期比对。
3. **界面显示**：在报告表格中明确列出每条报文的实际周期(s)、实际频率(Hz)、丢帧率。
4. **标色告警**：周期偏差 >5% 或丢帧率 >1% → 🟠周期不符（橙）；报文缺失 → 红；
   正常 → 🟢绿；输出比设定更密等提示项 → 🟡提示。

### Setup 图配置来源（核对基准）

- **北云 ICOM3**（`by_manual\icom3 setup.jpg`）：BESTPOSA 1.0s、INSPVAXA 0.1s、GPGGA 1.0s、
  BESTGNSSPOSA 0.2s、HEADINGA 0.2s、GPIMU ONNEW 0.01s、GPGSV/GPGSA/GPGST 0.1s、TRACKSTATA 0.2s。
  （ICOM4：RANGECMPB ONTIME 0.2s 二进制，记录为关注项，未实现二进制解码。）
- **华测 COM1**（`huace_manual\setup.jpg`）：GGA/GSV/GST/GSA/BESTDOPSA/BESTPA/RTKPA/RTKVA/
  BESTVA/ENVSTATUSA 全部 0.1s。

## 分析内容（两版同口径）

- 完整性检查与报文周期核对（见上）
- 采样率 / 丢帧统计
- 解类型分布（GNSS 固定解率，北云额外含 INS 固定解率）
- 卫星可见性、DOP 值
- 速度时间序列（北云: INSPVAXA 三分量；华测: BESTVA/RTKV 水平速度+航迹角分解）
- RTK 链路核对（基站ID、差分龄期，均源自 GNSS 位置报文）
- ENU 轨迹图、解算状态时间序列
- HTML + Markdown 报告

## 关键处理准则

- **GPIMU 报文直接放弃**，不做任何处理（仅计数显示）。
- **北云 GNSS 只用 `BESTGNSSPOSA`，不用 `BESTPOSA`**；惯导只用 `INSPVAXA`。
  （`BESTPOSA` 仅作信息提示，不参与分析。）
- **所有报文以实测数据为准**：带时间戳的以时间戳为最准；无时间戳按报文插入规律推断周期。

## 目录

| 目录 | 内容 |
|------|------|
| `by_data\` | 北云各场景原始数据(.dat) + 分析报告 |
| `huace_data\` | 华测原始数据(.log) + 分析报告 |
| `by_manual\` | 北云 UG016 协议手册 + ICOM3/ICOM4 setup 图 |
| `huace_manual\` | 华测 M7 系列协议手册 V2.7 + COM1 setup 图 |
| `.codebuddy\memory\` | 分析要点与准则记录 |

## 依据规范

- 北云科技 UG016 数据通信接口协议
- 华测 M7 系列模组用户指令及协议手册 V2.7
- NMEA 0183 标准
