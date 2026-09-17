# 2026-09-17 嵌入式报文提取 + DOP三件套（北云/华测）

## 会话目标
1. 华测 COM11 混合流里被二进制块切断的 ASCII 报文（如 #ENVSTATUSA/#BESTDOPSA）恢复提取。
2. DOP 由"仅 HDOP"升级为 PDOP/HDOP/VDOP 三件套，回答"DOP是卫星下发还是本地解算""多星座GSA如何整合"。

## 关键实测事实（非猜测，来自 20260916164216_M720_COM11.log 实测）
- 华测 COM11 是"不定长二进制块 + ASCII 报文"交错流。二进制块把一条 ASCII 报文拦腰切断，
  按行读取时该行含 \x00 被当"二进制行"整体跳过 → #ENVSTATUSA/#BESTDOPSA 漏解析。
- 实测：#ENVSTATUSA / #BESTDOPSA 字节流各 14448 次，但按行解析仅 ~13000 条。
- #BESTDOPSA 字段(表3-36): [0]PDOP [1]GDOP [2]HDOP [3]VDOP [4]TDOP [5]截止角 [6]卫星数。
  样本: 0.7621,0.8756,0.4163,0.4312,0.8475,0.0,44。
- 华测 $GNGSA 每周期多条(每星座1条)，尾部 PDOP,HDOP,VDOP；北云 $GPGSA 每周期仅1条(系统组合)，
  尾部 PDOP,HDOP,VDOP,systemId。GSA 字段: f[15]=PDOP f[16]=HDOP f[17]=VDOP。

## 嵌入式报文提取方案（已落地 gps_kinematic_analyzer_huace.py）
- 方法: 消息头锚点(_HASH_HEADS/_NMEA_HEADS) + 报文自带校验位验证。
  定位 '*' 校验结尾，#类用 CRC-32 验证、$类用 NMEA XOR 验证，只有校验通过才补入 parsed_data，
  天然剔除被二进制污染的残缺段，全程不猜测。
- **NovAtel CRC-32 是【反射】多项式 0xEDB88320、初值0**；`zlib.crc32` 为【非反射】不可用，
  必须用查表法 `_novatel_crc32`（_CRC_TAB/_crc_table）。实测与报文 CRC 完全吻合，14448/14448 通过。
- 合并策略: 对锚点清单内的头，若校验通过集合条数 > 逐行解析条数，则用前者替换。
- 结果: #ENVSTATUSA/#BESTDOPSA 均恢复到 14448 条(与字节流计数一致)，#BESTPOSA/#RANGEA=1445、
  #SATVIS2A=11560、$GNGSA=86598。

## DOP 三件套方案（两程序 analyze_dop 均重写）
- **DOP 是接收机本地解算的几何精度因子，非卫星下发**；取决于卫星几何分布，星座越多 DOP 越小。
- 多星座整合: 按 NMEA 0183 取 PRN 槽数最多/代表所有参与解算卫星的"组合条"，**不能对各星座 DOP 取平均**。
- 华测数据源三级回退: #BESTDOPSA(权威) → $GNGSA组合(parse_gngsa_combined, 按PRN回落切周期取PRN最多者) → $GNGGA(仅HDOP)。
- 北云数据源: $GPGSA(系统组合, parse_gpgsa) → $GPGGA(仅HDOP)。
- analyze_dop 结果含 dop_source + average/min/max_{pdop,hdop,vdop} + xxx_values；
  _plot_dop_time_series 画三条线；HTML/MD DOP 章节显示数据源+三件套表格(_row/_mrow 局部函数)。
- 阈值: HDOP <1优秀 /<2良好 /else一般(沿用)。

## 附带修复的既有 bug（与本次需求无关但阻塞报告生成）
北云 gps_kinematic_analyzer_beiyun.py 之前缺失 4 个绘图方法，调用即 AttributeError：
- _plot_velocity_time_series（新增，数据源 INSPVAXA 的 Vn/Ve/Vu，speed=sqrt(Vn^2+Ve^2+Vu^2)）
- _plot_gnss_enu_trajectory / _plot_ins_enu_trajectory / _plot_gnss_ins_combined_trajectory
- 配套新增: _detect_anomalies(相邻点速度超阈值标红X) + _enu_coords(经纬度→ENU,equirectangular近似,R=6371000)
- 着色统一用 kinematic_core.criteria.type_color / type_cn。

## 端到端验证（均 exit 0）
- 北云 by_data\20260916-164201\_com3.dat: DOP $GPGSA PDOP1.40/HDOP0.73(优秀)/VDOP1.19，章节1~10连续。
- 华测 huace_data\20260916164216_M720_COM11.log: DOP #BESTDOPSA PDOP0.73/HDOP0.41(优秀)/VDOP0.40，
  章节1~9连续，ENVSTATUSA/BESTDOPSA 恢复14448。

## 沿用约束（务必保持）
- 北云 C/N0 只用 COM3 的 TRACKSTATA，**不引入/不探测/不解码 COM4**（见 2026-09-17-com4-trackstata-decision.md）。
- 北云 GNSS 只用 BESTGNSSPOSA(不用BESTPOSA)，惯导只用 INSPVAXA，$GPIMU 放弃处理。
- 只做移动站模式；绘图标题/轴用英文避免乱码，中文仅在 HTML/MD 正文。
- GUI 横幅: 北云蓝(#1a5276)"北云 BEIYUN M21..."，华测红(#b03a2e)"华测 HUACE 纯GNSS..."。
- 改源码用临时 .py 脚本(@'..'@|Set-Content + python -X utf8)，改前 assert old in s，改后 ast.parse 验证。

---

## 追加：_plot_solution_status 性能优化（2026-09-17 下午）

- 问题: 华测程序跑到"绘制解算状态时间序列图..."卡很久。
- 根因: 该函数对【每个数据点】单独 plt.scatter()、对【每段相邻同状态线】单独 plt.plot(),
  14448~17830 条数据产生上万次 matplotlib Artist 创建, 开销极大(性能反模式)。
- 修复(北云/华测两文件同步): 改为向量化——
  (1) 按状态分组后每状态一次批量 scatter(组内颜色一致, 与原单点同色);
  (2) 折线按状态分组, 用 NaN 在状态切换处断开, 单次 plot 画出所有同色线段(仅相邻同状态相连, 与原逻辑相同)。
  视觉输出逐点逐色完全一致, 仅不再重复创建 Artist。
- 验证: 华测 78MB/17830历元 全程 18.9s(原需数分钟), solution_status.png 正常;
  图像确认: SOL_COMPUTED 绿色密带在底, NO_CONVERGENCE 灰点在顶, 中文Y轴/图例正常, 无错误。
  北云 6.5s 同步通过。两文件 ast.parse 通过。
- 注意: 华测数据文件已被用户重命名为 0916星扬上午.log / 0916星扬下午.log。

---

## GitHub 仓库（2026-09-17）

- 远程: https://github.com/mingzhewei/GNSS_KINEMATIC_TEST (branch: main)
- 本地: D:\software\gps分析程序\gps_test_kinematic (git init, user mingzhewei)
- 首次提交 f65d4f2, 仅含源码 5 文件: gps_kinematic_analyzer_beiyun.py /
  gps_kinematic_analyzer_huace.py / kinematic_core/criteria.py / README.md / .gitignore
- .gitignore 排除(体积极大不上传): by_data/ huace_data/ by_manual/ huace_manual/
  .codebuddy/ __pycache__/ .mypy_cache/ _fonttest.png _*.py
- 推送方式: https + Windows 凭据管理器已存 GitHub 登录(无 gh CLI, 禁用 GIT_TERMINAL_PROMPT)

---

## GitHub 追加：数据/手册/笔记已上传（2026-09-17，用户确认公开）

- 用户确认仓库为 Public 后, 放开 .gitignore, 追加提交 27693ea 并推送成功。
- 现仓库含: 源码 + by_data/ + huace_data/ + by_manual/ + huace_manual/ + .codebuddy/ (共153文件)。
- 单文件最大82.36MB(<100MB硬限制), 仅触发50MB警告(GH001提示可用Git LFS), 未被拒收。
- 注: 数据/手册/私有记忆笔记现已在公开互联网上可见; 后续若需撤回, 需重写历史(filter-repo)或删库。
