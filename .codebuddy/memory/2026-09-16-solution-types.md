# 2026-09-16 解类型规范化 + 特色C/N0分析 + 报告美化

## 本轮三大需求完成情况

### 1. 解类型(Pos Type)按产品区分命名 —— 已实现动态化
- 新增 `kinematic_core/criteria.py` 的 **SOLUTION_TYPES 注册表**：每类含 color/cn中文名/fixed布尔。
  覆盖 GNSS 公有(NARROW_INT/NARROW_FLOAT/SINGLE/PSRDIFF/SPPDIFF/PPP系列等) + INS组合(INS_RTKFIXED/RTKFLOAT/PSRSP等) + NONE。
- helper：`type_color()` `type_cn()` `is_fixed()` `present_types()` `legend_label(pos_type,count)`(生成"NARROW_INT(RTK固定解) 100")。
- ENU轨迹图/分布图改为 `sorted(set(...))` **按数据实际出现动态着色**，不再硬编码5类。
- **关键产品命名差异（务必记住）**：伪距差分 **北云=PSRDIFF、华测=SPPDIFF**，含义相同(手册表3-40/UG016表4-2)。
  原硬编码导致华测SPPDIFF在ENU图无色，已修复。

### 2. ⚠️ NONE 解漏统计 bug（重要，已修复）
- **现象**：分布图只显示4类，NONE(无解)缺失。
- **根因**：解析器 `if lat==0 or lon==0: continue` 把无解记录过滤了(无解时经纬度为0)。
- **实测确认 NONE 真实存在**：北云 BESTGNSSPOSA NONE=50(sol_status=RESIDUALS/VARIANCE时)、
  北云 INSPVAXA NONE=80、华测 BESTPA NONE=77(sol_status=NO_CONVERGENCE时)。
- **修复**：解析时保留无解记录并加 `valid_coord` 标记；绘图(位置时间序列/ENU轨迹/合并轨迹)前过滤 `valid_coord=False` 的点。
  这样解类型统计能拿到 NONE，轨迹图又不受零坐标污染。修改了北云 parse_bestgnss_posa+parse_inspvaxa、
  华测 parse_bestp_abbrev+parse_bestpa_ascii。

### 3. 特色字段 C/N0 载噪比分析（用户批准，标记"特色"）
- **北云用 TRACKSTATA**（0.2s ASCII已解码，比ICOM4 RANGECMPB二进制更及时丰富；RANGECMPB仍需二进制解码暂缓）。
  TRACKSTATA字段：数据段 f[3]=#chans，每通道10字段，C/N0在相对偏移+5。
- **华测用 #RANGEA**（伪距/载波/多普勒/CN0，华测独有采集）。RANGE结构：f[0]=#obs，每观测10字段，C/N0在idx+7。
  依据：华测手册正文无RANGE完整字段表，按NovAtel RANGE标准+与SATVIS2A的C/N0交叉验证(首观测32.4吻合)。
- 两版均新增 `parse_*_cn0()` `analyze_cn0()` `_plot_cn0()`，结果字段统一(available/n_epochs/n_obs_total/mean/median/min/max/low_ratio/epochs)。
- 报告新增"载噪比 C/N0 分析"章节(带 `.badge-feature` 紫色"特色"徽章)，附**解读指导**(怎么看直方图/时间序列、行业阈值含义)。
- **C/N0 行业共识阈值**(Kaplan《Understanding GPS》)：≥45强(开阔天空)/35~45中/<35弱(遮挡多路径)/跟踪门限约25~28 dB-Hz。

### 4. ⚠️ 华测报文头两种格式（关键，易错）
- **长格式头**(#ENVSTATUSA/#BESTPA等)：含FINESTEERING，`[6]=GPS周`、`[7]=TOW毫秒` → 时间戳用 `int(hp[7])/1000.0`。
- **短格式头**(#RANGEA/#SATVIS2A等)：无FINESTEERING，`[5]=GPS周`、`[6]=TOW周内秒(单位秒)`、`[7]=状态`。
- 曾出bug：parse_range_cn0误用长格式假设(int(hp[7])/1000)，导致C/N0时间序列图阶梯状错乱。已修复为 `float(hp[6])` 取TOW秒。

### 5. 报告章节编号修复 + 自适应
- 北云原HTML章节有bug：两个"4."(卫星可见性误标)、缺后续号。已重排：北云=1概览/2完整性/3GNSS解类型/4INS解类型/5卫星/6DOP/7速度/8C-N0/9位置/10结论；华测无INS章。
- **编号自适应**：位置/结论章节号用 `_pos_no`/`_conc_no` 变量，若无C/N0数据(如旧<BESTP缩写格式无RANGEA)则自动前移，避免跳号。
- MD与HTML同步重编号(含9.x/8.x子节)。
- 新增CSS类 `.badge-feature`(紫色徽章) `.note`(蓝色左侧边框注释框)。

### 6. 移除误导性"水平定位精度"评估
- Kinematic无真值，原 `analyze_position_accuracy` 写恒0结果导致报告出现"水平定位精度0.0000米-通过/满足厘米级"错误结论。
- 已改为不写 analysis_results，报告该项自动省略；清理两版MD/HTML死代码评估块。

## 评估算法一致性核对结论
- 两版 `run_analysis` 步骤顺序、结论评估逻辑、阈值**完全一致**：固定解率95/80、卫星10/6、HDOP 1/2。
- 北云多"INS固定解比率"评估项、多INS相关图(INS轨迹/合并轨迹) —— 属组合惯导产品正常差异，非遗漏。

## 数据目录现状(已变化，注意)
- 北云：`by_data\com3.dat` 和 `by_data\com3_data\` **已被移走**。现北云数据在 `by_data\20260916-164201\_com3.dat`(84MB,ASCII) + `_com4.dat`(16MB,二进制RANGECMPB未解码)。
- 华测三份log在 `huace_data\` 根目录：
  - `20260915162214_COM11.log`(595KB, 旧<BESTP缩写格式, 无BESTPA无RANGEA)
  - `20260916113643_M720_COM11.log`(78MB, 新格式, 含RANGEA/ENVSTATUSA)
  - `20260916164216_M720_COM11.log`(62MB, 新格式)
- 报告输出到各数据同名子目录(report.html/report.md/*.png)。

## 已生成报告(全部修复后代码, 已验证无乱码/编号正确/NONE显示/SPPDIFF着色/C-N0正确)
- 北云 _com3: GNSS 5类(NARROW_INT6596/PSRDIFF82/NARROW_FLOAT432/SINGLE106/NONE50), INS 4类(INS_RTKFIXED13193/RTKFLOAT1127/PSRSP132/NONE80), C/N0平均40.1dB-Hz
- 华测1113643: 4类(NARROW_INT15910/NARROW_FLOAT1183/SPPDIFF660/NONE77), C/N0平均38.3dB-Hz
- 华测164216: 新格式, C/N0平均约38dB-Hz(900s处遮挡下坠)
- 华测162214: 旧缩写格式, 无C/N0章(自适应编号位置7/结论8)

## 待办/可选
- 北云ICOM4 RANGECMPB二进制解码(多路径/伪距分析) —— 暂缓, TRACKSTATA已够C/N0用。
- matplotlib已配SimHei+unicode_minus=False, 中文/σ/±/Δ实测可渲染(_fonttest.png验证)。图表标题/轴仍用英文规避乱码, 中文仅在legend_label。
