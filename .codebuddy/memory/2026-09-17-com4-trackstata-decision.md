# 2026-09-17 北云 C/N0 数据源决策: 选 TRACKSTATA 而非 RANGECMPB

## 背景 / 用户准则
用户准则(反复强调): **哪个字段能给的载噪比(C/N0)信息更新鲜、更全面、更及时, 就用哪个字段**。
北云设备有两个串口文件都可取到 C/N0:
- COM3 的 `#TRACKSTATA` (ASCII)
- COM4 的 `RANGECMPB` (二进制压缩原始观测值)

## 结论(已确认, 用户认可)
**C/N0 分析统一用 COM3 的 `#TRACKSTATA`, COM4 的 `RANGECMPB` 暂缓不解码。**

## 决策依据(三维度对比, TRACKSTATA 胜出/持平)
| 维度 | TRACKSTATA (COM3) | RANGECMPB (COM4) |
|------|-------------------|------------------|
| 全面性 | 每颗卫星 x 每频点逐通道 C/N0(实测首条104通道) | 含C/N0但需先二进制解码 |
| 及时性 | 周期 0.2s (5Hz) | 周期 0.2s (5Hz) —— 持平, 无优势 |
| 可用性 | ASCII, 程序已直接解析 | 二进制Range压缩格式, 需另写解码器, 成本高 |

- **及时性**: 两者周期完全相同(都0.2s), RANGECMPB 在"更新鲜/更及时"上没有任何优势。
- **全面性**: TRACKSTATA 给的是逐卫星逐频点的细粒度 C/N0, 正是判断遮挡/多路径所需。
- **可用性**: TRACKSTATA 已解码可用; RANGECMPB 需额外二进制解码。

## TRACKSTATA 字段结构(UG016 4.2.26)
- 数据段 = sol status, pos type, cutoff, #chans, 然后每通道10字段。
- 每通道10字段: PRN, glofreq, ch-tr-status, psr伪距(m), Doppler(Hz), **C/No(dB-Hz)**, locktime(s), psr-res伪距残差(m), reject, psr-weight。
- C/N0 在每通道相对偏移 **+5**; 代码见 `parse_trackstat_cn0()`。
- 实测 `_com3.dat`: TRACKSTATA 7267条(0.2s), 首条 #chans=104, C/N0 平均约 40.1 dB-Hz。

## 唯一例外(避免日后误会)
RANGECMPB 比 TRACKSTATA 多的是 **伪距/载波相位原始观测值**, 仅用于"伪距残差/多路径定量建模"这类更深入分析。
- 只要目标是 **C/N0 载噪比**, TRACKSTATA 已完全够且更好拿, 选它是对的。
- 仅当将来要做 **多路径定量分析 / 伪距残差分析** 时, 才需回头实现 RANGECMPB 二进制解码。

## 关联
- 见 2026-09-16-solution-types.md 第3节(特色C/N0分析)。
- 华测侧对应: C/N0 用 `#RANGEA`(伪距/载波/多普勒/CN0)。

---

## 追加(2026-09-17 当日纠正): 撤销 COM4 引入 + 界面设备标识

### 背景
一度误以为北云需在 GUI 选 COM3+COM4 两个文件并自动探测 COM4。用户明确指出:
**"我们当时讨论只用 COM3, COM4 就不需要了, 也不需要有任何引入"** —— 已全部撤销。

### 已撤销(北云 gps_kinematic_analyzer_beiyun.py)
- 删除 `__init__` 的 `com4_file` 参数、`self.com4_file`/`input_format`/`com4_format` 属性
- 删除 `_find_com4_sibling()` 自动探测函数
- 删除 GUI 的 COM4 第二文件框(entry4/label4/browse4/select_file4)及自动带出逻辑
- 删除报告 HTML/MD 的 com4_html/com4_status/com4_md 变量与引用
- 删除命令行 `--com4` 参数
- 恢复为**只选 COM3 一个文件**。现已无任何 com4_file/com4_status/entry4 残留(已验证)。
- 报告中仅剩 1 处 "ICOM4 关注项" 静态说明文字(完整性章节, 说明ICOM4口有RANGECMPB未解码), 属内容注释, 保留。

### 保留(用户本轮明确要的): 界面/报告设备标识
两个程序 GUI 顶部加**醒目彩色横幅** + 窗口标题 + info_text + 报告H1/MD标题, 明确区分设备, 避免启动时混淆:
- 北云(beiyun): 蓝色横幅 `【适用设备：北云 BEIYUN M21 · GNSS/INS 组合惯导移动站】`
  - 窗口标题: 北云 BEIYUN M21 GNSS/INS组合惯导 Kinematic分析工具
  - 文件标签: 输入文件 - COM3 (ASCII文本, 必选)
  - 报告标题: ...【北云 BEIYUN M21 · GNSS/INS组合惯导】
- 华测(huace): 红色横幅 `【适用设备：华测 HUACE · 纯GNSS 移动站(无惯导)】`
  - 窗口标题: 华测 HUACE 纯GNSS Kinematic分析工具 (COM1)
  - 文件标签: 输入文件 - COM1 (ASCII文本, 必选; 支持.dat/.log), 文件对话框已支持 *.log
  - 报告标题: ...【华测 HUACE · 纯GNSS】

### 验证
- 两文件 ast.parse 语法OK; 实例化确认无 com4_file 属性
- 北云 _com3.dat 跑通(exit0), 报告H1含【北云 BEIYUN M21】, 章节1~10连续, COM4仅ICOM4关注项1处
- 华测 164216 跑通(exit0), 报告H1含【华测 HUACE · 纯GNSS】, 章节1~9连续, COM4出现0次

### 准则沉淀
**北云 C/N0 只用 COM3 TRACKSTATA; COM4(RANGECMPB二进制)不引入、不探测、不解码。**
仅当未来明确要做多路径/伪距残差定量分析时, 才单独评估是否启用 COM4 二进制解码。
