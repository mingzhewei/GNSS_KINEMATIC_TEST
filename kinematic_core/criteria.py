# -*- coding: utf-8 -*-
"""
统一评估标准引擎（北云组合惯导 / 华测纯GNSS 共用）

设计准则（用户要求）：
- 两版报告"评估逻辑、算法逻辑、代码逻辑"保持一致，仅字段与消息格式做设备映射。
- 所有阈值尽量标注行业共识或设备手册依据，无共识的明确标注为"工程经验值"。
- 报告对每个指标输出"测什么 / 用什么字段 / 怎么算好 / 怎么算坏"的注释。

重要说明（诚实性）：
- RTK 固定解率、HDOP、卫星数、差分龄期、C/N0 等的"好/坏"阈值在行业内
  **没有强制统一标准**（各厂商/项目自定义）。本模块采用的阈值是
  **主流工程实践常用口径 + 设备手册建议**，凡手册明确给出的优先用手册值。
- 华测手册直接给出的阈值：ENVSTATUS 环境分(85优秀/60-84一般/<60不合格)、
  avg SNR(>=38开阔/32-37半遮挡/<=31严重遮挡)、PDOP(<=2.5好/2.5-5一般/>5差)。
"""

# 评估状态
PASS = 'pass'
WARN = 'warn'
FAIL = 'fail'
INFO = 'info'

STATUS_LABEL = {'pass': '通过', 'warn': '警告', 'fail': '失败', 'info': '提示'}


class Metric(object):
    """一个评估指标：数值 + 状态 + 面向用户的注释(测什么/怎么算好坏/依据)"""

    def __init__(self, key, name, value, status, note, basis=''):
        self.key = key          # 机器键
        self.name = name        # 显示名
        self.value = value      # 显示值(字符串)
        self.status = status    # pass/warn/fail/info
        self.note = note        # 注释: 这个字段判断动态的什么、怎么好怎么坏
        self.basis = basis      # 依据(手册章节 / 行业共识 / 工程经验)

    def as_dict(self):
        return {'key': self.key, 'name': self.name, 'value': self.value,
                'status': self.status, 'status_label': STATUS_LABEL.get(self.status, ''),
                'note': self.note, 'basis': self.basis}


# ----------------------------------------------------------------------------
# RTK 固定解率（GNSS 与 INS 通用口径）
# 行业常用口径: >=95% 优, 80~95% 可用, <80% 差（工程经验值，非强制标准）
# ----------------------------------------------------------------------------
def eval_fix_ratio(fixed_ratio, kind='GNSS'):
    if fixed_ratio is None:
        return Metric(kind + '_fix', kind + '固定解率', '无数据', INFO,
                      'Kinematic下固定解(NARROW_INT/INS_RTKFIXED)占比, 反映持续高精度定位能力',
                      '无固定解数据')
    if fixed_ratio >= 95:
        st, txt = PASS, '满足高精度定位要求'
    elif fixed_ratio >= 80:
        st, txt = WARN, '基本满足要求'
    else:
        st, txt = FAIL, '需要改善'
    note = ('Kinematic下 %s 固定解占比。判断持续的厘米级定位能力。'
            '>=95%%好, 80~95%%可用, <80%%差。' % kind)
    return Metric(kind + '_fix', kind + '固定解率', '%.1f%%' % fixed_ratio, st,
                  note + ' 实测: ' + txt, '行业常用口径(工程经验值, 非强制标准)')


# ----------------------------------------------------------------------------
# 卫星可见数
# ----------------------------------------------------------------------------
def eval_sat_count(avg_sats):
    if avg_sats is None:
        return Metric('sat_count', '卫星可见数', '无数据', INFO,
                      '参与定位的卫星数量, 影响几何构型与解的稳健性', '无数据')
    if avg_sats >= 10:
        st, txt = PASS, '良好'
    elif avg_sats >= 6:
        st, txt = WARN, '基本满足'
    else:
        st, txt = FAIL, '需要改善'
    note = ('平均可见/参与解算卫星数。判断定位几何条件。'
            '>=10良好, 6~10基本满足, <6差。多系统(GPS+BDS+GLO+GAL)Kinematic通常>20。')
    return Metric('sat_count', '卫星可见数', '%.1f 颗' % avg_sats, st,
                  note + ' 实测: ' + txt, '工程经验值(多系统联合定位)')


# ----------------------------------------------------------------------------
# DOP (HDOP)
# ----------------------------------------------------------------------------
def eval_hdop(avg_hdop):
    if avg_hdop is None:
        return Metric('hdop', 'HDOP', '无数据', INFO,
                      '水平精度因子, 反映卫星几何分布对水平定位的影响', '无数据')
    if avg_hdop < 1:
        st, txt = PASS, '优秀'
    elif avg_hdop < 2:
        st, txt = PASS, '良好'
    elif avg_hdop < 5:
        st, txt = WARN, '一般'
    else:
        st, txt = FAIL, '差'
    note = ('水平精度因子HDOP(越小越好)。判断卫星几何构型。'
            '<1优秀, 1~2良好, 2~5一般, >5差。')
    return Metric('hdop', 'HDOP', '%.2f' % avg_hdop, st,
                  note + ' 实测: ' + txt, '行业共识(DOP通用定义)')


# ----------------------------------------------------------------------------
# RTK 差分龄期（移动站）
# ----------------------------------------------------------------------------
def eval_diff_age(avg_age, max_age):
    if avg_age is None:
        return Metric('diff_age', '差分龄期', '无数据', INFO,
                      '基准站差分改正数的新旧程度, 影响RTK精度与固定可靠性', '无数据')
    if avg_age <= 2 and (max_age is None or max_age <= 10):
        st, txt = PASS, '链路良好'
    elif avg_age <= 5:
        st, txt = WARN, '链路一般'
    else:
        st, txt = FAIL, '链路偏差'
    note = ('RTK差分龄期(越小越好)。判断基准站数据链实时性。'
            '平均<=2s好, 2~5s一般, >5s差; 偶发大值提示链路中断。')
    return Metric('diff_age', '差分龄期', '平均%.2fs/最大%.1fs' % (avg_age, max_age or 0),
                  st, note + ' 实测: ' + txt, '行业常用口径(RTK工程经验)')


# ----------------------------------------------------------------------------
# 华测 ENVSTATUS 特色评估（手册表3-51 直接给出阈值）
# ----------------------------------------------------------------------------
def eval_env_status(env_score, avg_snr, sat_vis_rate, iono_level):
    """华测 M720 环境感知特色指标。阈值来自华测 M7 手册表3-51 / 表3-52。"""
    out = []
    if env_score is not None:
        if env_score >= 85:
            st, txt = PASS, '优秀'
        elif env_score >= 60:
            st, txt = WARN, '一般可用'
        else:
            st, txt = FAIL, '不合格'
        out.append(Metric('env_score', '环境质量分', '%.0f 分' % env_score, st,
                          '华测特色: 综合反映观测环境对定位的影响。'
                          '>=85优秀, 60~84一般, <60不合格。', '华测M7手册表3-51'))
    if avg_snr is not None:
        if avg_snr >= 38:
            st, txt = PASS, '开阔地'
        elif avg_snr >= 32:
            st, txt = WARN, '半遮挡'
        else:
            st, txt = FAIL, '严重遮挡'
        out.append(Metric('avg_snr', '平均载噪比SNR', '%.0f dB-Hz' % avg_snr, st,
                          '华测特色: 信号强度, 判断遮挡程度。'
                          '>=38开阔, 32~37半遮挡, <=31严重遮挡。', '华测M7手册表3-51'))
    if sat_vis_rate is not None:
        if sat_vis_rate >= 80:
            st, txt = PASS, '可视性好'
        elif sat_vis_rate >= 50:
            st, txt = WARN, '一般'
        else:
            st, txt = FAIL, '遮挡严重'
        out.append(Metric('sat_vis', '卫星可视比率', '%.0f%%' % sat_vis_rate, st,
                          '华测特色(移动站): 跟踪卫星数/理论可视卫星数。'
                          '越高越好, 反映天空遮挡。', '华测M7手册表3-51(阈值为工程经验)'))
    if iono_level is not None:
        iono_map = {0: ('QUIET', PASS, '平静'), 1: ('WEAK', PASS, '轻微'),
                    2: ('MODERATE', WARN, '中等'), 3: ('STRONG', FAIL, '强烈'),
                    4: ('EXTREME', FAIL, '超强'), 255: ('UNKNOWN', INFO, '未知')}
        name, st, txt = iono_map.get(iono_level, ('UNKNOWN', INFO, '未知'))
        out.append(Metric('iono', '电离层活跃', '%s(%s)' % (name, txt), st,
                          '华测特色: 电离层扰动会加大RTK固定难度。'
                          'QUIET/WEAK好, MODERATE及以上需关注。', '华测M7手册表3-52'))
    return out


# ----------------------------------------------------------------------------
# 北云组合惯导特色评估（INSPVAX σ 精度系列, 来自 UG016 表4-15）
# ----------------------------------------------------------------------------
def eval_ins_sigma(lat_sig, hgt_sig, azim_sig):
    """北云特色: INSPVAX 输出的位置/姿态标准差, 反映组合导航实时精度估计。"""
    out = []
    if lat_sig is not None:
        if lat_sig <= 0.02:
            st, txt = PASS, '厘米级'
        elif lat_sig <= 0.10:
            st, txt = WARN, '分米级'
        else:
            st, txt = FAIL, '米级'
        out.append(Metric('pos_sigma', '水平位置精度σ', '%.3f m' % lat_sig, st,
                          '组合惯导特色: 位置标准差(接收机自估)。'
                          '<=0.02m厘米级, 0.02~0.1m分米级, >0.1m米级。', '北云UG016 INSPVAX字段(阈值为工程经验)'))
    if azim_sig is not None:
        if azim_sig <= 0.1:
            st, txt = PASS, '航向精度高'
        elif azim_sig <= 0.5:
            st, txt = WARN, '一般'
        else:
            st, txt = FAIL, '航向发散'
        out.append(Metric('azim_sigma', '航向角精度σ', '%.3f deg' % azim_sig, st,
                          '组合惯导特色: 航向角标准差, 判断姿态收敛。'
                          '<=0.1deg高, 0.1~0.5一般, >0.5发散。', '北云UG016 INSPVAX字段(阈值为工程经验)'))
    return out

# ----------------------------------------------------------------------------
# GNSS/INS 位置类型(Pos Type)注册表 —— 统一颜色/中文名/官方枚举依据
# 依据: 华测 M7 手册 表3-40 (Position or Velocity Type);
#       北云 UG016 表4-2 (定位状态描述, 与 NovAtel OEM 命名一致)。
# 说明: 两厂命名大体一致, 但伪距差分项 北云=PSRDIFF / 华测=SPPDIFF (同一含义不同命名);
#       INS 组合解(INS_*)仅北云(组合惯导)有, 纯GNSS(华测)无。
# ----------------------------------------------------------------------------
SOLUTION_TYPES = {
    # --- GNSS 公有 ---
    'NARROW_INT':    {'color': '#00C000', 'cn': 'RTK固定解', 'fixed': True},
    'NARROW_FLOAT':  {'color': '#0066FF', 'cn': 'RTK浮点解', 'fixed': False},
    'SINGLE':        {'color': '#FF0000', 'cn': '单点定位', 'fixed': False},
    'PSRDIFF':       {'color': '#FFAA00', 'cn': '伪距差分(北云命名)', 'fixed': False},
    'SPPDIFF':       {'color': '#FFAA00', 'cn': '伪距差分(华测命名)', 'fixed': False},
    'PPP':           {'color': '#9B59B6', 'cn': 'PPP', 'fixed': False},
    'PPP_FIXED':     {'color': '#8E44AD', 'cn': 'PPP固定', 'fixed': True},
    'PPP_CONVERGING':{'color': '#D2B4DE', 'cn': 'PPP收敛中', 'fixed': False},
    'SBAS':          {'color': '#E67E22', 'cn': 'SBAS', 'fixed': False},
    'WAAS':          {'color': '#E67E22', 'cn': 'WAAS', 'fixed': False},
    'L1_INT':        {'color': '#1ABC9C', 'cn': 'L1固定', 'fixed': True},
    'L1_FLOAT':      {'color': '#34495E', 'cn': 'L1浮点', 'fixed': False},
    'WIDE_INT':      {'color': '#9B59B6', 'cn': '宽巷固定', 'fixed': True},
    'FIXEDPOS':      {'color': '#7F8C8D', 'cn': '指定位置', 'fixed': False},
    # --- INS 组合(仅北云) ---
    'INS_RTKFIXED':  {'color': '#00C000', 'cn': 'INS固定解', 'fixed': True},
    'INS_RTKFLOAT':  {'color': '#0066FF', 'cn': 'INS浮点解', 'fixed': False},
    'INS_PSRSP':     {'color': '#FFAA00', 'cn': 'INS单点', 'fixed': False},
    'INS_PSRDIFF':   {'color': '#FFAA00', 'cn': 'INS伪距差分', 'fixed': False},
    'INS_PPP':       {'color': '#9B59B6', 'cn': 'INS_PPP', 'fixed': False},
    'INS_PPPFIXED':  {'color': '#8E44AD', 'cn': 'INS_PPP固定', 'fixed': True},
    # --- 无解 ---
    'NONE':          {'color': '#808080', 'cn': '无解', 'fixed': False},
}
UNKNOWN_COLOR = '#7F8C8D'


def type_color(pos_type):
    return SOLUTION_TYPES.get(pos_type, {}).get('color', UNKNOWN_COLOR)


def type_cn(pos_type):
    return SOLUTION_TYPES.get(pos_type, {}).get('cn', pos_type)


def is_fixed(pos_type):
    return SOLUTION_TYPES.get(pos_type, {}).get('fixed', False)


def present_types(type_count):
    """按数量降序返回数据实际出现的解类型列表(动态, 不硬编码)。"""
    return [k for k, _ in sorted(type_count.items(), key=lambda x: x[1], reverse=True)]


def legend_label(pos_type, count):
    return '%s(%s) %d' % (pos_type, type_cn(pos_type), count)

