#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
华测 M720 纯GNSS/RTK Kinematic定位数据综合分析程序

功能说明：
    针对华测 M720 模组（纯GNSS，无内置惯导、未外接IMU）的定位数据分析。
    依据《M7系列模组用户指令及协议手册 V2.7》解析报文，生成与北云版同口径的分析报告。
    主要功能：
    1. 解析 <BESTP(缩写主源,10Hz)/<RTKP/<RTKV/#BESTPOSA/#BESTDOPSA 等华测专有报文
    2. 解析 $GNGGA/$GNGSA/$GPGSV 等 NMEA 标准报文
    3. 定位质量分析：采样率/丢帧、解类型分布、卫星跟踪/参与解算数量、DOP、速度(RTKV)、RTK链路(基站ID/差分龄期)
    4. 生成可视化图表（轨迹图、时间序列、分布图）
    5. 自动生成 HTML 和 Markdown 分析报告

符合规范：
    - 华测 M7 系列模组用户指令及协议手册 V2.7
    - NMEA 0183 标准

数据源说明：
    - 主定位源: <BESTP 缩写格式(10Hz, 表3-38)；校验源: #BESTPOSA(基站ID/差分龄期)
    - 速度源: <RTKV(表3-97, 水平速度+航迹角分解为E/N分量, 仅取 SOL_COMPUTED 有效帧)
    - M720 为纯GNSS模组, 不输出 IMUOBS/INSSOLX/INSPVAXA, 程序已移除全部惯导处理
    
作者：GPS Kinematic Analyzer
版本：2.0
日期：2026-05-12
"""

# ============================================================================
# 导入必要的Python标准库
# ============================================================================
import os                     # 操作系统接口模块，用于文件和路径操作
import re                     # 正则表达式模块，用于文本模式匹配
import json                   # JSON数据处理模块
import math                   # 数学函数模块，提供三角函数、平方根等
import datetime               # 日期和时间处理模块
import argparse               # 命令行参数解析模块
import statistics             # 统计计算模块，提供均值、标准差等函数
from collections import defaultdict
from kinematic_core import criteria
from collections import Counter

# ============================================================================
# 导入第三方科学计算和可视化库
# ============================================================================
try:
    import numpy as np
    import matplotlib.pyplot as plt
    import pandas as pd
    from scipy import stats
    import seaborn as sns
except ImportError as e:
    print(f"缺少必要的依赖库: {e}")
    print("请运行以下命令安装依赖:")
    print("  pip install numpy matplotlib pandas scipy seaborn")
    import sys
    sys.exit(1)

# ============================================================================
# 设置matplotlib的中文字体配置
# ============================================================================
# 设置中文字体为SimHei（黑体），确保中文标签正常显示
plt.rcParams['font.sans-serif'] = ['SimHei']
# 设置负号正常显示（避免显示为方块）
plt.rcParams['axes.unicode_minus'] = False


# ============================================================================
# GPSKinematic数据分析器类
# ============================================================================
class GPSKinematicAnalyzer:
    # ========================================================================
    # COM1 口输出配置(设定周期), 来源: huace_manual/setup.jpg 截屏
    #   全部 OUTMSG,COM1,<消息>,0.100,NOHOLD -> 设定周期 0.1s (10Hz)
    # 用户准则(2026-09-16):
    #   - 完整性检查 + 实际周期与设定核对, 界面显示实际周期, 不符标色告警
    #   - 带时间戳的报文以时间戳为准; 无时间戳按报文插入规律推断
    #   - 数据是摆在那的事实, 以实测数据为准
    # 注: 配置中的 GGA/GSV/GST/GSA 实际以多系统 talker 输出($GNGGA/$GNGSA/
    #     $GNGST/$xxGSV), 按 NMEA 惯例归类统计
    # ========================================================================
    COM1_CONFIG = [
        # (显示名, 数据键名, 设定周期s, 是否必须出现)
        ('GGA',       'GGA',       0.1, True),
        ('GSV',       'GSV',       0.1, True),
        ('GST',       'GST',       0.1, True),
        ('GSA',       'GSA',       0.1, True),
        ('BESTDOPSA', 'BESTDOPSA', 0.1, True),
        ('BESTPA',    'BESTPA',    0.1, True),
        ('RTKPA',     'RTKPA',     0.1, True),
        ('RTKVA',     'RTKVA',     0.1, True),
        ('BESTVA',    'BESTVA',    0.1, True),
        ('ENVSTATUSA','ENVSTATUSA',0.1, True),
    ]
    # 报文类别 -> 实测头归组规则
    NMEA_GROUPS = {
        'GGA': ('$GNGGA', '$GPGGA'),
        'GST': ('$GNGST', '$GPGST'),
        'GSA_PREFIX': ('GSA',),
        'GSV_SUFFIX': ('GSV',),
    }

    """
    GPS/INSKinematic数据综合分析器
    
    该类封装了完整的GPS RTK和INS组合导航数据的解析、分析和可视化功能。
    支持北云科技设备的多种报文格式，能够生成详细的定位质量分析报告。
    
    Attributes:
        input_file (str): 输入的原始数据文件路径
        filename (str): 输入文件的文件名（不含扩展名）
        output_dir (str): 输出目录路径
        parsed_data (defaultdict): 解析后的报文数据字典，key为报文头，value为报文列表
        header_info (dict): 报文头部信息统计
        analysis_results (dict): 分析结果存储字典
        mark_anomalies (bool): 是否标记异常跳跃点
        speed_threshold (float): 速度阈值（m/s），用于检测异常点
    """
    def __init__(self, input_file, mark_anomalies=False, speed_threshold=2.0):
        """
        初始化GPSKinematic分析器
        
        Args:
            input_file (str): 输入的.dat文件完整路径
            mark_anomalies (bool): 是否在轨迹图中标记异常跳跃点，默认为False
            speed_threshold (float): 速度阈值（m/s），超过此阈值的点被认为是异常跳跃点，默认为2.0
            
        Returns:
            None
            
        Example:
            >>> analyzer = GPSKinematicAnalyzer("data.dat", mark_anomalies=True, speed_threshold=3.0)
        """
        # 保存输入文件路径
        self.input_file = input_file
        # 提取文件名（不含扩展名），用于创建输出目录
        self.filename = os.path.basename(input_file).split('.')[0]
        
        # 避免使用Windows保留设备名作为目录名（如COM1-9, LPT1-9, CON, PRN, NUL）
        if self.filename.lower() in ['com1', 'com2', 'com3', 'com4', 'com5', 
                                     'com6', 'com7', 'com8', 'com9', 
                                     'lpt1', 'lpt2', 'lpt3', 'lpt4', 'lpt5', 
                                     'lpt6', 'lpt7', 'lpt8', 'lpt9', 
                                     'con', 'prn', 'nul']:
            # 如果是保留名，在文件名后添加"_data"后缀
            self.filename = f"{self.filename}_data"
        
        # 构建输出目录路径（与输入文件同目录下的子目录）
        self.output_dir = os.path.join(os.path.dirname(input_file), self.filename)
        # 初始化解析数据存储（使用defaultdict，访问不存在的key时返回空列表）
        self.parsed_data = defaultdict(list)
        # 初始化报文头部信息统计字典
        self.header_info = {}
        # 初始化分析结果存储字典
        self.analysis_results = {}
        # 保存是否标记异常点的标志
        self.mark_anomalies = mark_anomalies
        # 保存速度阈值
        self.speed_threshold = speed_threshold
        
    # 华测COM1可能出现的报文头锚点(以实测数据为准, 表3-x)
    _HASH_HEADS = [b"#BESTPA", b"#BESTVA", b"#RTKPA", b"#RTKVA",
                   b"#ENVSTATUSA", b"#BESTDOPSA", b"#SATVIS2A",
                   b"#BESTPOSA", b"#RANGEA"]
    _NMEA_HEADS = [b"$GNGSA", b"$GNGGA", b"$GNGST", b"$GPGSV", b"$GBGSV",
                   b"$GAGSV", b"$GLGSV", b"$GQGSV", b"$GIGSV"]

    # NovAtel CRC-32 查表(表4-6: 反射多项式0xEDB88320, 初值0)
    _CRC_TAB = []
    @classmethod
    def _crc_table(cls):
        if not cls._CRC_TAB:
            for n in range(256):
                c = n
                for _ in range(8):
                    c = ((c >> 1) ^ 0xEDB88320) if (c & 1) else (c >> 1)
                cls._CRC_TAB.append(c)
        return cls._CRC_TAB

    @classmethod
    def _novatel_crc32(cls, data):
        """NovAtel CRC-32(反射, 初值0). 注意: zlib.crc32为非反射, 不可用。"""
        tab = cls._crc_table()
        crc = 0
        for b in data:
            crc = tab[(crc ^ b) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    @staticmethod
    def _nmea_xor(data):
        c = 0
        for x in data:
            c ^= x
        return c

    def _extract_embedded_messages(self):
        """从 文本+二进制混合流 按[消息头锚点+校验和验证]提取完整合法报文.

        二进制块会把一条ASCII报文拦腰切断, 按行读取时被丢弃。本方法以消息头
        为锚点, 定位其'*'校验结尾, 用CRC-32(#类)/NMEA XOR($类)验证完整性,
        只有校验通过的报文才补入 parsed_data(仅补原本缺失的头, 不重复)。
        返回补回的报文条数。
        """
        try:
            with open(self.input_file, 'rb') as fb:
                raw = fb.read()
        except Exception:
            return 0

        # 收集所有锚点 (pos, head)
        anchors = []
        for h in self._HASH_HEADS + self._NMEA_HEADS:
            start = 0
            while True:
                k = raw.find(h, start)
                if k < 0:
                    break
                anchors.append((k, h))
                start = k + 1
        anchors.sort()

        rescued = 0
        recovered = {}
        for pos, head in anchors:
            star = raw.find(b"*", pos, pos + 700)
            if star < 0:
                continue
            body = raw[pos + 1:star]          # '#'/'$' 之后、'*' 之前
            if head.startswith(b"#"):
                m = re.match(rb"[0-9a-fA-F]{8}", raw[star + 1:star + 9])
                if not m:
                    continue
                try:
                    want = int(m.group(0), 16)
                except ValueError:
                    continue
                if self._novatel_crc32(body) != want:
                    continue
                line_b = raw[pos:star + 9]
            else:
                m = re.match(rb"[0-9a-fA-F]{2}", raw[star + 1:star + 3])
                if not m:
                    continue
                try:
                    want = int(m.group(0), 16)
                except ValueError:
                    continue
                if self._nmea_xor(body) != want:
                    continue
                line_b = raw[pos:star + 3]
            try:
                line = line_b.decode("ascii")
            except UnicodeDecodeError:
                continue
            # 校验通过的合法报文 -> 按头收集
            header = line.split(',')[0]
            recovered.setdefault(header, []).append(line)
            rescued += 1

        # 合并策略: 对"按行解析不完整"的头, 用校验通过的集合替换/补全。
        # 逐行解析会把被二进制切断的报文丢掉, 而本集合只含校验通过的完整报文,
        # 数量通常 >= 逐行结果; 故对锚点清单内的头, 若本集合更多则采用之。
        for header, lines in recovered.items():
            existing = self.parsed_data.get(header, [])
            if len(lines) > len(existing):
                self.parsed_data[header] = lines
        return rescued

    def create_output_dir(self):
        """
        创建输出目录
        
        检查输出目录是否存在，如果不存在则创建该目录。
        所有分析结果（图表、报告、分类数据文件）都将保存在此目录中。
        
        Returns:
            None
            
        Side Effects:
            - 在文件系统中创建目录
            - 打印目录创建信息到控制台
        """
        # 检查输出目录是否已存在
        if not os.path.exists(self.output_dir):
            # 创建目录（包括所有必需的父目录）
            os.makedirs(self.output_dir)
            # 打印创建信息
            print(f"创建输出目录: {self.output_dir}")
    
    def parse_dat_file(self):
        """
        解析原始.dat文件
        
        读取输入的.dat文件，按报文头类型对数据进行分类存储。
        支持NovAtel格式（以#开头）和NMEA格式（以$开头）的报文。
        跳过包含二进制数据的行。
        
        Returns:
            bool: 解析成功返回True，失败返回False
            
        Side Effects:
            - 填充self.parsed_data字典，key为报文头，value为报文列表
            - 填充self.header_info字典，包含总行数和各报文类型计数
            - 打印解析统计信息到控制台
            
        Example:
            >>> analyzer = GPSKinematicAnalyzer("data.dat")
            >>> success = analyzer.parse_dat_file()
            >>> if success:
            ...     print(f"找到{len(analyzer.parsed_data)}种报文类型")
        """
        # 打印解析开始信息
        print(f"开始解析文件: {self.input_file}")
        
        # 尝试打开并读取文件
        try:
            # 以UTF-8编码打开文件，遇到无法解码的字符时忽略（errors='ignore'）
            with open(self.input_file, 'r', encoding='utf-8', errors='ignore') as f:
                # 读取所有行到列表中
                lines = f.readlines()
        except Exception as e:
            # 捕获所有异常（如文件不存在、权限不足等）
            print(f"读取文件失败: {e}")
            # 返回失败标志
            return False
        
        # 初始化二进制/无法识别行计数器(完整性检查用)
        binary_lines = 0
        garbage_lines = 0
        raw_line_count = 0
        # 遍历每一行数据
        for line in lines:
            # 去除行首尾的空白字符（空格、制表符、换行符等）
            line = line.strip()
            raw_line_count += 1
            # 跳过空行
            if not line:
                continue

            # 检查是否包含空字符（null character，表示二进制数据）
            if '\x00' in line:
                # 二进制数据行计数加1
                binary_lines += 1
                # 跳过该行，不进行解析
                continue

            # 提取报文头标识符
            if line.startswith('#'):
                # NovAtel专有格式：报文头是第一个逗号前的部分
                header = line.split(',')[0]
            elif line.startswith('$'):
                # NMEA 0183标准格式：报文头是第一个逗号前的部分
                header = line.split(',')[0]
            elif line.startswith('<'):
                # 华测 Abbreviated ASCII 缩写格式：报文头是第一个空格前的部分
                header = line.split()[0]
            else:
                # 既不是已知格式(乱码/残片行)
                garbage_lines += 1
                continue

            # 过滤乱码报文头：正常报文头仅含 ASCII 可打印字母/数字/符号(0x20-0x7E)
            # 串口丢字节会引入控制字符或扩展字符，产生非法文件名并干扰统计，直接跳过
            if not all(0x20 <= ord(c) <= 0x7E for c in header):
                garbage_lines += 1
                continue

            # 将该行添加到对应报文头的列表中
            self.parsed_data[header].append(line)

        # M720 为纯 GNSS 模组(无内置惯导、未外接IMU), 不处理惯导航文
        # 从 parsed_data 中移除, 避免后续统计/保存/分析引用
        for _k in ('#IMUOBSA', '#INSSOLXA', '#INSPVAXA', '#IMURATEPVAA'):
            self.parsed_data.pop(_k, None)

        # ====================================================================
        # 嵌入式ASCII报文提取(关键修复 2026-09-17)
        # 背景(实测, 非猜测): 华测COM11日志是"不定长二进制块 + ASCII报文"交错
        # 的原始流。二进制块会把本应一行一条的ASCII报文拦腰切断, 按行读取时
        # 该报文被劈成两段、整体丢弃(如#ENVSTATUSA字节流出现14448次, 按行
        # 仅收到部分)。本段以"消息头锚点 + 报文自带校验位验证"提取完整合法
        # 报文: #类用CRC-32(表4-6 反射多项式0xEDB88320 初值0, 域=#后*前),
        # $类用NMEA XOR($后*前)。只有校验通过才保留, 天然剔除被二进制污染
        # 的残缺段——不猜测, 以报文自带校验位为准。
        # ====================================================================
        try:
            rescued = self._extract_embedded_messages()
            if rescued:
                print(f"嵌入式ASCII报文恢复: 校验通过并补回 {rescued} 条"
                      f"(如 #ENVSTATUSA/#BESTDOPSA 等原本被二进制切断的报文)")
        except Exception as e:
            print(f"嵌入式报文提取失败(忽略): {e}")

        # 如果有二进制数据行，打印警告信息
        if binary_lines > 0:
            print(f"警告: 跳过了 {binary_lines} 行二进制数据")
        
        # 统计解析结果信息
        self.header_info = {
            # 原始文件总行数(完整性检查基准)
            'raw_line_count': raw_line_count,
            # 计算所有报文的总行数
            'total_lines': sum(len(lines) for lines in self.parsed_data.values()),
            # 统计每种报文类型的数量（报文头 -> 行数）
            'header_count': {header: len(lines) for header, lines in self.parsed_data.items()},
            # 二进制行 / 乱码残片行(串口丢字节等)
            'binary_lines': binary_lines,
            'garbage_lines': garbage_lines,
            # 缩写格式数据行(行首'<', 属于上一条缩写报文的数据部分)
            'abbrev_data_lines': self.header_info_extra.get('abbrev_data_lines', 0)
                              if hasattr(self, 'header_info_extra') else 0,
        }
        # 缩写数据行计数(parsed_data 中 '<' 键)
        self.header_info['abbrev_data_lines'] = len(self.parsed_data.get('<', []))
        
        # 打印解析完成信息和统计结果
        print(f"解析完成，找到 {len(self.parsed_data)} 种报文类型")
        # 遍历每种报文类型，打印其名称和数量
        for header, count in self.header_info['header_count'].items():
            print(f"  {header}: {count} 条")
        
        # 返回成功标志
        return True
    
    def save_header_files(self):
        """
        将各报文类型保存到独立的.dat文件
        
        将解析后的每种报文类型分别保存到输出目录中的独立文件中。
        文件名格式为：{报文头}.dat
        
        Returns:
            None
            
        Side Effects:
            - 在输出目录中创建多个.dat文件，每个文件包含一种报文类型的所有数据
            - 打印每个文件的保存信息到控制台
            
        Note:
            文件名中的特殊字符（<>:"/\\|?*）会被替换为下划线，以确保文件名合法
        """
        # 遍历每种报文类型及其对应的数据行列表
        for header, lines in self.parsed_data.items():
            # 清理文件名，将非法字符替换为下划线
            safe_header = re.sub(r'[<>:"/\\|?*]', '_', header)
            # 构建输出文件的完整路径
            output_file = os.path.join(self.output_dir, f"{safe_header}.dat")
            
            # 以写入模式和UTF-8编码打开文件
            with open(output_file, 'w', encoding='utf-8') as f:
                # 将所有行用换行符连接后写入文件
                f.write('\n'.join(lines))
            
            # 打印保存信息
            print(f"保存 {header} 到 {output_file}")
    
    # ========================================================================
    # Kinematic 检测: 完整性检查 + 报文周期核对(用户准则 2026-09-16)
    # ========================================================================

    def _collect_category_lines(self, key):
        """按配置类别归集实测报文行(多系统NMEA按类别合并)"""
        counts = self.header_info.get('header_count', {})
        if key == 'GGA':
            headers = [h for h in counts if h in self.NMEA_GROUPS['GGA']]
        elif key == 'GST':
            headers = [h for h in counts if h in self.NMEA_GROUPS['GST']]
        elif key == 'GSA':
            headers = [h for h in counts if h.endswith('GSA')]
        elif key == 'GSV':
            headers = [h for h in counts if h.endswith('GSV')]
        else:
            targets = {'#' + key}
            # 缩写格式头词: 键名去掉尾A(<BESTP 对应 BESTPA)
            abbrev = '<' + key.rstrip('A')
            headers = [h for h in counts
                       if h in targets or (h.startswith('<') and len(h) > 1
                                           and h.split(',')[0].split()[0] == abbrev)]
        lines = []
        for h in headers:
            lines.extend(self.parsed_data.get(h, []))
        return headers, lines

    def _extract_category_timestamps(self, key, lines):
        """提取类别报文时间戳(带时间戳的以时间戳为准, 数据为准)

        - #GMF报文(ASCII): 头字段[7]=TOW毫秒(华测表3-18), /1000转秒
        - #NovAtel风格(#BESTPOSA): 头字段[6]=周秒(秒)
        - $NMEA GGA/GST: 字段[1]=UTC hhmmss.sss -> 当天秒
        - GSA/GSV 无可靠时间戳 -> 返回空, 走报文插入规律推断
        """
        timestamps = []
        for line in lines:
            try:
                if line.startswith('#'):
                    hp = line.split(',', 8)
                    if line.startswith('#BESTPOSA'):
                        timestamps.append(float(hp[6]))
                    else:
                        timestamps.append(int(hp[7]) / 1000.0)
                elif line.startswith('<'):
                    # 华测缩写格式头行: <BESTP ... [6]=GPS周 [7]=TOW毫秒(空格分隔)
                    hp = line.split()
                    timestamps.append(int(hp[7]) / 1000.0)
                elif line.startswith('$') and key in ('GGA', 'GST'):
                    time_str = line.split(',')[1]
                    hour = int(time_str[:2])
                    minute = int(time_str[2:4])
                    second = float(time_str[4:])
                    timestamps.append(hour * 3600 + minute * 60 + second)
            except (ValueError, IndexError):
                continue
        return sorted(timestamps)

    def check_message_integrity(self):
        """步骤1-3: 完整性检查, 提取各配置报文实际周期并与COM1设定核对

        判定规则(与北云版一致):
        - 报文缺失 -> error(标红)
        - 实测周期与设定偏差>5% -> warn(标黄)
        - 估算丢帧率>1% -> warn(标黄)
        - 其余 -> ok(绿色)
        """
        print("完整性检查与报文周期核对(COM1 配置)...")
        results = []
        for display, key, configured, required in self.COM1_CONFIG:
            headers, lines = self._collect_category_lines(key)
            count = len(lines)
            entry = {
                'header': display, 'name': key, 'matched': headers,
                'configured_period': configured, 'count': count,
                'required': required, 'status': 'ok', 'note': '',
                'actual_period': None, 'actual_rate': None,
                'loss_rate': None, 'time_span': None,
            }
            if count == 0:
                entry['status'] = 'error' if required else 'info'
                entry['note'] = '报文缺失: 数据流中未找到, 与COM1配置不符'
                results.append(entry)
                print(f"  {display}: 缺失(设定{configured:.3f}s)")
                continue

            per_header_stats = []
            # 按报文头分别提取时间戳(同一头内计算周期, 避免跨头混排虚假丢帧)
            for h in headers:
                h_lines = self.parsed_data.get(h, [])
                ts = self._extract_category_timestamps(key, h_lines)
                if len(ts) >= 2:
                    iv = [ts[i+1] - ts[i] for i in range(len(ts)-1)
                          if ts[i+1] - ts[i] > 1e-6]
                    if iv:
                        med = statistics.median(iv)
                        span = ts[-1] - ts[0]
                        expected = span / med + 1 if med > 0 else 0
                        loss = (max(0.0, (1 - len(ts) / expected) * 100)
                                if expected > 0 else 0)
                        per_header_stats.append({
                            'header': h, 'count': len(h_lines),
                            'med': med, 'rate': 1.0 / med if med > 0 else None,
                            'span': span, 'loss': loss,
                        })
            if per_header_stats:
                # 类别代表值: 取帧数最多的头的统计; 多头的完整信息写入备注
                rep = max(per_header_stats, key=lambda x: x['count'])
                entry['actual_period'] = rep['med']
                entry['actual_rate'] = rep['rate']
                entry['time_span'] = rep['span']
                entry['loss_rate'] = rep['loss']
                entry['per_header'] = per_header_stats
            else:
                # 无可用时间戳: 按报文插入规律推断(以实测数据为准)
                entry['note'] = '无可用时间戳, 按报文插入规律推断(以实测数据为准)'

            notes = []
            if entry['actual_period'] is not None and configured > 0:
                dev = abs(entry['actual_period'] - configured) / configured
                if dev > 0.05:
                    entry['status'] = 'warn'
                    notes.append(
                        f"实际周期{entry['actual_period']:.3f}s与设定{configured:.3f}s"
                        f"偏差{dev*100:.1f}%, 请核对配置")
            if entry['loss_rate'] is not None and entry['loss_rate'] > 1.0:
                if entry['status'] == 'ok':
                    entry['status'] = 'warn'
                notes.append(f"估算丢帧率{entry['loss_rate']:.1f}%")
            if len(per_header_stats) > 1:
                detail = ' / '.join(
                    f"{p['header'].lstrip('$#<')}:{p['med']:.3f}s"
                    for p in sorted(per_header_stats, key=lambda x: -x['count']))
                notes.append(f"各头实际周期 {detail}")
            if notes:
                entry['note'] = ('; '.join(notes) if not entry['note']
                                 else entry['note'] + '; ' + '; '.join(notes))
            results.append(entry)
            if entry['actual_period'] is not None:
                print(f"  {display}({','.join(headers)}): 实际周期 "
                      f"{entry['actual_period']:.3f}s ({entry['actual_rate']:.2f}Hz), "
                      f"设定 {configured:.3f}s, {count}条, 状态 {entry['status']}"
                      + (f" - {entry['note']}" if entry['note'] else ''))
            else:
                print(f"  {display}({','.join(headers)}): {count}条, "
                      f"设定 {configured:.3f}s, 状态 {entry['status']}"
                      + (f" - {entry['note']}" if entry['note'] else ''))

        self.analysis_results['message_integrity'] = {
            'raw_line_count': self.header_info.get('raw_line_count', 0),
            'total_lines': self.header_info.get('total_lines', 0),
            'binary_lines': self.header_info.get('binary_lines', 0),
            'garbage_lines': self.header_info.get('garbage_lines', 0),
            'checks': results,
        }
        return results

    def analyze_sampling_rate(self):
        """分析采样率"""
        print("分析采样率...")
        
        # 华测: 优先用 <BESTP 缩写格式(10Hz) 的 GPS 周秒时间戳
        bestp = self.parse_bestp_abbrev()
        if bestp and len(bestp) >= 2:
            timestamps = sorted(d['timestamp'] for d in bestp)
            intervals = [timestamps[i+1]-timestamps[i] for i in range(len(timestamps)-1) if timestamps[i+1]-timestamps[i] > 1e-6]
            if intervals:
                # 用中位间隔表示真实配置采样率(抗丢帧干扰), 平均间隔反映含丢帧的整体情况
                med_interval = statistics.median(intervals)
                avg_interval = statistics.mean(intervals)
                med_rate = 1.0/med_interval if med_interval > 0 else 0
                avg_rate = 1.0/avg_interval if avg_interval > 0 else 0
                # 丢帧统计: 间隔明显大于中位间隔的视为丢帧段
                gap_thresh = med_interval * 5
                gaps = [iv for iv in intervals if iv > gap_thresh]
                total_span = timestamps[-1]-timestamps[0]
                expected = total_span/med_interval if med_interval>0 else 0
                loss_rate = max(0.0, (1 - len(bestp)/expected)*100) if expected>0 else 0
                self.analysis_results['sampling_rate'] = {
                    'average_interval': avg_interval,
                    'median_interval': med_interval,
                    'sampling_rate': med_rate,
                    'avg_rate': avg_rate,
                    'gap_count': len(gaps),
                    'loss_rate': loss_rate,
                    'intervals': intervals, 'source': '<BESTP(缩写格式)'}
                print(f"BESTP配置采样率(中位间隔): {med_rate:.2f}Hz (间隔{med_interval:.3f}秒)")
                print(f"BESTP有效采样率(含丢帧): {avg_rate:.2f}Hz")
                print(f"丢帧段数: {len(gaps)}, 估算丢帧率: {loss_rate:.1f}%")
            return

        # 回退: NMEA GGA 报文(华测输出 $GNGGA 多系统, 兼容 $GPGGA)
        gga_lines = self.parsed_data.get('$GNGGA', []) or self.parsed_data.get('$GPGGA', [])
        if not gga_lines:
            print("未找到GGA报文，无法分析采样率")
            return

        # 解析GGA时间
        timestamps = []
        for line in gga_lines:
            parts = line.split(',')
            if len(parts) >= 2 and parts[1]:
                time_str = parts[1]
                try:
                    # 格式: HHMMSS.sss
                    hour = int(time_str[:2])
                    minute = int(time_str[2:4])
                    second = float(time_str[4:])
                    timestamp = hour * 3600 + minute * 60 + second
                    timestamps.append(timestamp)
                except:
                    continue
        
        if len(timestamps) < 2:
            print("GGA报文数量不足，无法计算采样率")
            return
        
        # 计算时间间隔
        intervals = []
        for i in range(1, len(timestamps)):
            interval = timestamps[i] - timestamps[i-1]
            # 处理跨天的情况（23:59:59 -> 00:00:00）
            if interval < 0:
                interval += 86400
            intervals.append(interval)
        
        if intervals:
            avg_interval = statistics.mean(intervals)
            sampling_rate = 1.0 / avg_interval if avg_interval > 0 else 0
            
            self.analysis_results['sampling_rate'] = {
                'average_interval': avg_interval,
                'sampling_rate': sampling_rate,
                'intervals': intervals
            }
            
            print(f"GGA平均采样间隔: {avg_interval:.3f}秒")
            print(f"GGA采样率: {sampling_rate:.2f}Hz")
    
    def parse_bestp_abbrev(self):
        """解析华测 <BESTP 缩写格式报文(主定位源, 10Hz)

        数据特征(依据 M7 协议手册 表3-38 + Abbreviated ASCII 格式):
        - 头行:  <BESTP 59.0 0 0 COM1 FINESTEERING 2435 460382600 425d9794 00000200
                 字段: [6]=GPS周 [7]=TOW周内毫秒
        - 数据行: <\tSOL_COMPUTED NARROW_INT WGS84 00 31.16... 121.45... ...
                 空格分隔, 字段顺序同 BESTP:
                 [0]解算状态 [1]定位类型 [2]坐标系 [3]Reserved
                 [4]纬度 [5]经度 [6]海拔高 [7]高程异常
                 [8]纬度σ [9]经度σ [10]高程σ [11]差分龄期 [12]解算时间
                 [13]跟踪卫星数 [14]参与解算卫星数 ...
        - 数据中约13%为串口丢字节导致的乱码, 需逐帧校验跳过
        """
        if hasattr(self, '_cache_bestp_abbrev'):
            return self._cache_bestp_abbrev

        # 读取原始文件行(逐行扫描, 因为缩写格式跨两行)
        try:
            with open(self.input_file, 'r', encoding='utf-8', errors='ignore') as f:
                raw_lines = f.read().splitlines()
        except Exception:
            self._cache_bestp_abbrev = []
            return []

        data = []
        for i, ln in enumerate(raw_lines):
            ln = ln.strip()
            if not ln.startswith('<BESTP') or i + 1 >= len(raw_lines):
                continue
            hp = ln.split()
            if len(hp) < 8:
                continue
            try:
                timestamp = int(hp[7]) / 1000.0  # TOW毫秒 -> 秒
            except (ValueError, IndexError):
                continue

            d = raw_lines[i + 1].strip()
            if d.startswith('<'):
                d = d[1:].strip()
            f = d.split()
            if len(f) < 15:
                continue
            try:
                lat = float(f[4]); lon = float(f[5]); height = float(f[6])
                # 坐标有效性校验(过滤乱码); 无解(NONE)记录坐标为0但保留供解类型统计
                if abs(lat) > 90 or abs(lon) > 180:
                    continue
                valid_coord = not (lat == 0 or lon == 0)
                lat_sigma = float(f[8]) if f[8] else 0
                lon_sigma = float(f[9]) if f[9] else 0
                hgt_sigma = float(f[10]) if f[10] else 0
                num_svs = int(f[13]) if f[13].isdigit() else 0
                num_soln_svs = int(f[14]) if f[14].isdigit() else 0
            except (ValueError, IndexError):
                continue  # 乱码帧跳过

            horizontal_sigma = math.sqrt(lat_sigma ** 2 + lon_sigma ** 2)
            data.append({
                'timestamp': timestamp,
                'latitude': lat, 'longitude': lon, 'height': height,
                'valid_coord': valid_coord,
                'sol_status': f[0].strip(),
                'pos_type': f[1].strip(),
                'solution_type': f[1].strip(),
                'lat_sigma': lat_sigma, 'lon_sigma': lon_sigma, 'hgt_sigma': hgt_sigma,
                'horizontal_sigma': horizontal_sigma,
                'vertical_sigma': hgt_sigma,
                'num_satellites': num_svs,
                'num_soln_satellites': num_soln_svs,
            })
        self._cache_bestp_abbrev = data
        return data

    def parse_bestposa_station(self):
        """解析 #BESTPOSA (NovAtel风格, 5s), 补充基站ID等校验信息

        #BESTPOSA 字段: [10]=差分基站ID [11]=差分龄期 [12]=解算时间
        这些是 <BESTP 缩写格式所没有的元信息, 用于RTK链路核对
        """
        if hasattr(self, '_cache_bestposa_stn'):
            return self._cache_bestposa_stn
        data = []
        try:
            with open(self.input_file, 'r', encoding='utf-8', errors='ignore') as f:
                for ln in f:
                    ln = ln.strip()
                    if not ln.startswith('#BESTPOSA') or ';' not in ln:
                        continue
                    try:
                        h, d = ln.split(';', 1)
                        hp = h.split(',')
                        timestamp = float(hp[6])
                        f = d.split(',')
                        if len(f) < 13:
                            continue
                        stn = f[10].strip().strip('"')
                        data.append({
                            'timestamp': timestamp,
                            'sol_status': f[0].strip(),
                            'pos_type': f[1].strip(),
                            'station_id': stn,
                            'diff_age': f[11].strip(),
                            'sol_age': f[12].strip(),
                        })
                    except (ValueError, IndexError):
                        continue
        except Exception:
            pass
        self._cache_bestposa_stn = data
        return data

    def parse_range_cn0(self):
        """解析华测特色报文 RANGEA 的载噪比 C/N0 (原始观测值)

        RANGE 结构(NovAtel标准, 华测沿用): 数据段 = [#obs] + 每观测10字段
          [0]PRN [1]glofreq [2]psr伪距(m) [3]psr_std [4]ADR载波(周) [5]ADR_std
          [6]doppler多普勒(Hz) [7]C/No载噪比(dB-Hz) [8]locktime [9]ch-tr-status
        注: 华测手册正文未给出RANGE完整字段表, 本条按数据实际结构与NovAtel RANGE
            标准解析, 并经与SATVIS2A的C/N0交叉验证一致(首观测C/N0=32.4吻合)。
        用途: C/N0是判断遮挡/多路径/信号质量的行业通用指标。
        """
        if hasattr(self, '_cache_range_cn0'):
            return self._cache_range_cn0
        epochs = []
        for line in self.parsed_data.get('#RANGEA', []):
            if ';' not in line:
                continue
            try:
                header_str, data_str = line.split(';', 1)
                hp = header_str.split(',')
                # RANGEA为短格式头: [5]=GPS周 [6]=TOW周内秒(单位秒, 如272225.000)
                # (区别于ENVSTATUSA/BESTPA长格式头[7]=TOW毫秒), 据数据实测字段确认
                ts = None
                if len(hp) > 6:
                    try:
                        ts = float(hp[6])  # 短格式头TOW秒
                    except ValueError:
                        ts = None
                f = data_str.split(',')
                n_obs = int(f[0])
                cn0s = []
                idx = 1
                for _ in range(n_obs):
                    if idx + 9 >= len(f):
                        break
                    try:
                        cn0 = float(f[idx + 7])
                        if cn0 > 0:
                            cn0s.append(cn0)
                    except ValueError:
                        pass
                    idx += 10
                if cn0s:
                    epochs.append({'timestamp': ts, 'cn0_list': cn0s,
                                   'n_obs': len(cn0s),
                                   'mean_cn0': sum(cn0s) / len(cn0s)})
            except (ValueError, IndexError):
                continue
        self._cache_range_cn0 = epochs
        return epochs

    def analyze_cn0(self):
        """载噪比C/N0 分析(华测特色: RANGEA原始观测值)

        行业共识(教科书/接收机通用, 如 Kaplan & Hegarty《Understanding GPS》):
          C/N0 >= 45 dB-Hz 信号强; 35~45 中等; < 35 偏弱(遮挡/多路径风险)。
          接收机跟踪门限一般约 25~28 dB-Hz。
        输出: mean/median/percentiles, 以及低C/N0(<35)占比 -> 判遮挡程度。
        """
        import statistics as _st
        epochs = self.parse_range_cn0()
        res = {'available': False}
        if epochs:
            all_cn0 = [c for e in epochs for c in e['cn0_list']]
            mean_per_epoch = [e['mean_cn0'] for e in epochs]
            low = sum(1 for c in all_cn0 if c < 35)
            res = {
                'available': True,
                'n_epochs': len(epochs),
                'n_obs_total': len(all_cn0),
                'mean_cn0': _st.mean(all_cn0),
                'median_cn0': _st.median(all_cn0),
                'min_cn0': min(all_cn0),
                'max_cn0': max(all_cn0),
                'low_ratio': low / len(all_cn0) * 100 if all_cn0 else 0,
                'epochs': epochs,  # 供绘图
            }
        self.analysis_results['cn0'] = res
        if res['available']:
            print(f"C/N0分析: {res['n_epochs']}历元, 平均{res['mean_cn0']:.1f}dB-Hz, "
                  f"低C/N0占比{res['low_ratio']:.1f}%")
        return res

    def parse_envstatus(self):
        """解析华测特色报文 ENVSTATUSA (M7手册表3-51, 环境感知)

        数据字段(分号后):
        [0]env score 环境质量分 [1]pos type [2]sat vis rate 卫星可视比率
        [3]sat sol rate 解算卫星比率 [4]sol confidence 解置信度
        [5]moved 基站位移(移动站不用) [6]reserved [7]#SVs 跟踪卫星数
        [8]cv sats 共视卫星数 [9]IONO level 电离层活跃 [10]avg SNR 平均载噪比
        头字段[7]=TOW毫秒(GMF头)。
        """
        if hasattr(self, '_cache_envstatus'):
            return self._cache_envstatus
        data = []
        for line in self.parsed_data.get('#ENVSTATUSA', []):
            if ';' not in line:
                continue
            try:
                header_str, data_str = line.split(';', 1)
                hp = header_str.split(',')
                ts = int(hp[7]) / 1000.0 if len(hp) > 7 and hp[7].isdigit() else None
                f = data_str.split(',')
                if len(f) < 11:
                    continue
                iono_raw = f[9].strip()
                iono_map = {'QUIET': 0, 'WEAK': 1, 'MODERATE': 2, 'STRONG': 3,
                            'EXTREME': 4, 'UNKNOWN': 255}
                data.append({
                    'timestamp': ts,
                    'env_score': float(f[0]),
                    'pos_type': f[1].strip(),
                    'sat_vis_rate': float(f[2]),
                    'sat_sol_rate': float(f[3]),
                    'sol_confidence': float(f[4]),
                    'num_svs': float(f[7]),
                    'cv_sats': float(f[8]),
                    'iono_level': iono_map.get(iono_raw, 255) if not iono_raw.isdigit() else int(iono_raw),
                    'avg_snr': float(f[10]),
                })
            except (ValueError, IndexError):
                continue
        self._cache_envstatus = data
        return data

    def analyze_special_metrics(self):
        """纯GNSS特色指标(华测): ENVSTATUS 环境感知

        字段来源: ENVSTATUSA (M7手册表3-51)。
        用途: 判断观测环境(遮挡/电离层)对Kinematic定位的影响, 辅助解释固定解率波动。
        好/坏阈值直接采用华测手册表3-51/3-52。
        """
        env = self.parse_envstatus()
        metrics = []
        if env:
            import statistics as _st
            def med(key):
                vals = [d[key] for d in env if d.get(key) is not None]
                return _st.median(vals) if vals else None
            metrics.extend(criteria.eval_env_status(
                med('env_score'), med('avg_snr'), med('sat_vis_rate'), med('iono_level')))
        # C/N0 载噪比特色指标(RANGEA原始观测值)
        cn0 = self.analysis_results.get('cn0', {})
        if cn0.get('available'):
            mean_cn0 = cn0['mean_cn0']
            if mean_cn0 >= 45:
                st, txt = criteria.PASS, '信号强'
            elif mean_cn0 >= 35:
                st, txt = criteria.WARN, '中等'
            else:
                st, txt = criteria.FAIL, '偏弱(遮挡/多路径风险)'
            metrics.append(criteria.Metric(
                'cn0_mean', '平均载噪比C/N0',
                '%.1f dB-Hz (低C/N0占比%.1f%%)' % (mean_cn0, cn0['low_ratio']), st,
                '华测特色(RANGEA): 信号载噪比, 判断遮挡/多路径。'
                '>=45强, 35~45中等, <35偏弱, 跟踪门限约25~28。详见cn0_analysis.png。'
                '实测: %s。' % txt,
                'RANGEA原始观测(经SATVIS2A交叉验证); 阈值行业共识(Kaplan等)'))
        self.analysis_results['special_metrics'] = [m.as_dict() for m in metrics]
        print(f"环境感知特色指标(ENVSTATUS+C/N0): {len(metrics)}项")

    def analyze_rtk_link(self):
        """RTK链路核对: 用 #BESTPOSA 提供的基站ID/差分龄期/解算时间

        <BESTP 缩写格式无基站ID字段, 故用低密度的 #BESTPOSA 补充该元信息;
        新采集格式(#BESTPA)末字段即基站ID, 作为回退校验源.
        正常情况下整段RTK数据基站ID应恒定, 差分龄期应较小且稳定.
        """
        stn_data = self.parse_bestposa_station()
        if not stn_data:
            # 回退: #BESTPA 标准ASCII(末字段基站ID, [11]差分龄期)
            stn_data = [{'station_id': d.get('station_id', ''),
                         'diff_age': str(d.get('diff_age', 0))}
                        for d in self.parse_bestpa_ascii()
                        if d.get('station_id')]
        if not stn_data:
            self.analysis_results['rtk_link'] = None
            return
        stn_ids = [d['station_id'] for d in stn_data if d['station_id']]
        diff_ages = []
        for d in stn_data:
            try:
                v = float(d['diff_age'])
                if v > 0: diff_ages.append(v)
            except (ValueError, TypeError):
                pass
        from collections import Counter
        stn_count = Counter(stn_ids)
        self.analysis_results['rtk_link'] = {
            'station_ids': dict(stn_count),
            'primary_station': stn_count.most_common(1)[0][0] if stn_count else 'N/A',
            'avg_diff_age': statistics.mean(diff_ages) if diff_ages else 0,
            'max_diff_age': max(diff_ages) if diff_ages else 0,
            'check_points': len(stn_data),
        }
        if stn_count:
            print(f"RTK基站ID: {self.analysis_results['rtk_link']['primary_station']} (核对{len(stn_data)}次)")
            if diff_ages:
                print(f"差分龄期: 平均{self.analysis_results['rtk_link']['avg_diff_age']:.2f}s, 最大{self.analysis_results['rtk_link']['max_diff_age']:.2f}s")

    def parse_gngsa_combined(self):
        """解析华测 $GNGSA 并整合为"组合DOP" (2026-09-17)

        华测COM1中, 每个定位周期会输出多条 $GNGSA, 每条对应一个星座
        (通过12个PRN槽编号区分 GPS/BDS/GLO/GAL...), 尾部携带 PDOP,HDOP,VDOP。

        【DOP 来源 - 行业共识】
        DOP(精度因子)是**接收机本地解算**的几何强度指标, 并非卫星下发;
        它只取决于所用卫星相对接收机的几何分布, 与星座数量正相关:
        参与解算的卫星/星座越多, 几何越好, 组合DOP越小。

        【多星座DOP如何整合 - NMEA 0183 v4.1 共识】
        多星座时接收机按星座各输出一条 GSA; NMEA规定其中**PRN槽数最多、
        代表"所有参与解算卫星组合"的那一条**给出整体(组合)DOP。
        工程实现: GSA无独立时间戳, 但每个周期多条GSA会"连续突发"输出,
        周期间有间隔。因此按"行间隔>0.5s 视为新周期"分组, 每组取
        PRN槽数最多的一条作为该周期权威 PDOP/HDOP/VDOP。
        注意: **不能对不同星座的DOP取平均**(各星座子集DOP不代表整体)。

        返回: [{'pdop','hdop','vdop','n_prn'}...] 每周期一条。
        """
        if hasattr(self, '_cache_gngsa'):
            return self._cache_gngsa
        raw_lines = self.parsed_data.get('$GNGSA', [])
        # 需要行号定位周期间隔, 但 parsed_data 不含行号; 改为直接读文件保留顺序即可,
        # 因 parsed_data['$GNGSA'] 本身按文件顺序排列, 周期间隔用"记录间隔"无法得知,
        # 故采用更稳的"PRN回落"切分: 一个周期内多条GSA输出完后, 下一周期重新从
        # 某星座开始; 由于组合条(全星)PRN最多, 一个周期内PRN先递增至峰值再回落,
        # 每当当前PRN < 上一段最大PRN 时认为上一周期结束。
        recs = []
        for ln in raw_lines:
            ln = ln.strip()
            if not ln.startswith('$GNGSA'):
                continue
            body = ln[1:].split('*')[0]
            f = body.split(',')
            if len(f) < 18:
                continue
            try:
                prn_slots = f[3:15]
                n_prn = sum(1 for p in prn_slots if p.strip())
                pdop = float(f[15]); hdop = float(f[16]); vdop = float(f[17])
            except (ValueError, IndexError):
                continue
            recs.append((n_prn, pdop, hdop, vdop))

        combined = []
        best = None          # 当前周期内 PRN 最多的一条
        prev_n = -1
        for rec in recs:
            n = rec[0]
            if best is not None and n < prev_n:
                # PRN 回落 -> 上一周期结束, 收录其组合(峰值)条
                combined.append(best)
                best = rec
            else:
                if best is None or n >= best[0]:
                    best = rec
            prev_n = n
        if best is not None:
            combined.append(best)

        self._cache_gngsa = [
            {'pdop': p, 'hdop': h, 'vdop': v, 'n_prn': n}
            for (n, p, h, v) in combined
        ]
        return self._cache_gngsa

    def parse_bestdopsa(self):
        """解析华测 #BESTDOPSA 最佳DOP报文(权威DOP源, 但【不含VDOP】)

        依据手册表3-36(M7系列V2.7 第80页), 数据字段(分号后)严格为:
        [0]pdop 位置精度因子
        [1]gdop 几何精度因子
        [2]hdop 水平精度因子
        [3]tdop 时间精度因子(假设3D位置已知、仅钟差未知)  <- 注意: 是TDOP不是VDOP
        [4]htdop 水平位置+时间精度因子
        [5]截止高度角(elev mask)
        [6]卫星数, 之后为各卫星PRN...
        实测样本: 0.8071,0.9247,0.4114,0.4514,0.8627,0.0,43,...
                  -> pdop=0.8071 gdop=0.9247 hdop=0.4114 tdop=0.4514 htdop=0.8627
        【重要】本报文无VDOP字段; VDOP 须从 $GNGSA(表3-3 第7字段=垂直精度因子)获取。
        数据来自 parsed_data(已被嵌入式锚点+校验提取补全, 含二进制切断的报文)。
        """
        if hasattr(self, '_cache_bestdopsa'):
            return self._cache_bestdopsa
        data = []
        for ln in self.parsed_data.get('#BESTDOPSA', []):
            if ';' not in ln:
                continue
            try:
                h, d = ln.split(';', 1)
                d = d.split('*')[0]               # 去掉CRC校验尾
                hp = h.split(',')
                timestamp = int(hp[7]) / 1000.0    # GMF头TOW毫秒
                f2 = d.split(',')
                data.append({
                    'timestamp': timestamp,
                    'pdop': float(f2[0]),
                    'gdop': float(f2[1]),
                    'hdop': float(f2[2]),
                    'tdop': float(f2[3]),                       # f2[3]=TDOP(原误作VDOP)
                    'htdop': float(f2[4]) if len(f2) > 4 else None,  # f2[4]=HTDOP(原误作TDOP)
                })
            except (ValueError, IndexError):
                continue
        self._cache_bestdopsa = data
        return data

    def parse_gnss_pos(self):
        """解析华测 GNSS 位置报文(主定位源, 提供坐标/解类型/卫星数)

        【华测实际报文】华测无 BESTGNSSPOSA, 主源为:
          1. <BESTP 缩写格式(10Hz, 主源, M7手册表3-38)
          2. #BESTPA 标准ASCII(回退, 字段同表3-38)
        数据字段(空格/逗号分隔, 顺序一致):
        [0]解算状态 [1]定位类型(Pos Type) [2]坐标系 [3]Reserved
        [4]纬度 [5]经度 [6]海拔高 [7]高程异常
        [8]纬度σ [9]经度σ [10]高程σ [11]差分龄期 [12]解算时间
        [13]#SVs跟踪卫星数 [14]#solnSVs参与解算卫星数
        """
        if hasattr(self, '_cache_gnsspos'):
            return self._cache_gnsspos

        # 华测: 优先使用 <BESTP 缩写格式(10Hz高密度), 字段语义同 #BESTPA(表3-38)且时间分辨率高
        bestp_abbrev = self.parse_bestp_abbrev()
        if bestp_abbrev:
            self._cache_gnsspos = bestp_abbrev
            return bestp_abbrev

        # 回退: 标准 BESTP ASCII 报文(新采集格式, 统一走 parse_bestpa_ascii)
        data = self.parse_bestpa_ascii()
        if data:
            self._cache_gnsspos = data
            return data
        return []
    
    def parse_solution_status(self):
        """解析华测解算状态(sol stat)时间序列

        【华测实际报文】解算状态取自与位置同源的:
          1. <BESTP 缩写格式(10Hz, 主源, M7手册表3-38)
          2. #BESTPA 标准ASCII(回退)
        注: #BESTPOSA(1Hz)仅用于基站ID/差分龄期校验, 不作解算状态主源。
        """
        if hasattr(self, '_cache_solution_status'):
            return self._cache_solution_status

        # 华测: 优先复用 <BESTP 缩写格式(高密度主源)
        bestp = self.parse_bestp_abbrev()
        if bestp:
            data = [{
                'timestamp': d['timestamp'],
                'sol_stat_str': d.get('sol_status', ''),
                'sol_stat': -1,
                'pos_type': d.get('pos_type', '')
            } for d in bestp]
            self._cache_solution_status = data
            return data

        # 回退: 标准 #BESTPA ASCII 报文(新采集格式)
        bestpa = self.parse_bestpa_ascii()
        if bestpa:
            data = [{
                'timestamp': d['timestamp'],
                'sol_stat_str': d.get('sol_status', ''),
                'sol_stat': -1,
                'pos_type': d.get('pos_type', '')
            } for d in bestpa]
            self._cache_solution_status = data
            return data
        self._cache_solution_status = []
        return []

    def parse_bestpa_ascii(self):
        """解析华测 #BESTPA 标准ASCII位置报文(新采集格式, 主定位源回退)

        依据 M7 协议手册 表3-38(字段顺序与 <BESTP 缩写一致, 逗号分隔):
        头: #BESTPA,...,[6]=GPS周,[7]=TOW毫秒;数据字段:
        [0]解算状态 [1]定位类型 [2]坐标系 [3]Reserved [4]纬度 [5]经度 [6]海拔高
        [7]高程异常 [8]纬度σ [9]经度σ [10]高程σ [11]差分龄期 [12]解算时间
        [13]跟踪卫星数 [14]解算卫星数 ... [末]基站ID
        """
        if hasattr(self, '_cache_bestpa_ascii'):
            return self._cache_bestpa_ascii
        data = []
        lines = self.parsed_data.get('#BESTPA', [])
        for line in lines:
            if ';' not in line:
                continue
            try:
                header_str, data_str = line.split(';', 1)
                hp = header_str.split(',')
                if len(hp) < 8:
                    continue
                timestamp = int(hp[7]) / 1000.0  # GMF头TOW毫秒
                f = data_str.split(',')
                if len(f) < 15:
                    continue
                lat = float(f[4]); lon = float(f[5]); height = float(f[6])
                if abs(lat) > 90 or abs(lon) > 180:
                    continue
                valid_coord = not (lat == 0 or lon == 0)
                lat_sigma = float(f[8]) if f[8] else 0
                lon_sigma = float(f[9]) if f[9] else 0
                hgt_sigma = float(f[10]) if f[10] else 0
                num_svs = int(f[13]) if f[13].strip().isdigit() else 0
                num_soln = int(f[14]) if f[14].strip().isdigit() else 0
                try:
                    diff_age = float(f[11]) if f[11] else 0.0
                except ValueError:
                    diff_age = 0.0
                # 基站ID为末字段(去CRC *xx 与引号)
                stn = f[-1].split('*')[0].strip().strip('"') if f else ''
                data.append({
                    'timestamp': timestamp,
                    'latitude': lat, 'longitude': lon, 'height': height,
                    'valid_coord': valid_coord,
                    'sol_status': f[0].strip(),
                    'pos_type': f[1].strip(),
                    'solution_type': f[1].strip(),
                    'lat_sigma': lat_sigma, 'lon_sigma': lon_sigma,
                    'hgt_sigma': hgt_sigma,
                    'horizontal_sigma': math.sqrt(lat_sigma ** 2 + lon_sigma ** 2),
                    'vertical_sigma': hgt_sigma,
                    'num_satellites': num_svs,
                    'num_soln_satellites': num_soln,
                    'station_id': stn,
                    'diff_age': diff_age,
                })
            except (ValueError, IndexError):
                continue
        self._cache_bestpa_ascii = data
        return data

    def parse_bestva_ascii(self):
        """解析华测 #BESTVA 最佳可用速度报文(手册表3-47, 新采集格式)

        头: #BESTVA,...,[6]=GPS周,[7]=TOW毫秒;数据字段(实测10字段):
        [0]速度解状态 [1]速度类型 [2]Reserved [3]水平对地速度m/s
        [4]地面航迹角(度,真北) [5]垂直速度m/s [6]latency延迟s [7]差分龄期s [8]Reserved
        注: 字段顺序参照同族 BESTVEL/RTKV 布局(sol stat, vel type, Reserved,
        hor spd, trk gnd, vert spd, latency, age), 并经 20260916113643 实测数据
        交叉验证: 时段TOW272300-272900步行(位置位移速度≈0.8-1.1m/s)与 [3]字段
        (0.003-0.93m/s, 静态近零)吻合; [4]字段取值0-360°随机变化, 符合航迹角特征。
        分解: vel_n = hor_spd*cos(trk_gnd), vel_e = hor_spd*sin(trk_gnd),
              vel_u = vert_spd
        仅保留 速度解状态==SOL_COMPUTED 的有效帧。
        """
        if hasattr(self, '_cache_bestva_ascii'):
            return self._cache_bestva_ascii
        data = []
        lines = self.parsed_data.get('#BESTVA', [])
        for line in lines:
            if ';' not in line:
                continue
            try:
                header_str, data_str = line.split(';', 1)
                hp = header_str.split(',')
                if len(hp) < 8:
                    continue
                timestamp = int(hp[7]) / 1000.0
                f = data_str.split(',')
                if len(f) < 7:
                    continue
                sol_status = f[0].strip()
                vel_type = f[1].strip()
                hor_spd = float(f[3])
                trk_gnd = float(f[4])
                vert_spd = float(f[5])
                latency = float(f[6]) if len(f) > 6 and f[6] else 0.0
                age = float(f[7]) if len(f) > 7 and f[7] else 0.0
            except (ValueError, IndexError):
                continue
            if sol_status != 'SOL_COMPUTED':
                continue
            rad = math.radians(trk_gnd)
            data.append({
                'timestamp': timestamp,
                'sol_status': sol_status,
                'vel_type': vel_type,
                'hor_spd': hor_spd, 'trk_gnd': trk_gnd, 'vert_spd': vert_spd,
                'latency': latency, 'age': age,
                'velocity_north': hor_spd * math.cos(rad),
                'velocity_east': hor_spd * math.sin(rad),
                'velocity_up': vert_spd,
            })
        self._cache_bestva_ascii = data
        return data

    def get_velocity_data(self):
        """速度数据统一入口: 优先 <RTKV(缩写), 回退 #BESTVA(标准ASCII)"""
        data = self.parse_rtkv()
        if data:
            return data, '<RTKV'
        data = self.parse_bestva_ascii()
        if data:
            return data, '#BESTVA'
        return [], None

    def parse_rtkv(self):
        """解析华测 <RTKV RTK速度报文(缩写格式)

        依据 M7 协议手册 表3-97(第203-204页) + Abbreviated ASCII 两行结构:
        - 头行:  <RTKV 25.0 0 0 COM1 APPROXIMATE 2435 462117746 425d9794 00000000
                 [6]=GPS周 [7]=TOW周内毫秒
        - 数据行: <\tSOL_COMPUTED NARROW_INT 0 0.0068 337.573 -0.0017 0.000 1.000 0.000
                 [0]sol stat解算状态 [1]vel type速度类型 [2]Reserved
                 [3]hor spd水平对地速度(m/s) [4]trk gnd地面航迹角(度,真北)
                 [5]vert spd垂直速度(m/s,正向上) [6]latency延迟(s) [7]age差分龄期(s)
        注: 手册示例 #RTKVA 为逗号分隔的ASCII格式, 本数据为 <RTKV 缩写(空格分隔),
        字段顺序一致。RTKV无E/N分量, 由水平速度+航迹角分解:
            vel_n = hor_spd*cos(trk_gnd), vel_e = hor_spd*sin(trk_gnd), vel_u = vert_spd
        """
        if hasattr(self, "_cache_rtkv"):
            return self._cache_rtkv
        data = []
        try:
            with open(self.input_file, "r", encoding="utf-8", errors="ignore") as fh:
                raw_lines = fh.read().splitlines()
        except Exception:
            self._cache_rtkv = []
            return []
        for i, ln in enumerate(raw_lines):
            ln = ln.strip()
            if not ln.startswith("<RTKV") or i + 1 >= len(raw_lines):
                continue
            hp = ln.split()
            if len(hp) < 8:
                continue
            try:
                timestamp = int(hp[7]) / 1000.0
            except (ValueError, IndexError):
                continue
            d = raw_lines[i + 1].strip()
            if d.startswith("<"):
                d = d[1:].strip()
            f = d.split()
            if len(f) < 6:
                continue
            try:
                sol_status = f[0].strip()
                vel_type = f[1].strip()
                hor_spd = float(f[3])
                trk_gnd = float(f[4])
                vert_spd = float(f[5])
                latency = float(f[6]) if len(f) > 6 else 0.0
                age = float(f[7]) if len(f) > 7 else 0.0
            except (ValueError, IndexError):
                continue
            # 仅保留解算成功的速度(手册: sol stat指示数据是否有效)
            if sol_status != "SOL_COMPUTED":
                continue
            rad = math.radians(trk_gnd)
            vel_n = hor_spd * math.cos(rad)
            vel_e = hor_spd * math.sin(rad)
            vel_u = vert_spd
            data.append({
                "timestamp": timestamp,
                "sol_status": sol_status,
                "vel_type": vel_type,
                "hor_spd": hor_spd,
                "trk_gnd": trk_gnd,
                "vert_spd": vert_spd,
                "latency": latency,
                "age": age,
                "velocity_north": vel_n,
                "velocity_east": vel_e,
                "velocity_up": vel_u,
            })
        self._cache_rtkv = data
        return data
    def parse_gpgga(self):
        """解析GPGGA报文"""
        if hasattr(self, '_cache_gpgga'):
            return self._cache_gpgga

        # 华测输出 $GNGGA(多系统GNSS), 兼容 $GPGGA(单GPS)
        lines = self.parsed_data.get('$GNGGA', []) or self.parsed_data.get('$GPGGA', [])
        data = []
        
        for line in lines:
            parts = line.split(',')
            if len(parts) < 15:
                continue
            
            try:
                # 解析时间
                time_str = parts[1]
                hour = int(time_str[:2])
                minute = int(time_str[2:4])
                second = float(time_str[4:])
                timestamp = hour * 3600 + minute * 60 + second
                
                # 解析位置
                lat_deg = float(parts[2][:2])
                lat_min = float(parts[2][2:])
                lat = lat_deg + lat_min / 60
                if parts[3] == 'S':
                    lat = -lat
                
                lon_deg = float(parts[4][:3])
                lon_min = float(parts[4][3:])
                lon = lon_deg + lon_min / 60
                if parts[5] == 'W':
                    lon = -lon
                
                # 解析其他信息
                fix_quality = int(parts[6]) if parts[6] else 0
                num_sats = int(parts[7]) if parts[7] else 0
                hdop = float(parts[8]) if parts[8] else 0
                height = float(parts[9]) if parts[9] else 0
                
                data.append({
                    'timestamp': timestamp,
                    'latitude': lat,
                    'longitude': lon,
                    'fix_quality': fix_quality,
                    'num_satellites': num_sats,
                    'hdop': hdop,
                    'height': height
                })
            except Exception as e:
                continue

        self._cache_gpgga = data
        return data

    def parse_gngst(self):
        """解析华测 $GNGST 伪距误差信息报文 (M7手册 3.1.5 GST, 表3-9)

        字段(手册原表, $--GST 通用, 华测实测 talker=GN):
        [0]log头 [1]utc 与该语句有关的GGA/GNS语句的UTC时间 hhmmss.ss
        [2]rms  = 伪距、DGNSS修正值的标准偏差的均方根 (即伪距残差RMS)
        [3]smjrstd 误差椭圆半长轴标准偏差(m) [4]smnrstd 半短轴(m)
        [5]orient  误差椭圆半长轴方向(度, 与真北夹角)
        [6]latstd 纬度标准偏差(m) [7]lonstd 经度标准偏差(m) [8]altstd 高度标准偏差(m)
        [9]*hh 校验

        说明: GST 无 GPS 周秒时间戳, 仅 UTC 时刻; 与 <BESTP/#BESTPA(周秒)时间轴
        不同源, 残差图横轴用相对首历元秒, 不做跨报文强制对齐。
        华测 COM11 为二进制+ASCII 交错流, $GNGST 已在 _NMEA_HEADS 锚点内,
        行解析与嵌入式校验提取两路互补, 数据取自 parsed_data。
        """
        if hasattr(self, '_cache_gngst'):
            return self._cache_gngst
        data = []
        lines = self.parsed_data.get('$GNGST', []) + self.parsed_data.get('$GPGST', [])
        for ln in lines:
            ln = ln.strip()
            if not (ln.startswith('$GNGST') or ln.startswith('$GPGST')):
                continue
            body = ln.split('*')[0]
            f = body.split(',')
            if len(f) < 9:
                continue
            try:
                utc = f[1].strip()
                rms = float(f[2]) if f[2] else None
                smjr = float(f[3]) if f[3] else None
                smnr = float(f[4]) if f[4] else None
                orient = float(f[5]) if f[5] else None
                latstd = float(f[6]) if f[6] else None
                lonstd = float(f[7]) if f[7] else None
                altstd = float(f[8]) if f[8] else None
            except (ValueError, IndexError):
                continue
            if rms is None:
                continue
            sec_of_day = None
            if len(utc) >= 6:
                try:
                    sec_of_day = (int(utc[0:2]) * 3600 + int(utc[2:4]) * 60
                                  + float(utc[4:]))
                except ValueError:
                    sec_of_day = None
            data.append({
                'utc': utc, 'sec_of_day': sec_of_day,
                'rms': rms, 'smjr': smjr, 'smnr': smnr, 'orient': orient,
                'lat_std': latstd, 'lon_std': lonstd, 'alt_std': altstd,
            })
        self._cache_gngst = data
        return data

    def analyze_pos_quality(self):
        """位置标准差sigma收敛 + 伪距残差RMS 分析 (华测)

        位置sigma: <BESTP(主源10Hz)/#BESTPA(回退) 数据字段[8]/[9]/[10]
                   = 纬度/经度/高度标准差, 单位 m (M7手册 表3-38 字段10/11/12)
        伪距残差RMS: $GNGST 字段3 = 伪距、DGNSS修正值的标准偏差的均方根 (M7手册 3.1.5 表3-9)

        两者均为接收机自估精度, 不是外业真值误差; 仅统计 SOL_COMPUTED 且坐标有效历元。
        与北云同口径(见 gps_kinematic_analyzer_beiyun.analyze_pos_quality),
        单位统一为 m, 保证两模组可比。
        """
        print("分析位置sigma收敛与伪距残差RMS (<BESTP/#BESTPA / GNGST)...")
        pos = self.parse_gnss_pos()
        gst = self.parse_gngst()

        pos_v = [d for d in pos
                 if d.get('valid_coord') and d.get('sol_status') == 'SOL_COMPUTED']
        res = {'pos_src': '<BESTP/#BESTPA', 'gst_src': '$GNGST',
               'pos_epochs': len(pos_v), 'gst_epochs': len(gst)}

        def _s(lst):
            return (statistics.mean(lst), statistics.median(lst), min(lst), max(lst)) if lst else (None,) * 4
        for key, name in (('lat_sigma', 'lat'), ('lon_sigma', 'lon'), ('hgt_sigma', 'hgt')):
            vals = [d[key] for d in pos_v if d.get(key) and d[key] > 0]
            a, med, mn, mx = _s(vals)
            if a is not None:
                res[name + '_mean'], res[name + '_med'] = a, med
                res[name + '_min'], res[name + '_max'] = mn, mx
        if pos_v:
            t0 = pos_v[0]['timestamp']
            res['pos_t'] = [d['timestamp'] - t0 for d in pos_v]
            res['pos_lat'] = [d['lat_sigma'] for d in pos_v]
            res['pos_lon'] = [d['lon_sigma'] for d in pos_v]
            res['pos_hgt'] = [d['hgt_sigma'] for d in pos_v]

        if gst:
            secs = [g['sec_of_day'] for g in gst]
            if all(v is not None for v in secs) and len(secs) >= 2:
                t0 = secs[0]
                rel = []
                prev = None
                wrap = 0.0
                for v in secs:
                    vv = v + wrap
                    if prev is not None and vv < prev - 43200:
                        wrap += 86400.0
                        vv = v + wrap
                    rel.append(vv - t0)
                    prev = vv
                res['gst_t'] = rel
            else:
                res['gst_t'] = [float(i) for i in range(len(gst))]
            res['gst_rms'] = [g['rms'] for g in gst]
            a, med, mn, mx = _s(res['gst_rms'])
            res['rms_mean'], res['rms_med'], res['rms_min'], res['rms_max'] = a, med, mn, mx

        self.analysis_results['pos_quality'] = res
        if pos_v:
            print("  位置sigma(<BESTP/#BESTPA): %d历元, sigma_lat中位 %.4fm, sigma_lon中位 %.4fm, sigma_hgt中位 %.4fm" % (
                len(pos_v), res.get('lat_med', float('nan')), res.get('lon_med', float('nan')), res.get('hgt_med', float('nan'))))
        else:
            print("  警告: 无有效位置sigma历元")
        if gst:
            print("  伪距残差RMS(GNGST): %d历元, 中位 %.3fm, 最大 %.3fm" % (
                len(gst), res.get('rms_med', float('nan')), res.get('rms_max', float('nan'))))
        else:
            print("  警告: 未找到 $GNGST 数据, 跳过伪距残差RMS")
        return res

    def analyze_position_accuracy(self):
        """分析位置精度 - Kinematic数据无真值，不生成误导性指标

        Kinematic场景无参考真值, 恒0的"水平定位精度0.0000米-通过"是错误结论,
        故不写入 analysis_results, 报告中该项自动省略。
        """
        print("Kinematic数据无真值，跳过位置精度评估(不生成误导性指标)")
    
    def analyze_solution_type(self):
        """分析解类型分布

        根据 M7手册(表3-38/3-39, 数据源<BESTP/#BESTPA):
        - Sol Status (解算状态): 表示解算是否成功 (SOL_COMPUTED等)
        - Pos Type (位置类型): 表示定位模式和精度等级 (NARROW_INT等)

        行业实践中,Pos Type是评估定位质量的关键指标
        """
        print("分析解类型分布...")

        # 分析GNSS位置类型
        bestgnss_data = self.parse_gnss_pos()
        gnss_type_count = {}
        gnss_sol_status_count = {}

        if bestgnss_data:
            for d in bestgnss_data:
                pos_type = d.get('pos_type', 'NONE')
                gnss_type_count[pos_type] = gnss_type_count.get(pos_type, 0) + 1

                sol_status = d.get('sol_status', 'UNKNOWN')
                gnss_sol_status_count[sol_status] = gnss_sol_status_count.get(sol_status, 0) + 1

        total_gnss = len(bestgnss_data) if bestgnss_data else 0
        fixed_count = gnss_type_count.get('NARROW_INT', 0)
        fixed_ratio = (fixed_count / total_gnss * 100) if total_gnss > 0 else 0

        self.analysis_results['gnss_solution_type'] = {
            'type_count': gnss_type_count,
            'sol_status_count': gnss_sol_status_count,
            'fixed_ratio': fixed_ratio,
            'total_epochs': total_gnss
        }

        # 分析INS位置类型

        # 同时保留旧格式以兼容
        if bestgnss_data:
            self.analysis_results['solution_type'] = {
                'type_count': gnss_type_count,
                'fixed_ratio': fixed_ratio,
                'total_epochs': total_gnss
            }

        # 打印详细统计
        print(f"\n=== GNSS定位质量分析 ===")
        print(f"总历元数: {total_gnss}")
        print(f"固定解(NARROW_INT): {fixed_count} ({fixed_ratio:.1f}%)")
        print(f"\nGNSS位置类型分布:")
        for pos_type, count in gnss_type_count.items():
            pct = (count/total_gnss*100) if total_gnss > 0 else 0
            print(f"  {pos_type}: {count} ({pct:.1f}%)")

        print(f"\nGNSS解算状态分布:")
        for sol_status, count in gnss_sol_status_count.items():
            pct = (count/total_gnss*100) if total_gnss > 0 else 0
            print(f"  {sol_status}: {count} ({pct:.1f}%)")

    
    def analyze_satellite_visibility(self):
        """分析卫星数量(跟踪/可用两层, M7手册定义)

        - 跟踪卫星数 Tracked : <BESTP/#BESTPA #SVs     "跟踪到的卫星数"(手册 p83/p86)
        - 可用卫星数 Used    : <BESTP/#BESTPA #solnSVs "参与解算的卫星数"(手册 p84)
        用户要求(2026-09-17): 只保留"跟踪"与"参与解算"两层, 不使用GSV可见数。
        关系: 跟踪 >= 可用。主源 <BESTP(10Hz), 回退 #BESTPA。
        回退: 无 BESTP/BESTPA 时用 GGA 字段8, 手册定义为"使用中的卫星数"
              (M7手册 3.1.1 表3-1), 属"可用"口径, 故回退时填入 used。
        兼容: 保留 average_satellites 等旧键(=跟踪口径), 供报告模板沿用。
        """
        print("分析卫星数量(跟踪/可用)...")

        bestp = self.parse_bestp_abbrev() or self.parse_bestpa_ascii()
        gpgga_data = self.parse_gpgga()

        def _stats(lst):
            return (statistics.mean(lst), min(lst), max(lst)) if lst else (None, None, None)

        res = {}
        tr = [d['num_satellites'] for d in bestp if d.get('num_satellites', 0) > 0] if bestp else []
        us = [d['num_soln_satellites'] for d in bestp if d.get('num_soln_satellites', 0) > 0] if bestp else []
        # 回退: GGA 字段8("使用中的卫星数", 属可用口径)
        if not us and gpgga_data:
            us = [d['num_satellites'] for d in gpgga_data if d.get('num_satellites', 0) > 0]

        for name, lst in (('tracked', tr), ('used', us)):
            a, mn, mx = _stats(lst)
            if a is not None:
                res[f'average_{name}'] = a
                res[f'min_{name}'] = mn
                res[f'max_{name}'] = mx
                res[f'{name}_counts'] = lst

        if not res:
            print("未找到卫星数量数据(BESTP/BESTPA/GGA)")
            return

        main = tr if tr else us
        if main:
            res['average_satellites'] = statistics.mean(main)
            res['min_satellites'] = min(main)
            res['max_satellites'] = max(main)
            res['satellite_counts'] = main

        self.analysis_results['satellite_visibility'] = res

        label = {'tracked': '跟踪卫星数', 'used': '可用卫星数'}
        for name in ('tracked', 'used'):
            if f'average_{name}' in res:
                print(f"平均{label[name]}: {res[f'average_{name}']:.1f} "
                      f"(最小{res[f'min_{name}']} 最大{res[f'max_{name}']})")
    
    def analyze_dop(self):
        """分析DOP值 (PDOP/HDOP/VDOP 三件套)

        数据源优先级(2026-09-17):
          1. #BESTDOPSA —— 华测权威DOP(手册表3-36): pdop/gdop/hdop/tdop/htdop,
             【不含VDOP】; VDOP 唯一来源为 $GNGSA(表3-3 第7字段=垂直精度因子)
          2. $GNGSA 组合 —— 多星座整合后的 PDOP/HDOP/VDOP (NMEA共识, 取组合条)
          3. $GNGGA      —— 仅HDOP (NMEA GGA只携带HDOP一个值, 无PDOP/VDOP)
        三件套齐全时报告同时给出 PDOP/HDOP/VDOP 的均值/最值。
        """
        print("分析DOP值...")

        bestdops = self.parse_bestdopsa()   # PDOP/GDOP/HDOP/TDOP/HTDOP(高精度4位小数, 无VDOP)
        gngsa = self.parse_gngsa_combined() # PDOP/HDOP/VDOP(1位小数, 含VDOP)
        gpgga_data = self.parse_gpgga()

        def _stats(lst):
            return (statistics.mean(lst), min(lst), max(lst)) if lst else (None, None, None)
        res = {}

        if bestdops:
            # 【分字段混合, 两源同频(均10Hz)且同源(实测PDOP/HDOP逐周期一致)】
            #   PDOP/HDOP 取 #BESTDOPSA(精度高, 权威); VDOP 取 $GNGSA(BESTDOPSA无此字段, 唯一来源)
            #   另附 BESTDOPSA 独有的 GDOP/TDOP/HTDOP 作参考。
            dop_src = 'PDOP/HDOP=#BESTDOPSA, VDOP=$GNGSA(同频混合)'
            res['dop_source'] = dop_src
            res['note'] = ('#BESTDOPSA不含VDOP(表3-36: PDOP/GDOP/HDOP/TDOP/HTDOP); '
                           'VDOP取自$GNGSA(表3-3第7字段=垂直精度因子)。两报文同为10Hz同频, '
                           '且PDOP/HDOP数值同源(GSA为1位小数舍入版), 故按字段混合使用。')
            pd_l = [d['pdop'] for d in bestdops if d.get('pdop', 0) > 0]
            hd_l = [d['hdop'] for d in bestdops if d.get('hdop', 0) > 0]
            vd_l = [g['vdop'] for g in gngsa if g.get('vdop', 0) > 0]
            res['timestamps'] = [d['timestamp'] for d in bestdops if d.get('pdop', 0) > 0]
            for name, lst in (('pdop', pd_l), ('hdop', hd_l), ('vdop', vd_l),
                              ('gdop', [d['gdop'] for d in bestdops if d.get('gdop', 0) > 0]),
                              ('tdop', [d['tdop'] for d in bestdops if d.get('tdop', 0) > 0]),
                              ('htdop', [d['htdop'] for d in bestdops if d.get('htdop') and d['htdop'] > 0])):
                a, mn, mx = _stats(lst)
                if a is not None:
                    res[f'average_{name}'] = a
                    res[f'min_{name}'] = mn
                    res[f'max_{name}'] = mx
                    res[f'{name}_values'] = lst
            print(f"DOP数据源: {dop_src}")
            for name in ('pdop', 'hdop', 'vdop', 'gdop', 'tdop', 'htdop'):
                if f'average_{name}' in res:
                    print(f"平均{name.upper()}: {res[f'average_{name}']:.3f} "
                          f"(最小{res[f'min_{name}']:.3f} 最大{res[f'max_{name}']:.3f})")
        elif gngsa:
            # 无BESTDOPSA时, 三件套全部回退 $GNGSA(组合条, 含VDOP)
            dop_src = '$GNGSA(多星座组合)'
            res['dop_source'] = dop_src
            for name in ('pdop', 'hdop', 'vdop'):
                lst = [g[name] for g in gngsa if g.get(name, 0) > 0]
                a, mn, mx = _stats(lst)
                if a is not None:
                    res[f'average_{name}'] = a
                    res[f'min_{name}'] = mn
                    res[f'max_{name}'] = mx
                    res[f'{name}_values'] = lst
            print(f"DOP数据源: {dop_src}")
            for name in ('pdop', 'hdop', 'vdop'):
                if f'average_{name}' in res:
                    print(f"平均{name.upper()}: {res[f'average_{name}']:.2f} "
                          f"(最小{res[f'min_{name}']:.2f} 最大{res[f'max_{name}']:.2f})")
        elif gpgga_data:
            dop_src = '$GNGGA(仅HDOP)'
            res['dop_source'] = dop_src
            lst = [d['hdop'] for d in gpgga_data if d.get('hdop', 0) > 0]
            a, mn, mx = _stats(lst)
            if a is not None:
                res['average_hdop'] = a; res['min_hdop'] = mn; res['max_hdop'] = mx
                res['hdop_values'] = lst
            print(f"DOP数据源: {dop_src}, 平均HDOP: {a:.2f}")
        else:
            print("未找到DOP数据")
            return

        self.analysis_results['dop'] = res

    def analyze_velocity(self):
        """分析速度"""
        print("分析速度...")
        
        # 华测: 速度数据源 <RTKV(缩写,表3-97) 优先, 回退 #BESTVA(表3-47)
        rtkv_data, vel_src = self.get_velocity_data()
        if not rtkv_data:
            print("未找到有效速度数据(RTKV/BESTVA均无 SOL_COMPUTED 帧)")
            return
        print(f"速度数据源: {vel_src}")

        velocities = []
        for d in rtkv_data:
            vel_n = d['velocity_north']
            vel_e = d['velocity_east']
            vel_u = d['velocity_up']
            speed = math.sqrt(vel_n**2 + vel_e**2 + vel_u**2)
            velocities.append(speed)
        
        if velocities:
            avg_speed = statistics.mean(velocities)
            max_speed = max(velocities)
            
            self.analysis_results['velocity'] = {
                'average_speed': avg_speed,
                'max_speed': max_speed,
                'speeds': velocities
            }
            
            print(f"平均速度: {avg_speed:.2f} m/s")
            print(f"最大速度: {max_speed:.2f} m/s")
    
    def generate_plots(self):
        """生成图表"""
        print("生成分析图表...")
        
        # 位置时间序列图（GNSS - <BESTP/#BESTPA, 华测实际位置主源）
        bestgnss_data = self.parse_gnss_pos()
        if bestgnss_data:
            self._plot_position_time_series(bestgnss_data, 'gnss')
        
        # 解类型分布图（GNSS）
        if 'gnss_solution_type' in self.analysis_results:
            self._plot_gnss_solution_type_distribution()
        
        # 卫星数量时间序列
        if 'satellite_visibility' in self.analysis_results:
            self._plot_satellite_time_series()
        
        # DOP时间序列
        if 'dop' in self.analysis_results:
            self._plot_dop_time_series()

        # 位置sigma收敛+伪距残差RMS 四联图
        if 'pos_quality' in self.analysis_results:
            self._plot_pos_quality()
        
        # 速度时间序列
        if 'velocity' in self.analysis_results:
            self._plot_velocity_time_series()
        
        # GNSS ENU轨迹图（按解类型着色）
        if bestgnss_data:
            self._plot_gnss_enu_trajectory(bestgnss_data)
        
        # 解算状态时间序列图（华测: <BESTP 主源的sol stat）
        bestpos_data = self.parse_solution_status()
        if bestpos_data:
            self._plot_solution_status(bestpos_data)
        else:
            print("无解算状态数据(<BESTP/#BESTPA均无)，跳过解算状态图")

        # C/N0载噪比图(华测特色: RANGEA原始观测值)
        if self.analysis_results.get('cn0', {}).get('available'):
            self._plot_cn0()
    
    def _plot_cn0(self):
        """绘制C/N0载噪比分析图(华测特色: 直方图+时间序列, 判断遮挡/多路径)

        参考行业共识: >=45强, 35~45中, <35弱(遮挡/多路径), 跟踪门限约25~28。
        """
        cn0 = self.analysis_results['cn0']
        epochs = cn0['epochs']
        all_cn0 = [c for e in epochs for c in e['cn0_list']]
        times = []
        means = []
        t0 = epochs[0]['timestamp'] or 0
        for e in epochs:
            times.append((e['timestamp'] or t0) - t0)
            means.append(e['mean_cn0'])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5))
        # 直方图(按行业阈值着色)
        bins = [0, 25, 35, 45, 70]
        labels_b = ['<25 Unusable', '25-35 Weak', '35-45 Moderate', '>=45 Strong']
        colors_b = ['#808080', '#FF0000', '#FFAA00', '#00C000']
        counts = [0, 0, 0, 0]
        for c in all_cn0:
            if c < 25: counts[0] += 1
            elif c < 35: counts[1] += 1
            elif c < 45: counts[2] += 1
            else: counts[3] += 1
        bars = ax1.bar(labels_b, counts, color=colors_b)
        tot = len(all_cn0)
        for b, v in zip(bars, counts):
            ax1.text(b.get_x() + b.get_width()/2., b.get_height(),
                     '%d\n(%.1f%%)' % (v, v/tot*100 if tot else 0),
                     ha='center', va='bottom', fontsize=9)
        ax1.set_title('C/N0 Distribution (per observation)', fontsize=13, fontweight='bold')
        ax1.set_ylabel('Count', fontsize=11)
        ax1.tick_params(axis='x', rotation=15)

        # 时间序列(历元平均C/N0)
        ax2.plot(times, means, color='#0066FF', linewidth=1.2)
        ax2.axhline(45, color='#00C000', linestyle='--', linewidth=1, label='Strong >=45')
        ax2.axhline(35, color='#FFAA00', linestyle='--', linewidth=1, label='Weak <35')
        ax2.set_title('Mean C/N0 per Epoch (Time Series)', fontsize=13, fontweight='bold')
        ax2.set_xlabel('Time (s)', fontsize=11)
        ax2.set_ylabel('Mean C/N0 (dB-Hz)', fontsize=11)
        ax2.legend(fontsize=9)
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        out = os.path.join(self.output_dir, 'cn0_analysis.png')
        plt.savefig(out, dpi=150)
        plt.close()
        print(f"保存C/N0载噪比图: {out}")

    def _plot_position_time_series(self, data, source='gnss'):
        """绘制位置时间序列图
        
        Args:
            data: 位置数据列表
            source: 数据源类型 'gnss' 或 'ins'
        """
        # 过滤无有效坐标的记录(无解时坐标为0)
        data = [d for d in data if d.get('valid_coord', True)]
        timestamps = [d['timestamp'] for d in data]
        lats = [d['latitude'] for d in data]
        lons = [d['longitude'] for d in data]
        heights = [d['height'] for d in data]
        
        # 使用相对时间
        if timestamps:
            min_time = min(timestamps)
            relative_times = [t - min_time for t in timestamps]
        else:
            relative_times = timestamps
        
        plt.figure(figsize=(12, 8))
        
        plt.subplot(3, 1, 1)
        plt.plot(relative_times, lats)
        plt.title('Latitude Time Series')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Latitude (degrees)')
        
        plt.subplot(3, 1, 2)
        plt.plot(relative_times, lons)
        plt.title('Longitude Time Series')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Longitude (degrees)')
        
        plt.subplot(3, 1, 3)
        plt.plot(relative_times, heights)
        plt.title('Height Time Series')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Height (meters)')
        
        plt.tight_layout()
        
        # 根据数据源生成不同的文件名
        if source == 'gnss':
            output_file = os.path.join(self.output_dir, 'position_time_series_gnss.png')
        else:
            output_file = os.path.join(self.output_dir, 'position_time_series_ins.png')
        plt.savefig(output_file)
        plt.close()
        print(f"保存位置时间序列图: {output_file}")
    
    def _plot_gnss_solution_type_distribution(self):
        """绘制GNSS解类型分布图"""
        type_count = self.analysis_results['gnss_solution_type']['type_count']
        total = self.analysis_results['gnss_solution_type']['total_epochs']

        if not type_count or total == 0:
            return

        # 按数量降序排列
        sorted_types = sorted(type_count.items(), key=lambda x: x[1], reverse=True)
        labels = [item[0] for item in sorted_types]
        values = [item[1] for item in sorted_types]

        # 颜色映射(统一注册表, 与全报告同口径)
        colors = [criteria.type_color(label) for label in labels]

        plt.figure(figsize=(10, 6))
        bars = plt.bar(labels, values, color=colors)

        plt.title('GNSS Position Type Distribution', fontsize=14, fontweight='bold')
        plt.xlabel('Position Type', fontsize=12)
        plt.ylabel('Count', fontsize=12)
        plt.xticks(rotation=45, ha='right')

        for bar, val in zip(bars, values):
            height = bar.get_height()
            percentage = (val / total * 100) if total > 0 else 0
            plt.text(bar.get_x() + bar.get_width()/2., height,
                    f'{val}\n({percentage:.1f}%)',
                    ha='center', va='bottom', fontsize=9)

        plt.tight_layout()
        output_file = os.path.join(self.output_dir, 'gnss_solution_type_distribution.png')
        plt.savefig(output_file, dpi=150)
        plt.close()
        print(f"保存GNSS解类型分布图: {output_file}")
    
    def _plot_satellite_time_series(self):
        """绘制卫星数量时间序列(跟踪/可用两层)

        两层口径(M7 手册定义):
          - 跟踪 Tracked : <BESTP/#BESTPA #SVs     "跟踪到的卫星数"(p83/p86)
          - 可用 Used    : <BESTP/#BESTPA #solnSVs "参与解算的卫星数"(p84)
        用户要求(2026-09-17): 去掉"可见"层, 只保留跟踪与参与解算。
        主源 <BESTP(10Hz), 回退 #BESTPA; 再回退 GGA(字段8=使用中的卫星数, 属可用口径)。
        """
        best = self.parse_bestp_abbrev() or self.parse_bestpa_ascii()
        if not best:
            gpgga = self.parse_gpgga()
            if not gpgga:
                return
            t0 = min(d['timestamp'] for d in gpgga)
            x = [d['timestamp'] - t0 for d in gpgga]
            used = [d.get('num_satellites', 0) for d in gpgga]
            plt.figure(figsize=(12, 6))
            plt.plot(x, used, label='Used (GGA #sats)', color='#d62728',
                     linewidth=1.1)
            plt.title('Satellite Count Time Series (Used)')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Number of Satellites')
            plt.legend(loc='best', fontsize=9)
            plt.grid(True)
            plt.tight_layout()
            output_file = os.path.join(self.output_dir, 'satellite_time_series.png')
            plt.savefig(output_file)
            plt.close()
            print(f"保存卫星数量时间序列图(GGA回退, 仅可用): {output_file}")
            return

        t0 = min(d['timestamp'] for d in best)
        x = [d['timestamp'] - t0 for d in best]
        tracked = [d.get('num_satellites', 0) for d in best]
        used = [d.get('num_soln_satellites', 0) for d in best]

        plt.figure(figsize=(12, 6))
        plt.plot(x, tracked, label='Tracked (#SVs)', color='#1f77b4',
                 linewidth=1.1)
        plt.plot(x, used, label='Used (#solnSVs)', color='#d62728',
                 linewidth=1.1)
        plt.title('Satellite Count Time Series (Tracked / Used)')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Number of Satellites')
        plt.legend(loc='best', fontsize=9)
        plt.grid(True)
        plt.tight_layout()

        output_file = os.path.join(self.output_dir, 'satellite_time_series.png')
        plt.savefig(output_file)
        plt.close()
        print(f"保存卫星数量时间序列图(跟踪/可用两层): {output_file}")
    
    def _plot_dop_time_series(self):
        """绘制DOP时间序列 (分字段混合, 数据源自适应)

        常规情形: PDOP/GDOP/HDOP/TDOP/HTDOP 来自 #BESTDOPSA(带时间戳),
                  VDOP 来自 $GNGSA(组合条, 无时间戳, 用与BESTDOPSA同频的序号轴对齐)。
        回退: 三件套全部来自 $GNGSA(序号轴), 或仅 $GNGGA 的 HDOP。
        """
        dop = self.analysis_results.get('dop', {})
        src = dop.get('dop_source', '')

        plt.figure(figsize=(12, 6))
        plotted = False

        if '#BESTDOPSA' in src:
            # 主源 BESTDOPSA(带时间戳)
            series = self.parse_bestdopsa()
            if not series:
                plt.close(); return
            t0 = min(d['timestamp'] for d in series)
            x = [d['timestamp'] - t0 for d in series]
            for key, color in (('pdop', '#1f77b4'), ('hdop', '#2ca02c'),
                               ('gdop', '#9467bd'), ('tdop', '#8c564b'), ('htdop', '#7f7f7f')):
                ys = [d.get(key) for d in series]
                if any(v and v > 0 for v in ys):
                    plt.plot(x, [v if v else float('nan') for v in ys],
                             label=key.upper(), color=color, linewidth=1)
                    plotted = True
            # VDOP 来自 GNGSA(组合条, 无时间戳): 与BESTDOPSA同频10Hz, 用序号->相对时间对齐
            gsa = self.parse_gngsa_combined()
            if gsa:
                vd = [g['vdop'] for g in gsa if g.get('vdop', 0) > 0]
                if vd:
                    # 同频对齐: 用与x相同的时间跨度按序号均匀映射
                    xv = [x[0] + (x[-1]-x[0]) * i / max(len(vd)-1, 1) for i in range(len(vd))]
                    plt.plot(xv, vd, label='VDOP(GSA)', color='#d62728', linewidth=1)
                    plotted = True
            xlabel = 'Time (seconds)'
        elif '$GNGSA' in src:
            series = self.parse_gngsa_combined()
            if not series:
                plt.close(); return
            x = list(range(len(series)))
            for key, color in (('pdop', '#1f77b4'), ('hdop', '#2ca02c'), ('vdop', '#d62728')):
                ys = [g.get(key) for g in series if g.get(key) is not None and g.get(key) > 0]
                if ys:
                    plt.plot(x[:len(ys)], ys, label=key.upper(), color=color, linewidth=1)
                    plotted = True
            xlabel = 'Epoch index'
        else:  # $GNGGA 仅HDOP
            series = self.parse_gpgga()
            if not series:
                plt.close(); return
            t0 = min(d['timestamp'] for d in series)
            x = [d['timestamp'] - t0 for d in series]
            ys = [d['hdop'] for d in series if d.get('hdop', 0) > 0]
            if ys:
                plt.plot(x[:len(ys)], ys, label='HDOP', color='#2ca02c', linewidth=1)
                plotted = True
            xlabel = 'Time (seconds)'

        if not plotted:
            plt.close()
            return
        plt.title(f'DOP Time Series ({src})')
        plt.xlabel(xlabel)
        plt.ylabel('DOP')
        plt.legend(fontsize=8)
        plt.grid(True)
        plt.tight_layout()
        output_file = os.path.join(self.output_dir, 'dop_time_series.png')
        plt.savefig(output_file)
        plt.close()
        print(f"保存DOP时间序列图: {output_file}")

    def _plot_velocity_time_series(self):
        """绘制速度时间序列 - 三个分量+合成速度(数据源: <RTKV或#BESTVA)"""
        rtkv_data, _vel_src = self.get_velocity_data()
        if rtkv_data:
            # 使用实际的时间戳生成相对时间轴
            timestamps = [d['timestamp'] for d in rtkv_data]
            min_time = min(timestamps)
            relative_times = [t - min_time for t in timestamps]
            
            # 提取三个速度分量
            vel_n_list = []
            vel_e_list = []
            vel_u_list = []
            speed_list = []
            
            for d in rtkv_data:
                if 'velocity_north' in d and d['velocity_north'] is not None:
                    vel_n = d['velocity_north']
                else:
                    vel_n = 0.0

                if 'velocity_east' in d and d['velocity_east'] is not None:
                    vel_e = d['velocity_east']
                else:
                    vel_e = 0.0

                if 'velocity_up' in d and d['velocity_up'] is not None:
                    vel_u = d['velocity_up']
                else:
                    vel_u = 0.0

                speed = math.sqrt(vel_n**2 + vel_e**2 + vel_u**2)

                vel_n_list.append(vel_n)
                vel_e_list.append(vel_e)
                vel_u_list.append(vel_u)
                speed_list.append(speed)
            
            # 创建4个子图：三个分量 + 合成速度
            fig, axes = plt.subplots(4, 1, figsize=(14, 12))
            
            # North Velocity
            axes[0].plot(relative_times, vel_n_list, 'b-', linewidth=1)
            axes[0].set_title('North Velocity Time Series', fontsize=12, fontweight='bold')
            axes[0].set_xlabel('Time (seconds)')
            axes[0].set_ylabel('Velocity (m/s)')
            axes[0].grid(True, alpha=0.3)
            axes[0].axhline(y=0, color='r', linestyle='--', alpha=0.5)
            
            # 计算统计信息
            vel_n_mean = statistics.mean(vel_n_list)
            vel_n_std = statistics.stdev(vel_n_list) if len(vel_n_list) > 1 else 0
            axes[0].text(0.02, 0.95, f'Mean: {vel_n_mean:.3f} m/s\nStd: {vel_n_std:.3f} m/s', 
                        transform=axes[0].transAxes, fontsize=9, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
            # East Velocity
            axes[1].plot(relative_times, vel_e_list, 'g-', linewidth=1)
            axes[1].set_title('East Velocity Time Series', fontsize=12, fontweight='bold')
            axes[1].set_xlabel('Time (seconds)')
            axes[1].set_ylabel('Velocity (m/s)')
            axes[1].grid(True, alpha=0.3)
            axes[1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
            
            vel_e_mean = statistics.mean(vel_e_list)
            vel_e_std = statistics.stdev(vel_e_list) if len(vel_e_list) > 1 else 0
            axes[1].text(0.02, 0.95, f'Mean: {vel_e_mean:.3f} m/s\nStd: {vel_e_std:.3f} m/s', 
                        transform=axes[1].transAxes, fontsize=9, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
            # Up Velocity
            axes[2].plot(relative_times, vel_u_list, 'r-', linewidth=1)
            axes[2].set_title('Up Velocity Time Series', fontsize=12, fontweight='bold')
            axes[2].set_xlabel('Time (seconds)')
            axes[2].set_ylabel('Velocity (m/s)')
            axes[2].grid(True, alpha=0.3)
            axes[2].axhline(y=0, color='b', linestyle='--', alpha=0.5)
            
            vel_u_mean = statistics.mean(vel_u_list)
            vel_u_std = statistics.stdev(vel_u_list) if len(vel_u_list) > 1 else 0
            axes[2].text(0.02, 0.95, f'Mean: {vel_u_mean:.3f} m/s\nStd: {vel_u_std:.3f} m/s', 
                        transform=axes[2].transAxes, fontsize=9, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
            # Combined Speed (方差合成)
            axes[3].plot(relative_times, speed_list, 'purple', linewidth=1)
            axes[3].set_title('Combined Speed Time Series (Vector Magnitude)', fontsize=12, fontweight='bold')
            axes[3].set_xlabel('Time (seconds)')
            axes[3].set_ylabel('Speed (m/s)')
            axes[3].grid(True, alpha=0.3)
            
            speed_mean = statistics.mean(speed_list)
            speed_std = statistics.stdev(speed_list) if len(speed_list) > 1 else 0
            axes[3].text(0.02, 0.95, f'Mean: {speed_mean:.3f} m/s\nStd: {speed_std:.3f} m/s', 
                        transform=axes[3].transAxes, fontsize=9, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
            plt.tight_layout()
            output_file = os.path.join(self.output_dir, 'velocity_time_series.png')
            plt.savefig(output_file, dpi=150)
            plt.close()
            print(f"保存速度时间序列图: {output_file}")
    
    def _latlon_to_enu(self, lat, lon, height, ref_lat, ref_lon, ref_height):
        """将经纬度转换为ENU坐标"""
        # 地球参数
        a = 6378137.0  # WGS84长半轴
        f = 1/298.257223563  # WGS84扁率
        e2 = 2*f - f*f  # 第一偏心率的平方
        
        # 计算参考点的参数
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)
        
        # 计算参考点的卯酉曲率半径
        N_ref = a / math.sqrt(1 - e2 * math.sin(ref_lat_rad)**2)
        
        # 参考点的笛卡尔坐标
        x_ref = (N_ref + ref_height) * math.cos(ref_lat_rad) * math.cos(ref_lon_rad)
        y_ref = (N_ref + ref_height) * math.cos(ref_lat_rad) * math.sin(ref_lon_rad)
        z_ref = (N_ref * (1 - e2) + ref_height) * math.sin(ref_lat_rad)
        
        # 将当前点转换为笛卡尔坐标
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        # 计算当前点的卯酉曲率半径
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
        
        # 笛卡尔坐标（地心坐标系）
        x = (N + height) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + height) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e2) + height) * math.sin(lat_rad)
        
        # 计算ENU坐标
        dx = x - x_ref
        dy = y - y_ref
        dz = z - z_ref
        
        sin_lat_ref = math.sin(ref_lat_rad)
        cos_lat_ref = math.cos(ref_lat_rad)
        sin_lon_ref = math.sin(ref_lon_rad)
        cos_lon_ref = math.cos(ref_lon_rad)
        
        # E (East)
        E = -sin_lon_ref * dx + cos_lon_ref * dy
        # N (North)
        N_enu = -sin_lat_ref * cos_lon_ref * dx - sin_lat_ref * sin_lon_ref * dy + cos_lat_ref * dz
        # U (Up)
        U = cos_lat_ref * cos_lon_ref * dx + cos_lat_ref * sin_lon_ref * dy + sin_lat_ref * dz
        
        return E, N_enu, U
    
    def _plot_gnss_enu_trajectory(self, data):
        """绘制GNSS ENU轨迹图（按解类型着色，按时间顺序连线）"""
        print("绘制GNSS ENU轨迹图...")

        if not data or len(data) < 2:
            print("GNSS数据不足，无法绘制轨迹图")
            return

        # 过滤无有效坐标的记录(无解时坐标为0)
        data = [d for d in data if d.get('valid_coord', True)]
        if not data or len(data) < 2:
            print("GNSS有效坐标数据不足，无法绘制轨迹图")
            return
        print(f"GNSS数据点数量: {len(data)}")

        # 使用第一点作为参考点
        ref_lat = data[0]['latitude']
        ref_lon = data[0]['longitude']
        ref_height = data[0]['height']

        # 转换所有点到ENU，保留时间戳用于排序
        enu_data = []
        for d in data:
            E, N, U = self._latlon_to_enu(
                d['latitude'], d['longitude'], d['height'],
                ref_lat, ref_lon, ref_height
            )
            enu_data.append({
                'timestamp': d['timestamp'],
                'E': E,
                'N': N,
                'U': U,
                'solution_type': d['solution_type']
            })

        # 按时间戳排序
        enu_data.sort(key=lambda x: x['timestamp'])

        # 检查ENU坐标范围
        E_vals = [p['E'] for p in enu_data]
        N_vals = [p['N'] for p in enu_data]
        print(f"E范围: {min(E_vals):.3f} - {max(E_vals):.3f} 米")
        print(f"N范围: {min(N_vals):.3f} - {max(N_vals):.3f} 米")

        # 检测异常点
        anomaly_indices = set()
        if self.mark_anomalies and len(enu_data) > 1:
            print(f"检测异常点（速度阈值: {self.speed_threshold} m/s）...")
            for i in range(1, len(enu_data)):
                p1 = enu_data[i-1]
                p2 = enu_data[i]
                dt = p2['timestamp'] - p1['timestamp']
                if dt <= 0:
                    dt = 0.01
                dE = p2['E'] - p1['E']
                dN = p2['N'] - p1['N']
                dist = math.sqrt(dE*dE + dN*dN)
                speed = dist / dt
                if speed > self.speed_threshold:
                    anomaly_indices.add(i)

            print(f"发现 {len(anomaly_indices)} 个异常点")

        # 解类型颜色(按数据实际出现动态生成, 兼容各产品命名差异: 华测SPPDIFF/北云PSRDIFF)
        present = sorted(set(p['solution_type'] for p in enu_data))
        color_map = {t: criteria.type_color(t) for t in present}

        plt.figure(figsize=(14, 10))

        # 按时间顺序绘制线段，每个线段使用起点颜色
        for i in range(len(enu_data) - 1):
            p1 = enu_data[i]
            p2 = enu_data[i + 1]
            color = criteria.type_color(p1['solution_type'])
            plt.plot([p1['E'], p2['E']], [p1['N'], p2['N']],
                     c=color, linewidth=1.5, alpha=0.8)

        # 绘制正常点（圆点）
        normal_points = [p for i, p in enumerate(enu_data) if i not in anomaly_indices]
        if normal_points:
            for sol_type in present:
                points = [p for p in normal_points if p['solution_type'] == sol_type]
                if points:
                    E_vals = [p['E'] for p in points]
                    N_vals = [p['N'] for p in points]
                    plt.scatter(E_vals, N_vals, c=color_map[sol_type],
                               s=15, alpha=0.8, marker='o',
                               label=criteria.legend_label(sol_type, len(points)))

        # 绘制异常点（X符号）
        if self.mark_anomalies and anomaly_indices:
            anomaly_points = [enu_data[i] for i in anomaly_indices]
            anomaly_E = [p['E'] for p in anomaly_points]
            anomaly_N = [p['N'] for p in anomaly_points]
            plt.scatter(anomaly_E, anomaly_N, c='red', s=100, marker='X',
                       label=f'Anomaly Points ({len(anomaly_points)})', zorder=5)

        plt.xlabel('Easting (m)', fontsize=12)
        plt.ylabel('Northing (m)', fontsize=12)
        plt.title('GNSS Trajectory in ENU Coordinate (Colored by Position Type)', fontsize=14, fontweight='bold')
        plt.legend(loc='best', fontsize=10)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')

        plt.tight_layout()
        output_file = os.path.join(self.output_dir, 'gnss_enu_trajectory.png')
        plt.savefig(output_file, dpi=150)
        plt.close()
        print(f"保存GNSS ENU轨迹图: {output_file}")
    
    def _plot_pos_quality(self):
        """位置sigma收敛 + 伪距残差RMS 四联图 (华测: <BESTP/#BESTPA / GNGST)

        子图(按用户要求, 与北云同版式, 单位统一 m):
          1) 纬度标准差 lat sigma   2) 经度标准差 lon sigma
          3) 高度标准差 hgt sigma   4) 伪距残差RMS (GNGST field3)
        位置sigma横轴 = <BESTP/#BESTPA GPS周秒相对时间; RMS横轴 = GNGST UTC相对时间,
        两轴各自独立标注, 不做跨报文强制对齐。
        """
        res = self.analysis_results.get('pos_quality', {})
        pos_t = res.get('pos_t'); gst_t = res.get('gst_t')
        if not pos_t and not gst_t:
            print("无位置质量数据, 跳过四联图")
            return
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(14, 8))
        panels = [
            (axes[0][0], pos_t, res.get('pos_lat'), 'Latitude sigma (m)', '#1f77b4'),
            (axes[0][1], pos_t, res.get('pos_lon'), 'Longitude sigma (m)', '#2ca02c'),
            (axes[1][0], pos_t, res.get('pos_hgt'), 'Height sigma (m)', '#d62728'),
        ]
        for ax, x, y, title, color in panels:
            if x and y:
                ax.plot(x, y, color=color, linewidth=0.8)
                posvals = [v for v in y if v and v > 0]
                if posvals:
                    med = statistics.median(posvals)
                    ax.axhline(med, color=color, linestyle='--', linewidth=0.8, alpha=0.6,
                               label='median %.4f m' % med)
                    ax.legend(fontsize=8, loc='upper right')
                ax.set_xlabel('Time (s, rel. BESTP epoch0)')
            else:
                ax.text(0.5, 0.5, 'No data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title(title, fontsize=11)
            ax.set_ylabel('sigma (m)')
            ax.grid(True, alpha=0.3)
        ax = axes[1][1]
        if gst_t and res.get('gst_rms'):
            ax.plot(gst_t, res['gst_rms'], color='#9467bd', linewidth=0.8)
            med = statistics.median(res['gst_rms'])
            ax.axhline(med, color='#9467bd', linestyle='--', linewidth=0.8, alpha=0.6,
                       label='median %.3f m' % med)
            ax.legend(fontsize=8, loc='upper right')
            ax.set_xlabel('Time (s, rel. GNGST epoch0)')
        else:
            ax.text(0.5, 0.5, 'No GNGST data', ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Pseudorange residual RMS (m, GNGST field3)', fontsize=11)
        ax.set_ylabel('RMS (m)')
        ax.grid(True, alpha=0.3)
        fig.suptitle('Position Sigma Convergence & Pseudorange Residual RMS '
                     '(<BESTP/#BESTPA / GNGST, unit: m)', fontsize=13, fontweight='bold')
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        output_file = os.path.join(self.output_dir, 'pos_sigma_residual_rms.png')
        fig.savefig(output_file, dpi=150)
        plt.close(fig)
        print("保存位置sigma/伪距残差RMS四联图: %s" % output_file)

    def _plot_solution_status(self, data):
        """绘制解算状态时间序列图(数据源<BESTP/#BESTPA)
        
        Args:
            data: 解算状态数据列表(来自<BESTP/#BESTPA)
        """
        print("绘制解算状态时间序列图...")
        
        if not data or len(data) < 1:
            print("数据不足，无法绘制解算状态图")
            return
        
        # 使用实际的时间戳
        timestamps = [d['timestamp'] for d in data]
        min_time = min(timestamps)
        relative_times = [t - min_time for t in timestamps]
        
        # 解算状态和颜色映射
        sol_status_color_map = {
            'SOL_COMPUTED': '#2ECC71',
            'INSUFFICIENT_OBS': '#E74C3C',
            'NO_CONVERGENCE': '#95A5A6',
            'SINGULARITY': '#F39C12',
            'COV_TRACE': '#E67E22',
            'TEST_DIST': '#3498DB',
            'COLD_START': '#1ABC9C',
            'V_H_LIMIT': '#9B59B6',
            'VARIANCE': '#34495E',
            'RESIDUALS': '#8E44AD',
            'INTEGRITY_WARNING': '#C0392B',
            'PENDING': '#7F8C8D'
        }
        
        # 解算状态描述
        sol_status_desc = {
            'SOL_COMPUTED': '完全解算',
            'INSUFFICIENT_OBS': '观测量不足',
            'NO_CONVERGENCE': '不收敛',
            'SINGULARITY': '参数矩阵异常',
            'COV_TRACE': '协方差超限',
            'TEST_DIST': '测试距离超限',
            'COLD_START': '冷启动',
            'V_H_LIMIT': '高度/速度超限',
            'VARIANCE': '方差超限',
            'RESIDUALS': '残差过大',
            'INTEGRITY_WARNING': '完整性警告',
            'PENDING': '待处理'
        }
        
        # 提取状态序列
        sol_stat_list = [d['sol_stat_str'] for d in data]
        
        # 创建图
        plt.figure(figsize=(16, 8))
        
        # 绘制状态时间序列（使用阶梯图表示状态）
        # 首先找出所有不同状态并分配Y坐标
        unique_states = list(set(sol_stat_list))
        state_to_y = {state: i+1 for i, state in enumerate(unique_states)}
        
        # 向量化绘制(性能优化, 视觉输出与原逐点scatter/逐段plot完全一致):
        # 原实现对每个数据点单独调用 plt.scatter()、对每段相邻线单独 plt.plot(),
        # 14448条数据产生上万次Artist创建, 极其耗时。改为:
        #   (1) 按状态分组后, 每个状态一次批量scatter(组内颜色一致, 与原单点颜色相同);
        #   (2) 折线按状态分组, 用NaN在状态切换处断开, 单次plot画出所有同色线段
        #       (仅相邻同状态相连, 与原逻辑相同)。
        nan = float('nan')
        # 按状态分组收集 (t, y), 保持时间顺序
        groups = {}
        for i, state in enumerate(sol_stat_list):
            g = groups.setdefault(state, {'t': [], 'y': []})
            g['t'].append(relative_times[i])
            g['y'].append(state_to_y[state])
        # 批量散点(每状态一次)
        for state, g in groups.items():
            color = sol_status_color_map.get(state, '#808080')
            plt.scatter(g['t'], g['y'], c=color, s=80, zorder=5)
        # 折线(仅相邻同状态相连, 用NaN断开)
        line_groups = {}
        for i in range(1, len(sol_stat_list)):
            if sol_stat_list[i] == sol_stat_list[i-1]:
                lg = line_groups.setdefault(sol_stat_list[i], {'x': [], 'y': []})
                lg['x'].extend([relative_times[i-1], relative_times[i], nan])
                lg['y'].extend([state_to_y[sol_stat_list[i-1]],
                                state_to_y[sol_stat_list[i]], nan])
        for state, lg in line_groups.items():
            color = sol_status_color_map.get(state, '#808080')
            plt.plot(lg['x'], lg['y'], c=color, linewidth=2, zorder=3)
        
        # 添加图例
        handles = []
        labels = []
        for state in unique_states:
            color = sol_status_color_map.get(state, '#808080')
            handle = plt.scatter([], [], c=color, s=80, zorder=5)
            handles.append(handle)
            desc = sol_status_desc.get(state, state)
            labels.append(f"{state}: {desc}")
        
        plt.legend(handles, labels, loc='best', fontsize=10, bbox_to_anchor=(1, 1))
        
        plt.xlabel('Time (seconds)', fontsize=12)
        plt.yticks([state_to_y[state] for state in unique_states],
                  [sol_status_desc.get(state, state) for state in unique_states],
                  fontsize=10)
        plt.title('BESTP Solution Status Time Series', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, axis='x')
        plt.xlim(relative_times[0]-1, relative_times[-1]+1)
        plt.tight_layout()
        
        output_file = os.path.join(self.output_dir, 'solution_status.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"保存解算状态时间序列图: {output_file}")
    
    def generate_html_report(self):
        """生成HTML报告"""
        print("生成HTML报告...")
        
        html_content = f"""
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <title>GPS/RTK Kinematic定位分析报告 - {self.filename}</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            line-height: 1.6;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }}
        h1, h2, h3 {{
            color: #333;
            margin-top: 30px;
        }}
        h1 {{
            border-bottom: 3px solid #2c3e50;
            padding-bottom: 10px;
        }}
        h2 {{
            border-bottom: 2px solid #3498db;
            padding-bottom: 5px;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }}
        th, td {{
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }}
        th {{
            background-color: #3498db;
            color: white;
            font-weight: bold;
        }}
        tr:nth-child(even) {{
            background-color: #f9f9f9;
        }}
        .badge-feature {{
            display: inline-block;
            background-color: #8e44ad;
            color: #fff;
            font-size: 12px;
            font-weight: bold;
            padding: 2px 10px;
            border-radius: 12px;
            margin-left: 8px;
            vertical-align: middle;
        }}
        .note {{
            background-color: #eef6fb;
            border-left: 4px solid #3498db;
            padding: 12px 16px;
            margin: 15px 0;
            border-radius: 4px;
            font-size: 14px;
            color: #2c3e50;
        }}
        .metric-good {{
            color: green;
            font-weight: bold;
        }}
        .metric-warning {{
            color: orange;
            font-weight: bold;
        }}
        .metric-bad {{
            color: red;
            font-weight: bold;
        }}
        .chart {{
            margin: 20px 0;
            border: 1px solid #ddd;
            border-radius: 5px;
            overflow: hidden;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }}
        .chart img {{
            width: 100%;
            height: auto;
            display: block;
        }}
        .pass {{
            color: green;
            font-weight: bold;
        }}
        .fail {{
            color: red;
            font-weight: bold;
        }}
        .metric-info {{
            color: #17a2b8;
            font-weight: bold;
        }}
        .summary-table {{
            background-color: #f8f9fa;
            border: 2px solid #dee2e6;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>GPS/RTK Kinematic定位分析报告【华测 HUACE · 纯GNSS】</h1>
        <p><strong>分析输入(COM1/ASCII):</strong> {os.path.basename(self.input_file)} |
           <strong>生成时间:</strong> {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} |
           <strong>分析工具:</strong> 华测 HUACE Kinematic Analyzer</p>

        <!-- 数据概览 -->
        <h2>1. 数据概览</h2>
        <table class="summary-table">
            <tr>
                <th>报文类型</th>
                <th>数量</th>
                <th>占比</th>
            </tr>
        """
        
        # 添加报文统计
        total_lines = self.header_info.get('total_lines', 0)
        for header, count in self.header_info.get('header_count', {}).items():
            percentage = (count / total_lines * 100) if total_lines > 0 else 0
            html_content += f"""
            <tr>
                <td>{header}</td>
                <td>{count}</td>
                <td>{percentage:.1f}%</td>
            </tr>
            """
        
        html_content += f"""
        </table>
        <p><strong>数据完整性:</strong> {total_lines} 条有效报文
        (原始 {self.header_info.get('raw_line_count', total_lines)} 行,
        乱码残片 {self.header_info.get('garbage_lines', 0)} 行,
        二进制 {self.header_info.get('binary_lines', 0)} 行)</p>
        """

        # Kinematic检测: 完整性检查与报文周期核对(步骤1-4, 含标色告警)
        mi = self.analysis_results.get('message_integrity')
        if mi:
            html_content += """
        <h2>2. 完整性检查与报文周期核对(COM1配置)</h2>
        <table>
            <tr>
                <th>报文</th>
                <th>实测头</th>
                <th>条数</th>
                <th>设定周期(s)</th>
                <th>实际周期(s)</th>
                <th>实际频率(Hz)</th>
                <th>估算丢帧率</th>
                <th>状态</th>
                <th>备注</th>
            </tr>
        """
            status_style = {'ok': 'pass', 'info': 'metric-warning',
                            'warn': 'fail', 'error': 'fail'}
            status_text = {'ok': '正常', 'info': '提示',
                           'warn': '周期不符', 'error': '缺失'}
            for chk in mi['checks']:
                cls = status_style.get(chk['status'], 'metric-warning')
                actual_p = (f"{chk['actual_period']:.3f}"
                            if chk['actual_period'] is not None else '-')
                actual_r = (f"{chk['actual_rate']:.2f}"
                            if chk['actual_rate'] is not None else '-')
                loss = (f"{chk['loss_rate']:.1f}%"
                        if chk['loss_rate'] is not None else '-')
                matched = ','.join(chk.get('matched', [])) or '-'
                html_content += f"""
            <tr>
                <td>{chk['header']}</td>
                <td>{matched}</td>
                <td>{chk['count']}</td>
                <td>{chk['configured_period']:.3f}</td>
                <td class="{cls}">{actual_p}</td>
                <td class="{cls}">{actual_r}</td>
                <td>{loss}</td>
                <td class="{cls}">{status_text.get(chk['status'], chk['status'])}</td>
                <td>{chk['note']}</td>
            </tr>
                """
            html_content += """
        </table>
        <p style="color:#c0392b;font-weight:bold;">注意: 标红/标黄项表示实际报文周期与设定周期可能不符或存在缺失, 请核对设备输出配置。</p>
        """
        
        # 采样率分析
        if 'sampling_rate' in self.analysis_results:
            sr = self.analysis_results['sampling_rate']
            if 'median_interval' in sr:
                html_content += f"""
        <p><strong>采样率分析:</strong> 配置 {sr['sampling_rate']:.2f} Hz (中位间隔 {sr['median_interval']:.3f} 秒) ／
        含丢帧有效 {sr.get('avg_rate', sr['sampling_rate']):.2f} Hz ／
        丢帧段 {sr.get('gap_count', 0)} 个, 估算丢帧率 {sr.get('loss_rate', 0):.1f}% ／ 数据源: {sr.get('source','')}</p>
                """
            else:
                html_content += f"""
        <p><strong>采样率分析:</strong> {sr['sampling_rate']:.2f} Hz (平均间隔: {sr['average_interval']:.3f} 秒)</p>
                """

        # RTK链路核对
        rtk = self.analysis_results.get('rtk_link')
        if rtk:
            html_content += f"""
        <p><strong>RTK链路:</strong> 基站ID {rtk['primary_station']} (核对{rtk['check_points']}次) ／
        差分龄期 平均{rtk['avg_diff_age']:.2f}s, 最大{rtk['max_diff_age']:.2f}s</p>
            """
        
        # 解类型分析（GNSS和INS）
        if 'gnss_solution_type' in self.analysis_results:
            st = self.analysis_results['gnss_solution_type']
            html_content += f"""
        <h2>3. GNSS解类型分析</h2>
        <div class="chart">
            <img src="gnss_solution_type_distribution.png" alt="GNSS解类型分布"/>
        </div>
        <table>
            <tr>
                <th>解类型</th>
                <th>数量</th>
                <th>占比</th>
            </tr>
            """
            
            for sol_type, count in sorted(st['type_count'].items(), key=lambda x: x[1], reverse=True):
                percentage = (count / st['total_epochs'] * 100) if st['total_epochs'] > 0 else 0
                html_content += f"""
            <tr>
                <td>{sol_type}</td>
                <td>{count}</td>
                <td>{percentage:.1f}%</td>
            </tr>
                """

            html_content += f"""
        </table>
        <p><strong>GNSS固定解比率:</strong> <span class="{'pass' if st['fixed_ratio'] > 95 else 'metric-warning' if st['fixed_ratio'] > 80 else 'fail'}">{st['fixed_ratio']:.1f}%</span></p>
            """

        # 卫星数量分析(跟踪/可用两层, 手册定义)
        if 'satellite_visibility' in self.analysis_results:
            sv = self.analysis_results['satellite_visibility']
            rows = ''
            defs = (
                ('tracked', '跟踪卫星数', '#SVs"跟踪到的卫星数"(M7手册 BESTP 字段15/p83)'),
                ('used', '可用卫星数', '#solnSVs"参与解算的卫星数"(M7手册 BESTP 字段16/p84)'),
            )
            for key, cn, desc in defs:
                if f'average_{key}' not in sv:
                    continue
                rows += (f'            <tr>\n'
                         f'                <td>{cn}<br/><small>{desc}</small></td>\n'
                         f'                <td>平均 {sv[f"average_{key}"]:.1f} / '
                         f'最小 {sv[f"min_{key}"]} / 最大 {sv[f"max_{key}"]}</td>\n'
                         f'            </tr>\n')
            html_content += f"""
        <h2>4. 卫星数量分析</h2>
        <div class="chart">
            <img src="satellite_time_series.png" alt="卫星数量时间序列"/>
        </div>
        <table>
            <tr>
                <th>指标(口径依据M7手册)</th>
                <th>数值(平均/最小/最大)</th>
            </tr>
{rows}        </table>
        <p><small>两层关系: 跟踪 &ge; 可用。跟踪为接收机已锁定跟踪的卫星, '
        '可用为实际参与位置解算的卫星。</small></p>
            """
        
        # DOP分析 (PDOP/HDOP/VDOP 三件套, 数据源自适应)
        if 'dop' in self.analysis_results:
            dop = self.analysis_results['dop']
            src = dop.get('dop_source', '')
            def _row(cn, key):
                a = dop.get(f'average_{key}')
                if a is None:
                    return ''
                mn = dop.get(f'min_{key}'); mx = dop.get(f'max_{key}')
                if key == 'hdop':
                    ev = ('优秀' if a < 1 else '良好' if a < 2 else '一般')
                    cls = ('' if a < 1 else 'metric-warning' if a < 2 else 'metric-bad')
                    evtd = f'<td class="{cls}">{ev}</td>'
                else:
                    evtd = '<td>-</td>'
                return (f'<tr><td>平均{cn}</td><td>{a:.2f}</td>{evtd}</tr>'
                        f'<tr><td>最小{cn}</td><td>{mn:.2f}</td><td></td></tr>'
                        f'<tr><td>最大{cn}</td><td>{mx:.2f}</td><td></td></tr>')
            note = dop.get('note', '')
            # 主三件套 + BESTDOPSA独有的 GDOP/TDOP/HTDOP(若有)
            rows = (_row('PDOP', 'pdop') + _row('HDOP', 'hdop') + _row('VDOP', 'vdop')
                    + _row('GDOP', 'gdop') + _row('TDOP', 'tdop') + _row('HTDOP', 'htdop'))
            note_html = f'<p style="font-size:12px;color:#888;">{note}</p>' if note else ''
            html_content += f"""
        <h2>5. DOP分析</h2>
        <p><strong>数据源:</strong> {src} ｜
           <span style="color:#555;">DOP为接收机本地解算的几何精度因子(非卫星下发);
           多星座时按NMEA共识取"组合"DOP(参与解算的全部卫星几何), 星座越多DOP越小。</span></p>
        {note_html}
        <div class="chart">
            <img src="dop_time_series.png" alt="DOP时间序列"/>
        </div>
        <table>
            <tr>
                <th>指标</th>
                <th>数值</th>
                <th>评估</th>
            </tr>
            {rows}
        </table>
            """
        
        # 速度分析
        if 'velocity' in self.analysis_results:
            vel = self.analysis_results['velocity']
        # 位置sigma收敛与伪距残差RMS(<BESTP/#BESTPA/GNGST)
        pq = self.analysis_results.get('pos_quality', {})
        if pq.get('pos_epochs') or pq.get('gst_epochs'):
            def _row3(label, key):
                a = pq.get(key + '_mean')
                if a is None:
                    return ''
                return ('<tr><td>%s</td><td>%.4f</td><td>%.4f</td><td>%.4f</td><td>%.4f</td><td>m</td></tr>'
                        % (label, a, pq.get(key + '_med'), pq.get(key + '_min'), pq.get(key + '_max')))
            rms_row = ''
            if pq.get('rms_mean') is not None:
                rms_row = ('<tr><td>伪距残差RMS ($GNGST field3)</td><td>%.4f</td><td>%.4f</td><td>%.4f</td><td>%.4f</td><td>m</td></tr>'
                           % (pq['rms_mean'], pq['rms_med'], pq['rms_min'], pq['rms_max']))
            html_content += f"""
        <h2>6. 位置标准差σ收敛与伪距残差RMS</h2>
        <p><strong>数据源:</strong> 位置σ = &lt;BESTP/#BESTPA 数据字段[8]/[9]/[10] (纬度/经度/高度标准差, M7手册 表3-38 字段10/11/12);
           伪距残差RMS = $GNGST 字段3 "伪距、DGNSS修正值的标准偏差的均方根" (M7手册 3.1.5 表3-9)。单位统一 m。</p>
        <p style="color:#555;font-size:13px;">σ为接收机自估精度(协方差传播), 反映解算收敛质量而非外业真值误差;
           仅统计 SOL_COMPUTED 且坐标有效历元({pq.get('pos_epochs', 0)}历元)。
           伪距残差RMS越小越好, 其抬升通常指示遮挡/多径/电离层扰动。</p>
        <div class="chart">
            <img src="pos_sigma_residual_rms.png" alt="位置sigma收敛与伪距残差RMS"/>
        </div>
        <table>
            <tr><th>指标</th><th>均值</th><th>中位数</th><th>最小</th><th>最大</th><th>单位</th></tr>
            {_row3('纬度标准差 σ_lat', 'lat')}
            {_row3('经度标准差 σ_lon', 'lon')}
            {_row3('高度标准差 σ_hgt', 'hgt')}
            {rms_row}
        </table>
            """

            html_content += f"""
        <h2>7. 速度分析</h2>
        <div class="chart">
            <img src="velocity_time_series.png" alt="速度时间序列"/>
        </div>
        <table>
            <tr>
                <th>指标</th>
                <th>数值</th>
                <th>单位</th>
            </tr>
            <tr>
                <td>平均速度</td>
                <td>{vel['average_speed']:.2f}</td>
                <td>m/s</td>
            </tr>
            <tr>
                <td>最大速度</td>
                <td>{vel['max_speed']:.2f}</td>
                <td>m/s</td>
            </tr>
        </table>
            """

        # 载噪比 C/N0 分析(华测特色: RANGEA原始观测值)
        cn0 = self.analysis_results.get('cn0', {})
        if cn0.get('available'):
            html_content += f"""
        <h2>8. 载噪比 C/N0 分析<span class="badge-feature">特色(RANGEA)</span></h2>
        <div class="chart">
            <img src="cn0_analysis.png" alt="C/N0载噪比分析"/>
        </div>
        <table>
            <tr><th>指标</th><th>数值</th><th>说明</th></tr>
            <tr><td>数据来源</td><td>RANGEA</td><td>原始观测值(伪距/载波/多普勒/载噪比), 华测独有采集, 逐卫星逐频点</td></tr>
            <tr><td>统计历元数</td><td>{cn0['n_epochs']}</td><td>参与统计的观测历元</td></tr>
            <tr><td>观测值总数</td><td>{cn0['n_obs_total']}</td><td>全部卫星/频点的C/N0样本数</td></tr>
            <tr><td>平均C/N0</td><td>{cn0['mean_cn0']:.1f} dB-Hz</td><td>越高信号越好</td></tr>
            <tr><td>中位数C/N0</td><td>{cn0['median_cn0']:.1f} dB-Hz</td><td>抗离群值, 反映典型水平</td></tr>
            <tr><td>范围</td><td>{cn0['min_cn0']:.0f} ~ {cn0['max_cn0']:.0f} dB-Hz</td><td>最低/最高观测值</td></tr>
            <tr><td>低C/N0(&lt;35)占比</td><td>{cn0['low_ratio']:.1f}%</td><td>占比越高遮挡/多路径越明显</td></tr>
        </table>
        <div class="note">
            <b>解读指导(行业共识, 见 Kaplan《Understanding GPS》):</b><br/>
            C/N0(载噪比)衡量卫星信号质量, 单位 dB-Hz, 越高越好。一般判据:
            <b>&ge;45</b> 信号强(开阔天空); <b>35~45</b> 中等(正常可用);
            <b>&lt;35</b> 偏弱, 易受遮挡/多路径影响; 接收机跟踪门限约 <b>25~28</b>, 低于此值易失锁。<br/>
            <b>怎么看图:</b> 上图为全部观测值的C/N0分布直方图, 颜色按上述阈值区分(绿=强/黄=中/红=弱), 绿色占比越高越好;
            下图为历元平均C/N0时间序列, 若某时段明显下坠, 说明该时段存在遮挡(楼宇/树荫/隧道)或强多路径。
            分布直方图左偏(低值尾巴长)或时间序列出现深坑, 都提示观测环境欠佳。
        </div>
            """

        # 位置分析(编号自适应: 若无C/N0特色章则前移)
        _pos_no = 9 if self.analysis_results.get('cn0', {}).get('available') else 8
        _conc_no = _pos_no + 1
        bestgnss_data = self.parse_gnss_pos()
        if bestgnss_data:
            html_content += f"""
        <h2>{_pos_no}. 位置分析</h2>
        <div class="chart">
            <img src="position_time_series_gnss.png" alt="位置时间序列"/>
        </div>
        <p>位置时间序列显示了接收机在Kinematic环境下的位置变化。从图中可以观察到位置的连续性和稳定性。</p>
        
        <h3>{_pos_no}.1 GNSS轨迹分析（ENU坐标）</h3>
        <div class="chart">
            <img src="gnss_enu_trajectory.png" alt="GNSS ENU轨迹"/>
        </div>
        <p>GNSS轨迹图使用ENU（East-North-Up）坐标系显示，按数据实际出现的解类型动态着色（本设备GNSS解类型见上分布图；不同产品命名可能有差异，如伪距差分北云称PSRDIFF、华测称SPPDIFF，含义相同）：绿色=RTK固定解(NARROW_INT)，蓝色=浮点解(NARROW_FLOAT)，红色=单点(SINGLE)，橙色=伪距差分(PSRDIFF/SPPDIFF)，紫色=PPP，灰色=无解(NONE)。图例中标注各类型数量及中文名。</p>
        
            """
        if bestgnss_data:
            html_content += f"""
        <h3>{_pos_no}.2 BESTP解算状态时间序列</h3>
        <div class="chart">
            <img src="solution_status.png" alt="解算状态时间序列"/>
        </div>
        <p>该图展示了BESTP报文中解算状态（sol stat）的时间序列变化，帮助分析定位过程中解算状态的稳定性。</p>
            """

        # 结论
        html_content += f"""
        <h2>{_conc_no}. 结论与建议</h2>
        <div class="summary-table">
            <table>
                <tr>
                    <th>评估指标</th>
                    <th>状态</th>
                    <th>说明</th>
                </tr>
            """
        
        # 添加评估结果
        evaluations = []
        
        if 'gnss_solution_type' in self.analysis_results:
            fixed_ratio = self.analysis_results['gnss_solution_type']['fixed_ratio']
            status = 'pass' if fixed_ratio > 95 else 'metric-warning' if fixed_ratio > 80 else 'fail'
            evaluations.append({
                'metric': 'GNSS固定解比率',
                'status': status,
                'description': f'{fixed_ratio:.1f}%，{"满足高精度定位要求" if fixed_ratio > 95 else "基本满足要求" if fixed_ratio > 80 else "需要改善"}'
            })
        
        if 'satellite_visibility' in self.analysis_results:
            sv_eval = self.analysis_results['satellite_visibility']
            avg_tr = sv_eval.get('average_tracked')
            avg_us = sv_eval.get('average_used')
            if avg_tr is not None:
                status = 'pass' if avg_tr >= 10 else 'metric-warning' if avg_tr >= 6 else 'fail'
                desc = f'平均跟踪{avg_tr:.1f}颗'
                if avg_us is not None:
                    desc += f'，平均参与解算{avg_us:.1f}颗'
                desc += '，' + ('良好' if avg_tr >= 10 else '基本满足' if avg_tr >= 6 else '需要改善')
                evaluations.append({
                    'metric': '卫星数(跟踪/参与解算)',
                    'status': status,
                    'description': desc
                })
        
        if 'dop' in self.analysis_results:
            avg_hdop = self.analysis_results['dop']['average_hdop']
            status = 'pass' if avg_hdop < 1 else 'metric-warning' if avg_hdop < 2 else 'fail'
            evaluations.append({
                'metric': 'DOP值',
                'status': status,
                'description': f'平均HDOP {avg_hdop:.2f}，{"优秀" if avg_hdop < 1 else "良好" if avg_hdop < 2 else "一般"}'
            })
        
        for eval_item in evaluations:
            html_content += f"""
                <tr>
                    <td>{eval_item['metric']}</td>
                    <td class="{eval_item['status']}">{eval_item['status'].replace('metric-', '')}</td>
                    <td>{eval_item['description']}</td>
                </tr>
            """
        
        html_content += f"""
            </table>
        </div>
        """
        
        # 特色指标(设备专有字段, 带注释)
        special = self.analysis_results.get('special_metrics', [])
        if special:
            rows = ''
            for m in special:
                cls = {'pass': 'pass', 'warn': 'metric-warning', 'fail': 'fail', 'info': 'metric-info'}.get(m['status'], '')
                rows += ('<tr><td>%s</td><td>%s</td><td class="%s">%s</td>'
                         '<td>%s</td><td style="font-size:12px;color:#666">%s</td></tr>'
                         % (m['name'], m['value'], cls, m['status_label'], m['note'], m['basis']))
            html_content += f"""
        <h3>特色指标(设备专有字段)</h3>
        <p style="font-size:13px;color:#555">以下为该设备特有的字段/能力评估, 每项附"测什么·怎么算好·怎么算坏"注释及依据。</p>
        <div class="summary-table">
            <table>
                <tr><th>特色指标</th><th>实测值</th><th>状态</th><th>评级</th><th>注释(判断动态的什么/好坏)</th><th>依据</th></tr>
                {rows}
            </table>
        </div>
        """
        
        html_content += f"""
        <h3>建议</h3>
        <ul>
            <li>保持良好的卫星信号接收环境，确保天线无遮挡</li>
            <li>定期检查RTK基准站连接状态</li>
            <li>对于Kinematic应用，建议使用组合导航系统提高定位稳定性</li>
            <li>根据应用场景调整采样率，平衡精度和数据量</li>
        </ul>
    </div>
</body>
</html>
        """
        
        output_file = os.path.join(self.output_dir, 'report.html')
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(html_content)
        
        print(f"生成HTML报告: {output_file}")
    
    def generate_md_report(self):
        """生成Markdown报告"""
        print("生成Markdown报告...")
        
        md_content = f"""
# GPS/RTK Kinematic定位分析报告【华测 HUACE · 纯GNSS】

## 基本信息
- **分析输入(COM1/ASCII)**: {os.path.basename(self.input_file)}
- **生成时间**: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- **分析工具**: 华测 HUACE Kinematic Analyzer

## 1. 数据概览

| 报文类型 | 数量 | 占比 |
|---------|------|------|
        """
        
        # 添加报文统计
        total_lines = self.header_info.get('total_lines', 0)
        for header, count in self.header_info.get('header_count', {}).items():
            percentage = (count / total_lines * 100) if total_lines > 0 else 0
            md_content += f"| {header} | {count} | {percentage:.1f}% |\n"
        
        md_content += f"""

**数据完整性**: {total_lines} 条有效报文(原始 {self.header_info.get('raw_line_count', total_lines)} 行, 乱码残片 {self.header_info.get('garbage_lines', 0)} 行)

        """

        # Kinematic检测: 完整性检查与报文周期核对(标色告警)
        mi = self.analysis_results.get('message_integrity')
        if mi:
            md_content += """
## 2. 完整性检查与报文周期核对(COM1配置)

| 报文 | 实测头 | 条数 | 设定周期(s) | 实际周期(s) | 实际频率(Hz) | 丢帧率 | 状态 | 备注 |
|------|--------|------|------------|------------|-------------|--------|------|------|
"""
            status_icon = {'ok': '🟢正常', 'info': '🟡提示',
                           'warn': '🟠**周期不符**', 'error': '🔴**缺失**'}
            for chk in mi['checks']:
                actual_p = (f"{chk['actual_period']:.3f}"
                            if chk['actual_period'] is not None else '-')
                actual_r = (f"{chk['actual_rate']:.2f}"
                            if chk['actual_rate'] is not None else '-')
                loss = (f"{chk['loss_rate']:.1f}%"
                        if chk['loss_rate'] is not None else '-')
                matched = ','.join(chk.get('matched', [])) or '-'
                note = chk['note'] or '-'
                md_content += (f"| {chk['header']} | {matched} | {chk['count']} | "
                               f"{chk['configured_period']:.3f} | {actual_p} | "
                               f"{actual_r} | {loss} | "
                               f"{status_icon.get(chk['status'], chk['status'])} | "
                               f"{note} |\n")
            md_content += """
> ⚠️ 加粗标色项表示实际报文周期与设定周期可能不符或报文缺失, 请核对设备输出配置。
"""

        # 采样率分析
        if 'sampling_rate' in self.analysis_results:
            sr = self.analysis_results['sampling_rate']
            if 'median_interval' in sr:
                md_content += f"""
**采样率分析**: 配置 {sr['sampling_rate']:.2f} Hz (中位间隔 {sr['median_interval']:.3f} 秒) ／ 含丢帧有效 {sr.get('avg_rate', sr['sampling_rate']):.2f} Hz ／ 丢帧段 {sr.get('gap_count', 0)} 个, 估算丢帧率 {sr.get('loss_rate', 0):.1f}% ／ 数据源: {sr.get('source','')}

        """
            else:
                md_content += f"""
**采样率分析**: {sr['sampling_rate']:.2f} Hz (平均间隔: {sr['average_interval']:.3f} 秒)

        """

        # RTK链路核对
        rtk = self.analysis_results.get('rtk_link')
        if rtk:
            md_content += f"""
**RTK链路**: 基站ID {rtk['primary_station']} (核对{rtk['check_points']}次) ／ 差分龄期 平均{rtk['avg_diff_age']:.2f}s, 最大{rtk['max_diff_age']:.2f}s

        """
        
        # 解类型分析（GNSS和INS）
        if 'gnss_solution_type' in self.analysis_results:
            st = self.analysis_results['gnss_solution_type']
            md_content += f"""
## 3. GNSS解类型分析

![GNSS解类型分布](gnss_solution_type_distribution.png)

| 解类型 | 数量 | 占比 |
|--------|------|------|
"""
            for sol_type, count in sorted(st['type_count'].items(), key=lambda x: x[1], reverse=True):
                percentage = (count / st['total_epochs'] * 100) if st['total_epochs'] > 0 else 0
                md_content += f"| {sol_type} | {count} | {percentage:.1f}% |\n"

            md_content += f"""

**GNSS固定解比率**: {st['fixed_ratio']:.1f}%

"""

        # 卫星数量分析(跟踪/可用两层, 手册定义)
        if 'satellite_visibility' in self.analysis_results:
            sv = self.analysis_results['satellite_visibility']
            mrows = ''
            defs = (
                ('tracked', '跟踪卫星数', '#SVs跟踪到的卫星数(M7手册BESTP字段15/p83)'),
                ('used', '可用卫星数', '#solnSVs参与解算的卫星数(M7手册BESTP字段16/p84)'),
            )
            for key, cn, desc in defs:
                if f'average_{key}' not in sv:
                    continue
                mrows += (f'| {cn} | {sv[f"average_{key}"]:.1f} | {sv[f"min_{key}"]} | '
                          f'{sv[f"max_{key}"]} | {desc} |\n')
            md_content += f"""
## 4. 卫星数量分析

![卫星数量时间序列](satellite_time_series.png)

| 指标 | 平均 | 最小 | 最大 | 口径依据(M7手册) |
|------|------|------|------|----------------------|
{mrows}
> 两层关系: 跟踪 >= 可用。

        """
        
        # DOP分析 (PDOP/HDOP/VDOP 三件套, 数据源自适应)
        if 'dop' in self.analysis_results:
            dop = self.analysis_results['dop']
            src = dop.get('dop_source', '')
            def _mrow(cn, key):
                a = dop.get(f'average_{key}')
                if a is None:
                    return ''
                mn = dop.get(f'min_{key}'); mx = dop.get(f'max_{key}')
                ev = ''
                if key == 'hdop':
                    ev = ('优秀' if a < 1 else '良好' if a < 2 else '一般')
                return (f'| 平均{cn} | {a:.2f} | {ev} |\n'
                        f'| 最小{cn} | {mn:.2f} | |\n'
                        f'| 最大{cn} | {mx:.2f} | |\n')
            note = dop.get('note', '')
            mrows = (_mrow('PDOP', 'pdop') + _mrow('HDOP', 'hdop') + _mrow('VDOP', 'vdop')
                     + _mrow('GDOP', 'gdop') + _mrow('TDOP', 'tdop') + _mrow('HTDOP', 'htdop'))
            note_md = ('\n> ' + note + '\n') if note else ''
            md_content += f"""
## 5. DOP分析

**数据源**: {src}（DOP为接收机本地解算的几何精度因子，非卫星下发；多星座取组合DOP，星座越多DOP越小）
{note_md}
![DOP时间序列](dop_time_series.png)

| 指标 | 数值 | 评估 |
|------|------|------|
{mrows}
        """
        
        # 速度分析
        if 'velocity' in self.analysis_results:
            vel = self.analysis_results['velocity']
        # 位置sigma收敛与伪距残差RMS(<BESTP/#BESTPA/GNGST)
        pq = self.analysis_results.get('pos_quality', {})
        if pq.get('pos_epochs') or pq.get('gst_epochs'):
            def _mrow(label, key):
                a = pq.get(key + '_mean')
                if a is None:
                    return ''
                return ('| %s | %.4f | %.4f | %.4f | %.4f | m |\n'
                        % (label, a, pq.get(key + '_med'), pq.get(key + '_min'), pq.get(key + '_max')))
            rms_line = ''
            if pq.get('rms_mean') is not None:
                rms_line = ('| 伪距残差RMS ($GNGST field3) | %.4f | %.4f | %.4f | %.4f | m |\n'
                            % (pq['rms_mean'], pq['rms_med'], pq['rms_min'], pq['rms_max']))
            md_content += f"""
## 6. 位置标准差σ收敛与伪距残差RMS

**数据源**: 位置σ = &lt;BESTP/#BESTPA 数据字段[8]/[9]/[10] (纬度/经度/高度标准差, M7手册 表3-38 字段10/11/12);
伪距残差RMS = $GNGST 字段3 "伪距、DGNSS修正值的标准偏差的均方根" (M7手册 3.1.5 表3-9)。单位统一 m。

![位置sigma收敛与伪距残差RMS](pos_sigma_residual_rms.png)

σ为接收机自估精度(协方差传播), 反映解算收敛质量而非外业真值误差; 仅统计 SOL_COMPUTED 且坐标有效历元({pq.get('pos_epochs', 0)}历元)。
伪距残差RMS越小越好, 其抬升通常指示遮挡/多径/电离层扰动。

| 指标 | 均值 | 中位数 | 最小 | 最大 | 单位 |
|------|------|--------|------|------|------|
{_mrow('纬度标准差 σ_lat', 'lat')}{_mrow('经度标准差 σ_lon', 'lon')}{_mrow('高度标准差 σ_hgt', 'hgt')}{rms_line}
"""

            md_content += f"""
## 7. 速度分析

![速度时间序列](velocity_time_series.png)

| 指标 | 数值 | 单位 |
|------|------|------|
| 平均速度 | {vel['average_speed']:.2f} | m/s |
| 最大速度 | {vel['max_speed']:.2f} | m/s |

        """
        
        # 载噪比 C/N0 分析(华测特色: RANGEA)
        cn0 = self.analysis_results.get('cn0', {})
        if cn0.get('available'):
            md_content += f"""
## 8. 载噪比 C/N0 分析（特色：RANGEA）

![C/N0载噪比分析](cn0_analysis.png)

| 指标 | 数值 | 说明 |
|------|------|------|
| 数据来源 | RANGEA | 原始观测值(伪距/载波/多普勒/载噪比), 华测独有采集 |
| 统计历元数 | {cn0['n_epochs']} | 参与统计的观测历元 |
| 观测值总数 | {cn0['n_obs_total']} | 全部卫星/频点的C/N0样本数 |
| 平均C/N0 | {cn0['mean_cn0']:.1f} dB-Hz | 越高信号越好 |
| 中位数C/N0 | {cn0['median_cn0']:.1f} dB-Hz | 抗离群值, 反映典型水平 |
| 范围 | {cn0['min_cn0']:.0f} ~ {cn0['max_cn0']:.0f} dB-Hz | 最低/最高观测值 |
| 低C/N0(<35)占比 | {cn0['low_ratio']:.1f}% | 占比越高遮挡/多路径越明显 |

**解读指导(行业共识, 见 Kaplan《Understanding GPS》)**: C/N0(载噪比)衡量卫星信号质量, 单位 dB-Hz, 越高越好。一般判据: **≥45** 信号强(开阔天空); **35~45** 中等(正常可用); **<35** 偏弱, 易受遮挡/多路径影响; 接收机跟踪门限约 **25~28**, 低于此值易失锁。

**怎么看图**: 上图分布直方图按阈值着色(绿=强/黄=中/红=弱), 绿色占比越高越好; 下图历元平均C/N0时间序列若某时段明显下坠, 说明该时段存在遮挡(楼宇/树荫/隧道)或强多路径。分布直方图左偏(低值尾巴长)或时间序列出现深坑, 都提示观测环境欠佳。

        """

        # 位置分析(编号自适应: 若无C/N0特色章则前移)
        _pos_no = 9 if self.analysis_results.get('cn0', {}).get('available') else 8
        _conc_no = _pos_no + 1
        bestgnss_data = self.parse_gnss_pos()
        if bestgnss_data:
            md_content += f"""
## {_pos_no}. 位置分析

![位置时间序列](position_time_series_gnss.png)

位置时间序列显示了接收机在Kinematic环境下的位置变化。从图中可以观察到位置的连续性和稳定性。

### {_pos_no}.1 GNSS轨迹分析（ENU坐标）

![GNSS ENU轨迹](gnss_enu_trajectory.png)

GNSS轨迹图使用ENU（East-North-Up）坐标系显示，按数据实际出现的解类型动态着色（本设备GNSS解类型见上分布图；不同产品命名可能有差异，如伪距差分北云称PSRDIFF、华测称SPPDIFF，含义相同）：绿色=RTK固定解(NARROW_INT)，蓝色=浮点解(NARROW_FLOAT)，红色=单点(SINGLE)，橙色=伪距差分(PSRDIFF/SPPDIFF)，紫色=PPP，灰色=无解(NONE)。图例中标注各类型数量及中文名。

"""
        if bestgnss_data:
            md_content += f"""
### {_pos_no}.2 BESTP解算状态时间序列

![解算状态时间序列](solution_status.png)

该图展示了BESTP报文中解算状态（sol stat）的时间序列变化，帮助分析定位过程中解算状态的稳定性。
"""

        # 结论
        md_content += f"""
## {_conc_no}. 结论与建议

### 评估结果

| 评估指标 | 状态 | 说明 |
|---------|------|------|
        """
        
        # 添加评估结果
        evaluations = []
        
        if 'gnss_solution_type' in self.analysis_results:
            fixed_ratio = self.analysis_results['gnss_solution_type']['fixed_ratio']
            status = '通过' if fixed_ratio > 95 else '警告' if fixed_ratio > 80 else '失败'
            evaluations.append({
                'metric': 'GNSS固定解比率',
                'status': status,
                'description': f'{fixed_ratio:.1f}%，{"满足高精度定位要求" if fixed_ratio > 95 else "基本满足要求" if fixed_ratio > 80 else "需要改善"}'
            })
        
        if 'satellite_visibility' in self.analysis_results:
            sv_eval = self.analysis_results['satellite_visibility']
            avg_tr = sv_eval.get('average_tracked')
            avg_us = sv_eval.get('average_used')
            if avg_tr is not None:
                status = '通过' if avg_tr >= 10 else '警告' if avg_tr >= 6 else '失败'
                desc = f'平均跟踪{avg_tr:.1f}颗'
                if avg_us is not None:
                    desc += f'，平均参与解算{avg_us:.1f}颗'
                desc += '，' + ('良好' if avg_tr >= 10 else '基本满足' if avg_tr >= 6 else '需要改善')
                evaluations.append({
                    'metric': '卫星数(跟踪/参与解算)',
                    'status': status,
                    'description': desc
                })
        
        if 'dop' in self.analysis_results:
            avg_hdop = self.analysis_results['dop']['average_hdop']
            status = '通过' if avg_hdop < 1 else '警告' if avg_hdop < 2 else '失败'
            evaluations.append({
                'metric': 'DOP值',
                'status': status,
                'description': f'平均HDOP {avg_hdop:.2f}，{"优秀" if avg_hdop < 1 else "良好" if avg_hdop < 2 else "一般"}'
            })
        
        for eval_item in evaluations:
            md_content += f"| {eval_item['metric']} | {eval_item['status']} | {eval_item['description']} |\n"
        
        # 特色指标(设备专有字段, 带注释)
        special = self.analysis_results.get('special_metrics', [])
        if special:
            md_content += "\n### 特色指标(设备专有字段)\n\n"
            md_content += "以下为该设备特有的字段/能力评估, 每项附注释及依据。\n\n"
            md_content += "| 特色指标 | 实测值 | 评级 | 注释(判断动态的什么/好坏) | 依据 |\n"
            md_content += "|---------|--------|------|--------------------------|------|\n"
            for m in special:
                md_content += ("| %s | %s | %s | %s | %s |\n"
                               % (m['name'], m['value'], m['status_label'], m['note'], m['basis']))
        
        md_content += f"""

### 建议

1. 保持良好的卫星信号接收环境，确保天线无遮挡
2. 定期检查RTK基准站连接状态
3. 对于Kinematic应用，建议使用组合导航系统提高定位稳定性
4. 根据应用场景调整采样率，平衡精度和数据量
        """
        
        output_file = os.path.join(self.output_dir, 'report.md')
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(md_content)
        
        print(f"生成Markdown报告: {output_file}")
    
    def run_analysis(self):
        """运行完整分析"""
        print(f"\n=== 开始分析文件: {self.input_file} ===")
        
        # 创建输出目录
        self.create_output_dir()
        
        # 解析文件
        if not self.parse_dat_file():
            return False
        
        # 保存分类文件
        self.save_header_files()

        # 步骤1-3: 完整性检查与报文周期核对(COM1配置, 实际周期界面显示)
        self.check_message_integrity()

        # 分析采样率
        self.analyze_sampling_rate()
        
        # 分析定位精度
        self.analyze_position_accuracy()
        
        # 分析解类型
        self.analyze_solution_type()
        
        # 分析卫星数量(跟踪/可用)
        self.analyze_satellite_visibility()
        
        # 分析DOP
        self.analyze_dop()

        # 位置sigma收敛与伪距残差RMS(<BESTP/#BESTPA/GNGST)
        self.analyze_pos_quality()
        
        # 分析速度
        self.analyze_velocity()

        # RTK链路核对(基站ID/差分龄期)
        self.analyze_rtk_link()

        # 纯GNSS特色指标(ENVSTATUS环境感知)
        self.analyze_special_metrics()

        # 载噪比C/N0分析(RANGEA原始观测值)
        self.analyze_cn0()

        # 生成图表
        self.generate_plots()
        
        # 生成报告
        self.generate_html_report()
        self.generate_md_report()
        
        print(f"\n=== 分析完成! 结果保存在: {self.output_dir} ===")
        return True

def run_with_gui():
    """带GUI界面的主函数"""
    import tkinter as tk
    from tkinter import filedialog, messagebox

    def select_file():
        file_path = filedialog.askopenfilename(
            title="选择华测 COM1 数据文件(ASCII文本)",
            filetypes=[("华测COM1数据", "*.dat;*.log"), ("所有文件", "*")]
        )
        if file_path:
            entry.delete(0, tk.END)
            entry.insert(0, file_path)

    def start_analysis():
        file_path = entry.get().strip()
        if not file_path:
            messagebox.showerror("错误", "请选择输入文件")
            return

        if not os.path.exists(file_path):
            messagebox.showerror("错误", f"文件不存在: {file_path}")
            return

        mark_anomalies = mark_anomalies_var.get()
        speed_threshold_str = speed_threshold_entry.get().strip()

        try:
            speed_threshold = float(speed_threshold_str) if speed_threshold_str else 2.0
            if speed_threshold <= 0:
                messagebox.showerror("错误", "速度阈值必须大于0")
                return
        except ValueError:
            messagebox.showerror("错误", "速度阈值必须是有效的数字")
            return

        try:
            analyzer = GPSKinematicAnalyzer(file_path,
                                        mark_anomalies=mark_anomalies,
                                        speed_threshold=speed_threshold)
            result = analyzer.run_analysis()
            if result:
                # 分析完成后自动打开HTML报告和结果目录(Windows)
                report_html = os.path.join(analyzer.output_dir, 'report.html')
                if os.path.isfile(report_html):
                    os.startfile(report_html)  # Windows默认应用打开HTML(通常为浏览器)
                else:
                    messagebox.showwarning("提示", "HTML报告未找到，将打开结果目录")
                if os.path.isdir(analyzer.output_dir):
                    os.startfile(analyzer.output_dir)  # 打开结果目录
                messagebox.showinfo(
                    "分析完成",
                    "结果目录已打开：\n" + analyzer.output_dir +
                    "\n\nHTML报告：\n" + report_html
                )
            else:
                messagebox.showerror("错误", "分析过程出现错误")
        except Exception as e:
            messagebox.showerror("错误", f"分析失败: {str(e)}")

    # 创建主窗口
    root = tk.Tk()
    root.title("华测 HUACE 纯GNSS Kinematic分析工具 (COM1)")
    root.geometry("620x600")
    root.resizable(True, True)

    # 设置背景色
    root.configure(bg="#f0f0f0")

    # 创建标题
    title_label = tk.Label(root, text="华测 HUACE 纯GNSS Kinematic分析工具",
                          font=("微软雅黑", 15, "bold"), bg="#f0f0f0", fg="#b03a2e")
    title_label.pack(pady=(18, 4))

    # 醒目设备横幅: 明确本程序适用的设备, 避免与北云程序混淆
    device_banner = tk.Label(root,
                             text="【适用设备：华测 HUACE · 纯GNSS 移动站(无惯导)】",
                             font=("微软雅黑", 11, "bold"),
                             bg="#b03a2e", fg="white", padx=10, pady=6)
    device_banner.pack(pady=(0, 12), fill=tk.X, padx=20)

    # 创建文件选择区域
    frame = tk.Frame(root, bg="#f0f0f0")
    frame.pack(pady=10, padx=20, fill=tk.X)

    label = tk.Label(frame, text="输入文件 - COM1 (ASCII文本, 必选; 支持.dat/.log):",
                     font=("微软雅黑", 10, "bold"), bg="#f0f0f0", fg="#b03a2e")
    label.pack(anchor=tk.W, pady=(0, 5))

    entry_frame = tk.Frame(frame, bg="#f0f0f0")
    entry_frame.pack(fill=tk.X)

    entry = tk.Entry(entry_frame, width=50, font=("微软雅黑", 10))
    entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))

    browse_button = tk.Button(entry_frame, text="浏览", command=select_file,
                             font=("微软雅黑", 10), bg="#4CAF50", fg="white",
                             padx=10, pady=5)
    browse_button.pack(side=tk.RIGHT)

    # 创建异常点标记选项区域
    anomaly_frame = tk.Frame(root, bg="#f0f0f0")
    anomaly_frame.pack(pady=10, padx=20, fill=tk.X)

    # 勾选框
    mark_anomalies_var = tk.BooleanVar(value=False)
    mark_anomalies_check = tk.Checkbutton(anomaly_frame,
                                         text="在轨迹图中标记异常跳跃点",
                                         variable=mark_anomalies_var,
                                         font=("微软雅黑", 10),
                                         bg="#f0f0f0",
                                         command=lambda: speed_threshold_entry.configure(state='normal' if mark_anomalies_var.get() else 'disabled'))
    mark_anomalies_check.pack(anchor=tk.W, pady=(0, 5))

    # 速度阈值输入
    threshold_frame = tk.Frame(anomaly_frame, bg="#f0f0f0")
    threshold_frame.pack(anchor=tk.W, fill=tk.X)

    threshold_label = tk.Label(threshold_frame, text="速度阈值 (m/s):",
                             font=("微软雅黑", 10), bg="#f0f0f0")
    threshold_label.pack(side=tk.LEFT, padx=(20, 10))

    speed_threshold_entry = tk.Entry(threshold_frame, width=10,
                                    font=("微软雅黑", 10), state='disabled')
    speed_threshold_entry.pack(side=tk.LEFT)
    speed_threshold_entry.insert(0, "2.0")

    threshold_hint = tk.Label(threshold_frame, text="(超过此速度阈值的点将被标记为异常)",
                             font=("微软雅黑", 8), bg="#f0f0f0", fg="gray")
    threshold_hint.pack(side=tk.LEFT, padx=(10, 0))

    # 创建分析按钮
    analyze_button = tk.Button(root, text="开始分析", command=start_analysis,
                              font=("微软雅黑", 12, "bold"), bg="#2196F3", fg="white",
                              padx=20, pady=10)
    analyze_button.pack(pady=20)

    # 创建状态显示区域
    status_frame = tk.Frame(root, bg="#f0f0f0")
    status_frame.pack(pady=10, padx=20, fill=tk.X)

    status_label = tk.Label(status_frame, text="就绪", font=("微软雅黑", 10),
                           bg="#f0f0f0", fg="#4CAF50")
    status_label.pack(anchor=tk.W)

    # 创建说明文本
    info_text = """
【适用产品】华测 HUACE (纯GNSS 移动站, 无惯导)

输入文件说明：
- COM1 (必选): ASCII 文本文件(.dat/.log)
  包含 #BESTPA(GNSS定位) #ENVSTATUSA(华测特色状态)
  #RANGEA(载噪比C/N0, 华测特色) 及 $NMEA 报文
  本程序的解析、评估、绘图、报告全部基于 COM1

操作步骤：
1. 浏览选择 COM1 文件
2. 可勾选"标记异常跳跃点"并设置速度阈值
3. 点击"开始分析"，结果保存在与输入文件同名的目录中

特色字段：#ENVSTATUSA / #RANGEA 为华测独有，报告中标"特色"。
用户准则：华测为纯GNSS，定位以 <BESTP(缩写10Hz)/#BESTPA 为准(无BESTGNSSPOSA)；
带时间戳的以时间戳为准。

异常点说明：
- 相邻两点速度超过阈值时标记为异常，轨迹图用红色X显示
    """

    info_frame = tk.Frame(root, bg="#f0f0f0")
    info_frame.pack(pady=10, padx=20, fill=tk.BOTH, expand=True)

    info_label = tk.Label(info_frame, text=info_text, font=("微软雅黑", 9),
                         bg="#f0f0f0", justify=tk.LEFT)
    info_label.pack(fill=tk.BOTH, expand=True)

    root.mainloop()

def main():
    """主函数"""
    import sys

    parser = argparse.ArgumentParser(description='华测M720纯GNSS/RTK Kinematic定位数据综合分析程序')
    parser.add_argument('input_file', nargs='?', help='输入的dat文件路径')
    parser.add_argument('--mark-anomalies', action='store_true',
                       help='在轨迹图中标记异常跳跃点（基于速度阈值）')
    parser.add_argument('--speed-threshold', type=float, default=2.0,
                       help='速度阈值（m/s），超过此阈值认为异常跳跃，默认2.0')

    args = parser.parse_args()

    if args.input_file:
        # 命令行模式
        if not os.path.exists(args.input_file):
            print(f"错误: 文件不存在 - {args.input_file}")
            return

        analyzer = GPSKinematicAnalyzer(args.input_file,
                                     mark_anomalies=args.mark_anomalies,
                                     speed_threshold=args.speed_threshold)
        analyzer.run_analysis()
    else:
        # GUI模式
        run_with_gui()

if __name__ == '__main__':
    main()
