#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高精度KinematicGPS定位数据综合分析与处理程序【北云 M21 版本】

功能说明：
    本程序用于解析和分析GPS RTK Kinematic定位数据，支持北云科技 M21 模组的多种报文格式（北云专用版本）。
    主要功能包括：
    1. 解析BESTGNSSPOSA、INSPVAXA等北云自定义格式报文
    2. 解析GPGGA等NMEA标准报文
    3. 完整性检查: GPIMU报文按用户要求直接放弃处理, 所有报文以实测数据为准
       (带时间戳的报文以时间戳为准, 无时间戳的按报文插入规律推断时间规律)
    4. 进行多维度定位质量分析（精度、解类型、卫星跟踪/参与解算数量、DOP值等）
    5. 生成丰富的可视化图表（轨迹图、时间序列、分布图等）
    6. 自动生成HTML和Markdown格式的 detailed 分析报告
    
符合规范：
    - UG016 北云科技数据通信接口协议
    - RTCM 10403.x 差分GNSS服务标准
    - NMEA 0183 标准
    
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
    # ICOM3 口输出配置(设定周期), 来源: by_manual/icom3 setup.jpg 截屏
    # 用户准则(2026-09-16):
    #   - 北云 GNSS 只用 BESTGNSSPOSA, 不用 BESTPOSA;
    #     惯导只用 INSPVAXA
    #   - GPIMU 报文直接放弃, 不做任何处理
    #   - 所有报文以实测数据为准: 带时间戳的以时间戳为准;
    #     无时间戳的按报文插入规律推断时间规律
    #   - BESTPOSA 配置为 1.0s, 但 com3.dat 实测为 0.2s
    #     -> 不作为异常告警, 仅作信息提示(实际周期更密, 且不再使用)
    # ========================================================================
    ICOM3_CONFIG = [
        # (规范报文头, 数据键名, 设定周期s, 是否必须出现)
        ('#BESTPOSA',      'BESTPOSA',      1.0,  True),
        ('#INSPVAXA',      'INSPVAXA',      0.1,  True),
        ('$GPGGA',         'GPGGA',         1.0,  True),
        ('#BESTGNSSPOSA',  'BESTGNSSPOSA',  0.2,  True),
        ('#HEADINGA',      'HEADINGA',      0.2,  True),
        ('$GPIMU',         'GPIMU',         0.01, False),  # ONNEW 0.01s, 放弃处理
        ('$GPGSV',         'GPGSV',         0.1,  True),
        ('$GPGSA',         'GPGSA',         0.1,  True),
        ('$GPGST',         'GPGST',         0.1,  True),
        ('#TRACKSTATA',    'TRACKSTATA',    0.2,  True),
    ]
    # 实际周期更密但不算异常的豁免(报文头 -> 说明)
    RELAXED_PERIOD_NOTES = {
        '#BESTPOSA': '设定1.0s, 实测通常0.2s(输出更密); 该报文已不再使用, 仅信息提示',
        '$GPGGA': '设定1.0s, 实测通常0.2s(输出更密); 实际输出比设定更频繁, 不影响使用, 仅信息提示',
    }
    # ICOM4 关注项(setup图): RANGECMPB 二进制报文; 如需分析须另配二进制解码,
    # 当前按"静态相关内容关注项"记录, 不参与ASCII周期检查
    ICOM4_NOTES = [
        'RANGECMPB ONTIME 0.2s (二进制, 原始观测值压缩格式; 当前程序未实现二进制解码)',
    ]

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
        # 保存输入文件路径(北云COM3, ASCII文本主文件)
        # 本程序仅分析COM3; C/N0已从COM3的TRACKSTATA获取, 无需COM4
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
        gpimu_lines = 0  # 放弃处理的GPIMU报文计数
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

            # GPIMU报文: 用户要求直接放弃, 不做任何处理(仅计数, 不入库)
            if line.startswith('$GPIMU'):
                gpimu_lines += 1
                continue

            # 提取报文头标识符
            if line.startswith('#'):
                # NovAtel专有格式：报文头是第一个逗号前的部分
                header = line.split(',')[0]
            elif line.startswith('$'):
                # NMEA 0183标准格式：报文头是第一个逗号前的部分
                header = line.split(',')[0]
            else:
                # 既不是NovAtel格式也不是NMEA格式(乱码/残片行)
                garbage_lines += 1
                continue

            # 将该行添加到对应报文头的列表中
            self.parsed_data[header].append(line)

        # 如果有二进制数据行，打印警告信息
        if binary_lines > 0:
            print(f"警告: 跳过了 {binary_lines} 行二进制数据")
        
        # 统计解析结果信息
        self.header_info = {
            # 原始文件总行数(完整性检查基准)
            'raw_line_count': raw_line_count,
            # 有效报文总行数
            'total_lines': sum(len(lines) for lines in self.parsed_data.values()),
            # 统计每种报文类型的数量（报文头 -> 行数）
            'header_count': {header: len(lines) for header, lines in self.parsed_data.items()},
            # 二进制行 / 乱码残片行(串口丢字节等)
            'binary_lines': binary_lines,
            'garbage_lines': garbage_lines,
            # 放弃处理的GPIMU报文条数(用户要求不处理, 仅计数)
            'gpimu_dropped': gpimu_lines,
        }
        
        # 打印解析完成信息和统计结果
        if gpimu_lines > 0:
            print(f"按用户要求放弃处理 GPIMU 报文 {gpimu_lines} 条(仅计数, 不参与解析)")
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
    #   1. 先对整个报文做完整性检查
    #   2. 提取各报文实际周期, 与 ICOM3 设定周期核对
    #   3. 实际周期在界面/报告中明确显示
    #   4. 周期不符的报文标色告警(报告中标红/标黄)
    # ========================================================================

    def _extract_header_timestamp(self, header, lines):
        """按报文头提取 GPS 周内秒时间戳列表(时间戳优先, 数据为准)

        北云标准 ASCII 报文头字段[6] = GPS 周内秒(UG016 2.1.2.2);
        NMEA $GPGGA 字段[1] = UTC hhmmss.sss, 换算为当天秒。
        GPIMU 按用户要求放弃处理。
        """
        if header == '$GPIMU':
            return []
        timestamps = []
        if header.startswith('#'):
            for line in lines:
                try:
                    parts = line.split(',', 8)
                    if len(parts) >= 7:
                        timestamps.append(float(parts[6]))
                except (ValueError, IndexError):
                    continue
        elif header == '$GPGGA':
            for line in lines:
                try:
                    parts = line.split(',')
                    time_str = parts[1]
                    hour = int(time_str[:2])
                    minute = int(time_str[2:4])
                    second = float(time_str[4:])
                    timestamps.append(hour * 3600 + minute * 60 + second)
                except (ValueError, IndexError):
                    continue
        return timestamps

    def check_message_integrity(self):
        """步骤1+2+3: 完整性检查, 并提取各配置报文的实际周期与设定核对

        完整性口径(以实测数据为准):
        - 有效报文行 / 乱码残片行 / 二进制行 计数与占比
        - 实际周期: 优先用报文自带时间戳的中位间隔(抗丢帧);
          带时间戳报文以时间戳为最准
        判定规则:
        - 报文缺失 -> error(标红)
        - 实测周期与设定偏差>5%(且不在豁免清单) -> warn(标黄)
        - 估算丢帧率>1% -> warn(标黄)
        - 其余 -> ok(绿色)
        """
        print("完整性检查与报文周期核对(ICOM3 配置)...")
        results = []
        for header, name, configured, required in self.ICOM3_CONFIG:
            lines = self.parsed_data.get(header, [])
            count = len(lines)
            if header == '$GPIMU':
                # 用户要求放弃处理: 显示计数但不判定周期
                dropped = self.header_info.get('gpimu_dropped', 0)
                results.append({
                    'header': header, 'name': name,
                    'configured_period': configured, 'count': dropped,
                    'required': False, 'status': 'info',
                    'actual_period': None, 'actual_rate': None,
                    'loss_rate': None, 'time_span': None,
                    'note': '按用户要求放弃处理, 仅计数显示',
                })
                print(f"  {header}: {dropped}条, 按用户要求放弃处理, 仅计数显示")
                continue
            entry = {
                'header': header, 'name': name,
                'configured_period': configured, 'count': count,
                'required': required, 'status': 'ok', 'note': '',
                'actual_period': None, 'actual_rate': None,
                'loss_rate': None, 'time_span': None,
            }
            if count == 0:
                entry['status'] = 'error' if required else 'info'
                entry['note'] = ('报文缺失: 数据流中未找到, 与ICOM3配置不符'
                                 if required else '未输出(ONNEW事件触发或无数据)')
                results.append(entry)
                continue

            timestamps = self._extract_header_timestamp(header, lines)
            timestamps = sorted(t for t in timestamps)
            if len(timestamps) >= 2:
                intervals = [timestamps[i+1] - timestamps[i]
                             for i in range(len(timestamps)-1)
                             if timestamps[i+1] - timestamps[i] > 1e-6]
                if intervals:
                    med = statistics.median(intervals)
                    span = timestamps[-1] - timestamps[0]
                    expected = span / med + 1 if med > 0 else 0
                    loss = max(0.0, (1 - len(timestamps) / expected) * 100) if expected > 0 else 0
                    entry['actual_period'] = med
                    entry['actual_rate'] = 1.0 / med if med > 0 else None
                    entry['time_span'] = span
                    entry['loss_rate'] = loss
            else:
                # 无可用时间戳: 按报文插入规律推断(数据是事实, 以数据为准)
                entry['note'] = '无可用时间戳, 按报文插入规律推断(以实测数据为准)'

            # 状态判定
            notes = []
            if header in self.RELAXED_PERIOD_NOTES:
                entry['status'] = 'info'
                notes.append(self.RELAXED_PERIOD_NOTES[header])
            elif entry['actual_period'] is not None and configured > 0:
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
            if notes:
                entry['note'] = ('; '.join(notes) if not entry['note']
                                 else entry['note'] + '; ' + '; '.join(notes))
            results.append(entry)

            # 控制台输出(实际周期明确显示)
            if entry['actual_period'] is not None:
                print(f"  {header}: 实际周期 {entry['actual_period']:.3f}s "
                      f"({entry['actual_rate']:.2f}Hz), 设定 {configured:.3f}s, "
                      f"{count}条, 状态 {entry['status']}"
                      + (f" - {entry['note']}" if entry['note'] else ''))
            else:
                print(f"  {header}: {count}条, 设定 {configured:.3f}s, "
                      f"状态 {entry['status']}"
                      + (f" - {entry['note']}" if entry['note'] else ''))

        self.analysis_results['message_integrity'] = {
            'raw_line_count': self.header_info.get('raw_line_count', 0),
            'total_lines': self.header_info.get('total_lines', 0),
            'binary_lines': self.header_info.get('binary_lines', 0),
            'garbage_lines': self.header_info.get('garbage_lines', 0),
            'gpimu_dropped': self.header_info.get('gpimu_dropped', 0),
            'checks': results,
            'icom4_notes': self.ICOM4_NOTES,
        }
        return results

    def analyze_sampling_rate(self):
        """分析采样率"""
        print("分析采样率...")
        
        # 从GPGGA报文分析时间
        gga_lines = self.parsed_data.get('$GPGGA', [])
        if not gga_lines:
            print("未找到GPGGA报文，无法分析采样率")
            return
        
        # 解析GPGGA时间
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
            print("GPGGA报文数量不足，无法计算采样率")
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
            
            print(f"GPGGA平均采样间隔: {avg_interval:.3f}秒")
            print(f"GPGGA采样率: {sampling_rate:.2f}Hz")
    
    def parse_bestgnss_posa(self):
        """解析BESTGNSSPOSA报文

        根据UG016规范第4.2.2节,BESTGNSSPOSA数据部分格式:
        [0] Sol Status - 解算状态 (SOL_COMPUTED等)
        [1] Pos Type - 位置类型 (NARROW_INT等)
        [2] Lat - 纬度
        [3] Lon - 经度
        [4] Hgt - 海拔高
        [5] Undulation - 高程异常值
        [6] Datum ID - 坐标系ID
        [7] Lat σ - 纬度标准差
        [8] Lon σ - 经度标准差
        [9] Hgt σ - 高度标准差
        [10] Stn ID - 差分站台ID
        [11] Diff_age - 差分龄期
        [12] Sol_age - 解算时间
        [13] #SVs - 跟踪卫星数
        [14] #solnSVs - 参与解算卫星数
        ...
        """
        if hasattr(self, '_cache_bestgnssposa'):
            return self._cache_bestgnssposa

        lines = self.parsed_data.get('#BESTGNSSPOSA', [])
        data = []

        for line in lines:
            if ';' not in line:
                continue
            header_str, data_str = line.split(';', 1)
            parts = data_str.split(',')
            if len(parts) < 10:
                continue

            try:
                header_parts = header_str.split(',')
                if len(header_parts) < 7:
                    continue
                timestamp = float(header_parts[6])

                sol_status = parts[0].strip()
                pos_type = parts[1].strip()
                lat = float(parts[2])
                lon = float(parts[3])
                height = float(parts[4])

                # 无解(NONE)记录坐标为0, 不丢弃(解类型统计需保留NONE),
                # 仅标记有效坐标, 绘图时再过滤
                valid_coord = not (lat == 0 or lon == 0)

                lat_sigma = float(parts[7]) if len(parts) > 7 and parts[7] else 0
                lon_sigma = float(parts[8]) if len(parts) > 8 and parts[8] else 0
                hgt_sigma = float(parts[9]) if len(parts) > 9 and parts[9] else 0

                # UG016 4.2.2: [10]=Stn ID基站ID [11]=Diff_age差分龄期
                # [12]=Sol_age [13]=#SVs跟踪卫星数 [14]=#solnSVs解算卫星数
                station_id = parts[10].strip().strip('"') if len(parts) > 10 else ''
                try:
                    diff_age = float(parts[11]) if len(parts) > 11 and parts[11] else 0.0
                except ValueError:
                    diff_age = 0.0
                try:
                    num_svs = int(parts[13]) if len(parts) > 13 and parts[13] else 0
                except ValueError:
                    num_svs = 0
                try:
                    num_soln_svs = int(parts[14]) if len(parts) > 14 and parts[14] else 0
                except ValueError:
                    num_soln_svs = 0

                horizontal_sigma = math.sqrt(lat_sigma**2 + lon_sigma**2)
                vertical_sigma = hgt_sigma

                data.append({
                    'timestamp': timestamp,
                    'latitude': lat,
                    'longitude': lon,
                    'height': height,
                    'valid_coord': valid_coord,
                    'sol_status': sol_status,
                    'pos_type': pos_type,
                    'solution_type': pos_type,
                    'lat_sigma': lat_sigma,
                    'lon_sigma': lon_sigma,
                    'hgt_sigma': hgt_sigma,
                    'horizontal_sigma': horizontal_sigma,
                    'vertical_sigma': vertical_sigma,
                    'station_id': station_id,
                    'diff_age': diff_age,
                    'num_satellites': num_svs,
                    'num_soln_satellites': num_soln_svs
                })
            except Exception as e:
                print(f"解析BESTGNSSPOSA失败: {e}, 行: {line[:100]}")
                continue

        self._cache_bestgnssposa = data
        return data
    
    def parse_bestposa(self):
        """解析BESTPOSA报文，提取解算状态（sol stat）"""
        if hasattr(self, '_cache_bestposa'):
            return self._cache_bestposa

        lines = self.parsed_data.get('#BESTPOSA', [])
        data = []

        # 表4-1解算状态描述说明
        sol_status_map = {
            'SOL_COMPUTED': 0,
            'INSUFFICIENT_OBS': 1,
            'NO_CONVERGENCE': 2,
            'SINGULARITY': 3,
            'COV_TRACE': 4,
            'TEST_DIST': 5,
            'COLD_START': 6,
            'V_H_LIMIT': 7,
            'VARIANCE': 8,
            'RESIDUALS': 9,
            'INTEGRITY_WARNING': 13,
            'PENDING': 18,
            'INVALID_FIX': 19,
            'UNAUTHORIZED': 20,
            'INVALID_RATE': 22
        }

        for line in lines:
            try:
                if ';' not in line:
                    continue
                header_str, data_str = line.split(';', 1)
                header_parts = header_str.split(',')
                if len(header_parts) < 7:
                    continue
                timestamp = float(header_parts[6])

                data_part = data_str.split(',')
                if len(data_part) < 3:
                    continue

                sol_stat_str = data_part[0].strip()
                pos_type_str = data_part[1].strip() if len(data_part) > 1 else ''

                sol_stat = sol_status_map.get(sol_stat_str, -1)

                data.append({
                    'timestamp': timestamp,
                    'sol_stat_str': sol_stat_str,
                    'sol_stat': sol_stat,
                    'pos_type': pos_type_str
                })
            except Exception as e:
                continue

        self._cache_bestposa = data
        return data
    
    def parse_inspvaxa(self):
        """解析INSPVAXA报文

        根据UG016规范第4.2.15节,INSPVAXA数据部分格式:
        [0] INS Status - INS解算状态 (INS_ALIGNMENT_COMPLETE等)
        [1] Pos Type - 位置类型 (INS_RTKFIXED等)
        [2] Lat - 纬度
        [3] Lon - 经度
        [4] Hgt - 海拔高
        [5] Undulation - 高程异常值
        [6] North Velocity - 北向速度
        [7] East Velocity - 东向速度
        [8] Up Velocity - 天向速度
        [9] Roll - 横滚角
        [10] Pitch - 俯仰角
        [11] Azimuth - 航向角
        [12] Lat σ - 纬度标准差
        [13] Lon σ - 经度标准差
        [14] Hgt σ - 高度标准差
        [15] North Vel σ - 北向速度标准差
        [16] East Vel σ - 东向速度标准差
        [17] Up Vel σ - 天向速度标准差
        [18] Roll σ - 横滚角标准差
        [19] Pitch σ - 俯仰角标准差
        [20] Azimuth σ - 航向角标准差
        ...
        """
        if hasattr(self, '_cache_inspvaxa'):
            return self._cache_inspvaxa

        lines = self.parsed_data.get('#INSPVAXA', [])
        data = []

        for line in lines:
            if ';' not in line:
                continue
            header_str, data_str = line.split(';', 1)
            parts = data_str.split(',')
            if len(parts) < 12:
                continue

            try:
                header_parts = header_str.split(',')
                if len(header_parts) < 7:
                    continue
                timestamp = float(header_parts[6])

                ins_status = parts[0].strip()
                pos_type = parts[1].strip()

                lat = float(parts[2])
                lon = float(parts[3])
                height = float(parts[4])

                # 无解(NONE)记录坐标为0, 保留供解类型/状态统计, 绘图时再过滤
                valid_coord = not (lat == 0 or lon == 0)

                vel_n = float(parts[6]) if len(parts) > 6 and parts[6] else 0
                vel_e = float(parts[7]) if len(parts) > 7 and parts[7] else 0
                vel_u = float(parts[8]) if len(parts) > 8 and parts[8] else 0

                roll = float(parts[9]) if len(parts) > 9 and parts[9] else 0
                pitch = float(parts[10]) if len(parts) > 10 and parts[10] else 0
                azimuth = float(parts[11]) if len(parts) > 11 and parts[11] else 0

                lat_sigma = float(parts[12]) if len(parts) > 12 and parts[12] else 0
                lon_sigma = float(parts[13]) if len(parts) > 13 and parts[13] else 0
                hgt_sigma = float(parts[14]) if len(parts) > 14 and parts[14] else 0

                roll_sigma = float(parts[18]) if len(parts) > 18 and parts[18] else 0
                pitch_sigma = float(parts[19]) if len(parts) > 19 and parts[19] else 0
                azimuth_sigma = float(parts[20]) if len(parts) > 20 and parts[20] else 0

                data.append({
                    'timestamp': timestamp,
                    'valid_coord': valid_coord,
                    'latitude': lat,
                    'longitude': lon,
                    'height': height,
                    'ins_status': ins_status,
                    'pos_type': pos_type,
                    'velocity_north': vel_n,
                    'velocity_east': vel_e,
                    'velocity_up': vel_u,
                    'roll': roll,
                    'pitch': pitch,
                    'azimuth': azimuth,
                    'lat_sigma': lat_sigma,
                    'lon_sigma': lon_sigma,
                    'hgt_sigma': hgt_sigma,
                    'roll_sigma': roll_sigma,
                    'pitch_sigma': pitch_sigma,
                    'azimuth_sigma': azimuth_sigma
                })
            except Exception as e:
                print(f"解析INSPVAXA失败: {e}, 行: {line[:100]}")
                continue

        self._cache_inspvaxa = data
        return data
    
    def parse_gpgga(self):
        """解析GPGGA报文"""
        if hasattr(self, '_cache_gpgga'):
            return self._cache_gpgga

        lines = self.parsed_data.get('$GPGGA', [])
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

    def analyze_position_accuracy(self):
        """分析位置精度 - Kinematic数据无真值，不生成误导性指标

        Kinematic场景无参考真值, 恒0的"水平定位精度0.0000米-通过"是错误结论,
        故不写入 analysis_results, 报告中该项自动省略。
        """
        print("Kinematic数据无真值，跳过位置精度评估(不生成误导性指标)")
    
    def analyze_solution_type(self):
        """分析解类型分布

        根据UG016规范:
        - Sol Status (解算状态): 表示解算是否成功 (SOL_COMPUTED等)
        - Pos Type (位置类型): 表示定位模式和精度等级 (NARROW_INT等)

        行业实践中,Pos Type是评估定位质量的关键指标
        """
        print("分析解类型分布...")

        # 分析GNSS位置类型
        bestgnss_data = self.parse_bestgnss_posa()
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
        inspvax_data = self.parse_inspvaxa()
        ins_type_count = {}
        ins_status_count = {}

        if inspvax_data:
            for d in inspvax_data:
                pos_type = d.get('pos_type', 'NONE')
                ins_type_count[pos_type] = ins_type_count.get(pos_type, 0) + 1

                ins_status = d.get('ins_status', 'UNKNOWN')
                ins_status_count[ins_status] = ins_status_count.get(ins_status, 0) + 1

        total_ins = len(inspvax_data) if inspvax_data else 0
        ins_fixed_count = ins_type_count.get('INS_RTKFIXED', 0)
        ins_fixed_ratio = (ins_fixed_count / total_ins * 100) if total_ins > 0 else 0

        self.analysis_results['ins_solution_type'] = {
            'type_count': ins_type_count,
            'ins_status_count': ins_status_count,
            'fixed_ratio': ins_fixed_ratio,
            'total_epochs': total_ins
        }

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

        print(f"\n=== INS定位质量分析 ===")
        print(f"总历元数: {total_ins}")
        print(f"固定解(INS_RTKFIXED): {ins_fixed_count} ({ins_fixed_ratio:.1f}%)")
        print(f"\nINS位置类型分布:")
        for pos_type, count in ins_type_count.items():
            pct = (count/total_ins*100) if total_ins > 0 else 0
            print(f"  {pos_type}: {count} ({pct:.1f}%)")

        print(f"\nINS解算状态分布:")
        for ins_status, count in ins_status_count.items():
            pct = (count/total_ins*100) if total_ins > 0 else 0
            print(f"  {ins_status}: {count} ({pct:.1f}%)")
    
    def analyze_satellite_visibility(self):
        """分析卫星数量(跟踪/可用两层, UG016 手册定义)

        - 跟踪卫星数 Tracked : BESTGNSSPOSA #SVs     "跟踪到的卫星数" (4.2.2 字段15)
        - 可用卫星数 Used    : BESTGNSSPOSA #solnSVs "参与解算的卫星数" (4.2.2 字段16)
        用户要求(2026-09-17): 只保留"跟踪"与"参与解算"两层, 不使用GSV可见数。
        关系: 跟踪 >= 可用。用户准则: GNSS 以 BESTGNSSPOSA 为准。
        回退: 无 BESTGNSSPOSA 时用 GPGGA 字段8, 该字段手册定义为"参与定位解算
              卫星数"(UG016 4.1.5), 属"可用"口径, 故回退时填入 used。
        兼容: 保留 average_satellites 等旧键(=跟踪口径), 供报告模板沿用。
        """
        print("分析卫星数量(跟踪/可用)...")

        bestgnss_data = self.parse_bestgnss_posa()
        gpgga_data = self.parse_gpgga()

        def _stats(lst):
            return (statistics.mean(lst), min(lst), max(lst)) if lst else (None, None, None)

        res = {}
        tr = [d['num_satellites'] for d in bestgnss_data if d.get('num_satellites', 0) > 0]
        us = [d['num_soln_satellites'] for d in bestgnss_data if d.get('num_soln_satellites', 0) > 0]
        # 回退: 无 BESTGNSSPOSA 时用 GPGGA 字段8("参与定位解算卫星数", 属可用口径)
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
            print("未找到卫星数数据(BESTGNSSPOSA/GPGGA)")
            return

        # 兼容旧键: 以"跟踪"口径为主(无则用可用)
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
    
    def parse_gpgsa(self):
        """解析北云 $GPGSA 报文, 取 PDOP/HDOP/VDOP 三件套 (2026-09-17)

        北云COM3每个周期只输出1条 $GPGSA(系统组合, PRN跨星座),
        结构: $GPGSA,mode,fix, 12个PRN槽, PDOP,HDOP,VDOP, systemId*CS
        实测样本: $GPGSA,M,3,03,16,26,...,1.6,0.7,1.4,1*24
          -> 倒数第4/3/2 = PDOP=1.6 HDOP=0.7 VDOP=1.4
        DOP是接收机本地解算的几何精度因子(非卫星下发), 与几何分布相关。
        返回: [{timestamp,pdop,hdop,vdop}](timestamp为None, 由GGA对齐或序号)
        """
        if hasattr(self, '_cache_gpgsa'):
            return self._cache_gpgsa
        data = []
        for ln in self.parsed_data.get('$GPGSA', []):
            ln = ln.strip()
            if not ln.startswith('$GPGSA'):
                continue
            body = ln[1:].split('*')[0]
            f = body.split(',')
            if len(f) < 18:
                continue
            try:
                # 末尾可能带 systemId(数字), DOP固定在最后第4/3/2
                # f[0]=GPGSA [1]=mode [2]=fix [3:15]=12 PRN [15]=PDOP [16]=HDOP [17]=VDOP
                pdop = float(f[15]); hdop = float(f[16]); vdop = float(f[17])
                data.append({'timestamp': None, 'pdop': pdop,
                             'hdop': hdop, 'vdop': vdop})
            except (ValueError, IndexError):
                continue
        self._cache_gpgsa = data
        return data

    def analyze_dop(self):
        """分析DOP值 (PDOP/HDOP/VDOP 三件套)

        数据源: 优先 $GPGSA(含PDOP/HDOP/VDOP三件套, 系统组合, 每周期1条);
                回退 $GPGGA(仅HDOP)。
        """
        print("分析DOP值...")

        gpgsa = self.parse_gpgsa()
        gpgga_data = self.parse_gpgga()

        dop_src = None
        pdop_l, hdop_l, vdop_l = [], [], []
        if gpgsa:
            dop_src = '$GPGSA(系统组合)'
            pdop_l = [d['pdop'] for d in gpgsa if d.get('pdop', 0) > 0]
            hdop_l = [d['hdop'] for d in gpgsa if d.get('hdop', 0) > 0]
            vdop_l = [d['vdop'] for d in gpgsa if d.get('vdop', 0) > 0]
        elif gpgga_data:
            dop_src = '$GPGGA(仅HDOP)'
            hdop_l = [d['hdop'] for d in gpgga_data if d.get('hdop', 0) > 0]
        else:
            print("未找到DOP数据(GPGSA/GPGGA均无)")
            return

        print(f"DOP数据源: {dop_src}")

        def _stats(lst):
            return (statistics.mean(lst), min(lst), max(lst)) if lst else (None, None, None)

        res = {'dop_source': dop_src}
        for name, lst in (('pdop', pdop_l), ('hdop', hdop_l), ('vdop', vdop_l)):
            a, mn, mx = _stats(lst)
            if a is not None:
                res[f'average_{name}'] = a
                res[f'min_{name}'] = mn
                res[f'max_{name}'] = mx
                res[f'{name}_values'] = lst
                print(f"平均{name.upper()}: {a:.2f} (最小{mn:.2f} 最大{mx:.2f})")
        self.analysis_results['dop'] = res

    def analyze_rtk_link(self):
        """RTK链路核对: 基站ID / 差分龄期 (数据源: BESTGNSSPOSA, UG016 4.2.2)"""
        print("RTK链路核对(基站ID/差分龄期)...")
        data = self.parse_bestgnss_posa()
        if not data:
            print("未找到BESTGNSSPOSA数据, 跳过RTK链路核对")
            return
        station_ids = [d['station_id'] for d in data if d.get('station_id')]
        diff_ages = [d['diff_age'] for d in data if d.get('diff_age', 0) > 0]
        if not station_ids:
            print("BESTGNSSPOSA中无基站ID信息(可能为单点定位)")
            return
        primary_station = Counter(station_ids).most_common(1)[0][0]
        result = {
            'primary_station': primary_station,
            'check_points': len(station_ids),
            'station_count': Counter(station_ids),
        }
        if diff_ages:
            result['avg_diff_age'] = statistics.mean(diff_ages)
            result['max_diff_age'] = max(diff_ages)
        self.analysis_results['rtk_link'] = result
        print(f"基站ID: {primary_station} (核对{len(station_ids)}次)")
        if diff_ages:
            print(f"差分龄期: 平均{result['avg_diff_age']:.2f}s, 最大{result['max_diff_age']:.2f}s")

    def analyze_velocity(self):
        """分析速度"""
        print("分析速度...")
        
        inspvax_data = self.parse_inspvaxa()
        if not inspvax_data:
            print("未找到INSPVAXA数据")
            return
        
        # 修复(2026-09-17): 过滤INS无解(NONE)历元。原实现对全部历元求速,
        # NONE历元速度为0会严重拉低平均速度(实测上午数据被低估约65%)。
        # 仅统计 pos_type 有效(非NONE)且坐标有效的历元。
        velocities = []
        for d in inspvax_data:
            pos_type = (d.get('pos_type') or d.get('solution_type') or '').strip().upper()
            if pos_type in ('NONE', '', 'INS_INACTIVE'):
                continue
            if not d.get('valid_coord', True):
                continue
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
    
    def parse_trackstat_cn0(self):
        """解析北云 TRACKSTATA 的载噪比 C/N0 (UG016 4.2.26)

        TRACKSTAT结构: 数据段 = sol status, pos type, cutoff, #chans, 然后每通道10字段
          [ch基]PRN, glofreq, ch-tr-status, psr伪距(m), Doppler(Hz),
          C/No载噪比(dB-Hz), locktime(s), psr res伪距残差(m), reject, psr weight
        即每通道第6项(相对偏移+5)为C/No。
        用途: C/N0判断遮挡/多路径, 行业通用指标。
        """
        if hasattr(self, '_cache_trackstat_cn0'):
            return self._cache_trackstat_cn0
        epochs = []
        for line in self.parsed_data.get('#TRACKSTATA', []):
            if ';' not in line:
                continue
            try:
                header_str, data_str = line.split(';', 1)
                hp = header_str.split(',')
                ts = float(hp[6]) if len(hp) > 6 else None
                f = data_str.split(',')
                n_ch = int(f[3])
                cn0s = []
                idx = 4
                for _ in range(n_ch):
                    if idx + 9 >= len(f):
                        break
                    try:
                        cn0 = float(f[idx + 5])
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
        self._cache_trackstat_cn0 = epochs
        return epochs

    def analyze_cn0(self):
        """载噪比C/N0分析(北云特色: TRACKSTATA通道跟踪)

        行业共识: C/N0>=45强, 35~45中, <35弱(遮挡/多路径), 跟踪门限约25~28。
        """
        import statistics as _st
        epochs = self.parse_trackstat_cn0()
        res = {'available': False}
        if epochs:
            all_cn0 = [c for e in epochs for c in e['cn0_list']]
            low = sum(1 for c in all_cn0 if c < 35)
            res = {'available': True, 'n_epochs': len(epochs),
                   'n_obs_total': len(all_cn0),
                   'mean_cn0': _st.mean(all_cn0),
                   'median_cn0': _st.median(all_cn0),
                   'min_cn0': min(all_cn0), 'max_cn0': max(all_cn0),
                   'low_ratio': low / len(all_cn0) * 100 if all_cn0 else 0,
                   'epochs': epochs}
        self.analysis_results['cn0'] = res
        if res['available']:
            print(f"C/N0分析(TRACKSTATA): {res['n_epochs']}历元, 平均{res['mean_cn0']:.1f}dB-Hz")
        return res

    def analyze_special_metrics(self):
        """组合惯导特色指标(北云): 姿态 + INSPVAX σ精度系列

        字段来源: INSPVAXA (UG016 表4-15)。
        用途: 姿态(横滚/俯仰/航向)反映Kinematic载体姿态动态;
              σ为接收机自估精度, 反映组合导航收敛质量。
        好/坏阈值见 kinematic_core.criteria(手册字段 + 工程经验)。
        """
        inspvax = self.parse_inspvaxa()
        valid = [d for d in inspvax if d.get('pos_type', '').startswith('INS_')]
        src = valid if valid else inspvax
        metrics = []
        if src:
            import statistics as _st
            def med(key):
                vals = [d[key] for d in src if d.get(key) is not None]
                return _st.median(vals) if vals else None
            roll = med('roll'); pitch = med('pitch'); azim = med('azimuth')
            if roll is not None:
                metrics.append(criteria.Metric(
                    'attitude', '载体姿态',
                    '横滚%.2f° 俯仰%.2f° 航向%.2f°' % (roll, pitch, azim),
                    criteria.INFO,
                    '组合惯导特色: 实时输出Kinematic载体姿态(横滚/俯仰/航向)。'
                    '纯GNSS无此能力。姿态连续性反映惯导跟踪状态。',
                    '北云UG016 INSPVAX字段'))
            metrics.extend(criteria.eval_ins_sigma(med('lat_sigma'), med('hgt_sigma'), med('azimuth_sigma')))
        # C/N0 载噪比特色指标(TRACKSTATA通道跟踪)
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
                '北云特色(TRACKSTATA): 信号载噪比, 判断遮挡/多路径。'
                '>=45强, 35~45中等, <35偏弱, 跟踪门限约25~28。详见cn0_analysis.png。'
                '实测: %s。' % txt,
                'TRACKSTATA通道跟踪(UG016 4.2.26); 阈值行业共识(Kaplan等)'))
        self.analysis_results['special_metrics'] = [m.as_dict() for m in metrics]
        print(f"组合惯导特色指标: {len(metrics)}项")

    def generate_plots(self):
        """生成图表"""
        print("生成分析图表...")
        
        # 位置时间序列图（GNSS - BESTGNSSPOSA）
        bestgnss_data = self.parse_bestgnss_posa()
        if bestgnss_data:
            self._plot_position_time_series(bestgnss_data, 'gnss')
        
        # 位置时间序列图（INS - INSPVAXA）
        inspvax_data = self.parse_inspvaxa()
        if inspvax_data:
            self._plot_position_time_series(inspvax_data, 'ins')
        
        # 解类型分布图（GNSS）
        if 'gnss_solution_type' in self.analysis_results:
            self._plot_gnss_solution_type_distribution()
        
        # 解类型分布图（INS）
        if 'ins_solution_type' in self.analysis_results:
            self._plot_ins_solution_type_distribution()
        
        # 卫星数量时间序列
        if 'satellite_visibility' in self.analysis_results:
            self._plot_satellite_time_series()
        
        # DOP时间序列
        if 'dop' in self.analysis_results:
            self._plot_dop_time_series()
        
        # 速度时间序列
        if 'velocity' in self.analysis_results:
            self._plot_velocity_time_series()
        
        # GNSS ENU轨迹图（按解类型着色）
        if bestgnss_data:
            self._plot_gnss_enu_trajectory(bestgnss_data)
        
        # INS ENU轨迹图（按解类型着色）
        inspvax_data = self.parse_inspvaxa()
        if inspvax_data:
            self._plot_ins_enu_trajectory(inspvax_data)
        
        # GNSS和INS轨迹合并图
        if bestgnss_data and inspvax_data:
            self._plot_gnss_ins_combined_trajectory(bestgnss_data, inspvax_data)
        
        # 解算状态时间序列图(用户准则: GNSS只用BESTGNSSPOSA, 不用BESTPOSA)
        bestgnss_for_status = self.parse_bestgnss_posa()
        if bestgnss_for_status:
            self._plot_solution_status(bestgnss_for_status)

        # C/N0载噪比图(北云特色: TRACKSTATA)
        if self.analysis_results.get('cn0', {}).get('available'):
            self._plot_cn0()
    
    def _plot_velocity_time_series(self):
        """绘制速度时间序列 (数据源: INSPVAXA 的 N/E/U 三分量速度)

        与 analyze_velocity 一致: speed = sqrt(Vn^2+Ve^2+Vu^2)
        INSPVAXA 含时间戳, 以相对时间(秒)为横轴。
        标题/轴标签用英文避免字体乱码。
        """
        inspvax_data = self.parse_inspvaxa()
        if not inspvax_data:
            print("无INSPVAXA数据, 跳过速度时间序列图")
            return
        timestamps = [d['timestamp'] for d in inspvax_data]
        if not timestamps:
            return
        min_time = min(timestamps)
        rel = [t - min_time for t in timestamps]
        speeds = [math.sqrt(d['velocity_north']**2 + d['velocity_east']**2 + d['velocity_up']**2)
                  for d in inspvax_data]

        plt.figure(figsize=(12, 6))
        plt.plot(rel, speeds, color='#1f77b4', linewidth=0.8)
        plt.title('Speed Time Series (INSPVAXA)')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Speed (m/s)')
        plt.grid(True)
        plt.tight_layout()
        output_file = os.path.join(self.output_dir, 'velocity_time_series.png')
        plt.savefig(output_file)
        plt.close()
        print(f"保存速度时间序列图: {output_file}")

    def _detect_anomalies(self, data, speed_threshold):
        """检测异常跳跃点: 相邻两有效坐标点相对速度超过阈值即判定。

        相对速度 = 两点平面距离(经纬度近似equirectangular, 短距离精确) / 时间差。
        返回需标记为异常的【后一个点】的下标列表。
        仅在 self.mark_anomalies 为 True 时由轨迹图调用。
        """
        pts = [d for d in data if d.get('valid_coord', True)]
        anomalies = []
        prev = None
        R = 6371000.0  # 地球半径(m), 用于短距离平面近似
        for i, d in enumerate(data):
            if not d.get('valid_coord', True):
                continue
            if prev is not None:
                dt = d['timestamp'] - prev['timestamp']
                if dt > 1e-9:
                    lat1, lon1 = math.radians(prev['latitude']), math.radians(prev['longitude'])
                    lat2, lon2 = math.radians(d['latitude']), math.radians(d['longitude'])
                    dlat, dlon = lat2 - lat1, lon2 - lon1
                    x = dlon * math.cos((lat1 + lat2) / 2.0)
                    dist = math.sqrt(x * x + dlat * dlat) * R
                    if dist / dt > speed_threshold:
                        anomalies.append(i)
            prev = d
        return anomalies

    def _enu_coords(self, data):
        """经纬度 -> 相对首点的 ENU 平面坐标 (equirectangular 短距离近似, m)。

        只取 valid_coord 点, 返回 (east_list, north_list, 过滤后的data)。
        """
        pts = [d for d in data if d.get('valid_coord', True)]
        if not pts:
            return [], [], []
        R = 6371000.0
        lat0 = math.radians(pts[0]['latitude'])
        lon0 = math.radians(pts[0]['longitude'])
        east, north = [], []
        for d in pts:
            lat = math.radians(d['latitude'])
            lon = math.radians(d['longitude'])
            east.append((lon - lon0) * math.cos((lat0 + lat) / 2.0) * R)
            north.append((lat - lat0) * R)
        return east, north, pts

    def _plot_enu_trajectory(self, data, source, title, outfile):
        """按解类型着色的 ENU 轨迹图 (通用, GNSS/INS 复用)。

        每个点按其 pos_type 用 criteria.type_color 着色; 图例给出各解类型及中文名;
        若 self.mark_anomalies 为 True, 用红色X标出速度异常点。
        """
        east, north, pts = self._enu_coords(data)
        if not pts:
            print(f"无{source}有效坐标, 跳过{title}")
            return

        plt.figure(figsize=(9, 8))
        # 按解类型分组绘制(同类型同色, 便于图例)
        from collections import OrderedDict
        groups = OrderedDict()
        for e, n, d in zip(east, north, pts):
            pt = d.get('pos_type', d.get('solution_type', 'NONE'))
            groups.setdefault(pt, {'e': [], 'n': []})
            groups[pt]['e'].append(e)
            groups[pt]['n'].append(n)
        for pt, g in groups.items():
            color = criteria.type_color(pt)
            cn = criteria.type_cn(pt)
            plt.scatter(g['e'], g['n'], c=color, s=8, label=f'{pt} {cn}', zorder=3)

        # 异常跳跃点标记(红色X)
        if self.mark_anomalies:
            anom_idx = self._detect_anomalies(data, self.speed_threshold)
            if anom_idx:
                ax_ = [east[i] for i in anom_idx]
                ay = [north[i] for i in anom_idx]
                plt.scatter(ax_, ay, marker='x', c='red', s=90, linewidths=2,
                            label=f'Anomaly (>{self.speed_threshold:g} m/s)', zorder=5)

        plt.title(title, fontsize=13, fontweight='bold')
        plt.xlabel('East (m)', fontsize=11)
        plt.ylabel('North (m)', fontsize=11)
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=8, loc='best', markerscale=2)
        plt.tight_layout()
        out = os.path.join(self.output_dir, outfile)
        plt.savefig(out, dpi=150)
        plt.close()
        print(f"保存{source} ENU轨迹图: {out}")

    def _plot_gnss_enu_trajectory(self, data):
        """GNSS ENU 轨迹图 (数据源: BESTGNSSPOSA, 用户准则: 不用BESTPOSA)"""
        self._plot_enu_trajectory(data, 'GNSS',
                                  'GNSS ENU Trajectory (BESTGNSSPOSA, colored by solution type)',
                                  'gnss_enu_trajectory.png')

    def _plot_ins_enu_trajectory(self, data):
        """INS ENU 轨迹图 (数据源: INSPVAXA)"""
        self._plot_enu_trajectory(data, 'INS',
                                  'INS ENU Trajectory (INSPVAXA, colored by solution type)',
                                  'ins_enu_trajectory.png')

    def _plot_gnss_ins_combined_trajectory(self, gnss_data, ins_data):
        """GNSS 与 INS 轨迹合并对比图 (GNSS=BESTGNSSPOSA, INS=INSPVAXA)"""
        ge, gn, gpts = self._enu_coords(gnss_data)
        ie, in_, ipts = self._enu_coords(ins_data)
        if not gpts and not ipts:
            print("GNSS/INS均无有效坐标, 跳过合并轨迹图")
            return
        plt.figure(figsize=(9, 8))
        if gpts:
            plt.plot(ge, gn, '.', color='#0066FF', markersize=3,
                     label='GNSS (BESTGNSSPOSA)', zorder=3)
        if ipts:
            plt.plot(ie, in_, '-', color='#00C000', linewidth=1,
                     label='INS (INSPVAXA)', zorder=2)
        plt.title('GNSS vs INS ENU Trajectory', fontsize=13, fontweight='bold')
        plt.xlabel('East (m)', fontsize=11)
        plt.ylabel('North (m)', fontsize=11)
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=9, loc='best')
        plt.tight_layout()
        out = os.path.join(self.output_dir, 'gnss_ins_combined_trajectory.png')
        plt.savefig(out, dpi=150)
        plt.close()
        print(f"保存GNSS/INS合并轨迹图: {out}")

    def _plot_cn0(self):
        """绘制C/N0载噪比分析图(北云特色: TRACKSTATA, 判断遮挡/多路径)

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
        ax1.set_title('C/N0 Distribution (per tracking channel)', fontsize=13, fontweight='bold')
        ax1.set_ylabel('Count', fontsize=11)
        ax1.tick_params(axis='x', rotation=15)

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
    
    def _plot_ins_solution_type_distribution(self):
        """绘制INS解类型分布图"""
        type_count = self.analysis_results['ins_solution_type']['type_count']
        total = self.analysis_results['ins_solution_type']['total_epochs']

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

        plt.title('INS Position Type Distribution', fontsize=14, fontweight='bold')
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
        output_file = os.path.join(self.output_dir, 'ins_solution_type_distribution.png')
        plt.savefig(output_file, dpi=150)
        plt.close()
        print(f"保存INS解类型分布图: {output_file}")
    
    def _plot_solution_type_distribution(self):
        """绘制解类型分布图（兼容旧版本）"""
        self._plot_gnss_solution_type_distribution()
    
    def _plot_satellite_time_series(self):
        """绘制卫星数量时间序列(跟踪/可用两层, 与统计同源 BESTGNSSPOSA)

        两层口径(UG016 手册定义):
          - 跟踪 Tracked : BESTGNSSPOSA #SVs     "跟踪到的卫星数" (4.2.2 字段15)
          - 可用 Used    : BESTGNSSPOSA #solnSVs "参与解算的卫星数" (4.2.2 字段16)
        用户要求(2026-09-17): 去掉"可见"层, 只保留跟踪与参与解算。
        修复(2026-09-17): 原实现用 GGA 而统计用 BESTGNSSPOSA, 数据源不一致;
        现统一以 BESTGNSSPOSA 为准(用户准则)。回退 GPGGA 时其字段8按手册
        定义为"参与定位解算卫星数", 归入 Used 口径。
        """
        best = self.parse_bestgnss_posa()
        if not best:
            # 回退: GPGGA 字段8 = "参与定位解算卫星数"(UG016 4.1.5), 属可用口径
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
        """绘制DOP时间序列 - PDOP/HDOP/VDOP 三条曲线(GPGSA优先, GGA回退)

        北云GPGSA无独立时间戳, 用GGA时间轴对齐(同周期0.1s); 若数量不一致
        则用序号轴。
        """
        gpgsa = self.parse_gpgsa()
        gpgga_data = self.parse_gpgga()

        series = gpgsa if gpgsa else gpgga_data
        src = '$GPGSA(系统组合)' if gpgsa else '$GPGGA(仅HDOP)'
        if not series:
            return

        # 时间轴: GPGSA无timestamp, 用GGA相对时间对齐(同周期); 否则序号
        x = None
        xlabel = 'Epoch index'
        if gpgga_data and len(gpgga_data) >= len(series):
            t0 = min(d['timestamp'] for d in gpgga_data)
            x = [d['timestamp'] - t0 for d in gpgga_data[:len(series)]]
            xlabel = 'Time (seconds)'
        if x is None:
            x = list(range(len(series)))

        plt.figure(figsize=(12, 6))
        plotted = False
        for key, color in (('pdop', '#1f77b4'), ('hdop', '#2ca02c'), ('vdop', '#d62728')):
            # 修复(2026-09-17): x/y 必须配对过滤。原实现仅压缩ys后用x[:len(ys)],
            # 若series中间存在无效历元会造成x-y错位。现按历元配对, 仅保留该字段
            # 有效的(x,y)对。
            pairs = [(x[i], d.get(key)) for i, d in enumerate(series)
                     if i < len(x) and d.get(key) is not None and d.get(key) > 0]
            if pairs:
                xs = [p[0] for p in pairs]
                ys = [p[1] for p in pairs]
                plt.plot(xs, ys, label=key.upper(), color=color, linewidth=1)
                plotted = True
        if not plotted:
            plt.close()
            return
        plt.title(f'DOP Time Series ({src})')
        plt.xlabel(xlabel)
        plt.ylabel('DOP')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        output_file = os.path.join(self.output_dir, 'dop_time_series.png')
        plt.savefig(output_file)
        plt.close()
        print(f"保存DOP时间序列图: {output_file}")

    def _plot_solution_status(self, data):
        """绘制BESTGNSSPOSA解算状态时间序列图(用户准则: 不用BESTPOSA)
        
        Args:
            data: BESTGNSSPOSA解析数据列表(含sol_status字段)
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
        
        # 提取状态序列(BESTGNSSPOSA的sol_status)
        sol_stat_list = [d.get('sol_status', 'UNKNOWN') for d in data]
        
        # 创建图
        plt.figure(figsize=(16, 8))
        
        # 绘制状态时间序列（使用阶梯图表示状态）
        # 首先找出所有不同状态并分配Y坐标
        unique_states = list(set(sol_stat_list))
        state_to_y = {state: i+1 for i, state in enumerate(unique_states)}
        
        # 向量化绘制(性能优化, 视觉输出与原逐点scatter/逐段plot完全一致):
        # 按状态分组后批量scatter(组内颜色一致), 折线用NaN在状态切换处断开单次plot。
        nan = float('nan')
        groups = {}
        for i, state in enumerate(sol_stat_list):
            g = groups.setdefault(state, {'t': [], 'y': []})
            g['t'].append(relative_times[i])
            g['y'].append(state_to_y[state])
        for state, g in groups.items():
            color = sol_status_color_map.get(state, '#808080')
            plt.scatter(g['t'], g['y'], c=color, s=80, zorder=5)
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
        plt.title('BESTGNSSPOSA Solution Status Time Series', fontsize=14, fontweight='bold')
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
        <h1>GPS/RTK Kinematic定位分析报告【北云 BEIYUN M21 · GNSS/INS组合惯导】</h1>
        <p><strong>分析输入(COM3/ASCII):</strong> {os.path.basename(self.input_file)} |
           <strong>生成时间:</strong> {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} |
           <strong>分析工具:</strong> 北云 BEIYUN M21 Kinematic Analyzer</p>

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
        <h2>2. 完整性检查与报文周期核对(ICOM3配置)</h2>
        <table>
            <tr>
                <th>报文</th>
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
                html_content += f"""
            <tr>
                <td>{chk['header']}</td>
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
            if mi.get('icom4_notes'):
                html_content += """
        <h3>ICOM4 关注项(静态相关)</h3>
        <ul>
        """
                for note in mi['icom4_notes']:
                    html_content += f"            <li>{note}</li>\n"
                html_content += "        </ul>\n"

        # 采样率分析
        if 'sampling_rate' in self.analysis_results:
            sr = self.analysis_results['sampling_rate']
            html_content += f"""
        <p><strong>采样率分析:</strong> {sr['sampling_rate']:.2f} Hz (平均间隔: {sr['average_interval']:.3f} 秒)</p>
            """
        
        # RTK链路核对(BESTGNSSPOSA: 基站ID/差分龄期)
        rtk = self.analysis_results.get('rtk_link')
        if rtk:
            rtk_html = f"<p><strong>RTK链路:</strong> 基站ID {rtk['primary_station']} (核对{rtk['check_points']}次)"
            if 'avg_diff_age' in rtk:
                rtk_html += f" ／ 差分龄期 平均{rtk['avg_diff_age']:.2f}s, 最大{rtk['max_diff_age']:.2f}s"
            html_content += rtk_html + "</p>\n"

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

        if 'ins_solution_type' in self.analysis_results:
            ist = self.analysis_results['ins_solution_type']
            html_content += f"""
        <h2>4. INS解类型分析</h2>
        <div class="chart">
            <img src="ins_solution_type_distribution.png" alt="INS解类型分布"/>
        </div>
        <table>
            <tr>
                <th>解类型</th>
                <th>数量</th>
                <th>占比</th>
            </tr>
            """

            for sol_type, count in sorted(ist['type_count'].items(), key=lambda x: x[1], reverse=True):
                percentage = (count / ist['total_epochs'] * 100) if ist['total_epochs'] > 0 else 0
                html_content += f"""
            <tr>
                <td>{sol_type}</td>
                <td>{count}</td>
                <td>{percentage:.1f}%</td>
            </tr>
                """
            
            html_content += f"""
        </table>
        <p><strong>INS固定解比率:</strong> <span class="{'pass' if ist['fixed_ratio'] > 95 else 'metric-warning' if ist['fixed_ratio'] > 80 else 'fail'}">{ist['fixed_ratio']:.1f}%</span></p>
            """
        
        # 卫星数量分析(跟踪/可用两层, 手册定义)
        if 'satellite_visibility' in self.analysis_results:
            sv = self.analysis_results['satellite_visibility']
            rows = ''
            defs = (
                ('tracked', '跟踪卫星数', 'BESTGNSSPOSA #SVs"跟踪到的卫星数"(UG016 4.2.2 字段15)'),
                ('used', '可用卫星数', 'BESTGNSSPOSA #solnSVs"参与解算的卫星数"(UG016 4.2.2 字段16)'),
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
        <h2>5. 卫星数量分析</h2>
        <div class="chart">
            <img src="satellite_time_series.png" alt="卫星数量时间序列"/>
        </div>
        <table>
            <tr>
                <th>指标(口径依据UG016手册)</th>
                <th>数值(平均/最小/最大)</th>
            </tr>
{rows}        </table>
        <p><small>两层关系: 跟踪 &ge; 可用。跟踪为接收机已锁定跟踪的卫星, '
        '可用为实际参与位置解算的卫星。口径依据 UG016 4.2.2 字段15/16。</small></p>
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
            rows = _row('PDOP', 'pdop') + _row('HDOP', 'hdop') + _row('VDOP', 'vdop')
            html_content += f"""
        <h2>6. DOP分析</h2>
        <p><strong>数据源:</strong> {src} ｜
           <span style="color:#555;">DOP为接收机本地解算的几何精度因子(非卫星下发),
           取决于卫星几何分布。</span></p>
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

        # 载噪比 C/N0 分析(北云特色: TRACKSTATA通道跟踪)
        cn0 = self.analysis_results.get('cn0', {})
        if cn0.get('available'):
            html_content += f"""
        <h2>8. 载噪比 C/N0 分析<span class="badge-feature">特色(TRACKSTATA)</span></h2>
        <div class="chart">
            <img src="cn0_analysis.png" alt="C/N0载噪比分析"/>
        </div>
        <table>
            <tr><th>指标</th><th>数值</th><th>说明</th></tr>
            <tr><td>数据来源</td><td>TRACKSTATA</td><td>通道跟踪状态, 逐跟踪通道给出载噪比, 及时性好</td></tr>
            <tr><td>统计历元数</td><td>{cn0['n_epochs']}</td><td>参与统计的观测历元</td></tr>
            <tr><td>观测值总数</td><td>{cn0['n_obs_total']}</td><td>全部通道的C/N0样本数</td></tr>
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
        bestgnss_data = self.parse_bestgnss_posa()
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
        
        <h3>{_pos_no}.2 INS轨迹分析（ENU坐标）</h3>
        <div class="chart">
            <img src="ins_enu_trajectory.png" alt="INS ENU轨迹"/>
        </div>
        <p>INS轨迹图使用ENU坐标系显示，组合惯导系统的定位轨迹。颜色对应：绿色(INS_RTKFIXED)、橙色(INS_RTKFLOAT)、蓝色(INS_PSRSP)、黄色(INS_PSRDIFF)、灰色(NONE)。</p>
        
        <h3>{_pos_no}.3 GNSS与INS轨迹对比</h3>
        <div class="chart">
            <img src="gnss_ins_combined_trajectory.png" alt="GNSS与INS轨迹对比"/>
        </div>
        <p>该图展示了GNSS纯定位轨迹（蓝色）与INS组合导航轨迹（红色）的叠加对比，可以分析两者之间的一致性和差异。</p>
        
        <h3>{_pos_no}.4 BESTGNSSPOSA解算状态时间序列</h3>
        <div class="chart">
            <img src="solution_status.png" alt="解算状态时间序列"/>
        </div>
        <p>该图展示了BESTGNSSPOSA报文中解算状态（sol stat）的时间序列变化，帮助分析定位过程中解算状态的稳定性。</p>
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
        
        if 'ins_solution_type' in self.analysis_results:
            ins_fixed_ratio = self.analysis_results['ins_solution_type']['fixed_ratio']
            status = 'pass' if ins_fixed_ratio > 95 else 'metric-warning' if ins_fixed_ratio > 80 else 'fail'
            evaluations.append({
                'metric': 'INS固定解比率',
                'status': status,
                'description': f'{ins_fixed_ratio:.1f}%，{"满足高精度定位要求" if ins_fixed_ratio > 95 else "基本满足要求" if ins_fixed_ratio > 80 else "需要改善"}'
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
# GPS/RTK Kinematic定位分析报告【北云 BEIYUN M21 · GNSS/INS组合惯导】

## 基本信息
- **分析输入(COM3/ASCII)**: {os.path.basename(self.input_file)}
- **生成时间**: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- **分析工具**: 北云 BEIYUN M21 Kinematic Analyzer

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
## 2. 完整性检查与报文周期核对(ICOM3配置)

| 报文 | 条数 | 设定周期(s) | 实际周期(s) | 实际频率(Hz) | 丢帧率 | 状态 | 备注 |
|------|------|------------|------------|-------------|--------|------|------|
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
                note = chk['note'] or '-'
                md_content += (f"| {chk['header']} | {chk['count']} | "
                               f"{chk['configured_period']:.3f} | {actual_p} | "
                               f"{actual_r} | {loss} | "
                               f"{status_icon.get(chk['status'], chk['status'])} | "
                               f"{note} |\n")
            md_content += """
> ⚠️ 加粗标色项表示实际报文周期与设定周期可能不符或报文缺失, 请核对设备输出配置。
"""
            if mi.get('icom4_notes'):
                md_content += """
### ICOM4 关注项(静态相关)
"""
                for note in mi['icom4_notes']:
                    md_content += f"- {note}\n"
                md_content += "\n"

        # 采样率分析
        if 'sampling_rate' in self.analysis_results:
            sr = self.analysis_results['sampling_rate']
            md_content += f"""
**采样率分析**: {sr['sampling_rate']:.2f} Hz (平均间隔: {sr['average_interval']:.3f} 秒)

        """
        
        # RTK链路核对(BESTGNSSPOSA: 基站ID/差分龄期)
        rtk = self.analysis_results.get('rtk_link')
        if rtk:
            rtk_line = f"**RTK链路**: 基站ID {rtk['primary_station']}(核对{rtk['check_points']}次)"
            if 'avg_diff_age' in rtk:
                rtk_line += f" ／ 差分龄期 平均{rtk['avg_diff_age']:.2f}s, 最大{rtk['max_diff_age']:.2f}s"
            md_content += rtk_line + "\n\n"

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

        if 'ins_solution_type' in self.analysis_results:
            ist = self.analysis_results['ins_solution_type']
            md_content += f"""
## 4. INS解类型分析

![INS解类型分布](ins_solution_type_distribution.png)

| 解类型 | 数量 | 占比 |
|--------|------|------|
"""
            for sol_type, count in sorted(ist['type_count'].items(), key=lambda x: x[1], reverse=True):
                percentage = (count / ist['total_epochs'] * 100) if ist['total_epochs'] > 0 else 0
                md_content += f"| {sol_type} | {count} | {percentage:.1f}% |\n"

            md_content += f"""

**INS固定解比率**: {ist['fixed_ratio']:.1f}%

"""
        
        # 卫星数量分析(跟踪/可用两层, 手册定义)
        if 'satellite_visibility' in self.analysis_results:
            sv = self.analysis_results['satellite_visibility']
            mrows = ''
            defs = (
                ('tracked', '跟踪卫星数', 'BESTGNSSPOSA #SVs跟踪到的卫星数(UG016 4.2.2字段15)'),
                ('used', '可用卫星数', 'BESTGNSSPOSA #solnSVs参与解算的卫星数(UG016 4.2.2字段16)'),
            )
            for key, cn, desc in defs:
                if f'average_{key}' not in sv:
                    continue
                mrows += (f'| {cn} | {sv[f"average_{key}"]:.1f} | {sv[f"min_{key}"]} | '
                          f'{sv[f"max_{key}"]} | {desc} |\n')
            md_content += f"""
## 5. 卫星数量分析

![卫星数量时间序列](satellite_time_series.png)

| 指标 | 平均 | 最小 | 最大 | 口径依据(UG016手册) |
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
                if key == 'hdop':
                    ev = '优秀' if a < 1 else '良好' if a < 2 else '一般'
                else:
                    ev = '-'
                return (f"| 平均{cn} | {a:.2f} | {ev} |\n"
                        f"| 最小{cn} | {mn:.2f} |  |\n"
                        f"| 最大{cn} | {mx:.2f} |  |\n")
            rows = _mrow('PDOP', 'pdop') + _mrow('HDOP', 'hdop') + _mrow('VDOP', 'vdop')
            md_content += f"""
## 6. DOP分析

**数据源:** {src}

> DOP为接收机本地解算的几何精度因子(非卫星下发), 取决于卫星几何分布。

![DOP时间序列](dop_time_series.png)

| 指标 | 数值 | 评估 |
|------|------|------|
{rows}
        """
        
        # 速度分析
        if 'velocity' in self.analysis_results:
            vel = self.analysis_results['velocity']
            md_content += f"""
## 7. 速度分析

![速度时间序列](velocity_time_series.png)

| 指标 | 数值 | 单位 |
|------|------|------|
| 平均速度 | {vel['average_speed']:.2f} | m/s |
| 最大速度 | {vel['max_speed']:.2f} | m/s |

        """
        
        # 载噪比 C/N0 分析(北云特色: TRACKSTATA)
        cn0 = self.analysis_results.get('cn0', {})
        if cn0.get('available'):
            md_content += f"""
## 8. 载噪比 C/N0 分析（特色：TRACKSTATA）

![C/N0载噪比分析](cn0_analysis.png)

| 指标 | 数值 | 说明 |
|------|------|------|
| 数据来源 | TRACKSTATA | 通道跟踪状态, 逐跟踪通道给出载噪比, 及时性好 |
| 统计历元数 | {cn0['n_epochs']} | 参与统计的观测历元 |
| 观测值总数 | {cn0['n_obs_total']} | 全部通道的C/N0样本数 |
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
        bestgnss_data = self.parse_bestgnss_posa()
        if bestgnss_data:
            md_content += f"""
## {_pos_no}. 位置分析

![位置时间序列](position_time_series_gnss.png)

位置时间序列显示了接收机在Kinematic环境下的位置变化。从图中可以观察到位置的连续性和稳定性。

### {_pos_no}.1 GNSS轨迹分析（ENU坐标）

![GNSS ENU轨迹](gnss_enu_trajectory.png)

GNSS轨迹图使用ENU（East-North-Up）坐标系显示，按数据实际出现的解类型动态着色（本设备GNSS解类型见上分布图；不同产品命名可能有差异，如伪距差分北云称PSRDIFF、华测称SPPDIFF，含义相同）：绿色=RTK固定解(NARROW_INT)，蓝色=浮点解(NARROW_FLOAT)，红色=单点(SINGLE)，橙色=伪距差分(PSRDIFF/SPPDIFF)，紫色=PPP，灰色=无解(NONE)。图例中标注各类型数量及中文名。

### {_pos_no}.2 INS轨迹分析（ENU坐标）

![INS ENU轨迹](ins_enu_trajectory.png)

INS轨迹图使用ENU坐标系显示，组合惯导系统的定位轨迹。颜色对应：绿色(INS_RTKFIXED)、橙色(INS_RTKFLOAT)、蓝色(INS_PSRSP)、黄色(INS_PSRDIFF)、灰色(NONE)。

### {_pos_no}.3 GNSS与INS轨迹对比

![GNSS与INS轨迹对比](gnss_ins_combined_trajectory.png)

该图展示了GNSS纯定位轨迹（蓝色）与INS组合导航轨迹（红色）的叠加对比，可以分析两者之间的一致性和差异。

### {_pos_no}.4 BESTGNSSPOSA解算状态时间序列

![解算状态时间序列](solution_status.png)

该图展示了BESTGNSSPOSA报文中解算状态（sol stat）的时间序列变化，帮助分析定位过程中解算状态的稳定性。
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
        
        if 'ins_solution_type' in self.analysis_results:
            ins_fixed_ratio = self.analysis_results['ins_solution_type']['fixed_ratio']
            status = '通过' if ins_fixed_ratio > 95 else '警告' if ins_fixed_ratio > 80 else '失败'
            evaluations.append({
                'metric': 'INS固定解比率',
                'status': status,
                'description': f'{ins_fixed_ratio:.1f}%，{"满足高精度定位要求" if ins_fixed_ratio > 95 else "基本满足要求" if ins_fixed_ratio > 80 else "需要改善"}'
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

        # 步骤1-3: 完整性检查与报文周期核对(ICOM3配置, 实际周期界面显示)
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
        
        # 分析速度
        self.analyze_velocity()

        # RTK链路核对(BESTGNSSPOSA基站ID/差分龄期)
        self.analyze_rtk_link()

        # 组合惯导特色指标(姿态/σ精度)
        self.analyze_special_metrics()

        # 载噪比C/N0分析(TRACKSTATA)
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
        # 北云 COM3 = ASCII 文本主文件(本程序唯一输入)
        file_path = filedialog.askopenfilename(
            title="选择北云 COM3 数据文件(ASCII文本)",
            filetypes=[("COM3 ASCII数据", "*.dat"), ("所有文件", "*")]
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
                messagebox.showinfo("成功", f"分析完成！结果保存在: {analyzer.output_dir}")
            else:
                messagebox.showerror("错误", "分析过程出现错误")
        except Exception as e:
            messagebox.showerror("错误", f"分析失败: {str(e)}")

    # 创建主窗口
    root = tk.Tk()
    root.title("北云 BEIYUN M21 GNSS/INS组合惯导 Kinematic分析工具")
    root.geometry("620x560")
    root.resizable(True, True)

    # 设置背景色
    root.configure(bg="#f0f0f0")

    # 创建标题
    title_label = tk.Label(root, text="北云 BEIYUN M21 GNSS/INS组合惯导 Kinematic分析工具",
                          font=("微软雅黑", 15, "bold"), bg="#f0f0f0", fg="#1a5276")
    title_label.pack(pady=(18, 4))

    # 醒目设备横幅: 明确本程序适用的设备, 避免与华测程序混淆
    device_banner = tk.Label(root,
                             text="【适用设备：北云 BEIYUN M21 · GNSS/INS 组合惯导移动站】",
                             font=("微软雅黑", 11, "bold"),
                             bg="#1a5276", fg="white", padx=10, pady=6)
    device_banner.pack(pady=(0, 12), fill=tk.X, padx=20)

    # 创建文件选择区域
    frame = tk.Frame(root, bg="#f0f0f0")
    frame.pack(pady=10, padx=20, fill=tk.X)

    label = tk.Label(frame, text="输入文件 - COM3 (ASCII文本, 必选):",
                     font=("微软雅黑", 10, "bold"), bg="#f0f0f0", fg="#1a5276")
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
【适用设备】北云 BEIYUN M21 · GNSS/INS 组合惯导移动站

输入文件说明（只需一个文件）：
- COM3 (必选): ASCII 文本文件(.dat)
  包含 #INSPVAXA(惯导0.1s) #BESTGNSSPOSA(GNSS0.2s)
  #TRACKSTATA(载噪比0.2s) #HEADINGA 及 $NMEA 报文
  本程序的解析、评估、绘图、报告全部基于 COM3
  载噪比C/N0 已从 COM3 的 #TRACKSTATA 获取，无需 COM4

操作步骤：
1. 浏览选择 COM3 文件
2. 可勾选"标记异常跳跃点"并设置速度阈值
3. 点击"开始分析"，结果保存在与 COM3 同名的目录中

用户准则：GNSS 仅用 BESTGNSSPOSA，惯导仅用 INSPVAXA，
$GPIMU 按要求放弃处理；带时间戳的以时间戳为准。

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

    parser = argparse.ArgumentParser(description='高精度KinematicGPS定位数据综合分析与处理程序【北云 M21 版本】')
    parser.add_argument('input_file', nargs='?', help='输入的COM3 dat文件路径(ASCII, 本程序唯一输入)')
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
