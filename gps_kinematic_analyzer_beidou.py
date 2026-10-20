#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
和芯星通 UM980 / 北斗星通模组 GNSS/RTK Kinematic定位数据综合分析程序

依据《NebulasIV 系列高精度产品数据接口协议》(N4 Products Commands and Logs
Reference Book R1.15)解析 UM980 输出。UM980为无IMU、单天线GNSS模组，不做INS分析。

主要输入:
    ASCII: #BESTNAVA, #SATSINFOA, #BESTSATA, #RTKSTATUSA, $GNGGA/$GPGGA,
           $GNGSA/$GPGSA, $GNGST/$GPGST, $GNGRS/$GPGRS, $xxGSV
    BINARY: OBSVMB (Message ID=12) 混入在同一日志流中

主要功能:
    1. 二进制OBSVMB按0xAA44B5同步头、Header长度和CRC-32恢复，避免按行读入破坏ASCII
    2. 完整性检查与报文周期核对
    3. BESTNAVA主定位/速度分析、解类型、卫星数、DOP、σ、伪距残差RMS、RTK状态
    4. OBSVM C/N0、locktime、失锁/重捕获分析
    5. GRS逐星残差分布，辅助异常卫星识别
    6. ENU轨迹、HTML和Markdown报告

数据字段依据:
    - BESTNAV: 手册7.3.19表7-70
    - GGA: 手册7.1.1.3表7-4
    - GRS: 手册7.1.1.6表7-7
    - GSA: 手册7.1.1.7表7-8
    - GST: 手册7.1.1.8表7-9
    - OBSVM: 手册7.3.65表7-137/7-138/7-140
    - SATSINFO: 手册7.3.89表7-174~7-177
    - BESTSAT: 手册7.3.22表7-76~7-80
    - RTKSTATUS: 手册7.3.85表7-169
"""

import os
import re
import math
import datetime
import argparse
import statistics
import struct
from collections import defaultdict, Counter

from kinematic_core import criteria

try:
    import numpy as np
    import matplotlib.pyplot as plt
except ImportError as e:
    print(f"缺少必要的依赖库: {e}")
    print("请运行以下命令安装依赖:")
    print("  pip install numpy matplotlib")
    import sys
    sys.exit(1)

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False


# ============================================================================
# UM980数据分析器
# ============================================================================
class GPSKinematicAnalyzer:
    # 依据 beidou_manual/UM980_北云指令对照表.md 的推荐配置。该周期仅作为参考基准，
    # 不把“实测周期与参考周期不一致”直接判为错误：现场可能按试验目的特意重新设置。
    LOG_CONFIG = [
        ('BESTNAVA',  'BESTNAVA',  0.1, True),
        ('GGA',       'GGA',       0.1, True),
        ('GSA',       'GSA',       0.1, True),
        ('GRS',       'GRS',       0.1, True),
        ('GSV',       'GSV',       1.0, True),
        ('GST',       'GST',       1.0, True),
        ('SATSINFOA', 'SATSINFOA', 1.0, True),
        ('BESTSATA',  'BESTSATA',  1.0, True),
        ('RTKSTATUSA','RTKSTATUSA',1.0, True),
        ('OBSVMB',    'OBSVMB',    1.0, True),
    ]
    NMEA_GROUPS = {
        'GGA': ('$GNGGA', '$GPGGA'),
        'GST': ('$GNGST', '$GPGST'),
        'GRS': ('$GNGRS', '$GPGRS'),
        'GSA_PREFIX': ('GSA',),
        'GSV_SUFFIX': ('GSV',),
    }
    _CRC_TAB = []
    _OBSVM_RECORD = struct.Struct('<HHddHHfHHfI')

    def __init__(self, input_file, mark_anomalies=False, speed_threshold=2.0):
        self.input_file = input_file
        self.filename = os.path.basename(input_file).split('.')[0]
        if self.filename.lower() in {
                'com1','com2','com3','com4','com5','com6','com7','com8','com9',
                'lpt1','lpt2','lpt3','lpt4','lpt5','lpt6','lpt7','lpt8','lpt9',
                'con','prn','nul'}:
            self.filename += '_data'
        self.output_dir = os.path.join(os.path.dirname(input_file), self.filename)
        self.parsed_data = defaultdict(list)
        self.obsvm_records = []
        self.obsvm_events = []
        self.header_info = {}
        self.analysis_results = {}
        self.mark_anomalies = mark_anomalies
        self.speed_threshold = speed_threshold

    # ------------------------------------------------------------------
    # 基础工具
    # ------------------------------------------------------------------
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
    def _unicore_crc32(cls, data: bytes) -> int:
        """Unicore/NovAtel CRC-32: 反射多项式0xEDB88320，初值0。"""
        tab = cls._crc_table(); c = 0
        for b in data:
            c = tab[(c ^ b) & 0xff] ^ (c >> 8)
        return c

    @staticmethod
    def _nmea_xor(data: bytes) -> int:
        x = 0
        for b in data:
            x ^= b
        return x

    @staticmethod
    def _nmea_seconds(value: str):
        if not value or len(value) < 6:
            return None
        try:
            return int(value[:2]) * 3600 + int(value[2:4]) * 60 + float(value[4:])
        except ValueError:
            return None

    def create_output_dir(self):
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"创建输出目录: {self.output_dir}")

    # ------------------------------------------------------------------
    # 文件解析: 必须按字节流处理，先恢复二进制OBSVM，再恢复ASCII
    # ------------------------------------------------------------------
    def _split_and_parse_stream(self):
        with open(self.input_file, 'rb') as stream:
            raw = stream.read()
        segments = []; start = 0; bad_obsvm = 0
        while True:
            pos = raw.find(b'\xaa\x44\xb5', start)
            if pos < 0:
                break
            record = None
            if pos + 28 <= len(raw):
                msg_id, length = struct.unpack_from('<HH', raw, pos + 4)
                end = pos + 24 + length
                if msg_id == 12 and end + 4 <= len(raw):
                    expected = struct.unpack_from('<I', raw, end)[0]
                    if expected == self._unicore_crc32(raw[pos:end]):
                        record = self._decode_obsvm(raw, pos, length, end)
                        segments.append((pos, end + 4))
                if record is None:
                    bad_obsvm += 1
            if record is None:
                start = pos + 3
            else:
                self.obsvm_records.append(record)
                start = end + 4
        # 拼接去除合法二进制块后的ASCII流
        chunks = []; cursor = 0
        for a, z in segments:
            chunks.append(raw[cursor:a]); cursor = z
        chunks.append(raw[cursor:])
        ascii_raw = b''.join(chunks)

        raw_line_count = len(raw.splitlines())
        ascii_line_count = 0; partial_lines = 0; embedded_rescued = 0
        recovered = defaultdict(list)
        # 按ASCII报文锚点恢复。二进制块可能切断ASCII，但本数据验证后无切断；
        # 保留该逻辑用于以后串口混合流。
        anchors = []
        for head in [b'#BESTNAVA', b'#SATSINFOA', b'#BESTSATA', b'#RTKSTATUSA',
                     b'$GNGGA', b'$GPGGA', b'$GNGSA', b'$GPGSA', b'$GNGST',
                     b'$GPGST', b'$GNGRS', b'$GPGRS',
                     b'$GPGSV', b'$GLGSV', b'$GBGSV', b'$GAGSV', b'$GQGSV']:
            pos = 0
            while True:
                k = ascii_raw.find(head, pos)
                if k < 0: break
                anchors.append((k, head)); pos = k + 1
        anchors.sort()
        ascii_anchor_candidates = len(anchors)
        for pos, head in anchors:
            star = ascii_raw.find(b'*', pos, pos + 15000)
            if star < 0: continue
            body = ascii_raw[pos+1:star]
            if head.startswith(b'#'):
                m = re.match(rb'[0-9a-fA-F]{8}', ascii_raw[star+1:star+9])
                if not m: continue
                if self._unicore_crc32(body) != int(m.group(0),16): continue
                line_b = ascii_raw[pos:star+9]
            else:
                m = re.match(rb'[0-9a-fA-F]{2}', ascii_raw[star+1:star+3])
                if not m: continue
                if self._nmea_xor(body) != int(m.group(0),16): continue
                line_b = ascii_raw[pos:star+3]
            try: line = line_b.decode('ascii')
            except UnicodeDecodeError: continue
            header = line.split(',')[0]
            recovered[header].append(line)
            embedded_rescued += 1
        for header, lines in recovered.items():
            self.parsed_data[header].extend(lines)
        # 逐行统计仅用于描述清洗后文本；partial按不可解码/非报文行统计
        for line in ascii_raw.splitlines():
            try: text = line.decode('ascii').strip()
            except UnicodeDecodeError:
                partial_lines += 1; continue
            if not text: continue
            ascii_line_count += 1
            if not (text.startswith('#') or text.startswith('$')):
                partial_lines += 1
        self.header_info = {
            'raw_line_count': raw_line_count,
            'total_lines': sum(len(v) for v in self.parsed_data.values()),
            'header_count': {k: len(v) for k,v in self.parsed_data.items()},
            'binary_blocks': len(segments),
            'binary_lines': len(raw.splitlines()) - len(ascii_raw.splitlines()),
            'garbage_lines': partial_lines,
            'ascii_lines': ascii_line_count,
            'embedded_rescued': embedded_rescued,
            'ascii_anchor_candidates': ascii_anchor_candidates,
            'ascii_invalid_candidates': max(0, ascii_anchor_candidates - embedded_rescued),
            'bad_obsvm_blocks': bad_obsvm,
        }
        return True

    def _decode_obsvm(self, raw: bytes, pos: int, length: int, end: int):
        cpu_idle, msg_id, _len, time_ref, time_status, week, ms, version, reserved, leap, delay = \
            struct.unpack_from('<BHHBBHIIBBH', raw, pos+3)
        obs_number = struct.unpack_from('<I', raw, pos+24)[0]
        if 4 + obs_number * 40 != length:
            raise ValueError('OBSVM长度与观测数不符')
        rows = []; off = pos + 28
        for _ in range(obs_number):
            sf, prn, psr, adr, psr_std, adr_std, dopp, cn0, reserved2, locktime, status = \
                self._OBSVM_RECORD.unpack_from(raw, off)
            rows.append({
                'system_freq': sf, 'prn': prn,
                'psr': psr, 'adr': adr,
                'psr_std': psr_std / 100.0, 'adr_std': adr_std / 10000.0,
                'doppler': dopp, 'cn0': cn0 / 100.0,
                'locktime': locktime / 32.0,
                'status': status,
                'system': (status >> 16) & 0x0f,
                'signal': (status >> 21) & 0x1f,
                'l2c_flag': (status >> 26) & 1,
                'phase_valid': bool(status & 0x00000400),
                'pseudorange_valid': bool(status & 0x00001000),
            })
            off += 40
        return {
            'offset': pos, 'cpu_idle': cpu_idle, 'message_id': msg_id,
            'time_ref': time_ref, 'time_status': time_status,
            'week': week, 'ms': ms, 'timestamp': week * 604800.0 + ms / 1000.0,
            'obs_number': obs_number, 'rows': rows,
        }

    def parse_dat_file(self):
        print(f"开始解析文件: {self.input_file}")
        try:
            self._split_and_parse_stream()
        except Exception as e:
            print(f"读取/解析文件失败: {e}")
            return False
        print(f"解析完成，找到 {len(self.parsed_data)} 种ASCII报文类型")
        for header,count in self.header_info['header_count'].items():
            print(f"  {header}: {count} 条")
        print(f"  OBSVMB二进制: {len(self.obsvm_records)} 个合法历元"
              f"(CRC失败块 {self.header_info['bad_obsvm_blocks']})")
        return True

    def save_header_files(self):
        for header, lines in self.parsed_data.items():
            safe = re.sub(r'[<>:"/\\|?*]', '_', header)
            path = os.path.join(self.output_dir, f'{safe}.dat')
            with open(path,'w',encoding='utf-8',newline='\n') as f:
                f.write('\n'.join(lines) + ('\n' if lines else ''))
        # OBSVM导出为人类可读CSV，不伪造原始二进制。
        if self.obsvm_records:
            path = os.path.join(self.output_dir,'OBSVMB_decoded.csv')
            with open(path,'w',encoding='utf-8-sig',newline='') as f:
                f.write('week,tow_s,obs_number,system,prn,signal,psr_m,psr_std_m,adr_cycles,adr_std_cycles,doppler_hz,cn0_dbhz,locktime_s,phase_valid,pseudorange_valid,status_hex\n')
                for r in self.obsvm_records:
                    for x in r['rows']:
                        f.write('%d,%.3f,%d,%d,%d,%d,%.6f,%.4f,%.6f,%.6f,%.4f,%.2f,%.5f,%d,%d,0x%08x\n' % (
                            r['week'], r['ms']/1000.0, r['obs_number'], x['system'], x['prn'], x['signal'],
                            x['psr'], x['psr_std'], x['adr'], x['adr_std'], x['doppler'], x['cn0'], x['locktime'],
                            int(x['phase_valid']), int(x['pseudorange_valid']), x['status']))
        print("分类报文与OBSVM解码表已保存")

    # ------------------------------------------------------------------
    # 完整性/周期
    # ------------------------------------------------------------------
    def _collect_category_lines(self, key):
        counts = self.header_info.get('header_count', {})
        if key == 'GGA': headers = [h for h in counts if h in self.NMEA_GROUPS['GGA']]
        elif key == 'GST': headers = [h for h in counts if h in self.NMEA_GROUPS['GST']]
        elif key == 'GRS': headers = [h for h in counts if h in self.NMEA_GROUPS['GRS']]
        elif key == 'GSA': headers = [h for h in counts if h.endswith('GSA')]
        elif key == 'GSV': headers = [h for h in counts if h.endswith('GSV')]
        elif key == 'OBSVMB':
            return ['OBSVMB'], []
        else: headers = ['#' + key]
        lines = []
        for h in headers: lines.extend(self.parsed_data.get(h, []))
        return headers, lines

    def _extract_category_timestamps(self, key, lines):
        out = []
        for line in lines:
            try:
                if line.startswith('#'):
                    hp = line.split(',',8); out.append(int(hp[5])/1000.0)
                elif line.startswith('$') and key in ('GGA','GST','GRS'):
                    out.append(self._nmea_seconds(line.split(',')[1]))
            except (ValueError,IndexError,TypeError): pass
        return sorted(x for x in out if x is not None)

    def check_message_integrity(self):
        print("完整性检查与报文周期核对(推荐参考周期，不将差异直接判错)...")
        checks=[]
        for display,key,reference_period,required in self.LOG_CONFIG:
            headers,lines=self._collect_category_lines(key)
            if key=='OBSVMB': count=len(self.obsvm_records)
            else: count=len(lines)
            e={'header':display,'matched':headers,'reference_period':reference_period,
               'count':count,'required':required,'status':'ok','note':'',
               'actual_period':None,'actual_rate':None,'loss_rate':None,'time_span':None,
               'gap_count':0,'max_gap':None,'estimated_missing_slots':0}
            if count==0:
                e['status']='error' if required else 'info'
                e['note']='报文缺失: 未在数据流中找到'
                checks.append(e); continue
            if key=='OBSVMB':
                ts=[r['timestamp'] for r in self.obsvm_records]
            else:
                ts=self._extract_category_timestamps(key,lines)
            if len(ts)>=2:
                iv=[b-a for a,b in zip(ts,ts[1:]) if b-a>1e-6]
                if iv:
                    med=statistics.median(iv); span=ts[-1]-ts[0]
                    expected=span/med+1 if med>0 else 0
                    loss=max(0.0,(1-len(ts)/expected)*100) if expected>0 else 0
                    gap_iv=[x for x in iv if x > med*1.5 + 1e-6]
                    missing_slots=sum(max(0, int(round(x/med))-1) for x in iv)
                    e.update(actual_period=med,actual_rate=1/med,time_span=span,
                             loss_rate=loss,gap_count=len(gap_iv),
                             max_gap=max(gap_iv) if gap_iv else None,
                             estimated_missing_slots=missing_slots)
                    if abs(med-reference_period)/reference_period>0.05:
                        # 周期差异只提示核对现场配置，不判错；用户可能特意设置输出周期。
                        e['note']=(e['note']+'; ' if e['note'] else '') + (
                            f"实测周期{med:.3f}s与推荐参考{reference_period:.3f}s不一致，"
                            "需结合现场配置确认")
                    if gap_iv:
                        gap_note=(f"按实测节律检测到{len(gap_iv)}个时间跳变/缺槽区间"
                                  f"(最大{max(gap_iv):.3f}s，估算缺{missing_slots}个槽位)")
                        e['note']=(e['note']+'; ' if e['note'] else '')+gap_note
                        if loss>1.0:
                            e['status']='warn'
                        elif e['status']=='ok':
                            e['status']='info'
                    if loss>1.0:
                        e['status']='warn' if e['status']!='error' else e['status']
                        e['note']=(e['note']+'; ' if e['note'] else '')+f"按实测节律估算丢帧率{loss:.1f}%"
                else: e['note']='时间戳间隔不足'
            else:
                e['note']='无可用时间戳，未计算周期；按实测条数呈现'
            checks.append(e)
            print(f"  {display}({','.join(headers) or 'binary'}): {count}条, "
                  + (f"实际{e['actual_period']:.3f}s/{e['actual_rate']:.2f}Hz, " if e['actual_period'] else '')
                  + f"状态{e['status']}" + (f" - {e['note']}" if e['note'] else ''))
        self.analysis_results['message_integrity']={
            'raw_line_count':self.header_info['raw_line_count'],
            'total_lines':self.header_info['total_lines'],
            'binary_blocks':self.header_info['binary_blocks'],
            'bad_obsvm_blocks':self.header_info['bad_obsvm_blocks'],
            'ascii_anchor_candidates':self.header_info.get('ascii_anchor_candidates',0),
            'ascii_invalid_candidates':self.header_info.get('ascii_invalid_candidates',0),
            'checks':checks,
        }

    # ------------------------------------------------------------------
    # 主定位源BESTNAVA
    # ------------------------------------------------------------------
    def parse_bestnava(self):
        if hasattr(self,'_cache_bestnava'): return self._cache_bestnava
        data=[]
        for line in self.parsed_data.get('#BESTNAVA',[]):
            if ';' not in line: continue
            try:
                h,d=line.split(';',1); hp=h.split(','); f=d.split('*')[0].split(',')
                if len(hp)<8 or len(f)<30: continue
                timestamp=int(hp[5])/1000.0; week=int(hp[4])
                pos_status=f[0].strip(); pos_type=f[1].strip()
                lat=float(f[2]); lon=float(f[3]); height=float(f[4])
                lat_sigma=float(f[7]); lon_sigma=float(f[8]); hgt_sigma=float(f[9])
                station=f[10].strip().strip('"')
                diff_age=float(f[11]) if f[11] else 0.0; sol_age=float(f[12]) if f[12] else 0.0
                num_svs=int(f[13]) if f[13].strip().isdigit() else 0
                num_soln=int(f[14]) if f[14].strip().isdigit() else 0
                vel_status=f[21].strip(); vel_type=f[22].strip()
                hor_spd=float(f[23]) if f[23] else 0.0
                trk=float(f[24]) if f[24] else 0.0
                vert_spd=float(f[25]) if f[25] else 0.0
                vert_sigma=float(f[26]) if f[26] else None
                hor_sigma=float(f[27]) if f[27] else None
                # BESTNAV速度为水平速度+真北航迹角，可分解为E/N。
                rad=math.radians(trk)
                valid_coord=(pos_status=='SOL_COMPUTED' and abs(lat)<=90 and abs(lon)<=180 and not (lat==0 and lon==0))
                data.append({'timestamp':timestamp,'week':week,
                    'latitude':lat,'longitude':lon,'height':height,
                    'valid_coord':valid_coord,'sol_status':pos_status,'pos_type':pos_type,
                    'solution_type':pos_type,'lat_sigma':lat_sigma,'lon_sigma':lon_sigma,
                    'hgt_sigma':hgt_sigma,'station_id':station,'diff_age':diff_age,'sol_age':sol_age,
                    'num_satellites':num_svs,'num_soln_satellites':num_soln,
                    'vel_status':vel_status,'vel_type':vel_type,'hor_spd':hor_spd,'vert_spd':vert_spd,'trk_gnd':trk,
                    'vert_sigma':vert_sigma,'hor_sigma':hor_sigma,
                    'velocity_east':hor_spd*math.sin(rad),'velocity_north':hor_spd*math.cos(rad),
                    'velocity_up':vert_spd})
            except (ValueError,IndexError): continue
        self._cache_bestnava=data; return data

    def parse_solution_status(self):
        return [{'timestamp':d['timestamp'],'sol_stat_str':d['sol_status'],'sol_stat':-1,
                 'pos_type':d['pos_type']} for d in self.parse_bestnava()]

    def analyze_sampling_rate(self):
        print("分析采样率...")
        data=self.parse_bestnava()
        if len(data)<2:
            print("BESTNAVA不足两帧，无法分析采样率"); return
        ts=sorted(d['timestamp'] for d in data)
        iv=[b-a for a,b in zip(ts,ts[1:]) if b-a>1e-6]
        if not iv: return
        med=statistics.median(iv); avg=statistics.mean(iv); gaps=[x for x in iv if x>med*5]
        expected=(ts[-1]-ts[0])/med if med>0 else 0
        loss=max(0.0,(1-len(data)/expected)*100) if expected>0 else 0
        self.analysis_results['sampling_rate']={
            'sampling_rate':1/med if med>0 else 0,'median_interval':med,
            'average_interval':avg,'avg_rate':1/avg if avg>0 else 0,
            'gap_count':len(gaps),'loss_rate':loss,'intervals':iv,
            'source':'#BESTNAVA GPS周秒'}
        print(f"BESTNAVA采样率: {1/med:.2f}Hz; 丢帧率估算 {loss:.1f}%")

    def analyze_solution_type(self):
        print("分析解类型分布...")
        data=self.parse_bestnava(); tc={}; sc={}
        for d in data:
            tc[d['pos_type']]=tc.get(d['pos_type'],0)+1
            sc[d['sol_status']]=sc.get(d['sol_status'],0)+1
        fixed=sum(v for k,v in tc.items() if criteria.is_fixed(k))
        self.analysis_results['gnss_solution_type']={
            'type_count':tc,'sol_status_count':sc,'fixed_ratio':fixed/len(data)*100 if data else 0,
            'total_epochs':len(data)}
        print(f"总历元 {len(data)}; 固定解 {fixed} ({fixed/len(data)*100 if data else 0:.1f}%)")
        for k,v in sorted(tc.items(),key=lambda x:-x[1]): print(f"  {k}: {v}")

    def analyze_satellite_visibility(self):
        print("分析卫星数量(跟踪/参与解算)...")
        data=self.parse_bestnava()
        if not data:
            print("无BESTNAVA卫星数量数据")
            return
        def st(vals): return (statistics.mean(vals),min(vals),max(vals)) if vals else (None,None,None)
        res={}
        for key,src in (('tracked','num_satellites'),('used','num_soln_satellites')):
            vals=[d[src] for d in data if d.get(src,0)>0]; a,mn,mx=st(vals)
            if a is not None: res[f'average_{key}']=a; res[f'min_{key}']=mn; res[f'max_{key}']=mx; res[f'{key}_counts']=vals
        self.analysis_results['satellite_visibility']=res

    def analyze_rtk_link(self):
        print("分析RTK链路...")
        data=[d for d in self.parse_bestnava() if d.get('station_id')]
        if not data:
            self.analysis_results['rtk_link']=None; print("无基站ID数据"); return
        ids=Counter(d['station_id'] for d in data)
        ages=[d['diff_age'] for d in data if d.get('diff_age',0)>0]
        self.analysis_results['rtk_link']={
            'station_ids':dict(ids),'primary_station':ids.most_common(1)[0][0],
            'avg_diff_age':statistics.mean(ages) if ages else 0,
            'max_diff_age':max(ages) if ages else 0,'check_points':len(data)}
        print(f"基站ID {ids.most_common(1)[0][0]}; 差分龄期最大 {max(ages) if ages else 0:.3f}s")

    # ------------------------------------------------------------------
    # NMEA解析
    # ------------------------------------------------------------------
    def _nmea_group(self, *keys):
        out=[]
        for k in keys: out.extend(self.parsed_data.get(k,[]))
        return out

    def parse_gga(self):
        if hasattr(self,'_cache_gga'): return self._cache_gga
        data=[]
        for line in self._nmea_group('$GNGGA','$GPGGA'):
            try:
                f=line.split('*')[0].split(',')
                if len(f)<15: continue
                ts=self._nmea_seconds(f[1])
                if ts is None: continue
                lat=float(f[2][:2])+float(f[2][2:])/60
                if f[3]=='S': lat=-lat
                lon=float(f[4][:3])+float(f[4][3:])/60
                if f[5]=='W': lon=-lon
                data.append({'timestamp':ts,'latitude':lat,'longitude':lon,
                    'fix_quality':int(f[6] or 0),'num_satellites':int(f[7] or 0),
                    'hdop':float(f[8] or 0),'height':float(f[9] or 0)})
            except (ValueError,IndexError): continue
        self._cache_gga=data; return data

    def parse_gngsa_combined(self):
        """按GSA组内SysID代表组合条，取整体DOP；不平均星座DOP。

        UM980实测每组GSA最后携带SysID（表7-8字段19）：1=GPS、2=GLONASS、
        3=GAL、4=BDS、5=QZSS。多星座组合时SysID=4的BDS条PRN最多，代表
        全部参与解算卫星的组合DOP。该规则与实测报文及华测版“取PRN最多
        组合条”的工程逻辑一致，但不把不同星座DOP取平均。
        """
        if hasattr(self,'_cache_gsa'): return self._cache_gsa
        recs=[]
        for line in self._nmea_group('$GNGSA','$GPGSA'):
            try:
                f=line.split('*')[0].split(',')
                if len(f)<18: continue
                n=sum(1 for x in f[3:15] if x.strip())
                # SysID在DOP后；兼容空字段和CRC残留。
                sys_id=None
                if len(f)>=19:
                    v=f[18].strip()
                    if v:
                        sys_id=int(v,16) if re.fullmatch(r'[0-9A-Fa-f]+',v) else int(v)
                recs.append((n,float(f[15] or 0),float(f[16] or 0),
                             float(f[17] or 0),sys_id))
            except (ValueError,IndexError): continue

        combined=[]; group=[]
        for rec in recs:
            # SysID回到组内较小值时切组；SysID缺失时保持当前组。
            if group and rec[4] is not None and group and group[0][4] is not None and rec[4] < group[-1][4]:
                combined.append(max(group,key=lambda x:x[0]))
                group=[]
            group.append(rec)
        if group: combined.append(max(group,key=lambda x:x[0]))
        self._cache_gsa=[{'pdop':p,'hdop':h,'vdop':v,'n_prn':n,'sys_id':sys_id}
                         for n,p,h,v,sys_id in combined]
        return self._cache_gsa

    def parse_gst(self):
        if hasattr(self,'_cache_gst'): return self._cache_gst
        data=[]
        for line in self._nmea_group('$GNGST','$GPGST'):
            try:
                f=line.split('*')[0].split(',')
                if len(f)<9: continue
                ts=self._nmea_seconds(f[1])
                data.append({'timestamp':ts,'utc':f[1],
                    'rms':float(f[2] or 'nan'),'smjr':float(f[3] or 'nan'),
                    'smnr':float(f[4] or 'nan'),'orient':float(f[5] or 'nan'),
                    'lat_std':float(f[6] or 'nan'),'lon_std':float(f[7] or 'nan'),
                    'alt_std':float(f[8] or 'nan')})
            except (ValueError,IndexError): continue
        self._cache_gst=data; return data

    def parse_grs(self):
        """解析GNGRS逐星残差。同UTC多条对应不同星座/信号，不能合并成一条。"""
        if hasattr(self,'_cache_grs'): return self._cache_grs
        data=[]
        for line in self._nmea_group('$GNGRS','$GPGRS'):
            try:
                f=line.split('*')[0].split(',')
                if len(f)<17: continue
                ts=self._nmea_seconds(f[1])
                residuals=[]
                for x in f[3:15]:
                    if x: residuals.append(float(x))
                data.append({'timestamp':ts,'mode':f[2],'residuals':residuals,
                    'system_id':f[15] if len(f)>15 else '', 'signal_id':f[16].split('*')[0] if len(f)>16 else ''})
            except (ValueError,IndexError): continue
        self._cache_grs=data; return data

    def analyze_dop(self):
        print("分析DOP...")
        gsa=self.parse_gngsa_combined(); gga=self.parse_gga(); res={}
        def st(vals): return (statistics.mean(vals),min(vals),max(vals)) if vals else (None,None,None)
        if gsa:
            res['dop_source']='$GNGSA/$GPGSA组合条(SysID组内PRN槽最多)'
            for key in ('pdop','hdop','vdop'):
                vals=[d[key] for d in gsa if d.get(key,0)>0]; a,mn,mx=st(vals)
                if a is not None: res[f'average_{key}']=a; res[f'min_{key}']=mn; res[f'max_{key}']=mx; res[f'{key}_values']=vals
            res['timestamps']=list(range(len(gsa)))
        elif gga:
            res['dop_source']='$GNGGA(仅HDOP)'
            vals=[d['hdop'] for d in gga if d['hdop']>0]; a,mn,mx=st(vals)
            if a is not None: res['average_hdop']=a; res['min_hdop']=mn; res['max_hdop']=mx; res['hdop_values']=vals
            res['timestamps']=[d['timestamp'] for d in gga]
        else: print("未找到DOP数据"); return
        self.analysis_results['dop']=res
        for k in ('pdop','hdop','vdop'):
            if f'average_{k}' in res: print(f"平均{k.upper()}: {res[f'average_{k}']:.3f}")

    def analyze_pos_quality(self):
        print("分析位置σ与伪距残差RMS...")
        pos=[d for d in self.parse_bestnava() if d['valid_coord']]
        gst=self.parse_gst(); grs=self.parse_grs()
        res={'pos_src':'#BESTNAVA','gst_src':'$GNGST/$GPGST','grs_src':'$GNGRS/$GPGRS',
             'pos_epochs':len(pos),'gst_epochs':len(gst),'grs_records':len(grs)}
        def st(vals): return (statistics.mean(vals),statistics.median(vals),min(vals),max(vals)) if vals else (None,)*4
        for key,name in (('lat_sigma','lat'),('lon_sigma','lon'),('hgt_sigma','hgt')):
            vals=[d[key] for d in pos if d.get(key,0)>0]; a,med,mn,mx=st(vals)
            if a is not None: res[f'{name}_mean']=a; res[f'{name}_med']=med; res[f'{name}_min']=mn; res[f'{name}_max']=mx
        if pos:
            t0=pos[0]['timestamp']; res['pos_t']=[d['timestamp']-t0 for d in pos]
            res['pos_lat']=[d['lat_sigma'] for d in pos]; res['pos_lon']=[d['lon_sigma'] for d in pos]; res['pos_hgt']=[d['hgt_sigma'] for d in pos]
        if gst:
            t0=gst[0]['timestamp']; res['gst_t']=[d['timestamp']-t0 for d in gst]; res['gst_rms']=[d['rms'] for d in gst]
            a,med,mn,mx=st(res['gst_rms']); res.update(rms_mean=a,rms_med=med,rms_min=mn,rms_max=mx)
        if grs:
            all_res=[x for d in grs for x in d['residuals'] if x is not None]
            a,med,mn,mx=st(all_res); res.update(grs_mean=a,grs_med=med,grs_min=mn,grs_max=mx,grs_values=all_res)
        self.analysis_results['pos_quality']=res

    def analyze_velocity(self):
        print("分析速度...")
        data=[d for d in self.parse_bestnava() if d.get('vel_status')=='SOL_COMPUTED']
        if not data:
            print("无SOL_COMPUTED速度数据"); return
        speeds=[math.sqrt(d['velocity_east']**2+d['velocity_north']**2+d['velocity_up']**2) for d in data]
        self.analysis_results['velocity']={'average_speed':statistics.mean(speeds),'max_speed':max(speeds),'speeds':speeds,'source':'#BESTNAVA速度段'}
        print(f"平均速度 {statistics.mean(speeds):.3f} m/s; 最大 {max(speeds):.3f} m/s")

    # ------------------------------------------------------------------
    # UM980特色: RTKSTATUS / BESTSAT / SATSINFO / OBSVM
    # ------------------------------------------------------------------
    def parse_rtkstatus(self):
        if hasattr(self,'_cache_rtkstatus'): return self._cache_rtkstatus
        data=[]
        for line in self.parsed_data.get('#RTKSTATUSA',[]):
            if ';' not in line: continue
            try:
                h,d=line.split(';',1); hp=h.split(','); f=d.split('*')[0].split(',')
                if len(f)<16: continue
                def u32(x): return int(x,16) if x else 0
                status_map={0:'无差分源数据输入',1:'差分源可用观测值不足',2:'差分源延迟较大',
                            3:'电离层活跃',4:'ROVER端观测值不足',5:'满足RTK解算'}
                calc=int(f[12]) if f[12] else 0
                data.append({'timestamp':int(hp[5])/1000.0,'gps_source':u32(f[0]),
                    'bds_source1':u32(f[2]),'bds_source2':u32(f[3]),'glo_source':u32(f[5]),
                    'gal_source1':u32(f[7]),'gal_source2':u32(f[8]),'qzss_source':u32(f[9]),
                    'pos_type':f[11],'calc_status':calc,'calc_status_text':status_map.get(calc,'未知'),
                    'ion_detected':int(f[13]) if f[13] else 0,
                    'adr_number':int(f[15]) if f[15] else 0})
            except (ValueError,IndexError): continue
        self._cache_rtkstatus=data; return data

    def analyze_rtk_status(self):
        print("分析RTKSTATUS...")
        data=self.parse_rtkstatus()
        if not data:
            self.analysis_results['rtk_status']={'available':False}; return
        cnt=Counter(d['calc_status_text'] for d in data)
        poscnt=Counter(d['pos_type'] for d in data)
        ion=[d['ion_detected'] for d in data]
        adr=[d['adr_number'] for d in data]
        self.analysis_results['rtk_status']={'available':True,'epochs':len(data),
            'calc_status_count':dict(cnt),'pos_type_count':dict(poscnt),
            'ion_detected_min':min(ion),'ion_detected_max':max(ion),
            'adr_number_min':min(adr),'adr_number_max':max(adr),'records':data}
        print('RTK计算状态:',dict(cnt))

    def parse_bestsat(self):
        if hasattr(self,'_cache_bestsat'): return self._cache_bestsat
        data=[]
        for line in self.parsed_data.get('#BESTSATA',[]):
            if ';' not in line: continue
            try:
                h,d=line.split(';',1); hp=h.split(','); f=d.split('*')[0].split(',')
                n=int(f[0]); rows=[]
                idx=1
                for _ in range(n):
                    if idx+4>len(f): break
                    rows.append({'system':f[idx],'prn':f[idx+1],'status':f[idx+2],'signal_mask':f[idx+3]})
                    idx+=4
                data.append({'timestamp':int(hp[5])/1000.0,'rows':rows})
            except (ValueError,IndexError): continue
        self._cache_bestsat=data; return data

    def analyze_bestsat(self):
        print("分析BESTSAT参与解算卫星...")
        data=self.parse_bestsat()
        if not data:
            self.analysis_results['bestsat']={'available':False}; return
        systems=Counter(r['system'] for d in data for r in d['rows'])
        masks=Counter(r['signal_mask'] for d in data for r in d['rows'])
        self.analysis_results['bestsat']={'available':True,'epochs':len(data),
            'system_count':dict(systems),'mask_count':dict(masks),'records':data}
        print('参与解算卫星系统:',dict(systems))

    def parse_satsinfo(self):
        """仅收集SATSINFOA原始行，不解析变长频率段。"""
        if hasattr(self,'_cache_satsinfo'): return self._cache_satsinfo
        data=[{'raw':line} for line in self.parsed_data.get('#SATSINFOA',[])]
        self._cache_satsinfo=data; return data

    def analyze_satsinfo_presence(self):
        """仅统计SATSINFOA存在性和周期，不做变长频率字段定量解析。"""
        print("检查SATSINFOA存在性与周期...")
        data=self.parse_satsinfo()
        self.analysis_results['satsinfo']={'available':bool(data),'epochs':len(data),
            'note':'ASCII变长频率段字段切分规则不足以无歧义确认，避免猜测，不做天空分布/SNR统计'}
        if data:
            print(f"SATSINFOA: {len(data)}个历元；为避免字段猜测，未做定量解析")

    def analyze_obsvm(self):
        print("分析OBSVM C/N0、locktime与失锁/重捕获...")
        if not self.obsvm_records:
            self.analysis_results['obs']={'available':False}; return
        all_cn0=[]; per_epoch=[]; loss_events=[]; reacq_events=[]; prev={}
        for r in self.obsvm_records:
            cn0s=[x['cn0'] for x in r['rows'] if x['cn0']>0]
            per_epoch.append({'timestamp':r['timestamp'],'mean':statistics.mean(cn0s) if cn0s else None,'n':len(cn0s)})
            all_cn0.extend(cn0s)
            current={}
            for x in r['rows']:
                key=(x['system'],x['prn'],x['signal'])
                current[key]=x
                if key in prev:
                    if x['locktime']==0 and prev[key]['locktime']>0:
                        loss_events.append({'timestamp':r['timestamp'],'system':key[0],'prn':key[1],'signal':key[2]})
                    if prev[key]['locktime']==0 and x['locktime']>0:
                        reacq_events.append({'timestamp':r['timestamp'],'system':key[0],'prn':key[1],'signal':key[2]})
                prev[key]=x
            for key,x in prev.items():
                if key not in current:
                    loss_events.append({'timestamp':r['timestamp'],'system':key[0],'prn':key[1],'signal':key[2]})
            prev=current
        low=sum(1 for x in all_cn0 if x<35)
        res={'available':True,'epochs':len(self.obsvm_records),'obs_total':len(all_cn0),
             'mean_cn0':statistics.mean(all_cn0) if all_cn0 else None,
             'median_cn0':statistics.median(all_cn0) if all_cn0 else None,
             'min_cn0':min(all_cn0) if all_cn0 else None,'max_cn0':max(all_cn0) if all_cn0 else None,
             'low_ratio':low/len(all_cn0)*100 if all_cn0 else 0,
             'loss_events':loss_events,'reacq_events':reacq_events,'per_epoch':per_epoch,
             'records':self.obsvm_records}
        self.analysis_results['obs']=res
        print(f"OBSVM: {len(self.obsvm_records)}历元/{len(all_cn0)}频点观测, 平均C/N0 {res['mean_cn0']:.2f}dB-Hz")
        print(f"locktime归零/消失事件 {len(loss_events)}; 重捕获事件 {len(reacq_events)}")

    # ------------------------------------------------------------------
    # 绘图
    # ------------------------------------------------------------------
    def generate_plots(self):
        print("生成分析图表...")
        data=self.parse_bestnava()
        if data: self._plot_position_time_series(data,'gnss')
        if 'gnss_solution_type' in self.analysis_results: self._plot_gnss_solution_type_distribution()
        if 'satellite_visibility' in self.analysis_results and self.parse_bestnava(): self._plot_satellite_time_series()
        if 'dop' in self.analysis_results: self._plot_dop_time_series()
        if 'pos_quality' in self.analysis_results: self._plot_pos_quality()
        if 'velocity' in self.analysis_results: self._plot_velocity_time_series()
        if data: self._plot_gnss_enu_trajectory(data)
        if data: self._plot_solution_status(self.parse_solution_status())
        if self.analysis_results.get('obs',{}).get('available'): self._plot_cn0()
        # SATSINFO ASCII变长频率段字段顺序在当前PDF表格中不足以无歧义切分；
        # 为避免猜测，不生成天空分布图。
        if self.analysis_results.get('rtk_status',{}).get('available'): self._plot_rtk_status()

    def _plot_position_time_series(self,data,source='gnss'):
        data=[d for d in data if d.get('valid_coord')]
        if not data: return
        t0=min(d['timestamp'] for d in data); x=[d['timestamp']-t0 for d in data]
        plt.figure(figsize=(12,8))
        for i,(key,title,ylabel) in enumerate((('latitude','Latitude Time Series','degrees'),
            ('longitude','Longitude Time Series','degrees'),('height','Height Time Series','meters')),1):
            plt.subplot(3,1,i); plt.plot(x,[d[key] for d in data],linewidth=0.8)
            plt.title(title); plt.xlabel('Time (seconds, rel. BESTNAV epoch0)'); plt.ylabel(ylabel); plt.grid(alpha=.3)
        plt.tight_layout(); out=os.path.join(self.output_dir,'position_time_series_gnss.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _plot_gnss_solution_type_distribution(self):
        st=self.analysis_results['gnss_solution_type']; items=sorted(st['type_count'].items(),key=lambda x:-x[1])
        if not items: return
        labels=[x[0] for x in items]; vals=[x[1] for x in items]
        plt.figure(figsize=(10,6)); bars=plt.bar(labels,vals,color=[criteria.type_color(x) for x in labels])
        for b,v in zip(bars,vals): plt.text(b.get_x()+b.get_width()/2,b.get_height(),f'{v}\n{v/st["total_epochs"]*100:.1f}%',ha='center',va='bottom',fontsize=9)
        plt.title('GNSS Position Type Distribution'); plt.ylabel('Count'); plt.xticks(rotation=45,ha='right'); plt.tight_layout()
        out=os.path.join(self.output_dir,'gnss_solution_type_distribution.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _plot_satellite_time_series(self):
        data=self.parse_bestnava()
        if not data: return
        t0=min(d['timestamp'] for d in data); x=[d['timestamp']-t0 for d in data]
        plt.figure(figsize=(12,6)); plt.plot(x,[d['num_satellites'] for d in data],label='Tracked (#SVs)',color='#1f77b4')
        plt.plot(x,[d['num_soln_satellites'] for d in data],label='Used (#solnSVs)',color='#d62728')
        plt.title('Satellite Count Time Series (BESTNAV)'); plt.xlabel('Time (seconds)'); plt.ylabel('Number'); plt.legend(); plt.grid(alpha=.3); plt.tight_layout()
        out=os.path.join(self.output_dir,'satellite_time_series.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _plot_dop_time_series(self):
        dop=self.analysis_results['dop']; vals=dop.get('pdop_values') or dop.get('hdop_values')
        if not vals: return
        x=dop.get('timestamps',range(len(vals))); plt.figure(figsize=(12,6))
        for key,color in (('pdop','#1f77b4'),('hdop','#2ca02c'),('vdop','#d62728')):
            if dop.get(key+'_values'): plt.plot(x[:len(dop[key+'_values'])],dop[key+'_values'],label=key.upper(),color=color)
        plt.title(f'DOP Time Series ({dop["dop_source"]})'); plt.xlabel('Epoch index' if isinstance(x[0],int) else 'Time (seconds)')
        plt.ylabel('DOP'); plt.legend(); plt.grid(alpha=.3); plt.tight_layout()
        out=os.path.join(self.output_dir,'dop_time_series.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _plot_velocity_time_series(self):
        data=[d for d in self.parse_bestnava() if d['vel_status']=='SOL_COMPUTED']
        if not data: return
        t0=min(d['timestamp'] for d in data); x=[d['timestamp']-t0 for d in data]
        fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
        for ax,key,title,color in zip(axes,('velocity_north','velocity_east','velocity_up',None),
            ('North Velocity','East Velocity','Up Velocity','Combined Speed'),
            ('#1f77b4','#2ca02c','#d62728','#9467bd')):
            if key: ax.plot(x,[d[key] for d in data],color=color,linewidth=.8)
            else: ax.plot(x,[math.sqrt(d['velocity_north']**2+d['velocity_east']**2+d['velocity_up']**2) for d in data],color=color,linewidth=.8)
            ax.set_title(title); ax.grid(alpha=.3); ax.set_ylabel('m/s')
        axes[-1].set_xlabel('Time (seconds)'); fig.tight_layout()
        out=os.path.join(self.output_dir,'velocity_time_series.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _latlon_to_enu(self,lat,lon,height,ref_lat,ref_lon,ref_height):
        a=6378137.0; f=1/298.257223563; e2=2*f-f*f
        def xyz(lat,lon,h):
            p=math.radians(lat); l=math.radians(lon); N=a/math.sqrt(1-e2*math.sin(p)**2)
            return ((N+h)*math.cos(p)*math.cos(l),(N+h)*math.cos(p)*math.sin(l),(N*(1-e2)+h)*math.sin(p))
        xr,yr,zr=xyz(ref_lat,ref_lon,ref_height); x,y,z=xyz(lat,lon,height)
        p=math.radians(ref_lat); l=math.radians(ref_lon); dx,dy,dz=x-xr,y-yr,z-zr
        return (-math.sin(l)*dx+math.cos(l)*dy,
                -math.sin(p)*math.cos(l)*dx-math.sin(p)*math.sin(l)*dy+math.cos(p)*dz,
                math.cos(p)*math.cos(l)*dx+math.cos(p)*math.sin(l)*dy+math.sin(p)*dz)

    def _plot_gnss_enu_trajectory(self,data):
        data=[d for d in data if d.get('valid_coord')]
        if len(data)<2: return
        ref=data[0]; enu=[]
        for d in data:
            E,N,U=self._latlon_to_enu(d['latitude'],d['longitude'],d['height'],ref['latitude'],ref['longitude'],ref['height'])
            enu.append({'E':E,'N':N,'U':U,'solution_type':d['solution_type']})
        present=sorted(set(x['solution_type'] for x in enu))
        plt.figure(figsize=(14,10))
        for a,b in zip(enu[:-1],enu[1:]): plt.plot([a['E'],b['E']],[a['N'],b['N']],c=criteria.type_color(a['solution_type']),linewidth=1.2,alpha=.8)
        for t in present:
            pts=[x for x in enu if x['solution_type']==t]; plt.scatter([x['E'] for x in pts],[x['N'] for x in pts],c=criteria.type_color(t),s=12,label=criteria.legend_label(t,len(pts)))
        plt.xlabel('Easting (m)'); plt.ylabel('Northing (m)'); plt.title('GNSS Trajectory in ENU Coordinate (Colored by Position Type)')
        plt.legend(); plt.grid(alpha=.3); plt.axis('equal'); plt.tight_layout()
        out=os.path.join(self.output_dir,'gnss_enu_trajectory.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _plot_pos_quality(self):
        res=self.analysis_results['pos_quality']; fig,axes=plt.subplots(2,2,figsize=(14,8))
        panels=[(axes[0][0],res.get('pos_t'),res.get('pos_lat'),'Latitude sigma (m)','#1f77b4'),
                (axes[0][1],res.get('pos_t'),res.get('pos_lon'),'Longitude sigma (m)','#2ca02c'),
                (axes[1][0],res.get('pos_t'),res.get('pos_hgt'),'Height sigma (m)','#d62728')]
        for ax,x,y,title,color in panels:
            if x and y:
                ax.plot(x,y,color=color,linewidth=.8); vals=[v for v in y if v and v>0]
                if vals: ax.axhline(statistics.median(vals),color=color,ls='--',alpha=.6,label=f'median {statistics.median(vals):.4f} m'); ax.legend(fontsize=8)
                ax.set_xlabel('Time (s, rel. BESTNAV epoch0)')
            else: ax.text(.5,.5,'No data',ha='center',va='center',transform=ax.transAxes)
            ax.set_title(title); ax.set_ylabel('sigma (m)'); ax.grid(alpha=.3)
        ax=axes[1][1]
        if res.get('gst_t') and res.get('gst_rms'):
            ax.plot(res['gst_t'],res['gst_rms'],color='#9467bd',linewidth=.8)
            ax.axhline(statistics.median(res['gst_rms']),color='#9467bd',ls='--',alpha=.6,label=f'median {statistics.median(res["gst_rms"]):.3f} m'); ax.legend(fontsize=8)
            ax.set_xlabel('Time (s, rel. GST epoch0)')
        else: ax.text(.5,.5,'No GST data',ha='center',va='center',transform=ax.transAxes)
        ax.set_title('Pseudorange Residual RMS (m, GST field3)'); ax.set_ylabel('RMS (m)'); ax.grid(alpha=.3)
        fig.suptitle('Position Sigma & Pseudorange Residual RMS (BESTNAV / GST, unit: m)'); fig.tight_layout(rect=[0,0,1,.96])
        out=os.path.join(self.output_dir,'pos_sigma_residual_rms.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _plot_solution_status(self,data):
        if not data: return
        t0=min(d['timestamp'] for d in data); x=[d['timestamp']-t0 for d in data]
        states=sorted(set(d['sol_stat_str'] for d in data)); y={s:i+1 for i,s in enumerate(states)}
        plt.figure(figsize=(14,6))
        for s in states:
            pts=[(t,y[s]) for t,d in zip(x,data) if d['sol_stat_str']==s]; plt.scatter([p[0] for p in pts],[p[1] for p in pts],s=20,label=s)
        plt.yticks(list(y.values()),list(y.keys())); plt.xlabel('Time (seconds)'); plt.title('BESTNAV Solution Status Time Series'); plt.grid(alpha=.3,axis='x'); plt.legend()
        plt.tight_layout(); out=os.path.join(self.output_dir,'solution_status.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)

    def _plot_cn0(self):
        obs=self.analysis_results['obs']; cn0=[x['cn0'] for r in obs['records'] for x in r['rows'] if x['cn0']>0]
        fig,axes=plt.subplots(1,2,figsize=(14,5))
        axes[0].hist(cn0,bins=40,color='#1f77b4',alpha=.8); axes[0].axvline(35,color='#d62728',ls='--',label='35 dB-Hz'); axes[0].set_xlabel('C/N0 (dB-Hz)'); axes[0].set_ylabel('Frequency'); axes[0].set_title('OBSVM C/N0 Distribution'); axes[0].legend(); axes[0].grid(alpha=.3)
        ep=[e for e in obs['per_epoch'] if e['mean'] is not None]
        if ep:
            t0=ep[0]['timestamp']; axes[1].plot([e['timestamp']-t0 for e in ep],[e['mean'] for e in ep],color='#2ca02c',label='mean C/N0'); axes[1].set_xlabel('Time (s, rel. OBSVM epoch0)'); axes[1].set_ylabel('C/N0 (dB-Hz)'); axes[1].set_title('OBSVM Mean C/N0 Time Series'); axes[1].grid(alpha=.3); axes[1].legend()
        fig.tight_layout(); out=os.path.join(self.output_dir,'cn0_analysis.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)


    def _plot_rtk_status(self):
        rec=self.analysis_results['rtk_status']['records']; t0=rec[0]['timestamp']; x=[d['timestamp']-t0 for d in rec]
        states=sorted(set(d['calc_status_text'] for d in rec)); y={s:i+1 for i,s in enumerate(states)}
        plt.figure(figsize=(14,5))
        for s in states:
            pts=[(t,y[s]) for t,d in zip(x,rec) if d['calc_status_text']==s]; plt.scatter([p[0] for p in pts],[p[1] for p in pts],s=20,label=s)
        plt.yticks(list(y.values()),list(y.keys())); plt.xlabel('Time (s, rel. RTKSTATUS epoch0)'); plt.title('RTKSTATUS Calculate Status Time Series'); plt.grid(alpha=.3,axis='x'); plt.legend(fontsize=8); plt.tight_layout()
        out=os.path.join(self.output_dir,'rtk_status_timeline.png'); plt.savefig(out,dpi=150); plt.close(); print('保存',out)
    # ------------------------------------------------------------------
    # 报告
    # ------------------------------------------------------------------
    def _safe(self,x):
        if x is None: return '-'
        if isinstance(x,float): return f'{x:.3f}'
        return str(x)

    def generate_html_report(self):
        print("生成HTML报告...")
        mi=self.analysis_results.get('message_integrity',{})
        st=self.analysis_results.get('gnss_solution_type',{})
        sv=self.analysis_results.get('satellite_visibility',{})
        dop=self.analysis_results.get('dop',{})
        pq=self.analysis_results.get('pos_quality',{})
        vel=self.analysis_results.get('velocity',{})
        rtk=self.analysis_results.get('rtk_link')
        rts=self.analysis_results.get('rtk_status',{})
        obs=self.analysis_results.get('obs',{})
        si=self.analysis_results.get('satsinfo',{})
        bs=self.analysis_results.get('bestsat',{})
        css='''body{font-family:Arial,"Microsoft YaHei",sans-serif;line-height:1.6;background:#f5f5f5;margin:0;padding:20px}.container{max-width:1250px;margin:auto;background:#fff;padding:30px;border-radius:10px;box-shadow:0 2px 10px #0002}h1,h2{color:#2c3e50;border-bottom:3px solid #2c3e50;padding-bottom:8px}h2{border-bottom:2px solid #3498db}table{width:100%;border-collapse:collapse;margin:15px 0}th{background:#3498db;color:#fff}th,td{padding:8px;border:1px solid #ddd;text-align:left;vertical-align:top}.pass{color:green;font-weight:bold}.warn{color:#e67e22;font-weight:bold}.fail{color:#c0392b;font-weight:bold}.chart{margin:15px 0}.chart img{width:100%;border:1px solid #ddd}.note{background:#eef6fb;border-left:4px solid #3498db;padding:10px;margin:10px 0}small{color:#666}'''
        rows=''
        for x in mi.get('checks',[]):
            cls={'ok':'pass','info':'info','warn':'warn','error':'fail'}.get(x['status'],'warn')
            rows+=f'<tr><td>{x["header"]}</td><td>{",".join(x["matched"]) or "OBSVMB"}</td><td>{x["count"]}</td><td>{x["reference_period"]:.3f}</td><td>{self._safe(x["actual_period"])}</td><td>{self._safe(x["actual_rate"])}</td><td>{x["gap_count"]}</td><td>{self._safe(x["max_gap"])}</td><td>{x["estimated_missing_slots"]}</td><td class="{cls}">{x["status"]}</td><td><small>{x["note"] or "-"}</small></td></tr>'
        solrows=''.join(f'<tr><td>{k}</td><td>{v}</td><td>{v/st.get("total_epochs",1)*100:.1f}%</td></tr>' for k,v in sorted(st.get('type_count',{}).items(),key=lambda x:-x[1]))
        gsarows=''.join(f'<tr><td>{k}</td><td>{v}</td><td>{v/st.get("total_epochs",1)*100:.1f}%</td></tr>' for k,v in sorted(st.get('sol_status_count',{}).items(),key=lambda x:-x[1]))
        html=f'''<!DOCTYPE html><html lang="zh-CN"><head><meta charset="UTF-8"><title>UM980 GNSS Kinematic分析报告 - {self.filename}</title><style>{css}</style></head><body><div class="container">
<h1>UM980 / 北斗星通 GNSS Kinematic分析报告</h1>
<p><strong>输入:</strong>{os.path.basename(self.input_file)} | <strong>生成时间:</strong>{datetime.datetime.now():%Y-%m-%d %H:%M:%S} | <strong>协议依据:</strong>N4 Products Commands and Logs Reference Book R1.15</p>
<div class="note">UM980为无IMU、单天线GNSS模组。本报告只分析实测报文；位置σ和伪距残差是接收机自估质量指标，不是外业真值误差。</div>
<h2>1. 数据概览</h2><table><tr><th>项目</th><th>结果</th></tr>
<tr><td>ASCII合法报文</td><td>{self.header_info.get('total_lines',0)}条</td></tr>
<tr><td>原始行数</td><td>{self.header_info.get('raw_line_count',0)}行</td></tr>
<tr><td>OBSVMB合法二进制历元</td><td>{self.header_info.get('binary_blocks',0)}个；CRC失败块{self.header_info.get('bad_obsvm_blocks',0)}个</td></tr>
<tr><td>ASCII校验通过报文 / 锚点候选</td><td>{self.header_info.get('total_lines',0)}条 / {self.header_info.get('ascii_anchor_candidates',0)}个；校验失败锚点{self.header_info.get('ascii_invalid_candidates',0)}个</td></tr>
<tr><td>ASCII残片/不可识别行</td><td>{self.header_info.get('garbage_lines',0)}行</td></tr>
<tr><td>BESTNAVA时间跨度</td><td>{(max(d["timestamp"] for d in self.parse_bestnava())-min(d["timestamp"] for d in self.parse_bestnava())) if self.parse_bestnava() else 0}s</td></tr></table>
<h2>2. 完整性与周期核对</h2><div class="note">“参考周期”来自推荐配置，仅作对照。实测周期与参考周期不一致时不判为错误，需结合现场配置确认；完整性告警只基于实测时间戳节律的缺槽/跳变和校验结果。</div><table><tr><th>报文</th><th>实测头</th><th>条数</th><th>参考周期s</th><th>实际周期s</th><th>实际Hz</th><th>跳变数</th><th>最大跳变s</th><th>估算缺槽</th><th>状态</th><th>备注</th></tr>{rows}</table>
<h2>3. 解类型</h2><div class="chart"><img src="gnss_solution_type_distribution.png"></div><p><strong>固定解率:</strong><span class="{'pass' if st.get('fixed_ratio',0)>95 else 'warn' if st.get('fixed_ratio',0)>80 else 'fail'}">{st.get('fixed_ratio',0):.1f}%</span></p><table><tr><th>Pos Type</th><th>数量</th><th>占比</th></tr>{solrows}</table><h3>Sol Status</h3><table><tr><th>Sol Status</th><th>数量</th><th>占比</th></tr>{gsarows}</table>
<h2>4. 卫星与DOP</h2><div class="chart"><img src="satellite_time_series.png"></div><div class="chart"><img src="dop_time_series.png"></div><table><tr><th>指标</th><th>均值</th><th>最小</th><th>最大</th></tr>
<tr><td>跟踪卫星 #SVs</td><td>{sv.get('average_tracked','-')}</td><td>{sv.get('min_tracked','-')}</td><td>{sv.get('max_tracked','-')}</td></tr>
<tr><td>参与解算 #solnSVs</td><td>{sv.get('average_used','-')}</td><td>{sv.get('min_used','-')}</td><td>{sv.get('max_used','-')}</td></tr>
<tr><td>PDOP</td><td>{self._safe(dop.get('average_pdop'))}</td><td>{self._safe(dop.get('min_pdop'))}</td><td>{self._safe(dop.get('max_pdop'))}</td></tr>
<tr><td>HDOP</td><td>{self._safe(dop.get('average_hdop'))}</td><td>{self._safe(dop.get('min_hdop'))}</td><td>{self._safe(dop.get('max_hdop'))}</td></tr>
<tr><td>VDOP</td><td>{self._safe(dop.get('average_vdop'))}</td><td>{self._safe(dop.get('min_vdop'))}</td><td>{self._safe(dop.get('max_vdop'))}</td></tr></table><p><small>DOP源：{dop.get('dop_source','-')}。GSA无UTC时间戳，图横轴为组合条序号，不伪装成精确时间轴。</small></p>
<h2>5. 位置σ与伪距残差</h2><div class="chart"><img src="pos_sigma_residual_rms.png"></div><table><tr><th>指标</th><th>均值</th><th>中位</th><th>最小</th><th>最大</th></tr>
<tr><td>纬度σ m</td><td>{self._safe(pq.get('lat_mean'))}</td><td>{self._safe(pq.get('lat_med'))}</td><td>{self._safe(pq.get('lat_min'))}</td><td>{self._safe(pq.get('lat_max'))}</td></tr>
<tr><td>经度σ m</td><td>{self._safe(pq.get('lon_mean'))}</td><td>{self._safe(pq.get('lon_med'))}</td><td>{self._safe(pq.get('lon_min'))}</td><td>{self._safe(pq.get('lon_max'))}</td></tr>
<tr><td>高度σ m</td><td>{self._safe(pq.get('hgt_mean'))}</td><td>{self._safe(pq.get('hgt_med'))}</td><td>{self._safe(pq.get('hgt_min'))}</td><td>{self._safe(pq.get('hgt_max'))}</td></tr>
<tr><td>GST伪距残差RMS m</td><td>{self._safe(pq.get('rms_mean'))}</td><td>{self._safe(pq.get('rms_med'))}</td><td>{self._safe(pq.get('rms_min'))}</td><td>{self._safe(pq.get('rms_max'))}</td></tr>
<tr><td>GRS逐星残差 m</td><td>{self._safe(pq.get('grs_mean'))}</td><td>{self._safe(pq.get('grs_med'))}</td><td>{self._safe(pq.get('grs_min'))}</td><td>{self._safe(pq.get('grs_max'))}</td></tr></table>
<h2>6. 速度与轨迹</h2><div class="chart"><img src="velocity_time_series.png"></div><div class="chart"><img src="gnss_enu_trajectory.png"></div><p>平均速度 {self._safe(vel.get('average_speed'))} m/s；最大速度 {self._safe(vel.get('max_speed'))} m/s。ENU以首个有效BESTNAV点为参考点，仅用于轨迹形状，不是精度误差。</p>
<h2>7. RTK链路与状态</h2><p>基站ID {rtk['primary_station'] if rtk else 'N/A'}；差分龄期平均{self._safe(rtk.get('avg_diff_age') if rtk else None)}s，最大{self._safe(rtk.get('max_diff_age') if rtk else None)}s。</p><div class="chart"><img src="rtk_status_timeline.png"></div><table><tr><th>RTK计算状态</th><th>数量</th></tr>{''.join(f'<tr><td>{k}</td><td>{v}</td></tr>' for k,v in rts.get('calc_status_count',{}).items())}</table>
<h2>8. 原始观测质量</h2><div class="chart"><img src="cn0_analysis.png"></div><p>OBSVM {obs.get('epochs',0)}历元/{obs.get('obs_total',0)}频点观测；平均C/N0 {self._safe(obs.get('mean_cn0'))} dB-Hz；低C/N0(&lt;35)占比{obs.get('low_ratio',0):.1f}%；locktime归零/消失事件{len(obs.get('loss_events',[]))}个；重捕获事件{len(obs.get('reacq_events',[]))}个。事件判定仅基于OBSVM实测locktime，1Hz采集时间分辨率为1s。</p>
<h2>9. SATSINFO</h2><p>检测到 {si.get("epochs",0)} 个SATSINFOA历元。该ASCII变长频率段在手册表格中未给出足够无歧义的切分规则，为避免字段猜测，本版不生成天空分布/SNR统计。</p>
<h2>10. 依据与限制</h2><ul><li>BESTNAV字段依据手册7.3.19表7-70；GGA/GSA/GST/GRS依据7.1.1.3/7.1.1.7/7.1.1.8/7.1.1.6。</li><li>OBSVM字段依据7.3.65表7-137/7-138/7-140，仅保留CRC通过的二进制块。</li><li>SATSINFO仅做存在性检查（原始定义见7.3.89表7-174~7-177）；BESTSAT依据7.3.22表7-76~7-80；RTKSTATUS依据7.3.85表7-169。</li><li>本数据为Kinematic/固定场景实测，未提供外业真值，不能计算绝对定位误差。</li></ul></div></body></html>'''
        out=os.path.join(self.output_dir,'report.html'); open(out,'w',encoding='utf-8').write(html); print('生成',out)

    def generate_md_report(self):
        print("生成Markdown报告...")
        # Markdown内容与HTML同源要点，保持轻量。
        mi=self.analysis_results.get('message_integrity',{}); st=self.analysis_results.get('gnss_solution_type',{}); pq=self.analysis_results.get('pos_quality',{}); obs=self.analysis_results.get('obs',{})
        lines=[f'# UM980 / 北斗星通 GNSS Kinematic分析报告','',f'- 输入：`{os.path.basename(self.input_file)}`',f'- 生成时间：{datetime.datetime.now():%Y-%m-%d %H:%M:%S}',f'- 协议依据：N4 Products Commands and Logs Reference Book R1.15','','## 完整性与周期','说明：参考周期仅作对照，不将实测周期差异直接判错；告警基于校验和实测时间节律。','','| 报文 | 条数 | 参考周期s | 实际周期s | 实际Hz | 跳变数 | 估算缺槽 | 状态 | 备注 |','|---|---:|---:|---:|---:|---:|---:|---|---|']
        for x in mi.get('checks',[]): lines.append(f'| {x["header"]} | {x["count"]} | {x["reference_period"]:.3f} | {self._safe(x["actual_period"])} | {self._safe(x["actual_rate"])} | {x["gap_count"]} | {x["estimated_missing_slots"]} | {x["status"]} | {x["note"]} |')
        lines += ['','## 解类型',f'- 总历元：{st.get("total_epochs",0)}',f'- 固定解率：{st.get("fixed_ratio",0):.1f}%']
        for k,v in sorted(st.get('type_count',{}).items(),key=lambda x:-x[1]): lines.append(f'- {k}: {v}')
        lines += ['','## 位置σ / 伪距残差',f'- 纬度σ中位：{self._safe(pq.get("lat_med"))} m',f'- 经度σ中位：{self._safe(pq.get("lon_med"))} m',f'- 高度σ中位：{self._safe(pq.get("hgt_med"))} m',f'- GST RMS中位：{self._safe(pq.get("rms_med"))} m',f'- GRS逐星残差中位：{self._safe(pq.get("grs_med"))} m','','## OBSVM',f'- 历元：{obs.get("epochs",0)}；频点观测：{obs.get("obs_total",0)}',f'- C/N0均值：{self._safe(obs.get("mean_cn0"))} dB-Hz；低C/N0占比：{obs.get("low_ratio",0):.1f}%',f'- 失锁/消失事件：{len(obs.get("loss_events",[]))}；重捕获事件：{len(obs.get("reacq_events",[]))}','','## 限制','Kinematic数据无外业真值，不能计算绝对定位误差；接收机σ/RMS为自估质量指标。']
        out=os.path.join(self.output_dir,'report.md'); open(out,'w',encoding='utf-8',newline='\n').write('\n'.join(lines)+'\n'); print('生成',out)

    def run_analysis(self):
        print(f"\n=== 开始分析文件: {self.input_file} ===")
        self.create_output_dir()
        if not self.parse_dat_file(): return False
        self.save_header_files(); self.check_message_integrity(); self.analyze_sampling_rate()
        self.analyze_solution_type(); self.analyze_satellite_visibility(); self.analyze_dop()
        self.analyze_pos_quality(); self.analyze_velocity(); self.analyze_rtk_link()
        self.analyze_rtk_status(); self.analyze_bestsat(); self.analyze_satsinfo_presence(); self.analyze_obsvm()
        self.generate_plots(); self.generate_html_report(); self.generate_md_report()
        print(f"=== 分析完成! 结果保存在: {self.output_dir} ===")
        return True

def run_with_gui():
    import tkinter as tk
    from tkinter import filedialog,messagebox
    def select_file():
        return filedialog.askopenfilename(title='选择UM980数据文件',filetypes=[('UM980数据','*.log;*.dat'),('所有文件','*.*')])
    def start_analysis():
        path=select_file()
        if not path: return
        GPSKinematicAnalyzer(path).run_analysis()
    root=tk.Tk(); root.title('UM980 GNSS Kinematic Analyzer'); root.geometry('420x180')
    tk.Label(root,text='UM980 / 北斗星通 GNSS Kinematic Analyzer',font=('Microsoft YaHei',12,'bold')).pack(pady=20)
    tk.Button(root,text='选择数据并分析',command=start_analysis,width=24).pack(pady=10)
    tk.Label(root,text='结果保存于输入文件同名目录',foreground='#666').pack()
    root.mainloop()

def main():
    import sys
    parser=argparse.ArgumentParser(description='和芯星通UM980 GNSS/RTK Kinematic定位数据分析程序')
    parser.add_argument('input_file',nargs='?',help='输入log/dat文件路径')
    parser.add_argument('--mark-anomalies',action='store_true',help='轨迹图中标记速度异常点')
    parser.add_argument('--speed-threshold',type=float,default=2.0,help='异常速度阈值m/s')
    args=parser.parse_args()
    if args.input_file:
        if not os.path.exists(args.input_file): print(f'错误: 文件不存在 - {args.input_file}'); return
        GPSKinematicAnalyzer(args.input_file,mark_anomalies=args.mark_anomalies,speed_threshold=args.speed_threshold).run_analysis()
    else: run_with_gui()

if __name__=='__main__': main()
