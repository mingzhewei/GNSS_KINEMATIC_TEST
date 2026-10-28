"""UM980运动字段回归：N4 R1.15表7-70、表7-51及已有三份实测日志。"""
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from gps_kinematic_analyzer_beidou import GPSKinematicAnalyzer


# 手册PDF第223页/印刷190页原始示例，防止测试与错误字段映射一起偏移。
MANUAL_SAMPLE = (
    '#BESTNAVA,97,GPS,FINE,2294,472312000,0,0,18,16;'
    'SOL_COMPUTED,SINGLE,40.07895888272,116.23651029820,65.8312,-8.4925,'
    'WGS84,1.2221,1.1053,2.1970,"0",0.000,0.000,50,28,28,0,1,12,12,41,'
    'SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0046,335.592288,0.0045,'
    '0.0194,0.0123*c1b4f7fe'
)


def sample(changes=None, week=2294, ms=472312000, time_status='FINE'):
    header, body = MANUAL_SAMPLE.split(';')
    h = header.split(',')
    h[4:6] = [str(week), str(ms)]
    h[3] = time_status
    fields = body.split('*')[0].split(',')
    for index, value in (changes or {}).items():
        fields[index] = str(value)
    payload = ','.join(h)[1:] + ';' + ','.join(fields)
    crc = GPSKinematicAnalyzer._unicore_crc32(payload.encode('ascii'))
    return '#' + payload + f'*{crc:08x}'


def analyzer(*lines, **kwargs):
    result = GPSKinematicAnalyzer('fixture.log', **kwargs)
    result.parsed_data['#BESTNAVA'] = list(lines)
    return result


class BeidouMotionTests(unittest.TestCase):
    def test_manual_sample_and_field_units(self):
        d = analyzer(MANUAL_SAMPLE).parse_bestnava()[0]
        self.assertEqual(d['hor_spd'], 0.0046)
        self.assertEqual(d['trk_gnd'], 335.592288)
        self.assertEqual(d['velocity_up'], 0.0045)
        self.assertEqual(d['vert_sigma'], 0.0194)
        self.assertEqual(d['hor_sigma'], 0.0123)
        self.assertEqual(d['height'], 65.8312)
        self.assertAlmostEqual(d['ellipsoid_height'], 57.3387)

    def test_cardinal_directions_latency_and_separate_ages(self):
        for course, expected_e, expected_n in [(0,0,3),(90,3,0),(180,0,-3),(270,-3,0)]:
            a = analyzer(sample({11:7,23:0.25,24:9,25:3,26:course,27:-4}))
            d = a.parse_bestnava()[0]
            self.assertAlmostEqual(d['velocity_east'], expected_e)
            self.assertAlmostEqual(d['velocity_north'], expected_n)
            self.assertEqual(d['velocity_up'], -4)
            self.assertEqual(d['velocity_timestamp'], d['timestamp'] - 0.25)
            self.assertEqual(d['diff_age'], 7)
            self.assertEqual(d['velocity_diff_age'], 9)
            a.analyze_velocity()
            self.assertEqual(a.analysis_results['velocity']['average_speed'], 5)
            self.assertEqual(a.analysis_results['velocity']['average_horizontal_speed'], 3)

    def test_invalid_values_do_not_become_zero_or_erase_position(self):
        for index in (25,26,27):
            for value in ('','nan','inf','bad'):
                d = analyzer(sample({index:value})).parse_bestnava()[0]
                self.assertTrue(d['valid_coord'])
                self.assertFalse(d['valid_velocity'])
        a = analyzer(sample({25:0,27:0}), sample({25:''}), sample({21:'INSUFFICIENT_OBS'}))
        a.analyze_velocity()
        self.assertEqual(a.analysis_results['velocity']['valid_epochs'], 1)
        self.assertEqual(a.analysis_results['velocity']['excluded_epochs'], 2)
        self.assertEqual(a.analysis_results['velocity']['average_speed'], 0)

    def test_missing_latency_keeps_speed_but_not_corrected_epoch(self):
        a = analyzer(sample({23:''}))
        d = a.parse_bestnava()[0]
        self.assertTrue(d['valid_velocity'])
        self.assertFalse(d['valid_velocity_time'])
        a.analyze_velocity()
        self.assertEqual(a.analysis_results['velocity']['valid_epochs'],1)
        self.assertEqual(a.analysis_results['velocity']['missing_time_epochs'],1)

    def test_velocity_status_is_independent_of_position_status(self):
        for changes in ({0:'INSUFFICIENT_OBS',1:'NONE'}, {2:'',7:''}):
            d = analyzer(sample(changes)).parse_bestnava()[0]
            self.assertFalse(d['valid_coord'])
            self.assertTrue(d['valid_velocity'])
        self.assertTrue(analyzer(sample({2:0,3:0})).parse_bestnava()[0]['valid_coord'])

    def test_unknown_time_status_is_excluded_from_time_derived_metrics(self):
        lines = [sample(ms=472312000),
                 sample({25:3,26:90,27:4}, ms=472313000, time_status='UNKNOWN'),
                 sample(ms=472314000)]
        a = analyzer(*lines)
        records = a.parse_bestnava()
        self.assertEqual([d['time_valid'] for d in records], [True,False,True])
        self.assertTrue(records[1]['valid_coord'])
        self.assertTrue(records[1]['valid_velocity'])
        self.assertEqual(a._extract_category_timestamps('BESTNAVA',lines),
                         [records[0]['timestamp'],records[2]['timestamp']])

        a.analyze_sampling_rate()
        self.assertEqual(a.analysis_results['sampling_rate']['time_valid_epochs'],2)
        self.assertEqual(a.analysis_results['sampling_rate']['invalid_time_status_epochs'],1)
        self.assertEqual(a.analysis_results['sampling_rate']['median_interval'],2)
        a.analyze_velocity()
        self.assertEqual(a.analysis_results['velocity']['valid_epochs'],3)
        self.assertEqual(a.analysis_results['velocity']['invalid_time_status_epochs'],1)
        self.assertEqual(a.analysis_results['velocity']['missing_time_epochs'],1)

        a.mark_anomalies = True
        enu = [dict(E=0,N=0,U=0,timestamp=0,time_valid=True,source_index=0),
               dict(E=100,N=0,U=0,timestamp=1,time_valid=False,source_index=1),
               dict(E=200,N=0,U=0,timestamp=2,time_valid=True,source_index=2)]
        self.assertEqual(a._trajectory_anomalies(enu),[])

    def test_other_ascii_time_series_exclude_unknown_status(self):
        a = analyzer(sample())
        a.parsed_data['#RTKSTATUSA'] = [
            '#RTKSTATUSA,1,GPS,FINE,2294,472312000,0,0,18,16;0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,1*00000000',
            '#RTKSTATUSA,1,GPS,UNKNOWN,2294,472313000,0,0,18,16;0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,1*00000000']
        a.parsed_data['#BESTSATA'] = [
            '#BESTSATA,1,GPS,FINE,2294,472312000,0,0,18,16;0*00000000',
            '#BESTSATA,1,GPS,UNKNOWN,2294,472313000,0,0,18,16;0*00000000']
        self.assertEqual([d['time_valid'] for d in a.parse_rtkstatus()], [True, False])
        self.assertEqual([d['time_valid'] for d in a.parse_bestsat()], [True, False])

    def test_enu_trajectory_does_not_connect_unknown_time_point(self):
        lines = [sample(ms=472312000),
                 sample({2:40.07896}, ms=472313000, time_status='UNKNOWN'),
                 sample({2:40.07897}, ms=472314000)]
        a = analyzer(*lines)
        with tempfile.TemporaryDirectory() as tmp:
            a.output_dir = tmp
            with patch.object(plt,'close'):
                a._plot_gnss_enu_trajectory(a.parse_bestnava())
                self.assertEqual(len(plt.gcf().axes[0].lines),0)
            plt.close('all')

    def test_position_sigma_time_axis_breaks_at_unknown_time_status(self):
        a = analyzer(sample(), sample(time_status='UNKNOWN',ms=472313000),
                     sample(ms=472314000))
        a.analyze_pos_quality()
        times=a.analysis_results['pos_quality']['pos_t']
        self.assertTrue(math.isfinite(times[0]))
        self.assertTrue(math.isnan(times[1]))
        self.assertTrue(math.isfinite(times[2]))
        self.assertEqual(a.analysis_results['pos_quality']['pos_time_invalid_epochs'],1)

    def test_week_boundary_and_missing_epoch(self):
        lines = [sample(week=2441,ms=604799000), sample(week=2442,ms=0),
                 sample(week=2442,ms=1000), sample(week=2442,ms=3000)]
        a = analyzer(*lines)
        ts = [d['timestamp'] for d in a.parse_bestnava()]
        self.assertEqual([t-ts[0] for t in ts], [0,1,2,4])
        self.assertEqual(a._extract_category_timestamps('BESTNAVA',lines), ts)
        a.analyze_sampling_rate()
        self.assertEqual(a.analysis_results['sampling_rate']['sampling_rate'], 1)
        self.assertAlmostEqual(a.analysis_results['sampling_rate']['loss_rate'], 20)

    def test_enu_requires_ellipsoid_height_and_preserves_gaps(self):
        a = analyzer(sample({4:100,5:10}), sample({4:100,5:15},ms=472313000))
        points = a._trajectory_points(a.parse_bestnava())
        self.assertAlmostEqual(points[1]['E'], 0, places=7)
        self.assertAlmostEqual(points[1]['N'], 0, places=7)
        self.assertAlmostEqual(points[1]['U'], 5, places=7)
        for bad in ({5:''}, {6:'UNKNOWN'}, {0:'INSUFFICIENT_OBS'}):
            a = analyzer(sample(), sample(bad), sample(ms=472314000))
            self.assertEqual([p['source_index'] for p in a._trajectory_points(a.parse_bestnava())], [0,2])

    def test_anomaly_option_threshold_and_time_guards(self):
        points = [dict(E=0,N=0,U=0,timestamp=0,time_valid=True,source_index=0),
                  dict(E=3,N=0,U=0,timestamp=1,time_valid=True,source_index=1)]
        a = analyzer(mark_anomalies=True,speed_threshold=2)
        self.assertEqual(len(a._trajectory_anomalies(points)), 1)
        a.mark_anomalies=False
        self.assertEqual(a._trajectory_anomalies(points), [])
        a.mark_anomalies=True
        for end in (dict(points[1],timestamp=0), dict(points[1],timestamp=-1),
                    dict(points[1],source_index=2), dict(points[1],E=2),
                    dict(points[1],E=0,U=100), dict(points[1],time_valid=False)):
            self.assertEqual(a._trajectory_anomalies([points[0],end]), [])
        for threshold in (0,-1,math.nan,math.inf):
            with self.assertRaises(ValueError): analyzer(speed_threshold=threshold)

    def test_plot_uses_velocity_epoch_and_breaks_invalid_values(self):
        a = analyzer(sample({23:0.25,25:3,26:90,27:-4}), sample({25:''},ms=472313000),
                     sample({23:'',25:2,26:0,27:0},ms=472314000),
                     sample({25:2,26:0,27:0},ms=472315000,time_status='UNKNOWN'))
        with tempfile.TemporaryDirectory() as tmp:
            a.output_dir=tmp
            with patch.object(plt,'close'):
                a._plot_velocity_time_series()
                axes=plt.gcf().axes
                self.assertEqual(axes[0].lines[0].get_xdata()[0], -0.25)
                self.assertAlmostEqual(axes[1].lines[0].get_ydata()[0], 3)
                self.assertEqual(axes[2].lines[0].get_ydata()[0], -4)
                self.assertTrue(math.isnan(axes[0].lines[0].get_ydata()[1]))
                self.assertTrue(math.isnan(axes[0].lines[0].get_ydata()[2]))
                self.assertTrue(math.isnan(axes[0].lines[0].get_xdata()[3]))
                self.assertTrue(math.isnan(axes[0].lines[0].get_ydata()[3]))
            plt.close('all')

    def test_three_recorded_scenarios(self):
        expected = {670:(1.0424865598623645,2.6110139199169353),
                    1610:(0.890330561802937,2.818608644349194),
                    145:(1.1861665929241598,2.918402473957285)}
        files = list((Path(__file__).parent/'beidou_data').glob('*/#BESTNAVA.dat'))
        self.assertEqual(len(files), 3)
        for path in files:
            a = analyzer(*path.read_text(encoding='utf8').splitlines())
            a.analyze_velocity()
            data=a.parse_bestnava(); result=a.analysis_results['velocity']
            average,maximum=expected[len(data)]
            self.assertEqual(result['valid_epochs'],len(data))
            self.assertAlmostEqual(result['average_speed'],average,places=12)
            self.assertAlmostEqual(result['max_speed'],maximum,places=12)
            self.assertTrue(any(abs(d['velocity_east'])>0.1 for d in data))
            self.assertTrue(any(abs(d['velocity_north'])>0.1 for d in data))


if __name__ == '__main__':
    unittest.main()
