"""Numerical checks for the drowsiness model; no ROS graph is required."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drowsiness_node import (DrowsinessNode, THRESHOLD_DEFAULTS, decay_stimulus,
                             integrate_drowsiness, next_sleep_state)
from rclpy.parameter import Parameter

PARAMS = {'k_up': 0.03, 'alpha': 0.8, 'k_down': 0.34, 'wake_tau': 0.5}


def fake_node(clock, published, states):
    return SimpleNamespace(
        drowsiness=0.0, activity=0.0, stimulus=0.0, sleep_state='awake',
        previous_update_ns=None, thresholds=dict(THRESHOLD_DEFAULTS),
        get_clock=lambda: SimpleNamespace(now=lambda: clock),
        get_parameter=lambda name: Parameter(name, value=PARAMS[name]),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        publisher=SimpleNamespace(publish=published.append),
        state_publisher=SimpleNamespace(publish=states.append),
    )


def run_waves(start, waves, *, rate=20.0, duration=6.0):
    """0.5秒ごとに /wake を waves 回送り、(0.4未満になった時刻, 最終値) を返す。"""
    clock = SimpleNamespace(nanoseconds=0)
    node = fake_node(clock, [], [])
    DrowsinessNode.update(node)
    node.drowsiness, node.sleep_state = start, 'asleep' if start >= 0.85 else 'drowsy'
    step = int(round(1e9 / rate))
    wave_ticks = {int(round(0.5 * i * rate)) for i in range(waves)}
    woke_at = None
    for tick in range(int(duration * rate)):
        if tick in wave_ticks:
            DrowsinessNode.on_wake(node, None)
        clock.nanoseconds += step
        DrowsinessNode.update(node)
        if woke_at is None and node.sleep_state == 'awake':
            woke_at = (tick + 1) / rate
    return woke_at, node.drowsiness


class DrowsinessModelTest(unittest.TestCase):
    def test_stationary_matches_analytic_solution(self):
        state = 0.0
        for _ in range(600):
            state = integrate_drowsiness(state, 0.0, 0.05)
        self.assertAlmostEqual(state, 1.0 - math.exp(-0.03 * 30), delta=0.001)

    def test_activity_alone_does_not_reduce_drowsiness(self):
        state = 0.6
        for _ in range(140):
            state = integrate_drowsiness(state, 1.0, 0.05)
        self.assertGreater(state, 0.6)
        self.assertGreater(integrate_drowsiness(0.0, 1.0, 0.05), 0.0)

    def test_activity_slows_growth_without_external_stimulus(self):
        state = 0.0
        for _ in range(600):
            state = integrate_drowsiness(state, 1.0, 0.05)
        self.assertAlmostEqual(state, 1.0 - math.exp(-0.006 * 30), delta=0.001)
        self.assertEqual(integrate_drowsiness(0.6, 1.0, 0.05, k_down=100.0),
                         integrate_drowsiness(0.6, 1.0, 0.05, k_down=0.0))

    def test_saturation_and_activity_clamping(self):
        self.assertEqual(integrate_drowsiness(0.99, 0.0, 1000), 1.0)
        self.assertEqual(integrate_drowsiness(0.01, 1.0, 1000, external_stimulus=1.0), 0.0)
        self.assertEqual(integrate_drowsiness(0.5, -1.0, 0.05),
                         integrate_drowsiness(0.5, 0.0, 0.05))
        self.assertEqual(integrate_drowsiness(0.5, 2.0, 0.05),
                         integrate_drowsiness(0.5, 1.0, 0.05))

    def test_invalid_parameters_are_rejected(self):
        for name, value in [('alpha', 1.1), ('alpha', -0.1), ('k_up', -1.0),
                            ('k_down', -1.0), ('update_rate', 0.0), ('k_up', float('nan')),
                            ('wake_tau', 0.0), ('sleep_threshold', 1.5)]:
            with self.subTest(name=name, value=value):
                result = DrowsinessNode.validate_parameters([Parameter(name, value=value)])
                self.assertFalse(result.successful)

    def test_clock_startup_and_rewind_do_not_integrate_old_time(self):
        clock = SimpleNamespace(nanoseconds=1_000_000_000_000)
        published, states = [], []
        node = fake_node(clock, published, states)
        DrowsinessNode.update(node)
        self.assertEqual(node.drowsiness, 0.0)
        clock.nanoseconds += 50_000_000
        DrowsinessNode.update(node)
        self.assertAlmostEqual(node.drowsiness, 0.0015)
        clock.nanoseconds = 1_000_000_000
        DrowsinessNode.update(node)
        self.assertAlmostEqual(node.drowsiness, 0.0015)
        clock.nanoseconds += 50_000_000
        DrowsinessNode.update(node)
        self.assertGreater(node.drowsiness, 0.0015)
        self.assertEqual(len(published), 4)
        self.assertEqual([msg.data for msg in states], ['awake'] * 4)

    def test_one_wave_lowers_by_k_down_times_tau_at_any_rate(self):
        for rate in (20.0, 100.0, 7.0):
            with self.subTest(rate=rate):
                dt, stimulus, area = 1.0 / rate, 1.0, 0.0
                for _ in range(int(10 * rate)):
                    average, stimulus = decay_stimulus(stimulus, dt, 0.5)
                    area += average * dt
                self.assertAlmostEqual(0.34 * area, 0.17, delta=1e-4)

    def test_state_transitions_use_hysteresis(self):
        self.assertEqual(next_sleep_state('awake', 0.59), 'awake')
        self.assertEqual(next_sleep_state('awake', 0.6), 'drowsy')
        self.assertEqual(next_sleep_state('awake', 0.9), 'drowsy')  # 1回の更新で飛ばさない
        self.assertEqual(next_sleep_state('drowsy', 0.5), 'drowsy')
        self.assertEqual(next_sleep_state('drowsy', 0.85), 'asleep')
        self.assertEqual(next_sleep_state('drowsy', 0.39), 'awake')
        self.assertEqual(next_sleep_state('asleep', 0.5), 'asleep')  # 起こしかけでも眠ったまま
        self.assertEqual(next_sleep_state('asleep', 0.39), 'awake')

    def test_stillness_reaches_drowsy_and_asleep_on_schedule(self):
        clock = SimpleNamespace(nanoseconds=0)
        node = fake_node(clock, [], [])
        changes = {}
        DrowsinessNode.update(node)
        for tick in range(1, 20 * 70 + 1):
            clock.nanoseconds += 50_000_000
            before = node.sleep_state
            DrowsinessNode.update(node)
            if node.sleep_state != before:
                changes[node.sleep_state] = tick / 20
        self.assertAlmostEqual(changes['drowsy'], 30.5, delta=0.3)
        self.assertAlmostEqual(changes['asleep'], 63.2, delta=0.3)

    def test_waves_needed_grow_with_depth_of_sleep(self):
        self.assertIsNone(run_waves(1.0, 3)[0])      # 深い眠りは3回では起きない
        woke_at, final = run_waves(1.0, 4)
        self.assertAlmostEqual(woke_at, 2.1, delta=0.2)
        self.assertLess(final, 0.6)  # 起床後は再び増えるが、6秒後はまだうとうと前
        self.assertIsNone(run_waves(0.85, 2)[0])
        self.assertIsNotNone(run_waves(0.85, 3)[0])
        self.assertIsNone(run_waves(0.6, 1)[0])      # うとうとも1回では起きない
        self.assertIsNotNone(run_waves(0.6, 2)[0])


if __name__ == '__main__':
    unittest.main()
