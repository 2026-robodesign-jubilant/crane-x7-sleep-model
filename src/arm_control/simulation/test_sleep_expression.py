"""睡眠姿勢の向きと、肩・手首の連動を検証する。"""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from std_msgs.msg import Float32, String

from sleep_expression import SleepExpression, sleep_cycle, sleep_pose, waiting_pose


class SleepPoseTest(unittest.TestCase):
    def test_waiting_pose_is_close_and_points_down(self):
        origin = [.8, .4, .2, -2., .1, .4, -.3]
        waiting = waiting_pose(origin)
        sleeping = sleep_pose(origin)
        self.assertAlmostEqual(sum(waiting[i] for i in (1, 3, 5)), -math.pi)
        for i in range(7):
            self.assertAlmostEqual(waiting[i] - sleeping[i],
                                   math.radians({1: 10, 5: -10}.get(i, 0)))

    def test_sleep_pose_points_down_for_different_start_poses(self):
        for origin in ([0.0] * 7, [.8, .4, .2, -2., .1, .4, -.3]):
            pose = sleep_pose(origin)
            self.assertEqual(pose[0], origin[0])
            self.assertEqual(pose[6], origin[6])
            self.assertEqual((pose[2], pose[4]), (0., 0.))
            self.assertAlmostEqual(pose[1] + pose[3] + pose[5], -math.pi)

    def test_sway_keeps_tip_down(self):
        center = sleep_pose([0.] * 7)
        bounds = [(-math.pi, math.pi, 1.)] * 7
        points = sleep_cycle(center, math.radians(3), 6., bounds)
        for point in points:
            self.assertAlmostEqual(sum(point.positions[i] for i in (1, 3, 5)), -math.pi)
            self.assertAlmostEqual(point.velocities[1], -point.velocities[5])
        self.assertAlmostEqual(max(p.positions[1] for p in points), math.radians(13))
        self.assertAlmostEqual(min(p.positions[1] for p in points), math.radians(7))
        self.assertEqual(list(points[-1].positions), center)

    def test_wrist_limits_are_checked(self):
        bounds = [(-math.pi, math.pi, 1.)] * 7
        bounds[5] = (math.radians(-71), math.radians(90), 1.)
        with self.assertRaises(ValueError):
            sleep_cycle(sleep_pose([0.] * 7), math.radians(3), 6., bounds)


class StartupTest(unittest.TestCase):
    def make_node(self):
        node = SimpleNamespace(
            now=lambda: 10., last_tick=None, active=False, inputs_fresh=lambda now: True,
            model_verified=True, goal_handle=None, pending=False, next_start=0., blocked=False,
            bounds=[(-math.pi, math.pi, 1.)] * 7, arm=Mock(), planner=Mock(),
            validity=Mock(), expression=Mock(), get_logger=Mock(), plan_pose=Mock(),
            startup_attempted=False, sleepiness=0., sleep_state='awake', sleeping=False,
            positions=[0.] * 7, amplitude=math.radians(3), period=6.,
            make_sleep_cycle=Mock(return_value=['points']), check_cycle=Mock(),
        )
        return node

    def sleeping_node(self, state):
        node = self.make_node()
        node.startup_attempted = True
        node.sleeping = True
        node.awake_pose = waiting_pose(node.positions)
        node.sleep_center = sleep_pose(node.awake_pose)
        node.sleep_state = state
        return node

    def test_startup_moves_even_below_threshold_then_waits(self):
        node = self.make_node()
        SleepExpression.update(node)
        node.plan_pose.assert_called_once_with(waiting_pose(node.positions), 'waiting')
        SleepExpression.update(node)
        self.assertEqual(node.plan_pose.call_count, 1)

    def test_startup_waits_for_safe_inputs_and_services(self):
        for unavailable in ('model_verified', 'fresh', 'planner', 'arm'):
            node = self.make_node()
            if unavailable == 'model_verified':
                node.model_verified = False
            elif unavailable == 'fresh':
                node.inputs_fresh = lambda now: False
            elif unavailable == 'planner':
                node.planner.service_is_ready.return_value = False
            else:
                node.arm.server_is_ready.return_value = False
            SleepExpression.update(node)
            node.plan_pose.assert_not_called()
            self.assertFalse(node.startup_attempted)

    def test_drowsy_starts_sleep_from_waiting_pose(self):
        for state in ('drowsy', 'asleep'):  # 起動直後に asleep でも、まず睡眠姿勢へ下りる
            with self.subTest(state=state):
                node = self.make_node()
                node.startup_attempted = True
                node.positions = waiting_pose(node.positions)
                node.sleep_state = state
                SleepExpression.update(node)
                node.plan_pose.assert_called_once_with(sleep_pose(node.positions), 'lowering')
                self.assertEqual(node.awake_pose, node.positions)

    def test_awake_waits_and_high_drowsiness_alone_does_not_start(self):
        node = self.make_node()
        node.startup_attempted = True
        node.sleepiness = .9  # 判定は /sleep_state だけで行う
        SleepExpression.update(node)
        node.plan_pose.assert_not_called()

    def test_drowsy_sways_while_sleeping(self):
        node = self.sleeping_node('drowsy')
        SleepExpression.update(node)
        node.check_cycle.assert_called_once_with(['points'], 0)
        self.assertTrue(node.pending)

    def test_asleep_holds_still_at_sleep_pose(self):
        node = self.sleeping_node('asleep')
        SleepExpression.update(node)
        node.check_cycle.assert_not_called()
        node.plan_pose.assert_not_called()
        self.assertFalse(node.active)

    def test_awake_returns_to_pose_before_sleep(self):
        node = self.sleeping_node('awake')
        SleepExpression.update(node)
        node.plan_pose.assert_called_once_with(node.awake_pose, 'waking')

    def test_cycle_is_not_sent_after_leaving_drowsy(self):
        node = self.sleeping_node('asleep')
        node.send_trajectory = Mock()
        SleepExpression.check_cycle(node, ['points'], 1)
        node.send_trajectory.assert_not_called()
        self.assertFalse(node.pending)


class SleepStateInputTest(unittest.TestCase):
    def make_node(self):
        return SimpleNamespace(
            now=lambda: 10., sleep_state=None, state_stamp=None, blocked=True,
            sleepiness=0., sleep_stamp=None, joint_stamp=10., get_logger=Mock(),
        )

    def test_only_awake_clears_failure_block(self):
        node = self.make_node()
        for state in ('drowsy', 'asleep'):
            SleepExpression.on_sleep_state(node, String(data=state))
            self.assertTrue(node.blocked)
        SleepExpression.on_drowsiness(node, Float32(data=0.1))  # 眠気が低いだけでは解除しない
        self.assertTrue(node.blocked)
        SleepExpression.on_sleep_state(node, String(data='awake'))
        self.assertFalse(node.blocked)

    def test_unknown_state_is_ignored(self):
        node = self.make_node()
        SleepExpression.on_sleep_state(node, String(data='sleepy'))
        self.assertIsNone(node.sleep_state)
        self.assertIsNone(node.state_stamp)

    def test_state_must_be_fresh(self):
        node = self.make_node()
        SleepExpression.on_drowsiness(node, Float32(data=0.5))
        self.assertFalse(SleepExpression.inputs_fresh(node, 10.))  # /sleep_state 未受信
        SleepExpression.on_sleep_state(node, String(data='drowsy'))
        self.assertTrue(SleepExpression.inputs_fresh(node, 10.5))
        self.assertFalse(SleepExpression.inputs_fresh(node, 11.))


if __name__ == '__main__':
    unittest.main()
