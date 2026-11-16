#!/usr/bin/env python3
"""活動度 m で眠気の増加を抑え、手振り（/wake）の刺激で眠気を下げるROS 2ノード。"""

import math

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String

SLEEP_STATES = ('awake', 'drowsy', 'asleep')
THRESHOLD_DEFAULTS = {'drowsy_threshold': 0.6, 'sleep_threshold': 0.85, 'wake_threshold': 0.4}
PARAMETER_NAMES = ('k_up', 'alpha', 'k_down', 'wake_tau', 'update_rate', *THRESHOLD_DEFAULTS)


def integrate_drowsiness(state, activity, dt, *, external_stimulus=0.0,
                        k_up=0.03, alpha=0.8, k_down=0.34):
    """dS/dt = k_up*(1-alpha*m)*(1-S) - k_down*外部刺激 を積分。"""
    m = min(max(activity, 0.0), 1.0)
    derivative = k_up * (1.0 - alpha * m) * (1.0 - state) - k_down * external_stimulus
    return min(max(state + derivative * dt, 0.0), 1.0)


def decay_stimulus(stimulus, dt, tau):
    """dt の間に E=E0*exp(-t/tau) で減る刺激の (平均値, dt後の値) を返す。

    平均値×dt は E のグラフの面積と一致するので、計算間隔 dt が変わっても
    手振り1回あたりの効果（k_down×tau）が変わらない。
    """
    remaining = math.exp(-dt / tau)
    average = stimulus * tau * (1.0 - remaining) / dt
    return average, stimulus * remaining


def next_sleep_state(current, drowsiness, *, drowsy_threshold=0.6,
                     sleep_threshold=0.85, wake_threshold=0.4):
    """起きている→うとうと→睡眠、どちらからでも wake_threshold 未満で起きている。"""
    if current != 'awake' and drowsiness < wake_threshold:
        return 'awake'
    if current == 'awake' and drowsiness >= drowsy_threshold:
        return 'drowsy'
    if current == 'drowsy' and drowsiness >= sleep_threshold:
        return 'asleep'
    return current


class DrowsinessNode(Node):
    def __init__(self):
        super().__init__('drowsiness_node')
        self.declare_parameter('k_up', 0.03)
        self.declare_parameter('alpha', 0.8)
        self.declare_parameter('k_down', 0.34)
        self.declare_parameter('wake_tau', 0.5, ParameterDescriptor(
            description='手振り刺激が減衰する時定数 [s]。1回の効果は k_down×wake_tau。'))
        self.declare_parameter('update_rate', 20.0, ParameterDescriptor(
            description='更新頻度 [Hz]。起動時に設定する。', read_only=True))
        for name, default in THRESHOLD_DEFAULTS.items():
            self.declare_parameter(name, default, ParameterDescriptor(
                description='状態判定のしきい値。起動時に設定する。', read_only=True))
        params = [self.get_parameter(name) for name in PARAMETER_NAMES]
        result = self.validate_parameters(params)
        if not result.successful:
            raise ValueError(result.reason)
        self.thresholds = {name: self.get_parameter(name).value for name in THRESHOLD_DEFAULTS}
        if not (self.thresholds['wake_threshold'] < self.thresholds['drowsy_threshold']
                < self.thresholds['sleep_threshold']):
            raise ValueError('need wake_threshold < drowsy_threshold < sleep_threshold')
        self.add_on_set_parameters_callback(self.validate_parameters)

        self.drowsiness = 0.0
        self.activity = 0.0
        self.stimulus = 0.0  # 手振りによる外部刺激 E。/wake 1回で +1、時間とともに減衰する。
        self.sleep_state = 'awake'
        self.previous_update_ns = None
        self.create_subscription(Float32, '/activity', self.on_activity, 10)
        self.create_subscription(Empty, '/wake', self.on_wake, 10)
        self.publisher = self.create_publisher(Float32, '/drowsiness', 10)
        self.state_publisher = self.create_publisher(String, '/sleep_state', 10)
        self.timer = self.create_timer(1.0 / self.get_parameter('update_rate').value, self.update)
        self.get_logger().info('Activity m suppresses growth; each /wake adds stimulus that lowers drowsiness')

    @staticmethod
    def validate_parameters(parameters):
        for parameter in parameters:
            if parameter.name not in PARAMETER_NAMES:
                continue
            value = parameter.value
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                return SetParametersResult(successful=False, reason=f'{parameter.name} must be finite')
            if parameter.name == 'alpha' and not 0.0 <= value <= 1.0:
                return SetParametersResult(successful=False, reason='alpha must be in [0, 1]')
            if parameter.name in ('k_up', 'k_down') and value < 0.0:
                return SetParametersResult(successful=False, reason=f'{parameter.name} must be nonnegative')
            if parameter.name in ('update_rate', 'wake_tau') and value <= 0.0:
                return SetParametersResult(successful=False, reason=f'{parameter.name} must be positive')
            if parameter.name in THRESHOLD_DEFAULTS and not 0.0 <= value <= 1.0:
                return SetParametersResult(successful=False, reason=f'{parameter.name} must be in [0, 1]')
        return SetParametersResult(successful=True)

    def on_activity(self, msg):
        if math.isfinite(msg.data):
            self.activity = min(max(msg.data, 0.0), 1.0)
        else:
            self.get_logger().warning('Ignoring non-finite activity', throttle_duration_sec=1.0)

    def on_wake(self, msg):
        # Empty は中身がないので、届いたこと自体を手振り1回として数える。
        self.stimulus += 1.0

    def update(self):
        now_ns = self.get_clock().now().nanoseconds
        # /clock が最初に届いたとき、起動前の経過時間を積分しない。
        if self.previous_update_ns is not None and now_ns > self.previous_update_ns:
            dt = (now_ns - self.previous_update_ns) * 1e-9
            # 刺激 E はこの dt の間に減る。減りながら効いた分を平均値として式へ渡す。
            average_stimulus, self.stimulus = decay_stimulus(
                self.stimulus, dt, self.get_parameter('wake_tau').value)
            self.drowsiness = integrate_drowsiness(
                self.drowsiness, self.activity, dt,
                external_stimulus=average_stimulus,
                k_up=self.get_parameter('k_up').value,
                alpha=self.get_parameter('alpha').value,
                k_down=self.get_parameter('k_down').value,
            )
        # 時刻が巻き戻った場合は基準を更新し、負のdtを積分しない。
        self.previous_update_ns = now_ns

        state = next_sleep_state(self.sleep_state, self.drowsiness, **self.thresholds)
        if state != self.sleep_state:
            self.get_logger().info(
                f'sleep state: {self.sleep_state} -> {state} (drowsiness={self.drowsiness:.2f})')
            self.sleep_state = state
        self.publisher.publish(Float32(data=self.drowsiness))
        self.state_publisher.publish(String(data=self.sleep_state))


def main():
    rclpy.init()
    node = DrowsinessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
