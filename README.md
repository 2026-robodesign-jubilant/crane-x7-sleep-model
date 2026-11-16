# crane-x7-sleep-model

眠気を持つマニピュレータ。CRANE-X7 が時間とともにうとうとし、手を振ると起きる ROS 2（Jazzy）の睡眠モデルです。
授業「ロボットデザイン」のチーム Jubilant の一部として開発しています。

設計の軸は「状態 → パラメータ → 動作」です。関節の動きから活動度を求め、連続的な眠気を更新し、
眠気から決めた状態（起きている・うとうと・眠る）に応じて表現動作を生成します。

```text
/joint_states → activity_monitor → /activity → drowsiness_node → /drowsiness, /sleep_state
                                    /wake（手振り1回分）──┘                         ↓
                                                         sleep_expression → MoveIt / 関節軌道
```

| 状態 | 条件 | 動き |
|---|---|---|
| 起きている（awake） | 起動時、または眠気 0.4 未満 | 待機姿勢 |
| うとうと（drowsy） | 眠気 0.6 以上（静止から約 30 秒） | 先端を真下に向けて頭をゆっくり上下 |
| 眠る（asleep） | 眠気 0.85 以上（約 63 秒） | 睡眠姿勢で静止 |

手振り（`/wake`）を受け取るたびに眠気が下がり、深く眠っているときは約 2 秒振り続けると起きます。
手振り検出（MediaPipe＋深度）と LED は未実装で、今は `ros2 topic pub` で `/wake` を送ります。

## 構成

| パス | 内容 | ライセンス |
|---|---|---|
| `src/arm_control/` | 自作の活動度・眠気・睡眠表現ノード、launch、テスト、資料 | BSD 3-Clause（ルートの `LICENSE`） |
| `src/crane_x7_ros/` | [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros) を同梱（ローカル修正あり） | Apache License 2.0（同梱の `LICENSE`） |
| `src/crane_x7_description/` | **同梱しない**。`crane_x7.repos` で [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description) を取得 | RT Corporation 非商用使用許諾規約 |

固定コミットとローカル修正は [src/arm_control/THIRD_PARTY.md](src/arm_control/THIRD_PARTY.md) にあります。

## セットアップ

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo / MoveIt 2 を使います。

```bash
git clone https://github.com/2026-robodesign-jubilant/crane-x7-sleep-model.git ~/crane-x7-sleep-model
cd ~/crane-x7-sleep-model
source /opt/ros/jazzy/setup.bash
```

`crane_x7_description` を取得します（RT Corporation の非商用使用許諾規約に同意のうえ使用してください）。

```bash
sudo apt install python3-vcstool
vcs import src < crane_x7.repos
```

vcstool を使わない場合は、同じコミットを `git` で取得できます。

```bash
git clone https://github.com/rt-net/crane_x7_description.git src/crane_x7_description
git -C src/crane_x7_description checkout 61865950f741f0d5a35e28eb0a7d2501cac86e7b
```

依存パッケージを入れてビルドします。

```bash
rosdep install --from-paths src/crane_x7_ros src/crane_x7_description --ignore-src -r -y
sudo apt install ros-jazzy-rqt-plot ros-jazzy-gz-ros2-control
colcon build --symlink-install
source install/setup.bash
```

## 起動（シミュレーション）

```bash
ros2 launch ./src/arm_control/simulation/manual_activity.launch.py
```

Gazebo・RViz・グラフが起動し、約 30 秒でうとうと、約 63 秒で眠ります。眠ったら別の端末で手振りの代わりを送ります。

```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=87 && ros2 topic pub -r 2 /wake std_msgs/msg/Empty "{}"
```

2〜3 秒で Ctrl+C を押すと、起きて元の姿勢へ戻ります。詳細・実機の手順・テストは
[src/arm_control/README.md](src/arm_control/README.md) を参照してください。

## テスト（画面を起動しない）

```bash
python3 src/arm_control/simulation/test_drowsiness.py
python3 src/arm_control/simulation/test_sleep_expression.py
```
