# Diagnostic Safety Gate

このパッケージは、ROS2のdiagnosticsメッセージに基づいてモータへの指示値を遮断または通過させる安全ゲートノードを提供します。

## 概要

`diagnostic_safety_gate`ノードは、システムの診断状態を監視し、異常が検出された場合にモータコマンドを自動的に遮断します。これにより、システムの安全性を向上させることができます。

## 機能

- `/diagnostics` トピックをサブスクライブして診断状態を監視
- `cmd_vel_in` トピックからモータコマンドを受信
- 診断状態が安全な場合、コマンドを `cmd_vel` トピックに転送
- 診断状態が危険な場合、ゼロ速度を出力してモータを停止
- タイムアウト機能により、診断メッセージやコマンドが途絶えた場合も安全に停止

## トピック

### サブスクライブ

- `cmd_vel_in` (geometry_msgs/msg/Twist): モータへの入力コマンド
- `/diagnostics` (diagnostic_msgs/msg/DiagnosticArray): システムの診断情報

### パブリッシュ

- `cmd_vel` (geometry_msgs/msg/Twist): モータへの出力コマンド（遮断時はゼロ速度）

## パラメータ

- `diagnostics_timeout` (double, default: 5.0): 診断メッセージのタイムアウト時間（秒）
- `cmd_vel_timeout` (double, default: 0.5): コマンドメッセージのタイムアウト時間（秒）
- `monitor_names` (string[], default: []): 監視する診断名のリスト（空の場合は全て監視）
- `block_levels` (string[], default: ["ERROR"]): コマンドを遮断する診断レベル（OK, WARN, ERROR, STALE）
- `require_diagnostics` (bool, default: true): 最初の診断メッセージを受信するまでコマンドを遮断

## 使用方法

### スタンドアロンノードとして起動

```bash
ros2 launch diagnostic_safety_gate diagnostic_safety_gate.launch.py
```

### コンポーザブルノードとして起動

```bash
ros2 launch diagnostic_safety_gate diagnostic_safety_gate_composable.launch.py
```

### 設定のカスタマイズ

`config/params.yaml` を編集してパラメータをカスタマイズできます。

例：バッテリー監視のみを対象にし、WARNレベルでも遮断する場合：

```yaml
diagnostic_safety_gate:
  ros__parameters:
    monitor_names: ["bluetooth_battery_monitor"]
    block_levels: ["WARN", "ERROR"]
```

## システム統合例

既存のモータ制御システムに統合する場合、トピックのリマッピングを使用します：

```python
# joy_controllerなどからのcmd_velをcmd_vel_inにリマップ
Node(
    package='joy_controller',
    executable='joy_controller_node',
    remappings=[('cmd_vel', 'cmd_vel_in')]
)

# Safety Gate
Node(
    package='diagnostic_safety_gate',
    executable='diagnostic_safety_gate_node'
)

# モータ制御ノードはcmd_velをサブスクライブ
Node(
    package='esc_motor_control',
    executable='esc_motor_control_node'
)
```

この構成により、joy_controller → safety_gate → motor_control の順にメッセージが流れます。

## ビルド

```bash
cd ~/workspace/shr_ws/questy
colcon build --packages-select diagnostic_safety_gate
source install/setup.bash
```

## テスト

### 手動テスト

1. ノードを起動：

```bash
ros2 launch diagnostic_safety_gate diagnostic_safety_gate.launch.py
```

2. 別のターミナルでコマンドを送信：

```bash
ros2 topic pub /cmd_vel_in geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once
```

3. 診断メッセージを送信：

```bash
ros2 topic pub /diagnostics diagnostic_msgs/msg/DiagnosticArray "..." --once
```

4. `/cmd_vel` トピックを監視：

```bash
ros2 topic echo /cmd_vel
```

## ライセンス

Apache-2.0
