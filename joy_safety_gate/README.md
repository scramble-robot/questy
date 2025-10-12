# Joy Safety Gate

Joy入力をdiagnosticsメッセージに基づいてフィルタリングする安全ゲートノードです。

## 概要

`joy_safety_gate`は、システムの診断状態を監視し、異常が検出された場合にJoyメッセージを自動的に遮断します。これにより、バッテリー低下などの異常時に安全にロボットを停止できます。

## 機能

- `/diagnostics` トピックをサブスクライブして診断状態を監視
- `joy_in` トピックからJoyメッセージを受信
- 診断状態が安全な場合、メッセージを `joy_out` トピックに転送
- 診断状態が危険な場合、メッセージをブロック（joy_controllerへの入力が途絶え、自動停止）
- タイムアウト機能により、診断メッセージが途絶えた場合も安全に遮断

## システム構成

```
joy_node → joy_safety_gate → joy_controller → motor_control
                ↑
           /diagnostics
         (battery_monitor)
```

## トピック

### サブスクライブ

- `joy_in` (sensor_msgs/msg/Joy): Joy入力メッセージ
- `/diagnostics` (diagnostic_msgs/msg/DiagnosticArray): システムの診断情報

### パブリッシュ

- `joy_out` (sensor_msgs/msg/Joy): フィルタリングされたJoyメッセージ（安全時のみ）

## パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `diagnostics_timeout` | double | 5.0 | 診断メッセージのタイムアウト時間（秒） |
| `monitor_names` | string[] | [] | 監視する診断名のリスト（空=全て監視） |
| `block_levels` | string[] | ["ERROR"] | 遮断する診断レベル |
| `require_diagnostics` | bool | true | 最初の診断受信まで遮断 |
| `pass_through_when_unsafe` | bool | false | unsafe時も通過させるか |

### block_levels設定例

```yaml
# ERRORのみブロック（デフォルト）
block_levels: ["ERROR"]

# WARNINGとERRORでブロック（より保守的）
block_levels: ["WARN", "ERROR"]

# STALEでもブロック（診断が古い場合も停止）
block_levels: ["ERROR", "STALE"]
```

## 使用方法

### 1. スタンドアロンノードとして起動

```bash
ros2 launch joy_safety_gate joy_safety_gate.launch.py
```

### 2. システム全体を統合起動

```bash
ros2 launch joy_safety_gate system_with_safety.launch.py
```

### 3. 既存のlaunchファイルに統合

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('joy_safety_gate')
    params = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    return LaunchDescription([
        # Joy node
        Node(package='joy', executable='joy_node'),
        
        # Safety Gate
        Node(
            package='joy_safety_gate',
            executable='joy_safety_gate_node',
            parameters=[params],
            remappings=[
                ('joy_in', '/joy'),
                ('joy_out', '/joy_filtered'),
            ]
        ),
        
        # Joy Controller
        Node(
            package='joy_controller',
            executable='joy_controller_node',
            remappings=[('/joy', '/joy_filtered')]
        ),
    ])
```

## 設定のカスタマイズ

`config/params.yaml` を編集してパラメータをカスタマイズできます。

### 例1: バッテリー監視のみを対象

```yaml
joy_safety_gate:
  ros__parameters:
    monitor_names: ["bluetooth_battery_monitor"]
    block_levels: ["ERROR"]
```

### 例2: 警告レベルでも停止

```yaml
joy_safety_gate:
  ros__parameters:
    block_levels: ["WARN", "ERROR"]
    diagnostics_timeout: 3.0
```

### 例3: デバッグモード（警告のみでブロックしない）

```yaml
joy_safety_gate:
  ros__parameters:
    pass_through_when_unsafe: true  # 警告のみ、ブロックしない
```

## 動作確認

### ターミナル1: ノードの起動

```bash
ros2 launch joy_safety_gate joy_safety_gate.launch.py
```

### ターミナル2: Joyメッセージの送信

```bash
# 正常なJoyメッセージを送信
ros2 topic pub /joy_in sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.5], buttons: [0, 0]}" --once
```

### ターミナル3: 診断メッセージの送信

```bash
# OK診断（通過）
ros2 topic pub /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  "{status: [{level: 0, name: 'test', message: 'OK'}]}" --once

# ERROR診断（ブロック）
ros2 topic pub /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  "{status: [{level: 2, name: 'test', message: 'ERROR'}]}" --once
```

### ターミナル4: 出力の監視

```bash
# フィルタリングされた出力を確認
ros2 topic echo /joy_out
```

## トラブルシューティング

### Joyメッセージがブロックされる

1. 診断状態を確認:

   ```bash
   ros2 topic echo /diagnostics
   ```

2. ノードのログを確認:

   ```bash
   ros2 node info /joy_safety_gate
   ```

3. パラメータを確認:

   ```bash
   ros2 param list /joy_safety_gate
   ros2 param get /joy_safety_gate block_levels
   ```

### 診断メッセージが受信されない

```bash
# 診断トピックの確認
ros2 topic list | grep diagnostics
ros2 topic hz /diagnostics
```

## ビルド

```bash
cd ~/workspace/shr_ws/questy
colcon build --packages-select joy_safety_gate
source install/setup.bash
```

## テスト

```bash
# 手動テスト
ros2 launch joy_safety_gate joy_safety_gate.launch.py

# 別のターミナルでメッセージを送信してテスト
```

## アーキテクチャの利点

1. **単一責任の原則**: 各ノードが1つの役割のみを担当
2. **テスト容易性**: 独立したノードなので単体テストが簡単
3. **デバッグ容易性**: トピックをechoして問題箇所を特定しやすい
4. **再利用性**: 他のjoy_controller実装でも使える
5. **拡張性**: 将来的に他の安全機能を追加しやすい

## ライセンス

Apache-2.0

## 関連パッケージ

- `bluetooth_battery_monitor`: バッテリー状態を診断メッセージとして発行
- `joy_controller`: フィルタリングされたJoyメッセージをTwistに変換
- `diagnostic_safety_gate`: Twistメッセージ用の類似ゲート
