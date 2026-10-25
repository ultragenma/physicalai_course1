# Q-Learning Turtle Goal Navigator

Physical AI 応用1講座の最終課題向けの、ROS 2 と `turtlesim` を使う小規模な提出物です。タートルは目標座標をカメラ画像ではなく ROS 2 の `/turtle1/pose` から観測し、**Q-learning** で選んだ行動を `/turtle1/cmd_vel` に送って目標へ向かいます。

この題材では、状態（目標までの距離と方位誤差）・行動（前進、左旋回、右旋回）・報酬（目標へ近づくこと、到達、境界への接近、時間）を明示しています。したがって、単なる目標追従の数式制御ではなく、学習済みの Q 表により行動を選ぶ AI を実演できます。

## 必要な環境

- Ubuntu 上の ROS 2 **Humble、Iron、Jazzy のいずれか**
- `ros-<distro>-turtlesim`、`ros-<distro>-rclpy`、`python3-colcon-common-extensions`
- Python 3（追加の Python パッケージは不要）
- `turtlesim_node` のGUIを表示できるデスクトップセッション

ROS 2 のディストリビューション名を `<distro>` として、対象端末で一度だけ次を実行します。

```bash
sudo apt update
sudo apt install ros-<distro>-turtlesim ros-<distro>-rclpy python3-colcon-common-extensions
source /opt/ros/<distro>/setup.bash
```

## ビルド

```bash
cd submmit_easy_v1/ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select qlearn_turtle_nav
source install/setup.bash
```

## 実演手順

再現可能な学習記録、10回評価、実測値からのレポート表生成は [RUNBOOK.md](RUNBOOK.md) を使う。提出用の実行では `run_id` と出力先を必ず指定する。

端末 1 でシミュレータを起動します。

```bash
source /opt/ros/<distro>/setup.bash
ros2 run turtlesim turtlesim_node
```

端末 2 では [RUNBOOK.md](RUNBOOK.md) の「学習記録」「10回評価」のコマンドをそのまま使います。学習ノードは各エピソードの開始時、評価ノードは各試行の開始時に `/turtle1/teleport_absolute` を使ってタートルを開始位置 `(2.0, 2.0, 0.0)` に戻し、reset後は新しいPoseが開始位置の許容範囲内になるまで制御を再開しません。標準設定は60エピソード、各エピソード最大220ステップ、制御周期0.05秒です。学習済みQ表は学習runフォルダ内の `q_table.json` に保存されます。目標座標を変える場合はその座標で再学習します。

`target_x`、`target_y`、`episodes`、`seed` は ROS パラメータです。目標座標はシミュレータの安全範囲 `[0.5, 10.5]` 内に設定します。動画には、学習ログの `episode=... success=...` と、評価時の `GOAL REACHED` を映してください。実行時の成功率や所要時間は環境で測った値だけを報告します。

## ROS 2 インターフェース

| 役割 | 名前 | 型 |
| --- | --- | --- |
| 観測 | `/turtle1/pose` | `turtlesim/msg/Pose` |
| 行動 | `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` |
| リセット | `/turtle1/teleport_absolute` | `turtlesim/srv/TeleportAbsolute` |

## 提出前の確認

`REPORT_DRAFT.md` を実測値、使用したROS 2版、動画URL、GitHub URLで更新し、PDF化します。課題指定のPDF名は `説明資料_PAI最終課題_<OmniCampusアカウント名>.pdf` です。現状のPDFは未実行の草稿で、アカウント欄も未入力です。動画の構成は `VIDEO_PLAN.md` を使います。個人情報、認証情報、社内ネットワーク情報はレポート・動画・GitHubに含めません。詳細は [提出チェック](SUBMISSION_CHECKLIST.md) を参照してください。

## 機械可読な実行記録

実行データの`runs/`と学習済み`q_table.json`は`.gitignore`対象です。学習/評価summaryにQ表SHA-256が入り、レポート生成時にseed、開始/目標姿勢、制御周期、最大ステップ、ROS/Python版、Q表ハッシュの一致を確認します。

`RUNBOOK.md` の実行では `runs/<run_id>/` にJSON/JSONLを出力する。学習は `run_metadata.json`、`training_episodes.jsonl`、`training_summary.json`、評価は `run_metadata.json`、`evaluation_trials.jsonl`、`evaluation_summary.json` を作る。ログにはROSディストリビューション/rclpy/Python版、run_id、seed、目標/開始姿勢、制御周期、最大ステップ、各試行の成功・時間・最終距離、完了エピソード数・試行数・成功数・成功率・平均時間を記録する。値は実行時にのみ生成され、サンプル結果は含めない。
