# 実行・レポート生成手順

以下の例はROS 2 Humbleを使う。別のROS 2版では `humble` を実際のディストリビューション名へ置き換える。`run_id` は一度だけ使う。すでに同名の出力ディレクトリがある場合、ノードは上書きを防ぐため停止する。

## 1. ビルドとシミュレータ起動

端末1:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

端末2:

```bash
cd submmit_easy_v1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qlearn_turtle_nav
source install/setup.bash
```

## 2. 学習記録

```bash
cd submmit_easy_v1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run qlearn_turtle_nav navigator --ros-args \
  -p mode:=train -p episodes:=60 -p control_period_s:=0.05 -p seed:=7 \
  -p q_table_path:="$PWD/runs/train_001/q_table.json" \
  -p output_directory:="$PWD/runs" -p run_id:=train_001
```

実行後に `runs/train_001/run_metadata.json`、`runs/train_001/training_episodes.jsonl`、`runs/train_001/training_summary.json`、`runs/train_001/q_table.json` が作られる。training summaryはQ表のSHA-256を保存する。run記録とQ表は`.gitignore`対象。

## 3. 10回評価

```bash
cd submmit_easy_v1
bash scripts/run_repeated_evaluation.sh humble eval_001 train_001
```

評価スクリプトへ学習run IDを渡す。スクリプトは `runs/train_001/q_table.json` を評価に使い、存在しない場合は停止する。評価ノードは各試行前に開始姿勢 `(2.0, 2.0, 0.0)` へteleportし、`/turtle1/pose`が0.10 m・0.20 rad以内へ更新されるまで動作を再開しない。5秒以内に確認できない場合は停止して終了する。実行後に `runs/eval_001/run_metadata.json`、`runs/eval_001/evaluation_trials.jsonl`、`runs/eval_001/evaluation_summary.json` が作られ、評価に使ったQ表のSHA-256と学習run IDを保存する。

## 4. 実行値からレポート表を生成

```bash
cd submmit_easy_v1
python3 scripts/generate_report.py \
  --train-run runs/train_001 \
  --eval-run runs/eval_001 \
  --output REPORT_MEASUREMENTS.md
```

生成する `REPORT_MEASUREMENTS.md` は、ROSディストリビューション・rclpy/Python版・seed・目標/開始姿勢・学習エピソード・評価試行数・成功数/率・平均時間・Q表SHA-256をログから転記する。GitHub URL、YouTube URL、定性的な考察だけは `REPORT_DRAFT.md` に手動で追記する。

生成時には学習・評価のJSONL本文を読み、連番、型、要求回数、成功数、平均時間をmetadata/summaryと照合する。ログの欠落・途中切断・件数不一致ではレポートを生成しない。reset失敗またはreset後Pose待機タイムアウト時はnavigatorが終了コード1を返すため、評価shellも失敗終了する。既存runを上書きせず、原因を直して新しいrun IDで再実行する。

この生成器は学習記録と評価記録のseed、目標・開始姿勢、ROS/Python環境、制御周期、最大ステップ、Q表SHA-256が一致しない場合、混在した成功率を出さずエラーで停止する。パスやPC固有の絶対パスはレポートに出力しない。
