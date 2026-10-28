# 追加学習なしのMolmoAct2対象選択とMoveL実行

2026-10-28。タスクの成功率を前提にせず、観測から動作指令までつながる試行プログラムを実装した。
ユーザーのGPUはRTX 5090。SSH接続先は確認中で、実GPU・実AIWorkerでの実行は未検証。

## 構成

```text
AIWorker RGB + registered depth + CameraInfo + TF
    -> CVで白板・文字領域を検出し番号付き画像を作成
    -> RTX 5090上のMolmoAct2がJSONで消去対象IDを返す
    -> 深度・板面・TFから3Dの拭き取り経路を計算
    -> /r_goal_move (robotis_interfaces/msg/MoveL)
    -> /r_gripper_pose の位置応答
    -> 終了後の画像で消し残しを記録
```

ダウンロード済みMolmoAct2-DROIDの画像・言語生成部分を `model.generate()` で利用する。
Franka用の関節actionは使わず、モデル出力は検出済み領域のIDだけとする。
この使い方ではAIWorker用の19次元actionを学習する必要はなく、追加FT/LoRAは実施しない。
ただし、DROIDチェックポイントで文字領域を正しく選べるかはまだGPUで評価していない。

`vlm_server.py` の生成方法は、取得した公式 `modeling_molmoact2.py` の `forward()` に記載された
画像QAのchat-template / generate / decode例に合わせている。
出力が不正な場合も生テキストとエラーを記録する。模擬backendを実モデルの推論と扱わない。

## 1. RTX 5090側

このフォルダを作業ディレクトリとする。Python 3.12とNVIDIAドライバが利用できるGPU PCで実行する。
以下は専用venvにCUDA 12.8対応のPyTorchを導入し、sm_120と小さなCUDAカーネルを確認する。

```bash
python3 scripts/setup_molmoact2_gpu.py --download-model
# 既に環境がある場合の確認のみ
python3 scripts/setup_molmoact2_gpu.py --check-only

.venv-molmoact2/bin/python -m wb_eraser.vlm_server \
  --checkpoint checkpoints/molmoact2-droid \
  --host 127.0.0.1 --port 8211 --device cuda:0 --dtype bfloat16
```

使用モデルrevisionは `configs/molmoact2/model.lock.json` で固定している。
このPCには同revisionの全重みを取得・照合済みなので、GPU PCへコピーしてもよい。
GPU側サーバはROSへ接続せず、画像を受けて対象IDを返す。
既存の関節推論サーバ8210とは別の入口で、今回使うのは8211。

## 2. ROS側の準備

GPU PCへのSSH転送を用意する。実接続先に置換する。

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8211:127.0.0.1:8211 USER@GPU_HOST
```

既にトピックを受信できるAIWorker vendor環境でROSとSDKのsetupを読み込む。
既存のZenoh接続値とROS domainは保持し、テスト用のdomain 73へ切り替えない。
ROSコンテナ内からGPU側8211へ到達できることが必要。

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
python3 -m venv --system-site-packages .venv-ros-client
.venv-ros-client/bin/python -m pip install -r requirements-hybrid-ros-client.txt
.venv-ros-client/bin/python -c 'import rclpy, cv_bridge, message_filters, tf2_ros, cv2, PIL; from robotis_interfaces.msg import MoveL'
```

RGB、同じ光学座標系に登録済みのdepth、CameraInfo、`base_link`からカメラへのTFが必要。
MoveL controllerが `/r_goal_move` を購読し、時刻付きの `/r_gripper_pose` を配信していることも確認する。
プログラム自身はbringupやcontrollerの起動、Zenoh設定変更を行わない。

## 3. 対象選択と経路の確認

```bash
.venv-ros-client/bin/python -m wb_eraser.hybrid_run \
  --no-head --rounds 1 --selector-url http://127.0.0.1:8211 \
  --image-topic /zed/zed_node/left/image_rect_color \
  --depth-topic /zed/zed_node/depth/depth_registered \
  --info-topic /zed/zed_node/left/camera_info \
  --instruction 'Erase all writing from the whiteboard.' \
  --out runs/hybrid/plan
```

既定では計画までで、MoveLを送らない。画像・番号付き画像・モデル返答・対象ID・3D経路が保存される。
実際のtopicが異なる場合は引数で指定する。画像はPNGで転送し、画像hashとrequest IDで応答を対応付ける。
モデルが存在しないIDやJSON以外を返した場合、または応答が古い場合は実行へ進まない。

## 4. 動作指令を送る

同じコマンドへ **`--execute`** を追加する。

```bash
.venv-ros-client/bin/python -m wb_eraser.hybrid_run \
  --execute --no-head --rounds 1 --selector-url http://127.0.0.1:8211 \
  --instruction 'Erase all writing from the whiteboard.' \
  --out runs/hybrid/attempt
```

右腕のready移動、接近、押し当て、往復、離脱、readyへの復帰までMoveLを送信する。
工具の取り付けに合わせて `--tool-offset`、`--tool-quat X Y Z W`、`--ready X Y Z` を指定する。
既定の位置・姿勢は検証用の初期値で、実際の工具寸法や取り付け姿勢を測ったものではない。
カメラ外部校正・TF・workspace・板面推定の整合も必要だが、タスク専用の学習データは不要。
接触力の計測・力制御・衝突回避はこのプログラムの検証範囲に含めない。

AIのJSON応答が不正でも幾何経路の試行を優先する場合、`--selection-fallback all` を明示できる。
その場合は全検出領域を対象とし、結果を `explicit_cv_fallback` と記録する。
これはAIによる対象選択の成功とは扱わない。古い観測の拒否はfallbackでも保持する。

## 実行完了とタスク成功を分ける

`summary.json` の主要項目:

| 項目 | 意味 |
| --- | --- |
| `pipeline_complete` | 対象選択・計画・指令列の試行が実行上のエラーなく完了したか |
| `task_success` | 全消去を想定したCV指標で、文字が所定割合未満になったか。未評価ならnull |
| `task_success_measured` | 終了後の画像を取得・評価できたか |
| `movel_commands_sent` | 実際にpublishしたMoveLメッセージ数 |
| `move_records` | 各移動の到達判定・位置誤差。接触区間の時間待ちは力や消去成功の計測ではない |
| `ai_requests` / `selector` | 対象選択APIの呼出し回数とbackendの種類 |

**消し残しがあっても指令列の試行が完了すればexit 0** とする。
ROI選択・RGB-D/TFの実行エラーはexit 4、経路拒否や未到達はexit 3、白板検出・板面失敗はexit 2。
`task_success` は全消去用の画像指標で、任意の言語指示の意味的達成や実機の力制御を保証するものではない。

## 検証済みの範囲

全CPU回帰テスト63件が成功。外部ネットワークなし、FastDDS/domain 73の独立コンテナでも確認した。
実SDKの `robotis_interfaces/msg/MoveL` の型を使い、RGB-D・CameraInfo・TFから選択APIを経て、
MoveL subscriberとCartesian位置応答まで実際のROS通信を通している。
模擬boardは消さずに固定し、8件のMoveLが受信された後に `pipeline_complete=true` / `task_success=false` / exit 0となることを確認した。

他に、計画のみ、モデル応答不正、明示fallback、古い応答、controller未到達の各ケースを検証した。
試験backendは固定JSONを返すfixtureで、MolmoAct2の実モデルではない。IK・接触物理も模擬していない。
実GPU推論と実AIWorker上の動作は、SSH接続先・実機接続が整った後に確認する。

```bash
.venv/bin/python -B -m unittest discover -s tests -p 'test_*.py' -v
.venv/bin/python -B scripts/check_molmoact2_ros.py --fixture hybrid
```

ROS試験は既存AIWorkerコンテナを操作せず、使い捨ての隔離コンテナを使う。
現在の試験イメージ `physicalai-turtlesim:jazzy-v1` はvendor SDK、ROS、FastDDS、OpenCVを含む。
別環境では `docker build -f Dockerfile.molmoact2-ros-test -t physicalai-molmo-ros-test .` で試験用イメージを作り、
`scripts/check_molmoact2_ros.py --fixture hybrid --image physicalai-molmo-ros-test` と指定できる。
試験ログ・画像・MoveL記録は `runs/hybrid/ros_fixture/` に保持し、Gitへ追加しない。
