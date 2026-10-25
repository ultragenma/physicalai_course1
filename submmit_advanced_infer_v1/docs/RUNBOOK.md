# 実機・GPU 分離推論の実施手順

この手順は、AIWorkerを実機で動かしたという記録ではありません。各ゲートで記録を残し、失敗したら次のゲートへ進みません。AIWorker固有の配布ソフトウェア、topic名、関節構成、制御器はまだこのリポジトリに確定していません。

## 0. 推奨経路、GPU互換性、役割

ROBOTIS公式経路は、AIWorkerの現行Cyclo IntelligenceでGR00T N1.7 checkpointとtask instructionを指定し、**3D Sim Deploy → Real Robot Deploy**の順で進める経路である。3D Sim Deployは物理ロボットへのcommandを遮断する。GPU PCが対応し、Zenoh経路が確認できている場合はこちらを優先する。

このディレクトリには、NVIDIA公式server-client APIを直接使う別経路のコード例も用意した。確認済みのROS 2 graphを持つ会社ノートPCがAIWorkerから画像と`JointState`を受け、SSHトンネル越しにGPU PCのGR00T N1.7 serverへ推論を依頼する。GPU PC自体がAIWorkerのZenohネットワークへ接続できなくても、会社ノートPCとGPU PC間の承認済みSSH経路があれば推論できる。返却actionはノートPC上の候補topicへ出し、vendor controllerには直接送らない。

```text
AIWorker -- Zenoh / ROS 2 --> 会社ノートPC上のROS 2 client
                                      | RGB + JointState
                                      | local GR00T protocol adapter (ZMQ)
                                      +-- SSH local forward --> GPU PC GR00T N1.7 server
```

ただしCycloの現行prebuilt ACT/GR00T Docker backendは、Blackwellベースのamd64/arm64環境のみ対応と記載されている。GPU PCの型式は未確認なので、まずGPU型式とDockerからのGPU可視性を確認する。ROS 2 topicが取得できているのは会社ノートPCまでである。Cyclo経路ではGPU PCからZenohへの到達性を別途確認する。直接NVIDIA方式では、GPU PCにROS 2/Zenohを接続せず、会社ノートPCからSSHトンネルで推論要求を転送する。

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
```

* **BlackwellでCyclo backend対応、かつ許可済みZenoh経路がある:** Cyclo Intelligence inference panelを第一候補とする。
* **非Blackwell、Cyclo backendが起動不可、またはCyclo用Zenoh経路が未確定:** 会社ノートPCのROS 2 graphとGPU PC間の承認済みSSH経路を使い、NVIDIA公式Isaac-GR00TのGPU server + lightweight clientを評価する。これは本ディレクトリの`groot_zenoh_infer_node.py`が使う構成である。

Cyclo経路ではGPU PCからZenoh daemonへ接続できるかを会社ノートPCのROS graph確認と分けて記録する。直接NVIDIA方式では、会社ノートPCでtopicが見える環境をそのまま使う。どちらもRMW、domain ID、daemon endpointは実機のソフトウェア版と公式設定を見て確定し、推測値を使わない。

| 場所 | 担当 | 必要なもの |
| --- | --- | --- |
| AIWorker | vendor controller、安全機構、物理E-stop | vendorが対応するソフトウェアと安全ルール |
| 会社ノートPC | 確認済みZenoh ROS graph、`groot_zenoh_infer_node.py`、候補gateway | AIWorkerへ接続できるvendor環境、実topic型とjoint順 |
| GPU PC | Cyclo GR00T backend、またはIsaac-GR00T PolicyServer | 対応GPU、checkpoint、隔離環境 |
| PC間通信 | Cyclo経路はGPU PCからZenoh。直接NVIDIA経路はノートPCからSSH local forward | 会社承認済み閉域経路、時刻同期、SSHアクセス制御 |

直接NVIDIA方式は、GPU PC停止・遅延・通信断・古い観測に対して候補actionを発行しないように設計しているが、実機環境での挙動は未検証である。Cyclo Intelligence側の通信断時動作は採用versionの公式手順で確認します。ロボットの非常停止、速度制限、衝突制限および復帰手順は、必ずAIWorkerの公式手順と社内安全ルールを優先し、操作者が物理E-stopを使用できる状態にします。

## 0.1 ゲート0: GPU PCへCyclo Intelligenceを準備する

以下はROBOTISのCyclo Intelligenceセットアップ手順に沿った準備です。GPU PCのGPU型式、ネットワーク接続、会社のソフトウェア配布ルールを確認してから実施します。GPU PCからZenohへ到達できない場合、Cycloを無理に実機へ接続せず、ネットワーク担当者と公式構成を確認します。

```bash
# GPU PC: prerequisites
docker compose version       # Docker 24+ / Compose
git --version
df -h ~                      # Cyclo docs require 35 GB free
nvidia-smi
docker info | grep -i runtimes # output should include nvidia for model inference

# Official Cyclo Intelligence installer
curl -fsSL https://raw.githubusercontent.com/ROBOTIS-GIT/cyclo_intelligence/main/install.sh | bash
cd ~/cyclo_intelligence
./docker/container.sh start
./docker/container.sh enter
```

コンテナへ入った後、既存のZenoh設定とロボットネットワーク上で稼働しているdaemon endpointをベンダー手順に照らして確認します。既に設定がある場合は重複行を追加しません。別PC上のdaemonを使う場合の書式例は次のとおりです。`<AIWORKER_ZENOH_HOST>` は社内で確認した値に置換し、IPアドレスやホスト名は公開Gitへ保存しません。

```bash
grep ZENOH_CONFIG_OVERRIDE ~/.bashrc
# Example only; replace the private endpoint after checking the vendor setup.
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;mode="client";connect/endpoints=["tcp/<AIWORKER_ZENOH_HOST>:7447"]'
cyclo_intelligence
```

Cyclo UIはGPU PCの `http://localhost:7080/` から開きます。外部公開ポートとして開けず、必要なら社内承認済みの端末から安全な方法でアクセスします。Cyclo Intelligenceの正確なversion/commitと公式対応GPUを記録します。

## 1. ゲートA: 接続契約を確定する（read-only）

会社PCの各コマンド例にある `/path/to/verified_vendor_environment.bash` は、現在AIWorkerのtopic取得に成功している接続手順を復元する、確認済みのvendor環境scriptへ置き換えます。このscriptで必要なoverlay、`RMW_IMPLEMENTATION`、`ZENOH_CONFIG_OVERRIDE`、`ROS_DOMAIN_ID`を現在の成功設定どおりに読み込みます。すでに実topicが見えている既存shell/containerをそのまま使う場合だけ、この追加source行を省略できます。ROS Jazzyのsetupだけでは、新しい端末のZenoh接続設定は復元されません。

まずAIWorkerの型式、現行ソフトウェア版、更新手順をベンダー資料と照合し、更新が必要なら公式手順だけで行います。更新作業後にROS 2 topicが取れることを再確認するまでは、Cyclo、学習、推論、実機操作へ進みません。作業前には社内手順に従ってrobot状態・設定の保全方法を確認します。

会社ノートPCでAIWorkerのベンダー環境をsourceした後、コントローラを起動・操作せずにread-onlyで行います。

```bash
cd submmit_advanced_infer_v1
python3 src/inspect_ros_contract.py --output artifacts/ros_graph_before_setup.json
ros2 topic list -t
ros2 action list -t
ros2 service list -t
ros2 topic info --verbose <CAMERA_TOPIC>
ros2 topic info --verbose <JOINT_STATE_TOPIC>
ros2 interface show sensor_msgs/msg/Image
ros2 interface show sensor_msgs/msg/JointState
python3 src/inspect_ros_samples.py \
  --camera-topic <CAMERA_TOPIC> --joint-state-topic <JOINT_STATE_TOPIC> \
  --output artifacts/aiworker_samples_before_setup.json
```

次をベンダー資料と照合して `aiworker.local.yaml` に転記します。

1. `sensor_msgs/msg/JointState` の実topicと実データの `name` の並び。
2. カメラのRGB topic、実データの `frame_id` / encoding / 解像度、時間基準。
3. 公式に許可された制御器のtopic/action、受け付けるメッセージ型、位置／速度／努力の意味。
4. 可動域・速度・加速度、非常停止、制御権限の取得・解除方法。
5. `base_frame`、手首、消しゴム接触面、ホワイトボードを結ぶTF。

`ros_graph_before_setup.json` と設定の実値は社内情報を含み得るので、公開GitHubや提出PDFへ入れません。提出資料には構造図と匿名化済みの確認結果だけを示します。

**合格条件:** `JointState`、少なくとも1つのRGB、公式制御器型、停止方法が文書で確認できること。これらが欠ける間はgatewayもモデルも起動しません。

## 2. ゲートB: 計測・校正・デモ収集

1. 実機と同じ白板、消しゴム、照明、カメラ取付けを固定し、ボード座標系を定義します。
2. 手先座標と消しゴムの接触面中心との差を計測し、tool offsetとして保存します。
3. カメラ内部パラメータ、カメラ対base、base対boardを記録し、再投影誤差を確認します。
4. 低速のテレオペレーションで、接触前→接触→横方向に拭く→離脱の成功・失敗を収録します。人や障害物のない区画を確保し、最初は白板への押付けを使わず空中軌道で再生確認します。
5. 各エピソードに言語指示、開始／終了画像、状態、action、時刻、成功／失敗理由を関連付けます。

学習セットはGR00T LeRobot形式へ変換します。`observation.state` と `action` はfloat32の連結ベクトル、RGBはエピソードMP4、`meta/modality.json` は各成分の順序・単位・absolute/relativeを定義します。AIWorkerの実joint数・hand/arm構成は検証後にのみ入力します。

NVIDIAのcustom embodiment用設定ひな形は `config/aiworker_modality.template.py` です。GPU PCの同じIsaac-GR00T checkoutへコピーし、`TODO_*_KEY`をdatasetの`meta/modality.json`と一致させます。action vectorは関節順・単位・表現を実データに合わせます。`RELATIVE` actionではstate keyとaction keyが異なる場合、`ActionConfig.state_key`をstate modality keyへ合わせます。GR00TのPolicyServerは`ABSOLUTE`/`RELATIVE` actionをdecode後の絶対目標関節位置として返すため、このbridgeの`policy.action_mode`は`absolute_joint_position`のままです。学習済みのaction horizonと1 timestepの秒数も記録します。このtemplateはAIWorkerの実データを代替せず、TODOのままでは学習できません。

AIWorker demonstrationとデータセットを準備し、ゲートDのGPU環境を構築した後の学習コマンド例です。以下の`uv run`例はLinux x86_64のdGPU用です。dataset/config/output path、GPU数、batch size、学習stepは使用環境に合わせて置き換えます。

```bash
uv run python gr00t/experiment/launch_finetune.py \
  --base-model-path nvidia/GR00T-N1.7-3B \
  --dataset-path /secure/datasets/AIWORKER_WHITEBOARD \
  --embodiment-tag NEW_EMBODIMENT \
  --modality-config-path /path/to/aiworker_modality.py \
  --num-gpus 1 \
  --output-dir /secure/checkpoints/AIWORKER_WHITEBOARD \
  --save-steps 500 --max-steps 2000 --global-batch-size 32
```

`NEW_EMBODIMENT`はcustom embodimentとして登録された設定タグで、AIWorker用学習済みpolicyそのものではありません。fine-tuning時に使ったmodality configはcheckpointへ保存され、AIWorker用checkpointをPolicyServerに渡して読み込みます。SO-100のexample configやbase checkpointをそのまま実機へ使いません。

消去成功は「指定ROI内の前後画像でインク画素量がしきい値以下」だけで自動判定せず、反射・残像を避けるため人の確認と併記します。汎化確認には文字位置、太さ、照明、開始姿勢を分けたhold-out条件を使います。

## 3. ゲートC: Cyclo IntelligenceでのSim Deploy

1. Cyclo Intelligenceで対象AIWorkerの確認済みpolicy backend、モデル、センサー設定を選ぶ。
2. GR00T N1.7 checkpoint folder、task instruction、同期/非同期modeを記録する。初回は同期modeと最小execution horizonから始める。
3. **3D Sim Deploy**で起動し、物理AIWorkerにcommandが届かないことを確認する。
4. まずAIWorker用の3D Sim Deployで観測と最初のaction chunkを確認する。これは白板、消しゴム、目標文字を含む新規Isaac Lab/Gazebo/MuJoCo sceneの試験を代替しない。sceneを別途構築した場合は、視覚入力欠損、action限界、停止操作も確認する。
5. sim結果（revision、task、試行数、成功/失敗、動画）を保存する。ここを通過するまでReal Robot Deployを選ばない。

Cycloのpanel名・画面項目はversionで変わり得るため、提出資料にはCycloのgit commit、container image digest、AIWorker software version、checkpoint hash、データセット版、実GPU名と画面記録を添える。実機への切替時にはteleoperationを停止してcontrollerの所有権を明確化する。

## 4. ゲートD: GR00T N1.7 GPU serverとROS client

直接NVIDIA方式を使う場合は、GPU PCでIsaac-GR00T N1.7のrevision `51d4c89` に固定します。このサンプルの軽量ROSクライアントは、同revisionのNVIDIA PolicyServer MessagePack/ZMQ wire protocolに合わせています。古いローカル `Isaac-GR00T-n1.5` はN1.5であり、N1.7用の実行環境と混在させません。GR00T公式資料はカスタムembodiment用fine-tuneとGPU server / lightweight client構成を案内しています。AIWorker用checkpointとmodality configが未作成なら、serverを起動してもタスク推論はできません。

まずGPU型式とdriver、VRAM、OS、Python 3.12を記録します。必要VRAMはN1.7版・画像解像度・camera数・checkpointに依存するため、公式hardware recommendationと実測で決めます。

以下のGPUセットアップはLinux x86_64のNVIDIA dGPU PCを想定します。事前に`uv`、Git、必要なCUDA driver、FFmpeg 4–7のruntimeを用意します。標準dGPU環境の`torchcodec`ではFFmpeg 8を使いません。DGX Spark/Jetson Thor/Orin等のaarch64では、同revisionの[platform別セットアップ](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/scripts/deployment/README.md)を使用します。platform installer実行後は専用venvと`activate_<platform>.sh`をsourceし、以下の学習/server起動の`uv run python`を`python`へ置き換えます。aarch64の専用環境でrootの`uv sync`を再実行しません。

```bash
# GPU PC: use the reviewed NVIDIA Isaac-GR00T N1.7 revision.
uname -m
uv --version
ffmpeg -version
git clone --recurse-submodules https://github.com/NVIDIA/Isaac-GR00T.git
cd Isaac-GR00T
git checkout 51d4c89
git submodule sync --recursive
git submodule update --init --recursive
git rev-parse HEAD # record the full SHA with the experiment artifacts
git submodule status --recursive # record exact submodule SHAs too
uv sync --python 3.12
uv run python gr00t/eval/run_gr00t_server.py \
  --model-path /secure/checkpoints/AIWORKER_ERASER_TODO \
  --embodiment-tag NEW_EMBODIMENT \
  --device cuda:0 --host 127.0.0.1 --port 5555
```

`--host 127.0.0.1` はGPU PC内からだけserverを受け付ける設定です。会社ノートPCからは許可済みSSH local forwardingを作ります。ZMQ server port 5555をネットワークへ直接公開しません。

```bash
# Company laptop: run in the same vendor ROS 2 environment where topics are visible.
# Replace both placeholders with the approved GPU workstation SSH account/host.
# Keep this terminal open while the inference client runs.
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:5555:127.0.0.1:5555 \
  <GPU_PC_SSH_USER>@<GPU_PC_HOST>
```

会社ノートPCでは既にAIWorkerのtopicが見えるROS 2環境を使います。topicがvendor container内で見える場合は、そのcontainer内で以下を行います。Ubuntu/ROS JazzyのPython 3.12と同じPython ABIを使い、GR00T server全体、CUDA、PyTorch、transformersはノートPCへインストールしません。ローカルの `src/groot_policy_client.py` がNVIDIA PolicyServerのwire formatを扱います。

```bash
# Company laptop/vendor ROS container: source the same environment used for ROS checks.
cd /path/to/submmit_advanced_infer_v1
source /opt/ros/jazzy/setup.bash
source /path/to/verified_vendor_environment.bash
command -v ros2
python3 --version
python3 -c "import rclpy; print('rclpy import OK')"
uv --version
uv venv .venv-ros-client --python "$(command -v python3)" --system-site-packages
uv pip install --python .venv-ros-client/bin/python -r requirements-inference-client.txt
source .venv-ros-client/bin/activate
PYTHONPATH=src ./.venv-ros-client/bin/python -c "from groot_policy_client import GrootPolicyClient; print('local GR00T protocol adapter import OK')"
```

`RMW_IMPLEMENTATION`, `ZENOH_CONFIG_OVERRIDE`, `ROS_DOMAIN_ID`を、すでにtopicが見える環境から変えないでください。新しいshellでgraphが見えなくなった場合だけ、[ROBOTISのZenoh接続手順](https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/setup_overview/zenoh_communication/)と社内で確認した値を使って復旧します。外部PCからの一般的な例は`RMW_IMPLEMENTATION=rmw_zenoh_cpp`、AIWorker router TCP `7447`へのZenoh client接続です。domain IDは実環境と一致させ、推測値を設定しません。

## 4.1 ゲートD-1: topicを受けて推論だけ行う

設定例をローカル専用設定へコピーします。ROS topic/frame ID/encodingはグラフとvendor資料で実値を確認し、joint/action順序は学習データ・checkpointと同じにします。推論専用なら`command_topic`とgatewayの関節限界は未使用ですが、カメラtopic/frame、JointState topic、3つの関節順リスト、GR00T video/state/language/action keyは必須です。language keyもcheckpointのmodality configから転記します。`action.duration_s`はcandidateを作る段階まで実機制御には使いませんが、設定parserが必須値として検査します。

```bash
cd /path/to/submmit_advanced_infer_v1
cp config/aiworker.example.yaml config/aiworker.local.yaml
chmod 600 config/aiworker.local.yaml
```

推論serverとSSH tunnelの両方が稼働している状態で、`--publish-candidates`を付けずに起動します。これで1枚の最新RGB、同期したjoint position、language instructionから推論し、結果をlogへ出します。candidate topicもvendor command interfaceもpublishしません。

```bash
cd /path/to/submmit_advanced_infer_v1
source /opt/ros/jazzy/setup.bash
source /path/to/verified_vendor_environment.bash
./.venv-ros-client/bin/python src/groot_zenoh_infer_node.py --config config/aiworker.local.yaml
```

起動時にserverへ`ping`し、接続不可なら停止します。topicのframe IDが設定値と違う、topicのheader stampが0、画像encodingが非対応、camera/joint timestampがずれる、結果が遅すぎる、action key/dimensionが合わない場合はその推論結果を捨てます。カメラencodingは`rgb8`、`bgr8`、`rgba8`、`bgra8`、`mono8`に対応し、画像変換は`cv_bridge`を使いません。

bridgeはROS header時刻の鮮度とmonotonic時計による受信後の鮮度を別々に検査し、推論後にも元の観測時刻を再確認します。gatewayも候補を転送する瞬間にstateのheader時刻と受信時刻を再検査します。robot、camera、nodeのROS時刻基準が合わない場合は起動環境の時刻設定を確認し、鮮度上限を無条件に緩めません。

## 5. ゲートE: action / observation alignment

GR00Tに渡す入力とロボットに送る出力について、次の表をデータセット・推論・gatewayで同一に保ちます。

| 項目 | 確定方法 | 禁止事項 |
| --- | --- | --- |
| 関節順・単位 | `JointState.name` とベンダー仕様 | URDFだけで順番を推定すること |
| state/action key | fine-tuned checkpointのGR00T modality config | SO-100等の例をAIWorkerに流用すること |
| action意味 | データセットとmodality configのabsolute/relative定義 | 絶対角とdeltaを混在させること |
| RGBと時刻 | ROS headerと同期誤差のログ | 最新画像と古い関節状態の結合 |
| video key | fine-tuned checkpointのmodality config | 学習時と本番時でcamera viewを無言で交換 |
| tool/board frame | 手眼・ボード校正の結果 | 見た目だけで接触点を決めること |

このコード例は`video_key`、`state_key`、`language_key`、`action_key`を1つずつ受け付け、stateとactionは同じjoint orderの単一ベクトルを想定します。起動時にPolicyServerからmodality configを取得し、key名とvideo/state/languageの`delta_indices=[0]`を照合します。また、`action.delta_indices`に`chunk_index`が存在し、その選択値が`0`であること、`ActionConfig.rep`が`ABSOLUTE`/`RELATIVE`であり、RELATIVE時の参照state keyが観測state keyに一致することを確認します。GR00T APIのRGBは`uint8` `(B,T,H,W,3)`、stateは`float32` `(B,T,D)`、languageは`[[str]]`、actionは`(B,T,D)`です。ここでは`B=1, T=1`の観測を送り、現在stepに対応する絶対目標action pointを1つ選びます。`action.duration_s`は学習時のaction timestepと承認済みcontroller範囲の両方に合わせます。腕とgripperが別stream、Cartesian/EEF action、`DELTA` representation、複数カメラ、履歴複数frameのcheckpointにはdecoderを合わせる必要があります。AIWorker用actionは、base zero-shotで実機投入せず、AIWorker demonstrationと独自modality configでfine-tuneしてから評価します。

## 5.1 ゲートE-1: candidate topicの確認

起動時のaction検査では `ActionConfig.type=NON_EEF` と `format=DEFAULT` も必須です。EEF/Cartesian、XYZ回転形式、`DELTA` representationは、出力次元が偶然関節数に一致しても拒否します。

推論logを確認したあと、vendor資料で `JointTrajectory` command topicが確認でき、設定済みの場合に限り、実機を動かさずcandidate topicをpublishして別端末でmessage内容を確認できます。bridgeは誤remapを防ぐため、candidate publish時にvendor確認済みcommand topicの設定を必須とします。controller APIがaction/service等である場合、このサンプルはcandidate publishを拒否します。ROS 2/Zenoh graphにcandidate topic自体は見える可能性がありますが、vendor command/action topicとは必ず別名にし、vendor controllerへ接続しません。`ros2 topic echo`には設定した `policy_action_topic` の実値を指定します。

```bash
# Terminal A: use the same vendor ROS 2 / Zenoh environment as the topic checks.
cd /path/to/submmit_advanced_infer_v1
source /opt/ros/jazzy/setup.bash
source /path/to/verified_vendor_environment.bash
./.venv-ros-client/bin/python src/groot_zenoh_infer_node.py \
  --config config/aiworker.local.yaml --publish-candidates
```

別端末BもAIWorkerのtopicが見えるvendor環境で開き、同じRMW/Zenoh/domain設定を使用します。

```bash
# Terminal B: inherit the verified vendor Zenoh environment.
source /opt/ros/jazzy/setup.bash
source /path/to/verified_vendor_environment.bash
# Replace with interfaces.policy_action_topic from aiworker.local.yaml.
ros2 topic echo <POLICY_ACTION_TOPIC>
```

gatewayを使ったcandidate filterの確認は、command interfaceがvendor資料で特定され、設定値が埋まり、別の無効化済みcontroller/simulatorで行うまで保留します。gatewayの`arm_parameter:=false`はcandidateをすべて拒否し、vendor command topicへpublisher登録もしません。bridge/gatewayはcandidate topicとcommand topicの同一指定を拒否し、remapされたcandidate/joint-state/armed command topicでは起動しません。`true`は明示的な限定試験以外で使いません。実機arm前にZenoh/ROSネットワーク上で candidate topicは承認済みbridge nodeだけがpublishし、gatewayはsubscribeのみ、vendor command topicはgatewayだけがpublishできるACL/隔離を確認します。設定できない場合はarmしません。candidate filterはvendor safety controllerやE-stopの代替ではありません。

## 6. ゲートF: Real Robot Deploy とgeneric gatewayの段階試験

1. Cyclo経路なら3D Sim Deploy合格後、teleoperationを停止し、公式手順でReal Robot Deployへ移す。direct-NVIDIA経路なら、先にvendor controllerと全停止機能を確認し、別の無効化済みcontroller/simulatorでcandidate gatewayを評価する。
2. 候補フィルターがROS header時刻、関節順・範囲・duration・平均速度の違反を拒否することを確認する。これは一部のcandidate検査にすぎない。gatewayは非常停止、加速度、衝突、作業空間、接触力、制御権限、通信断後の停止を保証しない。メーカーのcontrollerと安全層が常に必要であり、現在のscaffoldは実機に接続しない。
3. 白板から離れた姿勢で、`max_step_rad` と1点だけのtrajectoryで低速replayを行う。
4. 操作者が非常停止に手を置き、空中で短いclosed-loopを確認する。
5. 白板に近づけるのは、空中replay、通信切断、再起動、E-stop、operator takeoverの全試験が記録された後である。

gatewayの初回起動例（実機topic/action契約と全limitを確認後。falseではcandidateを転送しない）:

```bash
cd /path/to/submmit_advanced_infer_v1
source /opt/ros/jazzy/setup.bash
source /path/to/verified_vendor_environment.bash
./.venv-ros-client/bin/python src/ros_action_gateway.py --config config/aiworker.local.yaml \
  --ros-args -p arm_parameter:=false
```

初回は常にfalseです。`true` はベンダー承認済みのaction interface、関節limit、非常停止が確認済みの限定試験でのみ使います。

## 7. 動画とレポートの実績記録

採択する動画では、AIの入力（言語指示・カメラ）、推論PC、ロボット動作、前後の白板を一続きで示します。成功時だけを編集して成功率を示しません。試行回数、成功定義、失敗例、速度倍率をPDFと概要欄に記載します。

実行未了の現時点では、`REPORT_DRAFT_ja.md` の `[未実施]` を成功表現に変更しません。実施後に日時、hardware/software revision、Cyclo commit/container digest、GPU名、checkpointとdataset hash、実行回数、成功数、観測された失敗を置換します。
