# AIWorkerの観測をMolmoAct2へ渡して推論する

2026-10-28。現在AIWorkerは未接続。このPCは観測・検査を担当し、モデルは別のCUDA GPU PCで実行する。
実装とCPU通信試験を完了した。モデルをダウンロード済みだが、このPCで実モデルのforwardは行っていない。

## 用意した2つのプロファイル

| プロファイル | 入力 | 出力 | 利用範囲 |
| --- | --- | --- | --- |
| `configs/molmoact2/droid_probe.json` | 頭カメラを2スロットへ複製、右手カメラ、右腕7関節とグリッパー | Frankaの絶対関節・グリッパー、15×8 | 公開モデルへの入出力を調べる診断用。AIWorkerの動作指令としては扱わない |
| `configs/molmoact2/sg2_native.json` | 頭・左右手カメラ、名前順の19次元 | 学習済みAIWorker方策の19次元 | AIWorker向けに微調整し、対応する正規化統計を持つチェックポイント用 |

[公式モデルカード](https://huggingface.co/allenai/MolmoAct2-DROID)ではDROIDモデルの対象はFrankaで、
`norm_tag=franka_droid`、絶対関節pose制御である。AIWorkerと同じ7関節の腕でも、軸・零点・機構・グリッパー尺度は一致しない。
診断プロファイルはAIWorkerの生の右腕状態を条件として送るが、これを校正済みFranka状態とは主張しない。
返却値は `franka_joint1` などの名前を保持し、`aiworker_action_layout=false` として記録する。

そのため、**公開DROIDモデルをダウンロードしただけではAIWorker向けの学習済み方策にはならない**。
19次元を単純に8次元へ切り詰めたり、出力を19次元へ水増しして制御する処理は実装していない。
現在のすべての入口は推論結果の検査・保存までで、ROSのアクチュエータpublisherを作らない。

## ファイルと処理の流れ

```text
AIWorkerのROS / Zenoh観測
  -> molmoact2_ros.py: 3カメラ・関節を同期、時刻とmetadataを確認
  -> molmoact2_client.py: 名前順state + RGB PNGをHTTP送信
  -> SSH転送 -> GPU PCのmolmoact2_server.py
  -> MolmoAct2.predict_action(..., continuous, norm_tag=...)
  -> 名前つきaction chunkを検査し、privateなruns/へ保存
```

- `wb_eraser/robot_schema.py`: 既存GR00T経路とも共有する関節・カメラ名、時刻検査
- `wb_eraser/molmoact2_contract.py`: checkpoint次元・正規化tag・カメラ順・全actionの検査
- `wb_eraser/molmoact2_server.py`: GPUモデルの読込と `/health`, `/schema`, `/infer`
- `wb_eraser/molmoact2_client.py`: GPU依存のないHTTPクライアントとファイル入力CLI
- `wb_eraser/molmoact2_ros.py`: 同期ROS観測からの連続した推論呼出し
- `scripts/molmoact2_smoke.py`: 公式DROIDサンプル観測を使うGPU側のforward確認
- `configs/molmoact2/model.lock.json`: 使用モデル、固定revision、公式実装commit、次元、ハッシュ

## 1. モデルの準備

Hugging Face CLIを利用する。保存先はGit管理外の `checkpoints/`。
この作業で次のrevisionを取得済み。5つの重みファイルの合計は21,768,955,912 bytes。

```bash
hf download allenai/MolmoAct2-DROID \
  --revision d8c1abd8a27d8e859455bbe514df2bcc617db0fb \
  --local-dir checkpoints/molmoact2-droid --max-workers 4

hf cache verify allenai/MolmoAct2-DROID \
  --revision d8c1abd8a27d8e859455bbe514df2bcc617db0fb \
  --local-dir checkpoints/molmoact2-droid --fail-on-missing-files

# CPUだけで実際のtag、カメラ順、state/action次元、重みの有無を確認する
python -m wb_eraser.molmoact2_server --checkpoint checkpoints/molmoact2-droid --inspect
```

公式照合では24ファイルのチェックサム確認に成功した。
`hf cache verify`が通知する追加ローカルファイルはダウンロード用cache/metadataであり、元のモデルファイルは改変していない。
GPU PCへコードと `checkpoints/molmoact2-droid/` をコピーするか、GPU PC上で同じrevisionを取得する。

## 2. 別GPU PCで起動する

GPU環境はPython 3.11または3.12。ROS側とは仮想環境を分ける。
依存の版は[公式実装](https://github.com/allenai/molmoact2/blob/6070080a20321b4f498ab30f28e1d09ac465edb7/pyproject.toml)に合わせた。
利用するGPU・ドライバにCUDA 12.8 wheelが適合することはGPU PC側で確認する。

```bash
python3.12 -m venv .venv-molmoact2
.venv-molmoact2/bin/python -m pip install --upgrade pip
.venv-molmoact2/bin/python -m pip install \
  torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128
.venv-molmoact2/bin/python -m pip install -r requirements-molmoact2-server.txt

# まず公式Franka観測で、GPU・重み・processor・forwardを確認する
.venv-molmoact2/bin/python scripts/molmoact2_smoke.py \
  --checkpoint checkpoints/molmoact2-droid --device cuda:0 --dtype bfloat16

.venv-molmoact2/bin/python -m wb_eraser.molmoact2_server \
  --checkpoint checkpoints/molmoact2-droid \
  --profile configs/molmoact2/droid_probe.json \
  --host 127.0.0.1 --port 8210 --device cuda:0 --dtype bfloat16 --warmup
```

CUDAが使えない場合、CPUへ自動で切り替えて巨大モデルを読込む処理は行わない。
BF16に非対応のGPUでは `--dtype float16` または十分なVRAMを確保して `float32` を選ぶ。
CUDA graphは無効。公式サーバにある入力テンソルのdtype変換をインスタンスに適用する。
固定revisionではflow trajectoryのdtypeとBF16からNumPyへの変換は公式側で修正済みで、cache内のソースは書き換えない。

## 3. このPCから接続する

既存のSSH接続先を使い、GPU PCの8210番を転送する。以下の `USER@GPU_HOST` は実環境の接続先に置換する。

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8210:127.0.0.1:8210 USER@GPU_HOST

.venv/bin/python -m pip install -r requirements-molmoact2-client.txt
.venv/bin/python -m wb_eraser.molmoact2_client probe \
  --profile configs/molmoact2/droid_probe.json --out runs/molmoact2/server_probe.json
```

probeでは画像や関節値を送らず、healthとmodel schemaだけを確認する。
ホストで張った転送をROSコンテナ内で使う場合は、そのコンテナから同じ転送先へ到達できるネットワーク構成が必要。
推論クライアントはIP、RMW、Zenohルータの設定を変更しない。

## 4. AIWorkerへ再接続したらROSから推論する

接続が成功しているvendor ROS/Jazzy環境を使う。カメラtopicは既存の取得手順に合わせる。
まず実受信件数を含むmetadataから `client_contract.json` を作る。

```bash
# ホスト側の既存取得ツール。既に確認した接続設定を引き継ぐ
.venv/bin/python scripts/capture_ros.py --out runs/connection/observations.json
.venv/bin/python -m wb_eraser.inference_contract runs/connection/observations.json \
  --out runs/connection/client_contract.json

# 以下はトピックを取得できるROS/Jazzy環境内
python3 -m venv --system-site-packages .venv-ros-client
.venv-ros-client/bin/python -m pip install 'numpy>=1.26,<2' -r requirements-molmoact2-client.txt
.venv-ros-client/bin/python -c 'import rclpy, cv_bridge, message_filters, PIL'

.venv-ros-client/bin/python -m wb_eraser.molmoact2_ros runs/connection/client_contract.json \
  --profile configs/molmoact2/droid_probe.json --url http://127.0.0.1:8210 \
  --instruction 'Erase all marks from the whiteboard.' --frames 3 \
  --out runs/molmoact2/ros_inference.json
```

3カメラとJointStateを同期待ちし、frame_id・解像度・encoding・時刻・関節名を照合する。
画像はRGBにしてPNGで転送し、診断時は `[head, head, right_wrist]` の順でモデルへ渡す。
モデルの返却値は全15ステップを検査し、関節ラベル・応答ID・元観測時刻を照合する。
`--max-result-age` は既定5秒。古い応答は診断用に保存するが `complete=false` とする。
画像は既定では保存せず、推論値と検査結果をGit管理外の `runs/` へ記録する。

`complete=true` は同期観測と推論応答の経路が通ったことを示す。
白板の消去成功や、AIWorkerでの動作妥当性を確認したことは示さない。

## 5. 録画済みの観測を入力する

privateなNPZファイルに、RGB画像配列と `joint_names`（文字列配列）、`positions`（数値配列）を保存する。
画像のキーは `cam_left_head`, `cam_left_wrist`, `cam_right_wrist`。
`allow_pickle=False` で読み、object配列は受け付けない。

```bash
.venv/bin/python -m wb_eraser.molmoact2_client infer-file \
  --input runs/molmoact2/observation.npz --instruction 'Erase all marks from the whiteboard.' \
  --out runs/molmoact2/file_inference.json
```

録画入力ではliveの鮮度確認を行ったとは記録しない。GPU接続ができればROSなしで入出力の調査ができる。

## 6. AIWorker用チェックポイントへ切り替える条件

MolmoAct2をAIWorkerの実演データで微調整し、その学習時の19次元正規化統計を同梱する必要がある。
公開DROIDモデルの正規化tagを書き換えるだけでは対応しない。
nativeプロファイルはcheckpointの `norm_stats.json` に次の情報がある場合だけ起動する。

```text
metadata_by_tag.aiworker_sg2_stationary:
  robot_type: robotis_aiworker_sg2
  state_joint_names: [左腕7, 左gripper, 右腕7, 右gripper, 頭2, lift]
  action_joint_names: 同じ19関節の名前順
  camera_order: [cam_left_head, cam_left_wrist, cam_right_wrist]
  action_representation: absolute_joint_positions
  state_stats / action_stats: 学習データから計算した19次元の統計
  action_horizon / n_action_steps: 実際に学習した値
```

実際の関節名は `configs/molmoact2/sg2_native.json` に列挙している。
モデルの最大horizon・最大action次元と整合していることも検査する。

```bash
.venv-molmoact2/bin/python -m wb_eraser.molmoact2_server \
  --checkpoint /path/to/aiworker_finetuned_molmoact2 \
  --profile configs/molmoact2/sg2_native.json --device cuda:0
```

この場合も実装範囲は推論結果の保存まで。joint actionをMoveLへ変換したり、関節制御へ配信したりする処理は含まない。

## 検証状況

- 全CPU回帰テスト57件を実行し成功。MolmoAct2追加分14件を含む。
- 3つのモデルカメラスロット、名前順state、PNGの可逆転送、HTTP実通信、action全体の形状・finite検査を確認。
- 19次元という長さだけで他ロボットのpolicyを受け入れず、nativeの関節順metadata不一致も拒否することを確認。
- checkpointの実ファイルをCPUで解析し、DROIDの15×8出力と5つの重みファイルを確認。
- 外部ネットワークなし・domain 73・localhost-onlyの独立ROS/Jazzyコンテナで、模擬3カメラと19関節を配信。
  実際の `molmoact2_ros` プロセスとHTTPサーバを通し、DROID診断の15×8とnative形式の15×19を各3回受信・保存した。
  このbackendとnative checkpointのmetadataは試験fixtureであり、学習済み方策ではない。
- 公式DROIDモデルによるGPU forward、実AIWorkerの同期ROS経路、AIWorkerのタスク成功は未検証。
- CPUテストのbackendは通信fixtureであり、AI方策や成功実績として扱わない。

試験の再現（Docker試験は実機へ接続せず、既存ai_workerコンテナを変更しない）:

```bash
.venv/bin/python -B -m unittest tests.test_molmoact2 -v
docker build -f Dockerfile.molmoact2-ros-test -t physicalai-molmo-ros-test .
.venv/bin/python -B scripts/check_molmoact2_ros.py --image physicalai-molmo-ros-test
```
