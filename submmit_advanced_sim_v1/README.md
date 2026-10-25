# AIWorker Whiteboard Eraser (simulation submission draft)

ROBOTIS AI Worker がホワイトボードの文字を消す課題の準備一式です。現時点ではAIWorkerの型式とシミュレータ、実機ソフトウェア更新が未確認のため、このv1は**提出用の設計・マスク評価器・幾何経路生成器・観測ノード雛形**を収録しています。AI方策、画像からマスクを作る処理、ロボットへの動作指令は未実装で、シミュレータ・実機での動作成功も主張しません。

## AI部分と現在の実装範囲

この最終課題では、別GPU PC上のGR00T N1.7をAI方策として利用するのが本案です。AIWorkerは本タスクの既知embodimentではないため、AIWorkerでテレオペレーションデモを収録してカスタムembodimentとしてfine-tuneする必要があります。データ収録・学習・推論はこのv1では実行していません。

実装済みの評価関数は、画像そのものではなく、外部で作成した二値の筆跡マスク・白板マスクを入力として消去率などを計算します。マスク生成モデル、再走査を決める閉ループ、衝突回避を含むモーション実行器はまだありません。したがって現状は最終提出用の実演ではなく、課題を実装する土台です。

## ディレクトリ

| パス | 内容 |
| --- | --- |
| `whiteboard_task/` | ROS 2 Pythonパッケージ雛形。二値マスク評価、ボード座標系の幾何経路生成、最大300件に制限した画像トピックのメタデータ取得を含む。動作指令は含まない。 |
| `config/aiworker_interface.example.yaml` | 利用環境で確定させるべきトピック、フレーム、MoveItアクション名。空文字は未確定値。 |
| `assets/` | マスク作成の説明だけを置く。画像・マスク・校正・実験JSONなどの実データはGit管理外の`artifacts/<run_id>/`に置く。 |
| `schema/`, `records/templates/`, `tools/` | 実験環境・試行・metricsをJSON/CSVで記録し、レポート転記用ファクトを生成する。 |
| `docs/` | 実装・実験手順、提出用レポート草稿、動画台本、実験・提出チェックリスト。 |
| `requirements-metrics.txt` | 二値マスク評価器を使うためのNumPy/OpenCV依存。画像セグメンテーションモデルは含まない。 |

## シミュレータの選び方

AIWorker向けの第一候補は、ROBOTIS `cyclo_lab` のIsaac Lab環境です。公開READMEにはFFW-BG2の例がありますが、使う型式はまだ未確認なので、その資産が対象型式に合うか最初に確認します。AIWorker Simulationの従来ページはGazebo URDF/SDFとMuJoCo MJCFを案内する一方、Isaac Sim USDを「upcoming」と記載しており、公式ページ間に情報差があります。利用するasset variant・取得リビジョン・ライセンスを実験時に固定します。

- [ROBOTIS cyclo_lab](https://github.com/ROBOTIS-GIT/cyclo_lab)
- [ROBOTIS AI Worker: Simulation](https://docs.robotis.com/docs/systems/aiworker/simulation/)
- [ROBOTIS AI Worker open-source inventory](https://docs.robotis.com/docs/systems/aiworker/resources/open_source/)
- [AI Worker Isaac VR teleoperation / SH5 DDS bridge](https://docs.robotis.com/docs/systems/aiworker/resources/technical_story/isaac_vr_teleoperation/)

## 実施順序

1. `config/aiworker_interface.example.yaml` をコピーして `config/aiworker_interface.yaml` とする。会社PCで取得できるROS graphのtopic/type、TF frame、MoveIt actionをベンダー資料と照合して、実際に確認できた値だけを入力する。型式、TF、MoveIt actionは未確認のため推測で補わない。これはGit管理しない。
2. 対象型式に対応した `cyclo_lab` asset が確認できる場合は、そのIsaac Labサンプルからカメラ、白板、消し具を追加する。対応しない場合は公式URDF/SDF（Gazebo）またはMJCF（MuJoCo）を選ぶ。座標系とカメラ内部・外部パラメータを保存する。
3. 消去前後の画像から筆跡マスク、白板マスク、対象文字だけを囲うROIマスクを作成し、マスクと実行ログを `artifacts/` に保存する。`erase_assessment.py` は対象ROI内の筆跡画素減少と、白板外で変化した筆跡マスク画素を算出する。後者は原画像全体の変化や白板損傷を測定するものではない。このv1にはマスク生成器がないため、使用した手法・モデル・閾値を記録する。
4. 低速、低い接触力、非常停止を設定したうえで、シミュレータから実機へ段階移行する。実機は担当者が安全確認を完了してからのみ実行する。

評価器の依存を隔離して準備する場合:

```bash
uv venv .venv-metrics
uv pip install --python .venv-metrics/bin/python -r requirements-metrics.txt
RUN_ID=sim_YYYYMMDD_001
mkdir -p "artifacts/${RUN_ID}/masks"
.venv-metrics/bin/python whiteboard_task/whiteboard_task/erase_assessment.py \
  --before-ink "artifacts/${RUN_ID}/masks/before_ink_mask.png" \
  --after-ink "artifacts/${RUN_ID}/masks/after_ink_mask.png" \
  --board-mask "artifacts/${RUN_ID}/masks/board_mask.png" \
  --target-roi-mask "artifacts/${RUN_ID}/masks/target_roi_mask.png" \
  --output-json "artifacts/${RUN_ID}/metrics_t001.json"
```

この単発評価例には、すでに作成済みの同解像度マスクが必要です。`target_roi_mask` は対象文字だけを囲み、`before_ink` を含む必要があります。v1はカメラ画像から筆跡を検出せず、白板外の原画像変化も測定しません。trial記録とレポート生成は[実験記録手順](docs/EXPERIMENT_RECORDING.md)を正本として使ってください。

実行後の記録とレポート転記の正確な手順は[実験記録手順](docs/EXPERIMENT_RECORDING.md)を参照してください。詳細は[実装計画](docs/IMPLEMENTATION_PLAN.md)、[運用チェックリスト](docs/EXPERIMENT_CHECKLIST.md)、[提出チェック](docs/SUBMISSION_CHECKLIST.md)にあります。

レポートPDF草稿は `説明資料_PAI最終課題_アカウント名要入力_草稿.pdf` です。提出前に実測値で `docs/REPORT_DRAFT.md` を更新し、課題指定のファイル名へ変更してください。

## 実行状態

この環境にはAIWorkerの型式・公式モデル・ROS 2ミドルウェア・MoveIt設定・実機接続がないため、実行検証は行っていません。実行済みの動画、評価値、成功率は含まれません。本案を最終課題として提出するには、AI方策の実装、シミュレーション、動画撮影、測定値を入れたPDFレポートが必要です。
