# Physical AI 応用1 最終課題 — 作業状況

更新日: 2026-10-28 (MolmoAct2追加)

| 成果物 | 内容 | 現状 | 次の実行ゲート |
| --- | --- | --- | --- |
| [`submmit_easy_v1`](submmit_easy_v1/README.md) | ROS 2 turtlesim のQ-learning目標到達 | コード・説明資料草稿・PDF草稿あり。ROS 2 build、学習、GUI実演は未確認。 | ROS 2環境でcolcon build、学習1回、独立評価、動画収録、実測記入。 |
| [`submmit_advanced_sim_v1`](submmit_advanced_sim_v1/README.md) | AIWorkerシミュレータ上のホワイトボード消去 | マスク評価・幾何経路・観測メタデータの準備コード。AI方策、画像認識、動作実行、シミュレーションは未実装・未実施。 | AIWorker型式と更新版を特定し、対象assetを固定して白板scene・認識・動作を実装。 |
| [`submmit_advanced_infer_v1`](submmit_advanced_infer_v1/README.md) | 別GPU PCでのGR00T N1.7推論サンプル | ノートPCからROS 2 topic取得済み（依頼者確認）。ROS観測bridge・軽量ZMQ client・候補trajectory・既定disarm gatewayを実装し3系統の静的レビュー済み。対象PCでの実行は未実施。 | AIWorker更新とvendor契約を確認。既存Zenoh環境、GPU/VRAM、AIWorker用checkpointを確定し、推論のみ→候補確認→段階試験へ進む。Cyclo利用時はSim Deployを先行。 |
| [`MolmoAct2`](submmit_advanced_infer_v1/molmoact2/README.md) | 別GPUのMolmoAct2にAIWorker観測を送る | 固定DROID重みの24ファイルを公式チェックサム照合。CPUテスト14件、隔離ROSの3カメラ・関節→HTTP→fixture応答を検証。 | GPU PCで公式サンプルのforwardを確認。公開Franka方策は診断用。AIWorkerでの利用には微調整済み19次元checkpointが必要。 |

## 実行後の記録とレポート

- MolmoAct2追加分は[実装・検証・起動手順](submmit_advanced_infer_v1/molmoact2/docs/MOLMOACT2.md)を参照。現在AIWorkerは未接続。実モデルのGPU推論と実機タスクは未検証で、以下の従来案とは検証状況を区別する。モデル重みと模擬ROS試験ログはローカルに保持し、Gitへ含めない。
- 簡易案: [RUNBOOK](submmit_easy_v1/RUNBOOK.md)の学習→10試行評価→[実測表生成](submmit_easy_v1/scripts/generate_report.py)を実施する。学習Q表は `runs/<train_run_id>/q_table.json` に保存し、評価run ID・Q表SHA-256・条件一致をレポート生成時に照合する。評価結果を使ってPDF草稿を更新する。
- 上級sim案: [実験記録手順](submmit_advanced_sim_v1/docs/EXPERIMENT_RECORDING.md)のrun metadata、metrics、trial JSON、プライバシーゲート付きreport factsを使う。実験データ・映像・校正情報は `artifacts/<run_id>/` に保存し、Git管理しない。公開用集計はmetadataの個人情報・社内ネットワーク情報・機密情報の確認を通過後にのみ生成する。
- 記録/レポート用コードと推論サンプル・手順書を整えたが、ROS 2 build、turtlesim実行、sim、AI推論、実機操作、metrics算出は実施していない。

PDF草稿: [easy](submmit_easy_v1/説明資料_PAI最終課題_アカウント名要入力_草稿.pdf) / [advanced sim](submmit_advanced_sim_v1/説明資料_PAI最終課題_アカウント名要入力_草稿.pdf) / [advanced inference](submmit_advanced_infer_v1/説明資料_PAI最終課題_アカウント名要入力_草稿.pdf)

3つのPDFはMarkdown草稿から生成済みですが、OmniCampusアカウント名は未入力です。未実測の結果を空欄/TODOとして残しています。現在の成果物は提出準備段階で、最終課題で必要なAI実演動画と実測結果が揃うまで最終提出できません。各フォルダの `REVIEW_LOG.md` に3回分のレビュー記録があります。

課題要項上の締切: **2026-11-02 10:00**。OmniCampusへのPDF/GitHub URL提出とYouTube動画公開設定は、各 `SUBMISSION_CHECKLIST.md` を参照してください。

上級案の公式参照: [ROBOTIS Cyclo Intelligence setup](https://docs.robotis.com/docs/systems/aiworker/imitation_learning/setup/)、[ROBOTIS model inference](https://docs.robotis.com/docs/systems/aiworker/imitation_learning/model_inference/)、[NVIDIA Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T)。
