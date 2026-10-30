# physicalai_course1

Physical AI 応用1の最終課題に向けたコード、実行手順、実験記録、レポート草稿です。

上級案の主経路は、更新された **MolmoAct2の対象選択＋RGB-D幾何計画＋MoveL実行**です。[主経路の手順](submmit_advanced_infer_v1/molmoact2/docs/HYBRID_ERASER.md)から進めます。従来のGR00T N1.7案、簡易案、sim評価コードも同じリポジトリに保持しています。統合内容と検証範囲は[統合レビュー記録](REVIEW_FOLLOWUP.md)を参照してください。

## 収録内容

| ディレクトリ | 内容 | 検証状況 |
| --- | --- | --- |
| [submmit_easy_v1](submmit_easy_v1/README.md) | ROS 2 turtlesimのQ-learning目標到達、学習・独立評価の記録、レポート表生成 | ROS 2 build・学習・評価は未実施 |
| [submmit_advanced_sim_v1](submmit_advanced_sim_v1/README.md) | AIWorker白板消去の二値マスク評価、幾何経路、観測metadata、実験記録 | AI方策・scene・動作実行は未実装。シミュレーション未実施 |
| [submmit_advanced_infer_v1](submmit_advanced_infer_v1/README.md) | AIWorkerのZenoh/ROS 2観測を別GPU PCのGR00T N1.7へ送る推論サンプル、候補trajectory、既定disarmのgateway | 3系統の静的レビュー済み。ROS/GPU/実機では未実行 |
| [MolmoAct2とMoveL実行](submmit_advanced_infer_v1/molmoact2/README.md) | 追加学習なしの文字領域選択→3D経路→MoveL実行。関節actionの診断用経路も収録 | CPUテスト20件、隔離ROSで8件のMoveL送受信・位置応答・最終観測を検証。実GPU・実機は未検証 |

ディレクトリ名の`submmit`は既存の提出物名を維持しています。

## 実行手順

1. 簡易案は [学習・評価RUNBOOK](submmit_easy_v1/RUNBOOK.md) を使用します。
2. 上級sim案は [実装計画](submmit_advanced_sim_v1/docs/IMPLEMENTATION_PLAN.md) と [実験記録手順](submmit_advanced_sim_v1/docs/EXPERIMENT_RECORDING.md) を使用します。
3. 上級案の主経路は[対象選択＋幾何計画＋MoveL](submmit_advanced_infer_v1/molmoact2/docs/HYBRID_ERASER.md)です。MolmoAct2の画像・言語部分を使い、既定は計画のみ、`hybrid_run --execute`で動作指令を送信します。文字消去の成功・失敗をプログラムの実行完了と区別します。
4. MolmoAct2の関節action診断は[専用手順](submmit_advanced_infer_v1/molmoact2/docs/MOLMOACT2.md)を使用します。公開DROID重みはFranka用であり、AIWorkerの動作方策ではありません。診断時は予測値を保存し、アクチュエータへ配信しません。
5. 従来のGR00T案は [GPU分離推論RUNBOOK](submmit_advanced_infer_v1/docs/RUNBOOK.md) を使用します。AIWorkerの実topic・型・関節順・制限値・vendor環境と、AIWorker用に学習したcheckpointが必要です。

主経路の構成:

```text
AIWorker RGB-D / TF --> 会社PC --> GPU PC / MolmoAct2 対象領域選択
                         |                        |
                         +<----- 領域ID -----------+
                         +--> 3D幾何経路 --> MoveL（--execute指定時）
```

従来のGR00T推論経路の構成:

```text
AIWorker -- Zenoh / ROS 2 --> 会社PC -- SSH / ZMQ --> GPU PC / GR00T N1.7
                               ^                     |
                               +--- action targets --+
                               |
                               +--> candidate trajectory --> gateway（既定disarm）
```

推論bridgeは観測とモデルの契約を検査します。実機commandへの転送は別gatewayで明示armした場合に限られ、vendorの`JointTrajectory`契約、ネットワークACL、安全機構、段階試験の実環境確認が必要です。action/service型のcontrollerには専用adapterが必要です。

## 提出資料と実行状況

[SUBMISSION_STATUS.md](SUBMISSION_STATUS.md) に各案の実装範囲と未確認事項を記載しています。PDFはアカウント名未入力・結果未実測の草稿です。実験後に各レポート草稿へ実測値、動画URL、環境revisionを記入します。

従来3案のコードは静的確認段階です。MolmoAct2の追加パッケージはCPUテストと隔離した模擬ROS観測・HTTP経路の実行を確認しましたが、GPUモデル・実機タスクの実行成功を示すものではありません。各パッケージの検証範囲を参照してください。

## ローカル情報

実IP、会社固有topic、認証情報、画像、rosbag、checkpoint、実験原データは含みません。設定例をローカル専用ファイルへコピーし、実設定と実行成果物をGitへ追加しないでください。

各パッケージのライセンス表記は個別の`package.xml`/`setup.py`を参照してください。ROBOTIS/NVIDIA等のモデル・asset・外部ソフトウェアは同梱していません。参照先は [上級推論の参考資料](submmit_advanced_infer_v1/docs/references.md) に記載しています。
