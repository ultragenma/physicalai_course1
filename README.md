# physicalai_course1

Physical AI 応用1の最終課題に向けたコード、実行手順、実験記録、レポート草稿です。

## 収録内容

| ディレクトリ | 内容 | 検証状況 |
| --- | --- | --- |
| [submmit_easy_v1](submmit_easy_v1/README.md) | ROS 2 turtlesimのQ-learning目標到達、学習・独立評価の記録、レポート表生成 | ROS 2 build・学習・評価は未実施 |
| [submmit_advanced_sim_v1](submmit_advanced_sim_v1/README.md) | AIWorker白板消去の二値マスク評価、幾何経路、観測metadata、実験記録 | AI方策・scene・動作実行は未実装。シミュレーション未実施 |
| [submmit_advanced_infer_v1](submmit_advanced_infer_v1/README.md) | AIWorkerのZenoh/ROS 2観測を別GPU PCのGR00T N1.7へ送る推論サンプル、候補trajectory、既定disarmのgateway | 3系統の静的レビュー済み。ROS/GPU/実機では未実行 |

ディレクトリ名の`submmit`は既存の提出物名を維持しています。

## 実行手順

1. 簡易案は [学習・評価RUNBOOK](submmit_easy_v1/RUNBOOK.md) を使用します。
2. 上級sim案は [実装計画](submmit_advanced_sim_v1/docs/IMPLEMENTATION_PLAN.md) と [実験記録手順](submmit_advanced_sim_v1/docs/EXPERIMENT_RECORDING.md) を使用します。
3. 上級推論案は [GPU分離推論RUNBOOK](submmit_advanced_infer_v1/docs/RUNBOOK.md) を使用します。AIWorkerの実topic・型・関節順・制限値・vendor環境と、AIWorker用に学習したcheckpointが必要です。

上級推論の構成:

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

このリポジトリのコードは静的確認段階です。ROS 2、GPU PolicyServer、シミュレータ、実機での実行成功を示すものではありません。確認内容は各ディレクトリの`REVIEW_LOG.md`に記録しています。

## ローカル情報

実IP、会社固有topic、認証情報、画像、rosbag、checkpoint、実験原データは含みません。設定例をローカル専用ファイルへコピーし、実設定と実行成果物をGitへ追加しないでください。

各パッケージのライセンス表記は個別の`package.xml`/`setup.py`を参照してください。ROBOTIS/NVIDIA等のモデル・asset・外部ソフトウェアは同梱していません。参照先は [上級推論の参考資料](submmit_advanced_infer_v1/docs/references.md) に記載しています。
