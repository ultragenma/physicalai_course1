# MolmoAct2主経路と既存レビュー修正の統合

更新日: 2026-10-30

## 統合対象

- 既存main: `eb10e03dfb1fc54e855b4bd54bae22efee3abaa0`
- 取り込み元: `feature/aiworker-molmoact2` の `40ed7e57002a9de1eca28ddf9de8a43635c4d2e3`。MolmoAct2推論・対象選択・MoveL実行パッケージをそのまま保持する。
- 上級案の主経路をMolmoAct2対象選択＋RGB-D幾何計画＋MoveLとしてREADMEから案内する。GR00T経路、簡易案、sim評価コードも保持する。
- MolmoAct2のMoveL経路とGR00Tの候補trajectory/gatewayは独立した実装のまま。方策出力やcommand topicを相互接続する変更は含まない。

## 再レビューで修正した内容

| 対象 | 指摘と修正 |
| --- | --- |
| 簡易案のレポート | 学習・評価JSONL本文を読み、連番、型、要求件数、成功数、評価平均時間をmetadata/summaryと照合。欠落・途中切断・不一致は生成エラーにする。 |
| 簡易案のnavigator | reset失敗・reset後Pose待機タイムアウトを終了コード1で返す。非有限の制御周期を拒否する。 |
| simのマスク評価 | 全画面のbefore/after筆跡マスクを受け付け、ROI内消去率と白板外差分の計算範囲を分離。空または2次元以外の入力を拒否する。 |
| simのROS観測 | ament用setup.cfgを追加し、sensor-data QoSを使用。run IDを必須化し、保存先をrun配下に制限。セッションディレクトリと個別ファイルの上書きを拒否する。 |
| simの試行・公開レポート | metricsありなら4枚のmask SHA-256を必須化。invalidを平均から除外しCSVに集計対象を記録。既定は公開確認済み要約を出力し、詳細環境値は明示オプションで出力する。 |
| GR00T bridge/gateway | ROS stampとtrajectory durationのsec/nanosecが非負・正規範囲か確認してから利用する。異常な時刻表現をそのままcontrollerへ渡さない。 |

SHA-256必須化は原記録の識別を補強する変更であり、生成器によるmask画像の再計算・照合を意味しない。ログの整合性検査も改ざん防止機構ではない。

## レビューと確認範囲

簡易案、sim案、推論案の分担による静的再レビュー後、統合時に別担当がeasy/simの最終差分とMolmoAct2/GR00Tの境界を確認した。文書の最終再確認も完了し、統合を止める指摘はなかった。主担当は全差分、依存定義、手順書を確認した。

実施した確認:

- `git fetch origin`、履歴・差分確認。統合用の文書変更を加える前に、取り込み元の変更ファイルと保全済みレビュー修正の変更ファイルに重複がないことを確認。
- `git diff --check` による差分の空白エラー検査。
- `perl -MJSON::PP -0777 -ne 'decode_json($_)' submmit_advanced_sim_v1/schema/trial_result.schema.json` によるJSON構文確認。JSON Schemaによるデータ検証の実行ではない。
- ソース、console entry point、setup.cfg、trial schemaとvalidator、README・RUNBOOKの静的照合。

workspaceのAGENTS.mdはsandboxでの実行検証を禁止しているため、この統合ではPython、colcon、ROS、GPU推論、sim、実機を実行していない。取り込み元のCPU/隔離ROS試験実績は元の文書を保持しており、今回の再実行実績とは区別する。

未検証の経路:

- turtlesimのbuild、学習、評価、JSONLからのレポート生成、reset失敗時の終了挙動。
- 観測ノードのcolcon配置・QoS通信・保存、mask計算、trial記録と集計の実行。
- GR00Tの非正規時刻入力の拒否、実モデル推論、candidate/gatewayのROS通信。
- MolmoAct2と実GPU、実AIWorkerの通信・推論・MoveL実行・白板消去。

実行時は各案のRUNBOOKを使用し、実測後にレポート草稿と動画を更新する。
