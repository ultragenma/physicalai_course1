# 説明資料：AIWorkerによるGR00T N1.7推論を用いたホワイトボード文字消去

> 提出PDF化前に、`[未実施]` と `TODO` を実測記録で置換する。本稿は準備段階の説明資料であり、実機成功を主張しない。

## 1. 課題の概要と目的

ROBOTIS AIWorkerが消しゴムを用い、白板上で指定された文字を消すタスクに取り組む。RGBカメラ、ロボット状態、自然言語指示をGR00T N1.7へ入力し、消去動作を生成することを目標とする。推論は別GPU PC、ロボットへの接続と安全検査は会社ノートPC上のROS 2で担当する。

本タスクを選んだ理由は、単純な到達動作だけではなく、白板位置・文字の見え方・消しゴム接触面・動作後の視覚的な確認を統合する必要があるためである。

## 2. AIの利用方法

GR00T N1.7を視覚・言語・proprioceptionからaction chunkを生成するVLAとして利用する。AIWorkerは本タスクの既知embodimentではないため、公開baseモデルのzero-shot性能を前提にしない。AIWorkerで収録したテレオペレーションのデモをGR00T互換LeRobot形式へ変換し、AIWorker固有のmodality configでfine-tuningする計画である。

## 3. システム構成と工夫

第一候補は、GPU PC上でCyclo IntelligenceのGR00T N1.7 backendを動かし、AIWorkerネットワークへZenoh経由で接続する構成である。会社ノートPCでは現在確認できているROS 2 graphのread-only記録とベンダー対応を行う。GPU PC側のZenoh経路はまだ確認していない。Cycloの対応外GPUやネットワーク構成の場合は、NVIDIA公式のGPU server/client方式を代替案として評価する。

リポジトリ内のgeneric ROS 2 gatewayは、代替方式の設計検討用scaffoldであり、GR00T clientでもCycloの一部でもない。現時点では実機controllerに接続していない。実機コマンドは、AI出力をそのまま送らず、関節名順、状態・actionの鮮度、可動域、1ステップ差分、軌道時間を検査する境界を設ける計画である。非常停止はロボット側の公式機構で担保する。

ボード座標系、カメラ、消しゴム接触面を校正し、学習時と推論時に同じ観測順序・単位・action定義を使う。消去結果は指定領域の前後画像と人による確認で評価する。

## 4. 実施結果

- ROS 2接続とtopic取得: 確認済み（社内PC、詳細topic名は非公開）。
- AIWorker型式／ソフトウェア更新: `[未確定・未実施 / TODO: 型式、日付、版]`
- GPU PC型式、Cyclo backend互換性、GPU PCからのZenoh到達: `[未確認]`
- AIWorkerシミュレーション: `[未実施 / TODO: 実行条件・結果]`
- デモ収録: `[未実施 / TODO: エピソード数]`
- GR00T N1.7 fine-tuning: `[未実施 / TODO: checkpoint・GPU・設定]`
- 実機closed-loop試行: `[未実施 / TODO: 試行数・成功数]`

成功は「指定された文字領域が消去され、意図しない領域への接触や安全停止がない」と定義する。報告時は試行数、成功数、文字位置・照明条件、失敗理由を併記する。

## 5. 今後の改善と考察

最初の実機ゲートは、AIWorkerの公式制御interface、JointStateの順序、カメラ/TF、非常停止を確定することである。次に空中replayと通信断試験を通し、白板への接触を段階的に導入する。失敗が生じた場合は、画像の反射、board/tool calibration、接触圧、action遅延、学習データの偏りをログと動画で切り分ける。

## 6. 動画・コード

- 動画URL: `TODO: YouTube限定公開URL`
- 公開GitHub URL: `TODO: public repository URL`
- 動画は1〜3分以内。倍速を用いる場合は動画内と概要欄に明記する。

個人情報、社内ネットワーク情報、認証情報、未公開のcamera映像やrosbagを含めない。

## 7. 参考資料

- ROBOTIS Cyclo Intelligence [Setup](https://docs.robotis.com/docs/systems/aiworker/imitation_learning/setup/) および [Model Inference](https://docs.robotis.com/docs/systems/aiworker/imitation_learning/model_inference/)
- NVIDIA [Isaac-GR00T N1.7](https://github.com/NVIDIA/Isaac-GR00T)
