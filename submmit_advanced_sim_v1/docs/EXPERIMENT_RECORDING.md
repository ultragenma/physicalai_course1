# 実験記録からレポートへの手順

この手順は、実行後の事実を機械可読で保存し、レポートへの転記漏れを減らすためのものです。AI認識、AI方策、白板scene、ロボット動作、シミュレータ実行を実装するものではありません。実施できていないゲートは記録で補完せず、`未実施`とします。

## 保存場所と公開範囲

各実験は `artifacts/<run_id>/` に保存します。このディレクトリはGit管理対象外です。`assets/`は説明資料専用です。公開用ファクトを生成する前に、metadataの各出力項目を人が確認し、個人情報・社内ネットワーク情報・機密情報がないことを確かめます。

| ファイル | 役割 | 公開前の扱い |
| --- | --- | --- |
| `metadata.json` | simulator/asset/software/model/GPUと課題の固定情報 | 既定は公開用要約のみ。詳細値の出力には全出力項目を目視で匿名化し追加flagを指定 |
| `metrics_<trial_id>.json` | `erase_assessment.py` が出す画素指標 | マスク生成法とともに確認 |
| `trials/<trial_id>.json` | status・時刻・所要時間・metrics・注記 | 失敗も保存し、公開は要約のみ |
| `report_facts.md` | プライバシー確認後の集計済み転記候補 | operator notes・動画参照・画像/マスク参照を含めずに生成 |

スキーマは [experiment_metadata.schema.json](../schema/experiment_metadata.schema.json) と [trial_result.schema.json](../schema/trial_result.schema.json) です。`record_validation.py`はrun/trial ID、UTC時刻、期間、metricsの型・範囲・画素数との整合、SHA-256形式を検査します。別途のJSON Schema実装は不要です。

## 0. 実行前: runを固定する

1. 提出フォルダ直下で、run IDを新しく決めて記録用ファイルを作る。

```bash
RUN_ID=sim_YYYYMMDD_001
mkdir -p "artifacts/${RUN_ID}/masks" "artifacts/${RUN_ID}/trials"
cp records/templates/experiment_metadata.template.json "artifacts/${RUN_ID}/metadata.json"
```

2. `metadata.json`のすべての`TODO`と`recorded_at_utc`を、実際に確認した値で埋める。特に`asset_revision`、`asset_variant`、scene revision、ROS/robot software、policy runtime revision、checkpoint ID、GPU名・driver・VRAMを記録する。テンプレートのままではtrial記録を作成できない。
3. `privacy_review`を人が確認し、個人情報・社内ネットワーク情報・機密情報の3つがない場合だけ対応するbooleanを`false`にする。IP、ホスト名、topic名、シリアル番号、社員名、認証情報、非公開asset/checkpoint情報、画像そのものは公開用ファクトへ出さない。`null`のままなら未レビューとして扱う。
4. 初版では`execution.kind`を`simulation`とする。実機値を混ぜない。

以降のコマンドもこのフォルダ直下で、同じshell sessionから実行する。別runを開始する場合は新しい`RUN_ID`にする。

## 1. 各試行: マスクからmetrics JSONを作る

4枚の同一解像度・同一画角の2次元二値マスクを用意する。`before_ink_mask`/`after_ink_mask`は白板外・対象文字以外の筆跡も含む全画面マスクとし、片側だけROIで切り取らない。`target_roi_mask`は対象文字を囲み、ROIと白板の共通領域に消去前の筆跡が1画素以上必要である。マスク作成モデル/アルゴリズム、閾値、版をmetadataの`task.mask_generation_method`へ記す。

```bash
TRIAL_ID=t001
.venv-metrics/bin/python whiteboard_task/whiteboard_task/erase_assessment.py \
  --before-ink "artifacts/${RUN_ID}/masks/${TRIAL_ID}_before_ink.png" \
  --after-ink "artifacts/${RUN_ID}/masks/${TRIAL_ID}_after_ink.png" \
  --board-mask "artifacts/${RUN_ID}/masks/${TRIAL_ID}_board.png" \
  --target-roi-mask "artifacts/${RUN_ID}/masks/${TRIAL_ID}_target_roi.png" \
  --output-json "artifacts/${RUN_ID}/metrics_${TRIAL_ID}.json"

BEFORE_HASH=$(sha256sum "artifacts/${RUN_ID}/masks/${TRIAL_ID}_before_ink.png" | awk '{print $1}')
AFTER_HASH=$(sha256sum "artifacts/${RUN_ID}/masks/${TRIAL_ID}_after_ink.png" | awk '{print $1}')
BOARD_HASH=$(sha256sum "artifacts/${RUN_ID}/masks/${TRIAL_ID}_board.png" | awk '{print $1}')
ROI_HASH=$(sha256sum "artifacts/${RUN_ID}/masks/${TRIAL_ID}_target_roi.png" | awk '{print $1}')
```

出力指標は対象ROI消去率と、白板外の**筆跡マスク**変化率だけです。対象ROI内に筆跡が増えた場合、消去率は負の値になることがあります。これは悪化を隠さないためです。白板面の損傷、物体接触、原画像全体の変化、把持失敗は測りません。該当する安全・失敗事象はtrialのstatusと注記へ残します。

## 2. 各試行: trial JSONを作る

時刻はUTCの`Z`表記で残します。`status=completed`は完走を示すだけで、成功を意味しません。評価不能な試行は`--metrics`を省略し、`failed`、`aborted`または`invalid`を正確に選びます。

metricsを記録する場合は4枚すべてのmask SHA-256が必須です。これは原記録を識別するための値であり、生成器がmask画像を再計算・照合したことを意味しません。

```bash
python3 tools/record_trial.py \
  --metadata "artifacts/${RUN_ID}/metadata.json" \
  --metrics "artifacts/${RUN_ID}/metrics_${TRIAL_ID}.json" \
  --output "artifacts/${RUN_ID}/trials/${TRIAL_ID}.json" \
  --trial-id "${TRIAL_ID}" \
  --status completed \
  --started-at-utc "2026-10-24T01:23:45Z" \
  --ended-at-utc "2026-10-24T01:24:10Z" \
  --video-id-or-redacted-path "local-redacted-video-id-only" \
  --before-mask-sha256 "${BEFORE_HASH}" \
  --after-mask-sha256 "${AFTER_HASH}" \
  --board-mask-sha256 "${BOARD_HASH}" \
  --target-roi-mask-sha256 "${ROI_HASH}" \
  --operator-notes "Record safety events or interventions here; this private note is not exported."
```

trial JSONは`artifacts/<run_id>/`外には保存できません。既存のtrialファイルを上書きせず、出力ファイル名は`<trial_id>.json`に固定します。訂正・再試行は新しいtrial IDで記録してください。実機または未実装のsimを実行したように見せる値を作らない。試行が行われなければtrial JSONも作らない。`operator_notes`、動画参照、画像/マスク参照は非公開記録にのみ残し、自動生成されるレポートfactsやCSVには出力しません。失敗原因を公開レポートに載せる場合は、個人・社内・機密情報を除いた表現に人が書き直します。

## 3. 全試行: report factsを生成する

```bash
python3 tools/generate_report_facts.py \
  --metadata "artifacts/${RUN_ID}/metadata.json" \
  --trials-dir "artifacts/${RUN_ID}/trials" \
  --output "artifacts/${RUN_ID}/report_facts.md" \
  --summary-csv "artifacts/${RUN_ID}/trial_summary.csv"
```

生成器はprivacy reviewの3フラグがすべて`false`でないと停止し、全trialの形式・run ID・時刻・計測範囲を検証します。出力も`artifacts/<run_id>/`内に限定します。生成された`report_facts.md`と`trial_summary.csv`から、`docs/REPORT_DRAFT.md`の「実験結果・提出情報」に環境、試行数、計測済み試行数、平均指標を転記する。成功数・成功率は、事前に定義した成功条件と人の確認に基づいて手動で記す。GitHub URL、YouTube URL、動画倍率、考察、匿名化した失敗注記も手動で記す。

平均指標はmetricsがある試行のうち`invalid`を除外して計算し、除外した計測済み件数も表示する。`failed`/`aborted`でも有効な計測値は含め、成功例だけを選ばない。試行表とCSVにはinvalidも残し、CSVの`included_in_metric_averages`で集計対象を示す。

既定の環境出力は`privacy_review.public_safe_summary`、run ID、実行種別だけです。asset/checkpoint等の詳細識別子を転記する場合は、全出力項目を公開可能な値へ匿名化・目視確認したうえで同じ生成コマンドへ`--include-reviewed-environment`を追加します。privacy flagがfalseでも、この追加指定なしでは生の環境識別子を出力しません。

## レポートのフィールド対応

| レポートの項目 | 記録元 |
| --- | --- |
| シミュレータ／AIWorker型式／ソフトウェア版 | `metadata.json`: `execution`, `robot`, `software` |
| 方策・checkpoint・GPU | `metadata.json`: `software` |
| 試行数／失敗数／所要時間 | `trials/*.json`: `status`, `duration_seconds` |
| 平均消去率／白板外筆跡マスク変化率 | `report_facts.md`: metrics summary |
| 1試行ごとの根拠 | `trials/*.json`と対応するmetrics/mask（公開しない原記録） |
| 成功数／成功率 | 事前に定めた成功条件 + 人の確認。自動集計しない。 |
| 動画URL・GitHub URL・再生速度 | 手動で入力 |

## 未実装ゲート

- 対象AIWorker型式とasset variantの確認
- 白板、文字、消し具、カメラを持つシーン
- 画像からの筆跡・ROIマスク生成
- GR00T N1.7用デモ収集、LeRobot変換、学習、推論
- collision/contact/forceを含むロボット動作実行
- シミュレータ実行、実機安全試験、動画撮影

これらが完了するまでは、出力ファイルやレポートを最終課題の実演結果として提出しない。
