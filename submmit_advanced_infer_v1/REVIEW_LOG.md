# 3回レビュー記録

レビュー日: 2026-10-24。文書・コードを静的に読み、PDFを表示確認した。実ROS/GPU/AIWorkerでは実行していない。

## 1回目 — 公式構成と課題要件

- ROBOTISのCyclo Intelligence Setup/Model InferenceとNVIDIA Isaac-GR00T資料を参照し、GPU PC上のCyclo Intelligence + Zenohを第一経路、NVIDIA server/clientを代替案として整理。
- GPU型式、AIWorker更新、GPU PCからのZenoh接続、checkpoint、デモ収録が未確認と記載。
- 3D Sim Deployと白板sceneのシミュレーションを同一視しないように区分。

## 2回目 — 独立コード/安全レビュー

- 独立レビュー担当 `advanced_sim` が、gatewayの短すぎるtrajectory、JointState header時刻不確認、E-stop/速度/加速度/衝突/作業空間/通信断/ROS publisher認証の未実装、Simulation資料のIsaac記述、ZMQ allowlist未実装、private graph artifactの追跡リスクを指摘。
- 修正: JointState/actionのROS header時刻とreceipt freshnessを検査、trajectoryの最小/最大時間とjoint position/平均速度範囲を追加、入力headerを保持。README/RUNBOOKで候補filterを安全controllerと区別し、E-stop・衝突/接触/加速度制約・アクセス制御が未実装であると明記。ZMQ例のbind/TLS/allowlistは構成されていないと警告。`.gitignore`を追加し、旧Simulation資料の記述を訂正。
- 平均速度検査はcontrollerのpeak velocity/acceleration保証ではないこと、未接続scaffoldであることを残す。

## 3回目 — PDF・公開情報・未確認事項

- PDFをレンダリングして目視確認（2ページ）。実施結果は空欄/未実施で、company IP/topic/credentials/camera imageが入っていないことを確認。
- Cyclo commit、container image digest、AIWorker software、GPU、checkpoint/dataset hashを記録する欄を運用手順へ追加。
- 提出チェックでGitHub public化、YouTube設定、OmniCampus提出、動画速度、個人/社内情報除外を確認。
- **残件:** AIWorkerのソフトウェア更新、型式、GPU、Zenoh endpoint、Cyclo/GR00T setup、fine-tuning、Sim Deploy、実機試験、動画、成功率は未確認。

## 2026-10-25 — Zenoh / GR00Tコード追加の3系統レビュー

今回の対象はROS観測bridge、軽量PolicyServer client、read-only metadata取得、modality config template、gateway、依存定義、README/RUNBOOKである。親担当を実装担当とし、3名が別々に静的レビューし、指摘修正後に再確認した。

### 1. Protocol・model contract — `infer_final_review`

- NVIDIA Isaac-GR00T revision `51d4c89` のPolicyServer endpoint/data envelope、MessagePack/NumPy serializer、ModalityConfig応答、`(action, info)`形式を照合。
- ROS PCでGR00T packageをimportするとmodel依存まで読み込む問題を避け、ローカル`groot_policy_client.py`と固定した軽量依存だけを使う構成へ変更。
- video/state/language/actionの単一key、観測delta `[0]`、actionの選択delta `0`、horizon、joint vector次元・有限値を検査。
- `decode_action()`後のRELATIVE actionは絶対目標へ戻っているため、bridgeでstateを二重加算しない。`ABSOLUTE`/`RELATIVE`、`NON_EEF`、`DEFAULT`、RELATIVE参照state keyを起動時検査し、EEF/rotation/DELTA形式を拒否。
- 最終再確認: 上記のprotocol/schema/absolute出力契約に重大な未解決点なし。API token付きserverはこのsample clientの対象外で、手順はtokenなしloopback server + SSH転送である。

### 2. 手順・依存・再現性 — `runbook_final_review`

- GPU source revisionとcheckout後submodule SHAを固定し、記録方法を追加。
- Linux x86_64 dGPUのPython 3.12/FFmpeg依存と、aarch64のplatform別installer/専用venv/`python`起動を明記。
- notebookはvendor Jazzy Pythonを参照するvenvへ固定。CWD、軽量依存、IPv4 SSH forward、`127.0.0.1`制限、実設定topicのecho、端末A/B分離を確認。
- 新規端末ではROS setupに加えて、既に成功しているvendor overlay/RMW/Zenoh/domain設定をsourceする手順を明記。
- 最終再確認: 文書とコードの実行導線に未解決の静的整合問題なし。

### 3. 候補filter・実機境界 — `gateway_final_review`

- candidate/command topicのraw・ROS解決後同名を拒否。bridgeの観測topicとcandidate publish時のcandidate/command、gatewayの観測/candidate/armed commandを絶対名・非remapに固定。
- YAMLのarmはfalse固定で、明示bool parameterでのみarmする。disarmed gatewayはvendor command publisher自体を生成しない。
- bridge/gatewayでjoint bounds、step、平均速度、trajectory durationを確認。bridgeはROS header時刻とmonotonic受信時刻の鮮度を推論前後に検査する。
- gatewayはJointState headerを保存し、candidate転送時にもheader ageとmonotonic receipt ageを再検査する。受信時だけの鮮度検査で生じる時間窓を修正。
- candidate publishは承認済みbridge、vendor command publishはgatewayというACL方向へ手順を訂正。
- 最終再確認: コード上の重大な未解決点なし。Zenoh/ROS側のpublisher認証・ACL、vendor command契約、安全controller/E-stop/ownershipの実環境確認は外部gateとして残る。

### 実施した確認と未実施範囲

このsandboxでは、AGENTS.mdの実行方針に従い、公式ソース/文書との照合と静的読解だけを実施した。最終確認で実際に使用したread-onlyコマンドは次のとおり。

```bash
rg --files submmit_advanced_infer_v1
rg -n 'ActionConfig|non_eef|absolute_joint_position|header_observation_age|image_header_age|receipt_age_ns|last_joint_state_stamp_ns|must not be ROS-remapped|127.0.0.1|create_timer' submmit_advanced_infer_v1/src/groot_zenoh_infer_node.py submmit_advanced_infer_v1/src/groot_policy_client.py submmit_advanced_infer_v1/src/ros_action_gateway.py
sed -n '145,305p' submmit_advanced_infer_v1/docs/RUNBOOK.md
cat submmit_advanced_infer_v1/requirements-inference-client.txt
```

Python実行、`py_compile`、依存install、unit/integration test、ROS commandの実行、GPU PolicyServer起動、SSH/Zenoh疎通、sim/実機動作は今回実施していない。README/RUNBOOK内の実行コマンドは対象PCで行う手順であり、成功実績ではない。AIWorker固有topic/type/joint order/limits、software更新、AIWorker用checkpoint・dataset、GPU環境、実機安全条件は引き続き未確認である。
