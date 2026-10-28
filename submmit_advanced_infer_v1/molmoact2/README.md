# AIWorker / MolmoAct2 対象選択・MoveL実行

**追加学習なしで試す実行構成**は、MolmoAct2の画像・言語部分で文字領域を選択し、
RGB-Dの板面推定・幾何経路・MoveLで拭く方式。`hybrid_run --execute` で動作指令まで送信する。
試行プログラムの実行完了と、文字消去の成功・失敗は別々に記録する。

| 経路 | 内容 | アクチュエータ指令 |
| --- | --- | --- |
| 対象選択＋幾何制御 | `vlm_server` → `hybrid_run` → MoveL | `--execute`で有効。既定は計画のみ |
| 関節action推論 | `molmoact2_server` → ROS/HTTP観測クライアント | 検査・保存のみ |

公開DROIDモデルの関節方策はFranka用。新しい対象選択経路はその関節出力を使用しない。
19次元の関節方策を直接使う場合は、従来どおりAIWorker向け微調整済みcheckpointが必要。

## 手順

[対象選択からMoveLまでの実行手順](docs/HYBRID_ERASER.md) を参照。
[関節actionの診断・native形式](docs/MOLMOACT2.md) は別手順。
クライアントはPython 3.10以上、GPUサーバはPython 3.11/3.12を使用する。

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-molmoact2-client.txt
.venv/bin/python -B -m unittest tests.test_molmoact2 tests.test_vlm_selector -v
```

モデルの固定revisionは `configs/molmoact2/model.lock.json`。
モデル重み・観測画像・関節値・実IP・認証情報・実測ログはGitへ含めない。

## 検証状況 (2026-10-28)

- 公式DROID重みを取得し、Hugging Face CLIで24ファイルをチェックサム照合した。
- このパッケージのCPUテスト20件に成功。元の開発ワークスペースでは既存機能を含む63件に成功。
- 別の隔離ROS/Jazzyコンテナ内で、模擬3カメラと19関節から実際のROSクライアント・HTTPを通し、
  DROID診断15×8とnative形式15×19を各3回受信・保存した。backendは試験fixtureでありAIモデルではない。
- RGB-D/TF → 対象選択 → 3D経路 → 実SDK型のMoveL送信 → Cartesian位置応答 → 最終観測も隔離ROSで確認。
  8件のMoveLを受信した後、文字が残っていても `pipeline_complete=true`, `task_success=false`, exit 0と記録した。
  応答不正・fallback・古い応答・controller未到達も確認。IK・接触物理は模擬していない。
- GPU上の実モデルforward、実AIWorkerからの推論、実機タスク成功は未検証。

公式参照と帰属は [NOTICE](LICENSES/NOTICE.md) に記載。
