# AIWorker / MolmoAct2 推論

AIWorkerの同期ROS観測を別GPU PCのMolmoAct2へ送り、返されたaction chunkを検査・保存する。
GPUサーバ、このPCの軽量HTTPクライアント、ROS同期ノードを含む。

**公開DROIDモデルはFranka用。現在の公開モデル経路は診断用で、AIWorkerの実行指令へ変換しない。**
AIWorkerの19次元native形式は、対応する微調整済みcheckpointと正規化・関節名metadataが必要。
このパッケージに学習済みAIWorker方策は含まない。すべての入口でアクチュエータ指令は送信しない。

## 手順

[モデル準備・GPU起動・SSH接続・ROS推論](docs/MOLMOACT2.md) を参照。
クライアントはPython 3.10以上、GPUサーバはPython 3.11/3.12を使用する。

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-molmoact2-client.txt
.venv/bin/python -B -m unittest tests.test_molmoact2 -v
```

モデルの固定revisionは `configs/molmoact2/model.lock.json`。
モデル重み・観測画像・関節値・実IP・認証情報・実測ログはGitへ含めない。

## 検証状況 (2026-10-28)

- 公式DROID重みを取得し、Hugging Face CLIで24ファイルをチェックサム照合した。
- このパッケージのCPUテスト14件に成功。元の開発ワークスペースでは既存機能を含む57件に成功。
- 別の隔離ROS/Jazzyコンテナ内で、模擬3カメラと19関節から実際のROSクライアント・HTTPを通し、
  DROID診断15×8とnative形式15×19を各3回受信・保存した。backendは試験fixtureでありAIモデルではない。
- GPU上の実モデルforward、実AIWorkerからの推論、実機タスク成功は未検証。

公式参照と帰属は [NOTICE](LICENSES/NOTICE.md) に記載。
