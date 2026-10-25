# 3回レビュー記録

レビュー日: 2026-10-24。ROS 2/turtlesimのコード実行とビルドは行わず、静的確認を実施した。

## 1回目 — 課題要件・提出物の確認

- Q-learningの状態・行動・報酬、ROS 2 interface、測定結果欄、1〜3分動画計画が最終課題資料と整合することを確認。
- OmniCampus account、評価値、動画/GitHub URLは未入力のままとし、未実行の値を実績として書かないことを明記。
- PDF名、public GitHub、OmniCampus提出、YouTubeの公開条件・2倍速表示、締切を提出チェックへ記載。

## 2回目 — 独立したコード・再現性レビュー

- 独立レビューでreset直後のPose競合、反復評価、Q表の作業ディレクトリ依存を指摘。追加レビューで、評価条件の差を検出しないことと、学習run内にQ表本体が残らない点も確認。
- 修正: reset後に位置・向きが許容範囲内のPoseを待つ。5秒以内に確認できなければ停止する。学習runにQ表snapshotとSHA-256を保存し、評価スクリプトへ学習run IDを渡す。
- `generate_report.py`は開始/目標姿勢、seed、ROS/rclpy/Python版、制御周期、最大ステップ、Q表の実ファイルとhashを照合し、不一致ならレポート生成を停止する。

## 3回目 — 独立最終レビュー・資料表示確認

- 独立レビュー担当 `advanced_infer` が更新後のコードとRUNBOOKを再確認し、追加指摘なしと報告。確認項目はQ表snapshot/hash/source ID、条件一致、reset pose timeout、レポート出力欄。
- 草稿PDFを再生成し、全2ページを画像にレンダリングして目視確認。日本語、表、ページ送り、脚注、フッターを確認。
- **未確認:** colcon build、ROS 2/turtlesim実行、Q-learning収束、成功率、動画、公開GitHub URL。レビューは静的確認と資料表示確認のみ。
