# 外部コードの帰属

`configs/gr00t/sg2_common.py` は ROBOTIS の SG2 modality 設定例を基にしています。
参照: https://github.com/ROBOTIS-GIT/Isaac-GR00T-n1.7/tree/e81d02b5c561dfafd0556f2a4a602016e670920c/examples/CYCLO/ffw_sg2_rev1

`wb_eraser/policy_client.py` の serializer と wire protocol は NVIDIA Isaac-GR00T の実装に基づいています。
参照: https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/gr00t/policy/server_client.py

`wb_eraser/molmoact2_server.py` のモデル読込方法、入力テンソルのdtype変換は Ai2 の公式MolmoAct2サーバを参照しています。
参照: https://github.com/allenai/molmoact2/blob/6070080a20321b4f498ab30f28e1d09ac465edb7/examples/droid/host_server_droid.py
カメラ順序、連続action API、公式サンプル観測は `allenai/MolmoAct2-DROID` のリビジョン
`d8c1abd8a27d8e859455bbe514df2bcc617db0fb` を参照しています。
参照: https://huggingface.co/allenai/MolmoAct2-DROID/tree/d8c1abd8a27d8e859455bbe514df2bcc617db0fb

`wb_eraser/vlm_server.py` の画像QA生成は、同revisionの `modeling_molmoact2.py` の `forward()` にある
chat-template / generate / decode例を参照しています。ロボット関節のaction出力とは別の呼出し経路です。

これらの参照実装のライセンスは Apache-2.0 です。本文を `Apache-2.0.txt` に収録しています。
モデル重みのライセンスはコードのライセンスと別途確認してください。
