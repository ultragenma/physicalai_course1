# 参照資料（2026-10-25確認）

* ROBOTIS, [Zenoh Communication](https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/setup_overview/zenoh_communication/): AIWorkerのROS 2 Jazzy / `rmw_zenoh_cpp`、外部PCからのZenoh client接続、TCP 7447とROS graph確認を参照。
* ROBOTIS, [AI Worker Simulation](https://docs.robotis.com/docs/systems/aiworker/simulation/): Gazebo向けURDF/SDFとMuJoCo向けMJCFを案内し、Isaac Sim USDはupcomingと記載。別資料の`cyclo_lab`にあるIsaac Lab/USD例は型式・variantを確認して使う。
* ROBOTIS, [Model Inference](https://docs.robotis.com/docs/systems/aiworker/imitation_learning/model_inference/): Cyclo IntelligenceのGR00T N1.7 inference panel、3D Sim DeployからReal Robot Deployへの導線、prebuilt ACT/GR00T Docker backendのBlackwell-only注意を参照。
* NVIDIA, [Isaac-GR00T N1.7 README](https://github.com/NVIDIA/Isaac-GR00T/tree/51d4c89): GPU policy serverと軽量ZMQ client、new embodimentのmodality config/fine-tuning、ハードウェア別推奨を参照。GPU側serverをrevision `51d4c89` に固定する。
* NVIDIA, [Deployment and platform setup](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/scripts/deployment/README.md): Linux x86_64 dGPUとSpark/Thor/Orinの依存環境・起動方法の違い、platform専用venvの使用を参照。
* NVIDIA, [Policy API and deployment](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/getting_started/policy.md): `PolicyClient`のZMQ接続、`get_action()`、観測形状（RGB `uint8` `(B,T,H,W,3)`、state `float32` `(B,T,D)`、language `[[str]]`）、action chunk形式を参照。
* NVIDIA, [PolicyServer implementation at the pinned revision](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/gr00t/policy/server_client.py): `src/groot_policy_client.py` のMessagePack/ZMQ endpoint、ndarrayおよび`ModalityConfig` wire protocolの照合先。ROS PCではGR00T package自体をimportせず、NVIDIA server側だけを起動する。
* NVIDIA, [Gr00tPolicy implementation at the pinned revision](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/gr00t/policy/gr00t_policy.py) and [action data configuration](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/getting_started/data_config.md): policy action is denormalized/decoded using observation state; the bridge expects the resulting absolute joint targets, including when training used `RELATIVE` representation.
* NVIDIA, [Custom embodiment fine-tuning](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/getting_started/finetune_new_embodiment.md): AIWorkerのようなcustom robotでは独自modality config、データセット、`NEW_EMBODIMENT` fine-tuningが必要であることを参照。
* NVIDIA, [Real-World Deployment Guide](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89/getting_started/real_world_deployment.md): hardware準備、デモ収集、LeRobot変換、open-loop、server-client、closed-loopを段階化。
* ローカル資料 `lerobot-v0.6.0/docs/source/groot.mdx`: 現行LeRobotのGR00T N1.7サポート、および`nvidia/GR00T-N1.7-3B`を確認。
* ローカル資料 `Isaac-GR00T-n1.5/getting_started/LeRobot_compatible_data_schema.md`: GR00T LeRobotデータのstate/action/video/modality定義を参照。ただしN1.5実装をN1.7本番環境へ混在させない。

製品のversion、ROS 2 distro、AIWorker controller APIは変更され得るため、実機実施日にベンダー資料で再確認する。
