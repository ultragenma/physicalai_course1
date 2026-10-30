# AIWorker Whiteboard Erasing — MolmoAct2 / GR00T

上級案の主経路は[MolmoAct2の対象選択＋幾何計画＋MoveL](molmoact2/docs/HYBRID_ERASER.md)です。
以下は引き続き利用できるGR00T N1.7経路の準備資料です。両経路は独立しており、
MolmoAct2のMoveL指令をGR00T gatewayへ接続する統合は行っていません。

2026-10-28: [MolmoAct2用の独立パッケージ](molmoact2/README.md)を追加しました。
GPUサーバ、同期ROS/HTTPクライアント、公開DROID方策の診断とAIWorker native形式の検査を含みます。
MolmoAct2の関節action経路は保存まで。新しい[対象選択＋幾何計画＋MoveL](molmoact2/docs/HYBRID_ERASER.md)は動作指令まで実行します。
この経路は以下のGR00T gatewayへ接続せず、MolmoAct2の関節actionも使いません。

This directory is a **deployment preparation deliverable**, not evidence that an
AIWorker has already been controlled by GR00T.  The target task is: use an
eraser held by the robot to erase specified writing on a vertical whiteboard,
then present a camera view in which the target writing is absent.

For the GR00T route, the vendor deployment workflow uses ROBOTIS **Cyclo Intelligence** running on the
separate GPU workstation. Connect it to the AIWorker network over Zenoh, choose
the GR00T N1.7 inference backend, run **3D Sim Deploy** first (which blocks
physical-robot commands), and progress to **Real Robot Deploy** only after the
simulation gate passes.

```
AIWorker ROS 2 / Zenoh  <---- approved private robot network ---->  GPU PC
                                                                  ├─ Cyclo Intelligence
                                                                  └─ GR00T N1.7 backend
```

The company laptop is the known-good ROS 2 / Zenoh client because its ROS graph
is already visible. For the code sample, it subscribes to the verified AIWorker
camera and `JointState` topics, sends observations through the NVIDIA
PolicyServer MessagePack/ZMQ protocol to the GR00T server on the GPU PC over an
SSH tunnel, then emits a candidate `JointTrajectory`. The small local adapter
in `src/groot_policy_client.py` avoids importing model-side PyTorch packages on
the company laptop. The separate gateway remains disarmed by default. No
AIWorker command topic, camera, joint order, or action semantics are guessed.

```text
AIWorker ROS 2 / Zenoh <---7447---> company laptop ROS client
                                       | camera + JointState
                                       | candidate topic
                                       +--- SSH tunnel / ZMQ 5555 ---> GPU PC GR00T N1.7 server
```

The official alternative is Cyclo Intelligence's native GR00T backend and
3D Sim Deploy → Real Robot Deploy workflow. The custom bridge below is useful
when using NVIDIA's server-client API directly; it is not part of Cyclo.

## Status and scope

* Confirmed by the project owner: the company laptop can connect to AIWorker
  and list ROS 2 topics.
* Not yet confirmed: AIWorker software update, controller/action interface,
  camera topic and calibration, teleoperation, model compatibility, data
  collection, fine-tuning, or any closed-loop robot motion.
* The new bridge is configurable and fail-closed: without a reachable GR00T
  server it exits; without `--publish-candidates` it performs inference only;
  even with candidate publishing enabled, the gateway rejects commands while
  `arm_parameter` is false.
* Cyclo Intelligence currently documents native GR00T N1.7 inference and
  AIWorker-specific training/deployment. Its prebuilt ACT/GR00T Docker backend
  is documented for Blackwell-based amd64/arm64 hosts. Since the GPU PC model
  is not yet known, check compatibility before installing the backend.
* Treat a base-checkpoint zero-shot result as an experiment, never as a
  deployment claim.  AIWorker-specific demonstrations and fine-tuning are the
  primary path for the whiteboard task.

The official AIWorker simulation page describes Gazebo URDF/SDF and MuJoCo
MJCF models, and says Isaac Sim USD support is upcoming. Separately, ROBOTIS
`cyclo_lab` contains Isaac Lab/USD examples for particular AIWorker variants.
Confirm the target model and asset revision before selecting that path. The
official GR00T N1.7 project documents a GPU-server / lightweight-client pattern
and custom-embodiment fine-tuning. Links and exact assumptions are recorded in
[`docs/references.md`](docs/references.md).

## Contents

| Item | Purpose |
| --- | --- |
| `config/aiworker.example.yaml` | Fill-in interface contract. It starts disabled and has no guessed topics. |
| `config/aiworker_modality.template.py` | NVIDIA custom-embodiment modality config template for the GPU-side fine-tuning environment. |
| `src/groot_zenoh_infer_node.py` | ROS 2 camera/JointState subscriber, GR00T PolicyServer client, and candidate publisher. No direct controller publish. |
| `src/groot_policy_client.py` | Lightweight ZMQ/MessagePack adapter for NVIDIA's PolicyServer wire protocol; no GR00T model imports on the ROS PC. |
| `src/ros_action_gateway.py` | Unconnected ROS 2 candidate filter. It is not a safety controller and must not replace vendor safety systems. |
| `src/inspect_ros_contract.py` | Captures the discovered ROS graph/types to a reviewable JSON file. |
| `src/inspect_ros_samples.py` | Reads camera metadata and the live `JointState.name` order; it never saves image pixels. |
| `docs/RUNBOOK.md` | Ordered setup, data, fine-tuning, and staged deployment procedure. |
| `requirements-inference-client.txt` | Small dependencies for the ROS-side client; ROS/Jazzy is provided by the vendor environment. |
| `docs/REPORT_DRAFT_ja.md` | OmniCampus PDF report source. Replace only factual placeholders after execution. |
| `docs/VIDEO_SCRIPT_ja.md` | 1–3 minute video plan. |
| `docs/SUBMISSION_CHECKLIST.md` | Assignment title, visibility, PDF, and upload requirements. |

PDF draft: `説明資料_PAI最終課題_アカウント名要入力_草稿.pdf`. Replace the account-name placeholder and factual TODOs only after the required experiment has been run.

## First safe actions on the company laptop

Do not start a controller or publish commands. After sourcing the vendor ROS
environment, record the actual graph:

```bash
cd submmit_advanced_infer_v1
# Replace the topic placeholders with names from `ros2 topic list -t`.
python3 src/inspect_ros_contract.py --output artifacts/ros_graph_before_setup.json
ros2 topic list -t
ros2 action list -t
ros2 service list -t
ros2 topic info --verbose <CAMERA_TOPIC>
ros2 topic info --verbose <JOINT_STATE_TOPIC>
ros2 interface show sensor_msgs/msg/Image
ros2 interface show sensor_msgs/msg/JointState
python3 src/inspect_ros_samples.py \
  --camera-topic <CAMERA_TOPIC> --joint-state-topic <JOINT_STATE_TOPIC> \
  --output artifacts/aiworker_samples_before_setup.json
```

Copy verified topic names, message/action types, camera `frame_id` and encoding,
live `JointState.name` order, and limits into a **private local** copy of
`config/aiworker.example.yaml`. Keep the command topic empty unless the vendor
confirms that it accepts `trajectory_msgs/msg/JointTrajectory`. Never commit
IP addresses, credentials, camera images, rosbag data, serial identifiers, or
company-internal topic names to a public repository.

## What the sample supports

`groot_zenoh_infer_node.py` expects one RGB camera, one `JointState`, and one
ordered joint-position action vector. Its video, state, language, and action
keys must each match the AIWorker-specific GR00T fine-tuning modality
configuration. It sends the configured `chunk_index` point (default `0`) from
the predicted action chunk, and startup verifies that this point has action
`delta_indices` value `0`. NVIDIA's GR00T policy decodes supported
`ABSOLUTE`/`RELATIVE` joint outputs to absolute targets, so the bridge accepts
only `absolute_joint_position` and never adds the current state a second time.
Startup also requires `ActionConfig.type=NON_EEF` and `format=DEFAULT`; EEF,
rotation formats, and raw `DELTA` representations are rejected.
Set `action.duration_s` to one trained action
timestep and the approved controller duration. For Cartesian/end-effector actions, additional
arms, hands, history frames, or normalized actions, adapt the decoder to the
checkpoint's actual schema before running; the sample rejects mismatched
dimensions instead of guessing.

The node publishes only to `/groot_candidate_trajectory` (or the configured
candidate topic), which may be visible to other connected Zenoh peers. It never
publishes to the vendor controller. The
separate gateway requires the exact same ordered joint list, valid limits, a
fresh state/action, and explicit `arm_parameter:=true` before forwarding. Run
the gateway only after the vendor command interface and all configured limits
have been verified. It still does not provide collision, force, acceleration,
E-stop, or controller-ownership checks.

## Candidate-filter limits

The unconnected gateway applies these automated candidate checks:

1. configuration provides exact joint names, position limits, positive velocity limits, and explicit arming;
2. the `JointState` header is recent in the node's ROS clock and its receipt is recent on a monotonic clock; both ages are checked again before forwarding;
3. candidate names/order and timestamp are valid, and the point is position-only;
4. targets stay within configured position and per-joint average-velocity limits;
5. the trajectory duration falls within configured minimum/maximum bounds.
Before any future integration, an operator must separately confirm the exact command interface and vendor safety configuration; the gateway cannot verify those facts itself.

The bridge and gateway both check configured position, per-step, average
velocity, and duration limits before a candidate is published or forwarded.
These local checks use operator-supplied limits and do not prove that those
limits match the vendor controller. The gateway's action input is
`trajectory_msgs/msg/JointTrajectory` on a candidate topic. A per-joint average-velocity estimate is not a controller-level
peak-velocity or acceleration guarantee. The gateway does not check collision,
workspace, force/torque, hand actuation, emergency-stop state, controller
ownership, or ROS/Zenoh publisher identity. Use the vendor's supported safety
controller and physical E-stop; do not connect this scaffold to the AIWorker
as-is. Candidate topics require network isolation/access control outside this
code. ROS timestamps must use the same clock; otherwise the filter rejects the
messages.

The gateway is not a GR00T client, is separate from Cyclo Intelligence, and does
not imply policy or robot compatibility.

## Review checklist

After creating the client environment described in `docs/RUNBOOK.md`, a
maintainer can run these static checks:

```bash
./.venv-ros-client/bin/python -m py_compile src/inspect_ros_contract.py src/inspect_ros_samples.py src/ros_action_gateway.py src/groot_policy_client.py src/groot_zenoh_infer_node.py
./.venv-ros-client/bin/python -c "import yaml; yaml.safe_load(open('config/aiworker.example.yaml'))"
```

These source-level checks need Python and PyYAML, but no running ROS graph or
robot connection. They do not validate physical deployment. The deployment
gates are in `docs/RUNBOOK.md`.
