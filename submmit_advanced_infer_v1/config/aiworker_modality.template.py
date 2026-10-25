"""GR00T N1.7 custom-embodiment template for one AIWorker arm vector.

Replace every TODO key with the matching key from the dataset's
meta/modality.json. This file is imported only on the GPU training machine.
It does not describe or imply the AIWorker's real joint count/order.
"""
from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)


aiworker_modality_config = {
    "video": ModalityConfig(
        delta_indices=[0],
        modality_keys=["TODO_CAMERA_KEY"],
    ),
    "state": ModalityConfig(
        delta_indices=[0],
        modality_keys=["TODO_ORDERED_JOINT_STATE_KEY"],
    ),
    "action": ModalityConfig(
        delta_indices=list(range(0, 16)),
        modality_keys=["TODO_ORDERED_JOINT_ACTION_KEY"],
        action_configs=[
            ActionConfig(
                # ABSOLUTE matches absolute target actions. RELATIVE may be used
                # when the source dataset stores absolute state/action and GR00T
                # should train on relative targets; state_key must match a state key.
                # PolicyServer decodes supported ABSOLUTE/RELATIVE outputs to targets.
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="TODO_ORDERED_JOINT_STATE_KEY",
            ),
        ],
    ),
    "language": ModalityConfig(
        delta_indices=[0],
        modality_keys=["TODO_LANGUAGE_KEY"],
    ),
}

register_modality_config(
    aiworker_modality_config,
    embodiment_tag=EmbodimentTag.NEW_EMBODIMENT,
)
