#/bin/bash

export MAGNUM_LOG=quiet
export HABITAT_SIM_LOG=quiet

set -x
python -u -m habitat_baselines.run \
   --config-name ovmm/rl_skill.yaml \
   benchmark/ovmm=gaze \
   habitat_baselines.checkpoint_folder=data/new_checkpoints/ovmm/gaze