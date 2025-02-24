# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents
from .automate_env import AutomateEnv
from .automate_env_cfg import AutomateTaskPlugInsertCfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Automate-Direct-v0",
    entry_point="isaaclab_tasks.direct.automate:AutomateEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": AutomateTaskPlugInsertCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
)