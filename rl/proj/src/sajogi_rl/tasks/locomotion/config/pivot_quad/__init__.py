"""로보인 pivot 사족보행 — 평지 속도 추종.

등록 형식: IsaacLab 3.0 source/isaaclab_tasks/isaaclab_tasks/core/velocity/config/go2/__init__.py
"""

import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Velocity-Flat-PivotQuad",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:PivotQuadFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PivotQuadFlatPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
