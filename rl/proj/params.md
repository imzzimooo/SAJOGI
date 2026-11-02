[velocity_env_cfg.py](IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/velocity/velocity_env_cfg.py)
- `ranges.lin_vel_x`: 목표 x축 선형 속도 범위
- `ranges.lin_vel_y`: 목표 y축 (전진) 성형 속도 범위
- `ranges.ang_vel_z`: 목표 z축 (방향) 각도 범위
- `RewardsCfg`: `RewTerm`의 `func`로 실제 보상을 계산한다.
- `feet_air_time`: 실제로 발이 떨어져 있는 시간이 `thresold`값보다 크면 보상을 준다
- `undesired_contacts`: 원하지 않는 접촉력의 총합이 `thresold`값보다 크면 접촉들의 개수만큼 페널티
- `dof_torques_l2`: 토크 크기 자체에 페널티
- `base_height_l2`: base가 목표 위치에서 벗어난 크기만큼 페널티
- `flat_orientation_l2`: base가 평평하지 않게 방향이 잡힌 경우 페널티

[velocity/rewards.py](IsaacLab/source/isaaclab_tasks/isaaclab_tasks/core/velocity/mdp/rewards.py)
- `feet_air_time` 함수가 여기 정의되어 있음

[envs/rewards.py](IsaacLab/source/isaaclab/isaaclab/envs/mdp/rewards.py)
[isaac lab](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/api/lab/isaaclab.envs.mdp.html#module-isaaclab.envs.mdp.rewards)
- `LocomotionVelocityRoughEnvCfg` 관련 보상 함수들이 여기 정의되어 있음

[RewTerm](IsaacLab/source/isaaclab/isaaclab/managers/manager_term_cfg.py)
- `LocomotionVelocityRoughEnvConfig`의 `RewardTermCfg` (alias `RewTerm`)의 정의