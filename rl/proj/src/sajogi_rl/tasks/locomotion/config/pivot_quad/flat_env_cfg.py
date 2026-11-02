"""평지 속도 추종 환경.

기반:
  - IsaacLab 3.0 core/velocity/velocity_env_cfg.py 의 LocomotionVelocityRoughEnvCfg (공통 MDP)
  - IsaacLab 3.0 core/velocity/config/go2/{rough,flat}_env_cfg.py (소형 사족보행 오버라이드 방식)
Go2와 다른 부분에는 [변경] 주석.
"""

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass, replace

from isaaclab_tasks.core.velocity import mdp
from isaaclab_tasks.core.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg

from .robot_cfg import PIVOT_QUAD_CFG, PIVOT_QUAD_STANDING_HEIGHT

BASE = "base_link"  # [변경] 공통 cfg는 "base"를 가정 (velocity_env_cfg.py 213, 225, 235, 318행)


@configclass
class PivotQuadRewards(RewardsCfg):
    # go2/rough_env_cfg.py 21-35행과 같은 이유로 추가: 다리 메시에도 충돌체가 있어서
    # 웅크린 채 버티는 정책을 base 접촉 종료나 flat_orientation 으로는 못 잡음
    base_height_l2 = RewTerm(
        func=mdp.base_height_l2,
        weight=0.0,
        params={"target_height": PIVOT_QUAD_STANDING_HEIGHT},
    )


@configclass
class PivotQuadFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: PivotQuadRewards = PivotQuadRewards()

    def __post_init__(self):
        super().__post_init__()

        # ---- scene
        self.scene.robot = replace(PIVOT_QUAD_CFG, prim_path="{ENV_REGEX_NS}/Robot")
        # importer 3.0 출력은 링크가 Robot/Geometry/... 아래 트리로 중첩됨 → 깊이 제한 없이 매칭
        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/Robot/.*"
        # [변경] 8GB VRAM 고려. 근거 없는 시작값이며 CLI --num_envs 로 덮어쓸 수 있음
        self.scene.num_envs = 1024
        self.scene.env_spacing = 1.0
        # 평지 (go2/flat_env_cfg.py 26-30, 36행)
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.curriculum.terrain_levels = None

        # ---- actions: go2 값 (go2/rough_env_cfg.py 61행)
        self.actions.joint_pos.scale = 0.25

        # ---- commands: [변경] 공통값 ±1.0 (velocity_env_cfg.py 146행)은 2 kg 로봇에 과함. 근거 없는 초기값
        ranges = self.commands.base_velocity.ranges
        ranges.lin_vel_x = (-1.0, 1.0)
        ranges.lin_vel_y = (-3.0, 3.0)
        ranges.ang_vel_z = (-0.5, 0.5)

        # cmd = self.commands.base_velocity
        # cmd.heading_command = False        # 무작위 목표 방향 끄기 → 지난번 '휘는 궤적'의 원인 제거
        # cmd.ranges.ang_vel_z = (0.0, 0.0)  # 회전 명령 0 → 직진 유지
        # cmd.ranges.lin_vel_x = (0.0, 0.0)
        # cmd.ranges.lin_vel_y = (0.1, 1.0)
        # cmd.ranges.heading = None

        # ---- events: 바디 이름 교체
        self.events.add_base_mass.params["asset_cfg"].body_names = BASE
        self.events.base_com.params["asset_cfg"].body_names = BASE
        # [변경] 공통 COM 범위 x,y ±5 cm 는 몸통 길이 ~0.3 m 로봇엔 큼 → 축소 (근거 없는 값)
        self.events.base_com.params["com_range"] = {"x": (-0.01, 0.01), "y": (-0.01, 0.01), "z": (-0.005, 0.005)}
        self.events.base_external_force_torque.params["asset_cfg"].body_names = BASE
        # [변경] 처음엔 밀기 끔. 서는 것부터 확인한 뒤 켤 것
        self.events.push_robot = None

        # ---- rewards
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = "calf.*"  # 발 링크 없음 → 종아리 끝 접지
        self.rewards.feet_air_time.weight = 0.25  # go2 flat 값
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = "thigh.*"
        self.rewards.flat_orientation_l2.weight = -2.5  # go2 flat 값
        self.rewards.dof_torques_l2.weight = -0.0002  # go2 값
        self.rewards.base_height_l2.weight = -30.0  # go2 값
        self.rewards.base_height_l2.params["sensor_cfg"] = None  # 평지: 높이 스캐너 없이 절대 높이 사용

        # ---- terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = BASE
