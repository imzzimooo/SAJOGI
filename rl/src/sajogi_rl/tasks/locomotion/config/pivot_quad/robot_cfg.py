"""pivot 사족보행 로봇 ArticulationCfg.

형식: IsaacLab 3.0 source/isaaclab_assets/isaaclab_assets/robots/unitree.py 의 UNITREE_GO2_CFG,
URDF spawn 부분은 source/isaaclab/test/sim/test_spawn_from_files.py 의 UrdfFileCfg 사용 예.
"""

from pathlib import Path

from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# src/<project>/tasks/locomotion/config/pivot_quad/robot_cfg.py 기준으로
# parents[4] == src/<project>  →  src/<project>/assets/data/pivot_quad/urdf/robot_isaaclab.urdf
# (생성된 assets/__init__.py 의 <PROJECT>_ASSETS_DIR 로 바꿔 써도 됨)
_USD = Path(__file__).resolve().parents[4] / "assets" / "data" / "pivot_quad" / "usd" / "robot_isaaclab" / "robot_isaaclab.usda"

# MuJoCo에서 관절각 0일 때 base가 z=0.225 m에 서는 것을 확인함
PIVOT_QUAD_STANDING_HEIGHT = 0.225

PIVOT_QUAD_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(_USD),
        activate_contact_sensors=True,
        rigid_props=PhysxRigidBodyCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=[
            PhysxArticulationCfg(
                enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
            ),
            NewtonArticulationCfg(self_collision_enabled=False),
        ],
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.26),
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # 전부 임시값. MX-64 실제 토크·속도·반사관성은 확인 안 함.
        # joint_effort_limit: 3.0에서 effort_limit_sim 대신 쓰는 이름 (actuator_base_cfg.py 73, 86행)
        "legs": ImplicitActuatorCfg(
            joint_names_expr=["revolute_.*"],
            joint_effort_limit=6.0,
            stiffness=20.0,
            damping=0.5,
            armature=0.01,
        ),
    },
)
