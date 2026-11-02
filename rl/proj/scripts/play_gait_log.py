"""학습된 정책을 `isaaclab play` 와 똑같이 돌리면서 매 env step 의 관절·접촉 데이터를 기록한다.

동작 방식 (로컬 IsaacLab 소스 기준, 라이브러리 파일은 수정하지 않음):
  1. `RslRlVecEnvWrapper.step` 을 감싸서, 원래 step 직후 값을 읽는다.
     (play 루프: isaaclab_rl/entrypoints/backends/play_rsl_rl.py 154-158행에서 env.step 호출)
  2. 외부 태스크 등록: isaaclab/cli/__init__.py 40-43행 `_load_external_tasks()`
     → pyproject.toml 의 entry point `isaaclab.tasks: sajogi_rl = "sajogi_rl.tasks"` 를 import
     (`isaaclab play` 는 210행에서 서브커맨드 실행 전에 209행에서 이것을 호출함.
      dispatch.run_play_cli 를 직접 부르면 이 단계가 빠져서 NameNotFound 가 난다.)
  3. isaaclab/cli/__init__.py 60-64행 `play(args)` 호출 = `isaaclab play` 와 같은 경로.
  4. --log_steps 만큼 기록하면 저장 후 KeyboardInterrupt 를 던져 재생을 끝낸다
     (entrypoints/common.py 1032행 run_playback 이 KeyboardInterrupt 를 삼키고 정상 종료).

실행 (작업 디렉터리 = sajogi_rl 루트):
  uv run python scripts/play_gait_log.py --log_steps 600 --log_out gait_log.npz -- \
      --task Isaac-Velocity-Flat-PivotQuad --num_envs 1 \
      --checkpoint 2026-10-05_18-02-30/model_299.pt \
      env.commands.base_velocity.heading_command=false \
      env.commands.base_velocity.rel_standing_envs=0.0 \
      env.commands.base_velocity.ranges.lin_vel_x="[0.0,0.0]" \
      env.commands.base_velocity.ranges.lin_vel_y="[0.3,0.3]" \
      env.commands.base_velocity.ranges.ang_vel_z="[0.0,0.0]"
  `--` 앞: 이 스크립트 인자 / 뒤: `isaaclab play` 에 주던 인자 그대로 (--rl_library 는 없으면 rsl_rl 로 채움)

기록 항목 (env 0 하나만, 매 env step = 0.02 s):
  raw      정책 출력                                  joint_actions.py 171행 _raw_actions
  target   위치 목표 [rad] = raw*scale + default      joint_actions.py 173-174행 (이 로봇: 0.25*raw + 0)
  q, qd    관절각 [rad], 관절속도 [rad/s]
  tau      추정 토크 [N·m], clamp 후                 envs/mdp/rewards.py 176행 dof_torques_l2 가 쓰는 값
  tau_raw  추정 토크, clamp 전 (tau 와 다르면 effort limit 포화)
  contact  calf 링크 접지 여부 (current_contact_time > 0)
  fz       calf 링크 수직 접촉력 [N]
  base_pos_w, base_lin_vel_b, cmd   몸통 위치·속도, 속도 명령
  done     리셋 발생 여부
"""

from __future__ import annotations

import argparse
import sys

import numpy as np


def _split_argv(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    if "--" in argv:
        i = argv.index("--")
        own, rest = argv[:i], argv[i + 1 :]
    else:
        own, rest = [], argv
    p = argparse.ArgumentParser(description="Record joint/contact data while playing an RSL-RL policy.")
    p.add_argument("--log_steps", type=int, default=600, help="기록할 env step 수 (0.02 s/step). 기본 600 = 12 s")
    p.add_argument("--log_out", type=str, default="gait_log.npz")
    p.add_argument("--foot_regex", type=str, default="calf.*", help="접지 판정할 링크 (feet_air_time 과 같은 calf.*)")
    return p.parse_args(own), rest


def main() -> None:
    args, play_argv = _split_argv(sys.argv[1:])
    if "--rl_library" not in play_argv:
        play_argv = ["--rl_library", "rsl_rl", *play_argv]

    from isaaclab.cli import _load_external_tasks, play
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

    rec: dict[str, list] = {}
    meta: dict = {}
    h: dict = {}  # 첫 step 에서 채우는 핸들

    def setup(base) -> None:
        h["robot"] = base.scene["robot"]
        h["act"] = base.action_manager.get_term("joint_pos")
        h["sensor"] = base.scene.sensors["contact_forces"]  # velocity_env_cfg.py 117행
        foot_ids, foot_names = h["sensor"].find_bodies(args.foot_regex)  # base_contact_sensor.py 258행
        h["foot_ids"] = foot_ids
        meta["dt"] = float(base.step_dt)
        meta["joint_names"] = np.array(h["robot"].joint_names)
        meta["action_joint_names"] = np.array(h["act"]._joint_names)
        meta["foot_names"] = np.array(foot_names)
        print(f"[GAIT] joint order : {list(h['robot'].joint_names)}")
        print(f"[GAIT] foot bodies : {foot_names}")

    def grab(base, dones) -> dict:
        r, a, s, f = h["robot"], h["act"], h["sensor"], h["foot_ids"]
        fn = s.data.net_normal_forces_w.torch[0, f]  # (n_feet, 3)
        return {
            "raw": a.raw_actions[0],
            "target": a.processed_actions[0],
            "q": r.data.joint_pos.torch[0],
            "qd": r.data.joint_vel.torch[0],
            "tau": r.actuators.applied_effort.torch[0],
            "tau_raw": r.actuators.computed_effort.torch[0],
            "contact": s.data.current_contact_time.torch[0, f] > 0.0,
            "fz": fn[:, 2],
            "base_pos_w": r.data.root_pos_w.torch[0],
            "base_lin_vel_b": r.data.root_lin_vel_b.torch[0],
            "cmd": base.command_manager.get_command("base_velocity")[0],
            "done": dones[0],
        }

    def save() -> None:
        n = len(rec.get("done", []))
        if n == 0:
            print("[GAIT] 기록된 step 이 없어 저장하지 않음")
            return
        out = {k: np.stack([v.detach().cpu().numpy() for v in vals]) for k, vals in rec.items()}
        out.update(meta)
        out["t"] = np.arange(n) * meta["dt"]
        np.savez(args.log_out, **out)
        print(f"[GAIT] {n} steps ({n * meta['dt']:.2f} s) 저장 → {args.log_out}")
        if out["done"].any():
            print(f"[GAIT] 주의: 리셋 발생 step {np.flatnonzero(out['done']).tolist()}")

    original_step = RslRlVecEnvWrapper.step

    def logged_step(self, actions):
        result = original_step(self, actions)  # (obs, rew, dones, extras): play_rsl_rl.py 156행
        base = self.unwrapped
        if not h:
            setup(base)
        for k, v in grab(base, result[2]).items():
            rec.setdefault(k, []).append(v.clone())
        if len(rec["done"]) >= args.log_steps:
            save()
            raise KeyboardInterrupt  # run_playback 이 삼키고 env 를 정상 종료
        return result

    RslRlVecEnvWrapper.step = logged_step
    _load_external_tasks()
    try:
        play(play_argv)
    finally:
        if rec and len(rec["done"]) < args.log_steps:  # Ctrl+C 등으로 일찍 끝난 경우
            save()


if __name__ == "__main__":
    main()
