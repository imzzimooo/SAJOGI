"""play_gait_log.py 가 만든 gait_log.npz 에서 보행 2주기를 잘라내고, 관절↔모터 매핑을 붙여 저장한다.

주기 판정: 기준 다리(calf 링크)의 착지(접지 False→True) 시점 3개 = 2주기.
  - 처음 --warmup_s 초는 기동 과도구간이라 버림
  - 구간 안에 리셋(done)이 있으면 그 구간은 건너뜀
  - 접지 데이터로 못 찾으면 관절 목표의 자기상관으로 주기를 추정 (fallback)

관절↔모터 매핑은 하드코딩하지 않고 URDF 를 직접 파싱해서 만든다:
  - 각 revolute 관절의 parent 링크 = 그 관절을 구동하는 모터 링크 (mx_64__N__1)
  - 관절 origin 을 base_link 까지 누적 = 관절각 0 에서의 회전축 위치 (URDF 의 모든 rpy 가 0 일 때만 유효; 아니면 중단)
  - 다리 이름은 위치 사분면: A(x-,y-) B(x-,y+) C(x+,y-) D(x+,y+), 축이 z 인 관절은 pivot
  - 역할: child 가 thigh* 면 고관절(hip), calf* 면 무릎(knee)
  - revolute_4(pivot) 의 하위 체인에 있으면 '피벗측', 아니면 '몸통측'

실행 (sajogi_rl 루트):
  uv run python scripts/extract_gait_cycles.py gait_log.npz
  → gait_2cycles.csv, joint_motor_map.csv, gait_2cycles.png (matplotlib 있을 때)
"""

from __future__ import annotations

import argparse
import csv
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_URDF = Path(__file__).resolve().parents[1] / "src/sajogi_rl/assets/data/pivot_quad/urdf/robot_isaaclab.urdf"


# --------------------------------------------------------------------------- URDF → 관절/모터 매핑
def build_joint_map(urdf_path: Path) -> tuple[dict[str, dict], dict[str, str]]:
    root = ET.parse(urdf_path).getroot()
    joints = root.findall("joint")
    by_child = {j.find("child").get("link"): j for j in joints}

    def chain(link: str) -> list[ET.Element]:
        out = []
        while link in by_child:
            j = by_child[link]
            out.append(j)
            link = j.find("parent").get("link")
        return out

    def pos_in_base(link: str) -> np.ndarray:
        p = np.zeros(3)
        for j in chain(link):
            o = j.find("origin")
            if o is not None:
                if any(abs(float(v)) > 1e-9 for v in o.get("rpy", "0 0 0").split()):
                    raise ValueError(f"{j.get('name')} 에 rpy 회전이 있어 단순 누적으로는 위치를 못 구함")
                p += np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        return p

    revs = [j for j in joints if j.get("type") == "revolute"]
    pivot_names = set()
    info: dict[str, dict] = {}
    for j in revs:
        axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
        if abs(axis[2]) > 0.9:
            pivot_names.add(j.get("name"))
    for j in revs:
        name = j.get("name")
        child = j.find("child").get("link")
        parent = j.find("parent").get("link")
        p = pos_in_base(child)
        axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
        lim = j.find("limit")
        if name in pivot_names:
            leg, role = "pivot", "pivot"
        else:
            leg = {(-1, -1): "A", (-1, 1): "B", (1, -1): "C", (1, 1): "D"}[(int(np.sign(p[0])), int(np.sign(p[1])))]
            role = "hip" if child.startswith("thigh") else "knee" if child.startswith("calf") else child
        side = "-" if leg == "pivot" else (
            "pivot_side" if any(a.get("name") in pivot_names for a in chain(child)[1:]) else "body_side"
        )
        info[name] = {
            "joint": name,
            "leg": leg,
            "quadrant": "" if leg == "pivot" else f"x{'+' if p[0] > 0 else '-'},y{'+' if p[1] > 0 else '-'}",
            "role": role,
            "mount": side,
            "motor_link": parent,
            "child_link": child,
            "pos_x": p[0], "pos_y": p[1], "pos_z": p[2],
            "axis": " ".join(f"{a:+.3f}" for a in axis),
            "lower": float(lim.get("lower")), "upper": float(lim.get("upper")),
        }
    foot_leg = {v["child_link"]: v["leg"] for v in info.values() if v["role"] == "knee"}
    return info, foot_leg


# --------------------------------------------------------------------------- 주기 찾기
def touchdowns(contact: np.ndarray) -> np.ndarray:
    c = contact.astype(bool)
    return np.flatnonzero(~c[:-1] & c[1:]) + 1


def find_two_cycles(d, warmup_steps: int, ref: int | None):
    contact, done = d["contact"], d["done"].astype(bool)
    feet = range(contact.shape[1]) if ref is None else [ref]
    for k in feet:
        td = touchdowns(contact[:, k])
        td = td[td >= warmup_steps]
        for i in range(len(td) - 2):
            a, b = td[i], td[i + 2]
            if not done[a:b].any():
                return a, b, k, np.diff(td[i : i + 3])
    return None


def autocorr_period(x: np.ndarray, min_lag: int = 5) -> int:
    s = x - x.mean()
    ac = np.correlate(s, s, "full")[len(s) - 1 :]
    return int(np.argmax(ac[min_lag : len(s) // 2]) + min_lag)


# --------------------------------------------------------------------------- main
def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("log", type=Path, help="play_gait_log.py 출력 npz")
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    p.add_argument("--warmup_s", type=float, default=3.0)
    p.add_argument("--ref_foot", type=str, default=None, help="기준 calf 링크 이름 (기본: 처음 조건을 만족하는 다리)")
    p.add_argument("--out_prefix", type=str, default="gait_2cycles")
    a = p.parse_args()

    d = dict(np.load(a.log, allow_pickle=False))
    dt = float(d["dt"])
    names = [str(n) for n in d["joint_names"]]
    feet = [str(n) for n in d["foot_names"]]
    jmap, foot_leg = build_joint_map(a.urdf)

    # ---- 매핑 표 (sim 인덱스 = npz 열 순서)
    map_rows = []
    for i, n in enumerate(names):
        row = {"sim_index": i, **jmap.get(n, {"joint": n})}
        map_rows.append(row)
    with open("joint_motor_map.csv", "w", newline="") as f:
        fields = list(dict.fromkeys(k for r in map_rows for k in r))
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(map_rows)
    print("sim  joint        leg    role   mount       motor_link     pos(x,y,z) [m]")
    for r in map_rows:
        if "leg" in r:
            print(f"{r['sim_index']:>3}  {r['joint']:<12} {r['leg']:<6} {r['role']:<6} {r['mount']:<11} "
                  f"{r['motor_link']:<14} ({r['pos_x']:+.3f},{r['pos_y']:+.3f},{r['pos_z']:+.3f})")
        else:
            print(f"{r['sim_index']:>3}  {r['joint']:<12} (URDF 에 없음)")
    print("feet:", {f: foot_leg.get(f, "?") for f in feet})

    # ---- 2주기 구간
    ref = feet.index(a.ref_foot) if a.ref_foot else None
    res = find_two_cycles(d, int(round(a.warmup_s / dt)), ref)
    tgt = d["target"]
    if res is not None:
        i0, i1, k, periods = res
        how = f"접지 기준 ({feet[k]} = 다리 {foot_leg.get(feet[k], '?')}), 주기 {periods * dt} s"
    else:
        j = int(np.argmax(tgt[int(a.warmup_s / dt) :].std(axis=0)))
        lag = autocorr_period(tgt[int(a.warmup_s / dt) :, j])
        i0 = int(round(a.warmup_s / dt))
        i1 = i0 + 2 * lag
        how = f"접지 판정 실패 → {names[j]} 목표의 자기상관, 주기 {lag * dt:.3f} s"
    if i1 > len(tgt):
        raise SystemExit("기록이 짧아 2주기를 못 자름. --log_steps 를 늘려서 다시 기록하세요.")
    print(f"2주기 구간: step {i0}..{i1 - 1} (t = {i0 * dt:.2f}–{i1 * dt:.2f} s, {i1 - i0} 샘플)  [{how}]")

    seg = slice(i0, i1)
    sat = (np.abs(d["tau_raw"][seg] - d["tau"][seg]) > 1e-6).any(axis=0)
    if sat.any():
        print("토크 포화(effort limit) 발생 관절:", [names[i] for i in np.flatnonzero(sat)])
    v = d["base_lin_vel_b"][seg].mean(axis=0)
    print(f"구간 평균 몸통 속도 (base frame) vx={v[0]:+.3f} vy={v[1]:+.3f} m/s, 명령={d['cmd'][i0][:3]}")

    # ---- CSV: 행 = env step, 열 = 시간 + 관절별(목표/실제/토크) + 다리별 접지
    def label(n):
        m = jmap.get(n)
        return f"{n}[{m['leg']}_{m['role']}|{m['motor_link']}]" if m else n

    header = ["step", "t_s", "t_rel_s"]
    for n in names:
        L = label(n)
        header += [f"{L}_target_rad", f"{L}_target_deg", f"{L}_q_rad", f"{L}_q_deg", f"{L}_qd_rad_s", f"{L}_tau_Nm"]
    header += [f"contact_{f}[{foot_leg.get(f, '?')}]" for f in feet]
    with open(f"{a.out_prefix}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for s in range(i0, i1):
            row = [s, f"{s * dt:.3f}", f"{(s - i0) * dt:.3f}"]
            for j in range(len(names)):
                row += [
                    f"{tgt[s, j]:.5f}", f"{np.degrees(tgt[s, j]):.2f}",
                    f"{d['q'][s, j]:.5f}", f"{np.degrees(d['q'][s, j]):.2f}",
                    f"{d['qd'][s, j]:.5f}", f"{d['tau'][s, j]:.4f}",
                ]
            row += [int(c) for c in d["contact"][s]]
            w.writerow(row)
    print(f"저장: {a.out_prefix}.csv, joint_motor_map.csv")

    # ---- 그림 (선택)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 없음 → 그림 생략")
        return
    t = (np.arange(i0, i1) - i0) * dt
    fig, axes = plt.subplots(len(names), 3, figsize=(16, 2.0 * len(names)), sharex=True)
    for j, n in enumerate(names):
        ax = axes[j, 0]
        ax.plot(t, np.degrees(tgt[seg, j]), label="target (position command)")
        ax.plot(t, np.degrees(d["q"][seg, j]), label="q (joint position)", alpha=0.8)
        ax.set_ylabel(f"{label(n)}\n[deg]", fontsize=7)
        axes[j, 1].plot(t, d["qd"][seg, j], color="tab:green")
        axes[j, 1].set_ylabel("qd [rad/s]", fontsize=7)
        axes[j, 2].plot(t, d["tau"][seg, j], color="tab:red")
        axes[j, 2].set_ylabel("tau est. [N·m]", fontsize=7)
    axes[0, 0].legend(fontsize=7)
    axes[0, 0].set_title("joint position", fontsize=9)
    axes[0, 1].set_title("joint velocity", fontsize=9)
    axes[0, 2].set_title("joint torque (Isaac Lab estimate)", fontsize=9)
    for c in range(3):
        axes[-1, c].set_xlabel("t from cycle start [s]")
    fig.tight_layout()
    fig.savefig(f"{a.out_prefix}.png", dpi=120)
    print(f"저장: {a.out_prefix}.png")


if __name__ == "__main__":
    main()
