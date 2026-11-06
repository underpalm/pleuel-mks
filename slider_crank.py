"""Slider-crank mechanism in MuJoCo: real connecting rod vs. two- and three-mass equivalent systems.

Three identical slider-crank mechanisms (crank, connecting rod, piston) run side by side at
constant speed; only the connecting rod differs:
    real   - rod from Onshape (mesh, mass, full inertia tensor)
    two    - two-mass model (point masses at A and B)
    three  - three-mass model (point masses at A, S and B)

The mechanism is a closed kinematic loop: crank -> rod (kinematic tree) and piston (slider on
the frame); the loop is closed by a joint constraint (equality/connect) at the piston pin.

Usage:   uv run python slider_crank.py [--rpm 3000] [--gravity]
Output:  results/*.png, results/history_*.csv, results/SUMMARY.md,
         slider_crank.xml (for the viewer)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from equivalent_system import ConRod, three_mass, two_mass  # noqa: E402

HERE = Path(__file__).parent
RESULTS = HERE / "results"
MESH = HERE / "assets" / "conrod.stl"

R_CRANK = 0.040  # m
M_PISTON = 0.350  # kg (piston + pin + rings, typical small engine)
DT = 1e-5  # s
VARIANTS = {
    "real": ("Real connecting rod (CAD)", "tab:blue", 0.0),
    "two": ("Two-mass model", "tab:red", 0.12),
    "three": ("Three-mass model", "tab:green", 0.24),
}
TINY = 1e-10  # inertia of a point mass (MuJoCo requires > 0)


def _rod_body(v: str, p: ConRod) -> str:
    """Body XML of the connecting rod of variant v (origin = big end A, x axis -> B)."""
    L, s = p.L, p.s
    if v == "real":
        I = p.inertia_full
        full = f"{I[0][0]:.9g} {I[1][1]:.9g} {I[2][2]:.9g} {I[0][1]:.9g} {I[0][2]:.9g} {I[1][2]:.9g}"
        return f"""
          <inertial pos="{s:.9g} 0 0" mass="{p.m:.9g}" fullinertia="{full}"/>
          <geom type="mesh" mesh="conrod" rgba="0.55 0.65 0.8 1"/>"""
    masses = (
        [("A", 0.0, two_mass(p).m_A), ("B", L, two_mass(p).m_B)]
        if v == "two"
        else [("A", 0.0, three_mass(p).m_A), ("S", s, three_mass(p).m_S), ("B", L, three_mass(p).m_B)]
    )
    xml = f"""
          <inertial pos="{L / 2:.6g} 0 0" mass="1e-6" diaginertia="{TINY} {TINY} {TINY}"/>
          <geom type="capsule" fromto="0 0 0 {L:.6g} 0 0" size="0.002" rgba="0.3 0.3 0.3 1" mass="0"/>"""
    for name, x, m in masses:
        radius = 0.006 + 0.012 * (m / p.m) ** (1 / 3)
        xml += f"""
          <body name="mass_{name}_{v}" pos="{x:.9g} 0 0">
            <inertial pos="0 0 0" mass="{m:.9g}" diaginertia="{TINY} {TINY} {TINY}"/>
            <geom type="sphere" size="{radius:.4f}" rgba="0.85 0.25 0.2 1" mass="0"/>
          </body>"""
    return xml


def build_model(p: ConRod, omega: float, gravity: bool) -> str:
    L, r = p.L, R_CRANK
    world = []
    for v, (_, _color, z) in VARIANTS.items():
        world.append(f"""
    <body name="crank_{v}" pos="0 0 {z}">
      <joint name="crank_{v}" type="hinge" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="1e-3 1e-3 1e-3"/>
      <geom type="capsule" fromto="0 0 -0.012 {r} 0 -0.012" size="0.008" rgba="0.4 0.4 0.4 1" mass="0"/>
      <body name="rod_{v}" pos="{r} 0 0">
        <joint name="rod_{v}" type="hinge" axis="0 0 1"/>
        <site name="A_{v}" pos="0 0 0"/>{_rod_body(v, p)}
      </body>
    </body>
    <body name="piston_{v}" pos="{r + L:.9g} 0 {z}">
      <joint name="piston_{v}" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="{M_PISTON}" diaginertia="1e-4 1e-4 1e-4"/>
      <geom type="cylinder" size="0.03 0.025" quat="0.7071068 0 0.7071068 0" pos="0.03 0 0" rgba="0.7 0.7 0.75 0.6" mass="0"/>
      <site name="K_{v}" pos="0 0 0"/>
    </body>""")
    eq = "".join(
        f'\n    <connect name="pin_{v}" body1="rod_{v}" body2="piston_{v}" anchor="{L:.9g} 0 0" solref="0.00003 1"/>'
        for v in VARIANTS
    )
    act = "".join(
        f'\n    <velocity name="motor_{v}" joint="crank_{v}" kv="20000" ctrlrange="0 400" ctrllimited="true"/>'
        for v in VARIANTS
    )
    sens = "".join(
        f'\n    <actuatorfrc name="M_{v}" actuator="motor_{v}"/>'
        f'\n    <force name="FA_{v}" site="A_{v}"/>'
        f'\n    <force name="FK_{v}" site="K_{v}"/>'
        for v in VARIANTS
    )
    g = "0 -9.81 0" if gravity else "0 0 0"
    return f"""<mujoco model="slider_crank_conrod_comparison">
  <compiler angle="radian" meshdir="assets" inertiafromgeom="false"/>
  <option timestep="{DT}" gravity="{g}" integrator="implicitfast">
    <flag contact="disable"/>
  </option>
  <asset>
    <mesh name="conrod" file="{MESH.name}" scale="0.001 0.001 0.001"/>
  </asset>
  <worldbody>
    <light pos="0.1 0.3 1" dir="0 0 -1" directional="true"/>{"".join(world)}
  </worldbody>
  <equality>{eq}
  </equality>
  <actuator>{act}
  </actuator>
  <sensor>{sens}
  </sensor>
</mujoco>
"""


def load_model(xml: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(xml, {MESH.name: MESH.read_bytes()})


# --------------------------------------------------------------------------- kinematics
def kinematics(phi: float, omega: float, p: ConRod):
    """Exact position/velocity of the rod angle β (relative to the crank) and the piston."""
    r, L = R_CRANK, p.L
    sin_psi = -r * math.sin(phi) / L  # absolute rod angle ψ (piston on the x axis)
    psi = math.asin(sin_psi)
    x = r * math.cos(phi) + L * math.cos(psi)
    dpsi = -r * math.cos(phi) * omega / (L * math.cos(psi))
    dx = -r * math.sin(phi) * omega - L * math.sin(psi) * dpsi
    beta, dbeta = psi - phi, dpsi - omega
    return beta, dbeta, x - (r + L), dx


def piston_acceleration(phi: np.ndarray, omega: float, p: ConRod) -> np.ndarray:
    """Analytic piston acceleration (exact x(φ), differentiated numerically)."""
    r, L = R_CRANK, p.L
    x = r * np.cos(phi) + np.sqrt(L**2 - (r * np.sin(phi)) ** 2)
    h = 1e-4
    xp = r * np.cos(phi + h) + np.sqrt(L**2 - (r * np.sin(phi + h)) ** 2)
    xm = r * np.cos(phi - h) + np.sqrt(L**2 - (r * np.sin(phi - h)) ** 2)
    return (xp - 2 * x + xm) / h**2 * omega**2


# --------------------------------------------------------------------------- simulation
def simulate(xml: str, p: ConRod, omega: float, revolutions: float = 3.0):
    m = load_model(xml)
    d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
    sens = lambda n: m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, n)]  # noqa: E731
    # consistent initial state at φ = 0 with full speed (no run-up phase)
    beta, dbeta, xk, dxk = kinematics(0.0, omega, p)
    for v in VARIANTS:
        d.qvel[m.jnt_dofadr[jid(f"crank_{v}")]] = omega
        d.qpos[m.jnt_qposadr[jid(f"rod_{v}")]] = beta
        d.qvel[m.jnt_dofadr[jid(f"rod_{v}")]] = dbeta
        d.qpos[m.jnt_qposadr[jid(f"piston_{v}")]] = xk
        d.qvel[m.jnt_dofadr[jid(f"piston_{v}")]] = dxk
    d.ctrl[:] = omega
    mujoco.mj_forward(m, d)
    n = int(revolutions * 2 * math.pi / omega / DT)
    keys = ("phi", "M", "FAx", "FAy", "FN", "FBx", "FBy", "xk", "ak", "omega")
    rec = {v: {k: np.empty(n) for k in keys} for v in VARIANTS}
    for i in range(n):
        # sensors/accelerations of a step belong to the state BEFORE integration
        before = {
            v: (
                d.qpos[m.jnt_qposadr[jid(f"crank_{v}")]],
                d.qvel[m.jnt_dofadr[jid(f"crank_{v}")]],
                d.qpos[m.jnt_qposadr[jid(f"piston_{v}")]],
            )
            for v in VARIANTS
        }
        mujoco.mj_step(m, d)
        for v in VARIANTS:
            r = rec[v]
            pj = jid(f"piston_{v}")
            r["phi"][i], r["omega"][i], xk_before = before[v]
            r["M"][i] = d.sensordata[sens(f"M_{v}")]
            # force crank -> rod at the big end (sensor in the rod frame) -> world frame
            site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"A_{v}")
            fa = d.site_xmat[site].reshape(3, 3) @ d.sensordata[sens(f"FA_{v}") : sens(f"FA_{v}") + 3]
            r["FAx"][i], r["FAy"][i] = fa[0], fa[1]
            # piston: wall (normal) force from the slider joint, pin force from Newton's 2nd law
            fk = d.sensordata[sens(f"FK_{v}") : sens(f"FK_{v}") + 3]
            ak = d.qacc[m.jnt_dofadr[pj]]
            r["FN"][i] = fk[1]
            r["FBx"][i], r["FBy"][i] = M_PISTON * ak - fk[0], -fk[1]
            r["xk"][i], r["ak"][i] = xk_before, ak
    # evaluate the last full revolution
    for v, r in rec.items():
        phi = r["phi"]
        sel = phi >= phi[-1] - 2 * math.pi
        for k in r:
            r[k] = r[k][sel]
        r["phi_deg"] = np.degrees(np.mod(r["phi"], 2 * math.pi))
        order = np.argsort(r["phi_deg"])
        for k in list(r):
            r[k] = r[k][order]
    return m, rec


# --------------------------------------------------------------------------- evaluation
def evaluate(rec, p: ConRod, omega: float, rpm: float, gravity: bool) -> str:
    RESULTS.mkdir(exist_ok=True)
    real = rec["real"]
    phi = np.radians(real["phi_deg"])
    # check 1: piston acceleration vs. analytic solution
    a_ana = piston_acceleration(phi, omega, p)
    err_a = np.max(np.abs(real["ak"] - a_ana)) / np.max(np.abs(a_ana))
    # check 2: speed ripple of the drive
    dw = max(np.ptp(r["omega"]) for r in rec.values()) / omega
    # check 3: theory for the torque error of the two-mass model
    #   ΔM ≈ ½ (J_2 - J_S) λ² ω² sin 2φ,  λ = r/L  (first-order approximation in λ)
    lam = R_CRANK / p.L
    dM_theory = 0.5 * (two_mass(p).J_model - p.J_S) * lam**2 * omega**2
    dM_sim = np.max(np.abs(rec["two"]["M"] - real["M"]))
    # check 4: mean crank torque over one revolution (lossless -> ~0)
    m_mean = max(abs(np.trapezoid(r["M"], np.radians(r["phi_deg"])) / (2 * math.pi)) for r in rec.values())

    quantities = {
        "M": ("Crank torque M", "Nm", lambda r: r["M"]),
        "FA": ("Crank pin force |F_A|", "N", lambda r: np.hypot(r["FAx"], r["FAy"])),
        "FB": ("Piston pin force |F_B|", "N", lambda r: np.hypot(r["FBx"], r["FBy"])),
        "FN": ("Piston side force F_N", "N", lambda r: r["FN"]),
    }
    fig, ax = plt.subplots(2, 2, figsize=(13, 8.5), sharex=True)
    for a, (title, unit, f) in zip(ax.flat, quantities.values()):
        for v, (label, color, _) in VARIANTS.items():
            ls = "--" if v == "three" else "-"
            a.plot(rec[v]["phi_deg"], f(rec[v]), ls, color=color, lw=1.6 if v != "three" else 2.2, label=label)
        a.set_title(title)
        a.set_ylabel(unit)
        a.grid(alpha=0.3)
    for a in ax[1]:
        a.set_xlabel("Crank angle φ in °  (0° = top dead centre)")
        a.set_xticks(range(0, 361, 45))
    ax[0, 0].legend()
    fig.suptitle(
        f"Slider-crank r = {R_CRANK * 1000:.0f} mm, L = {p.L * 1000:.0f} mm, {rpm:.0f} rpm"
        " – real connecting rod vs. equivalent systems"
    )
    fig.tight_layout()
    fig.savefig(RESULTS / "comparison_forces_torques.png", dpi=130)

    # deviations from the real rod
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
    rows = []
    for key, (title, unit, f) in quantities.items():
        ref = f(real)
        peak = np.max(np.abs(ref))
        row = {"quantity": f"{title} [{unit}]".replace("|", "\\|"), "peak": f"{peak:.2f}"}
        for v in ("two", "three"):
            diff = f(rec[v]) - ref
            row[v] = f"{np.max(np.abs(diff)):.3f} ({100 * np.max(np.abs(diff)) / peak:.2f} %)"
            if key in ("M", "FA"):
                i = 0 if key == "M" else 1
                ax[i].plot(real["phi_deg"], diff, color=VARIANTS[v][1], label=f"{VARIANTS[v][0]} − real")
                ax[i].set_title(f"Deviation {title}")
                ax[i].set_ylabel(unit)
        rows.append(row)
    for a in ax:
        a.set_xlabel("Crank angle φ in °")
        a.set_xticks(range(0, 361, 45))
        a.grid(alpha=0.3)
        a.legend()
    fig.tight_layout()
    fig.savefig(RESULTS / "deviations.png", dpi=130)

    for v, r in rec.items():
        header = "phi_deg,M_Nm,FAx_N,FAy_N,FBx_N,FBy_N,FN_N,x_piston_m,a_piston_m_s2"
        data = np.column_stack([r["phi_deg"], r["M"], r["FAx"], r["FAy"], r["FBx"], r["FBy"], r["FN"], r["xk"], r["ak"]])
        np.savetxt(
            RESULTS / f"history_{v}.csv", data[:: max(1, len(data) // 720)],
            delimiter=",", header=header, comments="", fmt="%.6g",
        )

    z, dm = two_mass(p), three_mass(p)
    tab = ["| Quantity | Peak (real) | max. deviation two-mass | max. deviation three-mass |", "|---|---|---|---|"]
    tab += [f"| {w['quantity']} | {w['peak']} | {w['two']} | {w['three']} |" for w in rows]
    text = f"""# Connecting rod: real part vs. equivalent system (MuJoCo {mujoco.__version__})

## Connecting rod (Onshape, steel)
- Mass m = {p.m * 1000:.2f} g, eye distance L = {p.L * 1000:.1f} mm
- Centre of mass s = {p.s * 1000:.2f} mm from the big end
- Moment of inertia about S (axis ⟂ plane of motion) J_S = {p.J_S:.5e} kg·m²

## Equivalent systems
| Model | m_A (big end) | m_S (centre of mass) | m_B (small end) | J about S | Error J |
|---|---|---|---|---|---|
| Two-mass (statically equivalent) | {z.m_A * 1000:.2f} g | – | {z.m_B * 1000:.2f} g | {z.J_model:.4e} | {z.delta_J_percent:+.1f} % |
| Three-mass (dynamically equivalent) | {dm.m_A * 1000:.2f} g | {dm.m_S * 1000:.2f} g | {dm.m_B * 1000:.2f} g | {dm.J_model:.4e} | 0 % |

## Simulation
Crank radius {R_CRANK * 1000:.0f} mm, piston mass {M_PISTON * 1000:.0f} g, speed {rpm:.0f} rpm (held constant),
time step {DT:g} s, gravity {"on" if gravity else "off (inertia forces dominate)"}, no gas force, frictionless.

{chr(10).join(tab)}

## Validation
- Piston acceleration vs. analytic solution: max. error {100 * err_a:.3f} %
- Speed ripple of the drive: {100 * dw:.4f} %
- Mean crank torque over one revolution (lossless → 0): {m_mean:.2e} Nm
- Torque error of the two-mass model: simulation {dM_sim:.3f} Nm, theory ½·ΔJ·λ²·ω² = {dM_theory:.3f} Nm

## Figures
- comparison_forces_torques.png – curves over one revolution
- deviations.png – difference equivalent system − real rod
"""
    (RESULTS / "SUMMARY.md").write_text(text)
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rpm", type=float, default=3000.0, help="crank speed in rpm (default 3000)")
    ap.add_argument("--gravity", action="store_true", help="enable gravity")
    args = ap.parse_args()
    p = ConRod.from_onshape()
    omega = args.rpm * 2 * math.pi / 60
    xml = build_model(p, omega, args.gravity)
    (HERE / "slider_crank.xml").write_text(xml)
    _, rec = simulate(xml, p, omega)
    print(evaluate(rec, p, omega, args.rpm, args.gravity))


if __name__ == "__main__":
    main()
