"""Equivalent mass systems for a connecting rod, computed from CAD mass properties (Onshape).

Notation (everything in the plane of motion, rotation axis z):
    A   centre of the big end (crank pin)       x = 0
    B   centre of the small end (piston pin)    x = L
    S   centre of mass                          x = s
    m   rod mass,  J_S  moment of inertia about S (z axis)

1) Two-mass model (statically equivalent) - point masses at A and B
       m_A + m_B = m                      (mass)
       m_A * s   = m_B * (L - s)          (centre of mass)
   =>  m_A = m (L - s) / L,   m_B = m s / L
   Moment of inertia of the model about S:  J_2 = m_A s² + m_B (L - s)² = m s (L - s)
   Error compared with the real rod:  ΔJ = J_S - J_2   (generally ≠ 0)

2) Three-mass model (dynamically equivalent) - point masses at A, S and B
       m_A + m_S + m_B = m
       m_A s = m_B (L - s)
       m_A s² + m_B (L - s)² = J_S
   =>  m_A = J_S / (s L),   m_B = J_S / ((L - s) L),   m_S = m - m_A - m_B
   Mass, centre of mass AND moment of inertia match exactly -> identical planar dynamics.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

HERE = Path(__file__).parent


@dataclass
class ConRod:
    m: float  # kg
    s: float  # m, distance of the centre of mass from the big end
    L: float  # m, distance between the eye centres
    J_S: float  # kg m², about the centre of mass, axis normal to the plane of motion
    inertia_full: list  # 3x3 tensor about S (rows), from Onshape

    @classmethod
    def from_onshape(cls, path: Path = HERE / "conrod_data.json") -> "ConRod":
        d = json.loads(path.read_text())
        A, B, S = d["big_end_m"], d["small_end_m"], d["center_of_mass_m"]
        I = d["inertia_about_com_kgm2"]
        return cls(m=d["mass_kg"], s=S[0] - A[0], L=B[0] - A[0], J_S=I[8], inertia_full=[I[0:3], I[3:6], I[6:9]])


@dataclass
class TwoMass:
    m_A: float
    m_B: float
    J_model: float
    delta_J: float
    delta_J_percent: float


@dataclass
class ThreeMass:
    m_A: float
    m_S: float
    m_B: float
    J_model: float


def two_mass(p: ConRod) -> TwoMass:
    m_A = p.m * (p.L - p.s) / p.L
    m_B = p.m * p.s / p.L
    J2 = m_A * p.s**2 + m_B * (p.L - p.s) ** 2
    return TwoMass(m_A, m_B, J2, p.J_S - J2, 100.0 * (J2 - p.J_S) / p.J_S)


def three_mass(p: ConRod) -> ThreeMass:
    m_A = p.J_S / (p.s * p.L)
    m_B = p.J_S / ((p.L - p.s) * p.L)
    m_S = p.m - m_A - m_B
    if m_S < 0:
        raise ValueError("m_S < 0: J_S > m s (L - s) - no three-mass model with positive masses exists")
    J3 = m_A * p.s**2 + m_B * (p.L - p.s) ** 2  # m_S sits at S and contributes nothing
    return ThreeMass(m_A, m_S, m_B, J3)


def report(p: ConRod) -> str:
    z, d = two_mass(p), three_mass(p)
    g = lambda kg: f"{kg * 1000:8.2f} g"  # noqa: E731
    return "\n".join(
        [
            "Connecting rod (from Onshape)",
            f"  m   = {g(p.m)}",
            f"  L   = {p.L * 1000:8.2f} mm   (distance between eye centres)",
            f"  s   = {p.s * 1000:8.2f} mm   (centre of mass from big end)",
            f"  J_S = {p.J_S:.5e} kg m²",
            "",
            "Two-mass model (statically equivalent)",
            f"  m_A = {g(z.m_A)}   (big end, rotating)",
            f"  m_B = {g(z.m_B)}   (small end, reciprocating)",
            f"  J_model = m s (L-s) = {z.J_model:.5e} kg m²",
            f"  error ΔJ = J_S - J_model = {z.delta_J:.5e} kg m²  ({z.delta_J_percent:+.1f} % too large)",
            "",
            "Three-mass model (dynamically equivalent)",
            f"  m_A = {g(d.m_A)}   m_S = {g(d.m_S)}   m_B = {g(d.m_B)}",
            f"  J_model = {d.J_model:.5e} kg m²  (= J_S)",
        ]
    )


if __name__ == "__main__":
    p = ConRod.from_onshape()
    print(report(p))
    out = {"conrod": asdict(p), "two_mass": asdict(two_mass(p)), "three_mass": asdict(three_mass(p))}
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results" / "equivalent_system.json").write_text(json.dumps(out, indent=2))
