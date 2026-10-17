#!/usr/bin/env python3
"""Generate independent Featherstone-style ABA oracle data for WP-01.1.

The generator intentionally lives outside `openbmp-multibody` and implements the
angular-over-linear Spatial_v2/RBDA articulated-body recursion directly in
NumPy. It is used to materialize committed TOML tolerance data; it is not a
runtime dependency and does not call the Rust implementation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


def skew(value: Iterable[float]) -> np.ndarray:
    x, y, z = value
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def motion_cross(value: np.ndarray) -> np.ndarray:
    out = np.zeros((6, 6), dtype=float)
    out[:3, :3] = skew(value[:3])
    out[3:, :3] = skew(value[3:])
    out[3:, 3:] = skew(value[:3])
    return out


def force_cross(value: np.ndarray) -> np.ndarray:
    return -motion_cross(value).T


def rotation_about_axis(axis: Iterable[float], angle_rad: float) -> np.ndarray:
    unit = np.asarray(axis, dtype=float)
    unit = unit / np.linalg.norm(unit)
    x, y, z = unit
    sin_angle = math.sin(angle_rad)
    cos_angle = math.cos(angle_rad)
    one_minus_cos = 1.0 - cos_angle
    return np.array(
        [
            [
                cos_angle + x * x * one_minus_cos,
                x * y * one_minus_cos - z * sin_angle,
                x * z * one_minus_cos + y * sin_angle,
            ],
            [
                y * x * one_minus_cos + z * sin_angle,
                cos_angle + y * y * one_minus_cos,
                y * z * one_minus_cos - x * sin_angle,
            ],
            [
                z * x * one_minus_cos - y * sin_angle,
                z * y * one_minus_cos + x * sin_angle,
                cos_angle + z * z * one_minus_cos,
            ],
        ],
        dtype=float,
    )


def rotation_parent_from_child_from_quaternion(q: Iterable[float]) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def motion_transform_matrix(rot_child_from_parent: np.ndarray, translation_parent_m: Iterable[float]) -> np.ndarray:
    out = np.zeros((6, 6), dtype=float)
    translation_parent_m = np.asarray(translation_parent_m, dtype=float)
    out[:3, :3] = rot_child_from_parent
    out[3:, :3] = -rot_child_from_parent @ skew(translation_parent_m)
    out[3:, 3:] = rot_child_from_parent
    return out


def compose_transform(
    first: tuple[np.ndarray, np.ndarray],
    next_: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    first_rot, first_translation = first
    next_rot, next_translation = next_
    return next_rot @ first_rot, first_translation + first_rot.T @ next_translation


def spatial_inertia(
    mass_kg: float = 10.0,
    com_body_m: Iterable[float] = (0.2, -0.3, 0.4),
    inertia_diag_kg_m2: Iterable[float] = (3.0, 4.0, 5.0),
) -> np.ndarray:
    com_cross = skew(com_body_m)
    inertia_body = np.diag(np.asarray(inertia_diag_kg_m2, dtype=float))
    out = np.zeros((6, 6), dtype=float)
    out[:3, :3] = inertia_body - mass_kg * com_cross @ com_cross
    out[:3, 3:] = mass_kg * com_cross
    out[3:, :3] = -mass_kg * com_cross
    out[3:, 3:] = mass_kg * np.eye(3)
    return out


@dataclass(frozen=True)
class Body:
    parent: int | None
    joint: str
    axis: tuple[float, float, float] | None
    inertia: np.ndarray
    fixed_rot: np.ndarray
    fixed_translation: np.ndarray


@dataclass(frozen=True)
class OracleCase:
    case: str
    description: str
    bodies: list[Body]
    q: list[float]
    qd: list[float]
    generalized_forces: list[float]
    root_acceleration: list[float]
    external_forces: list[list[float]]


RNG_SEED = 20260628


def joint_sizes(joint: str) -> tuple[int, int]:
    if joint == "free":
        return 7, 6
    if joint in {"revolute", "prismatic"}:
        return 1, 1
    if joint == "spherical":
        return 4, 3
    if joint == "welded":
        return 0, 0
    raise ValueError(f"unsupported joint {joint}")


def joint_transform(body: Body, q: list[float]) -> tuple[np.ndarray, np.ndarray]:
    if body.joint == "free":
        return rotation_parent_from_child_from_quaternion(q[:4]).T, np.asarray(q[4:7], dtype=float)
    if body.joint == "revolute":
        return rotation_about_axis(body.axis, q[0]), np.zeros(3)
    if body.joint == "prismatic":
        axis = np.asarray(body.axis, dtype=float)
        return np.eye(3), axis / np.linalg.norm(axis) * q[0]
    if body.joint == "spherical":
        return rotation_parent_from_child_from_quaternion(q).T, np.zeros(3)
    if body.joint == "welded":
        return np.eye(3), np.zeros(3)
    raise ValueError(f"unsupported joint {body.joint}")


def joint_subspace(body: Body) -> list[np.ndarray]:
    if body.joint == "free":
        columns = []
        for axis in range(3):
            value = np.zeros(6)
            value[axis] = 1.0
            columns.append(value)
        for axis in range(3):
            value = np.zeros(6)
            value[3 + axis] = 1.0
            columns.append(value)
        return columns
    if body.joint == "revolute":
        value = np.zeros(6)
        value[:3] = np.asarray(body.axis, dtype=float) / np.linalg.norm(body.axis)
        return [value]
    if body.joint == "prismatic":
        value = np.zeros(6)
        value[3:] = np.asarray(body.axis, dtype=float) / np.linalg.norm(body.axis)
        return [value]
    if body.joint == "spherical":
        columns = []
        for axis in range(3):
            value = np.zeros(6)
            value[axis] = 1.0
            columns.append(value)
        return columns
    if body.joint == "welded":
        return []
    raise ValueError(f"unsupported joint {body.joint}")


def offsets(bodies: list[Body]) -> tuple[list[int], list[int], int, int]:
    q_offsets = []
    qd_offsets = []
    q_total = 0
    qd_total = 0
    for body in bodies:
        q_offsets.append(q_total)
        qd_offsets.append(qd_total)
        q_len, qd_len = joint_sizes(body.joint)
        q_total += q_len
        qd_total += qd_len
    return q_offsets, qd_offsets, q_total, qd_total


def articulated_body_forward_dynamics(case: OracleCase) -> np.ndarray:
    q_offsets, qd_offsets, q_count, qd_count = offsets(case.bodies)
    assert len(case.q) == q_count
    assert len(case.qd) == qd_count
    assert len(case.generalized_forces) == qd_count
    transforms = []
    subspaces = []
    for body_index, body in enumerate(case.bodies):
        q_len, _ = joint_sizes(body.joint)
        joint = joint_transform(body, case.q[q_offsets[body_index] : q_offsets[body_index] + q_len])
        transform = compose_transform((body.fixed_rot, body.fixed_translation), joint)
        transforms.append(motion_transform_matrix(*transform))
        subspaces.append(joint_subspace(body))

    velocities = [np.zeros(6) for _ in case.bodies]
    coriolis = [np.zeros(6) for _ in case.bodies]
    articulated_inertias = [body.inertia.copy() for body in case.bodies]
    articulated_biases = [np.zeros(6) for _ in case.bodies]
    joint_u_columns: list[list[np.ndarray]] = [[] for _ in case.bodies]
    joint_d_matrices: list[np.ndarray] = [np.zeros((0, 0)) for _ in case.bodies]
    joint_u_vectors: list[np.ndarray] = [np.zeros(0) for _ in case.bodies]
    external_forces = [np.asarray(force, dtype=float) for force in case.external_forces]

    for body_index, body in enumerate(case.bodies):
        parent_velocity = (
            transforms[body_index] @ velocities[body.parent]
            if body.parent is not None
            else np.zeros(6)
        )
        joint_velocity = np.zeros(6)
        for local, motion in enumerate(subspaces[body_index]):
            joint_velocity += motion * case.qd[qd_offsets[body_index] + local]
        velocity = parent_velocity + joint_velocity
        velocities[body_index] = velocity
        coriolis[body_index] = motion_cross(velocity) @ joint_velocity
        articulated_biases[body_index] = force_cross(velocity) @ (body.inertia @ velocity) - external_forces[body_index]

    for body_index in reversed(range(len(case.bodies))):
        body = case.bodies[body_index]
        subspace = subspaces[body_index]
        u_columns = [articulated_inertias[body_index] @ motion for motion in subspace]
        d_matrix = np.array([[motion @ column for column in u_columns] for motion in subspace], dtype=float)
        u_vector = np.array(
            [
                case.generalized_forces[qd_offsets[body_index] + local]
                - motion @ articulated_biases[body_index]
                for local, motion in enumerate(subspace)
            ],
            dtype=float,
        )
        if len(subspace) == 0:
            reduced_inertia = articulated_inertias[body_index]
            reduced_bias = articulated_biases[body_index] + reduced_inertia @ coriolis[body_index]
        else:
            d_inv_u = np.linalg.solve(d_matrix, u_vector)
            d_inverse = np.linalg.inv(d_matrix)
            projected = np.zeros((6, 6))
            for left, left_column in enumerate(u_columns):
                for right, right_column in enumerate(u_columns):
                    projected += np.outer(left_column, right_column) * d_inverse[left, right]
            reduced_inertia = articulated_inertias[body_index] - projected
            weighted = np.zeros(6)
            for column, weight in zip(u_columns, d_inv_u):
                weighted += column * weight
            reduced_bias = articulated_biases[body_index] + reduced_inertia @ coriolis[body_index] + weighted
        joint_u_columns[body_index] = u_columns
        joint_d_matrices[body_index] = d_matrix
        joint_u_vectors[body_index] = u_vector
        articulated_inertias[body_index] = reduced_inertia
        articulated_biases[body_index] = reduced_bias

        if body.parent is not None:
            parent = body.parent
            articulated_inertias[parent] += transforms[body_index].T @ reduced_inertia @ transforms[body_index]
            articulated_biases[parent] += transforms[body_index].T @ reduced_bias

    accelerations = [np.zeros(6) for _ in case.bodies]
    qdd = np.zeros(qd_count)
    for body_index, body in enumerate(case.bodies):
        parent_acceleration = (
            transforms[body_index] @ accelerations[body.parent]
            if body.parent is not None
            else transforms[body_index] @ np.asarray(case.root_acceleration, dtype=float)
        )
        acceleration = parent_acceleration + coriolis[body_index]
        if subspaces[body_index]:
            rhs = joint_u_vectors[body_index].copy()
            for local, u_column in enumerate(joint_u_columns[body_index]):
                rhs[local] -= u_column @ acceleration
            joint_qdd = np.linalg.solve(joint_d_matrices[body_index], rhs)
            for local, value in enumerate(joint_qdd):
                qdd[qd_offsets[body_index] + local] = value
                acceleration += subspaces[body_index][local] * value
        accelerations[body_index] = acceleration
    return qdd


def fmt(value: float) -> str:
    return f"{value:.17e}"


def vector(values: Iterable[float], indent: str = "  ") -> str:
    return "[\n" + "\n".join(f"{indent}{fmt(value)}," for value in values) + "\n]"


def flat_row_major(matrix: np.ndarray) -> list[float]:
    return [float(matrix[row, col]) for row in range(matrix.shape[0]) for col in range(matrix.shape[1])]


def inertia_fields(inertia: np.ndarray) -> tuple[float, list[float], list[float]]:
    mass = float(inertia[3, 3])
    com_cross = inertia[:3, 3:] / mass
    com = [float(com_cross[2, 1]), float(com_cross[0, 2]), float(com_cross[1, 0])]
    inertia_body = inertia[:3, :3] + mass * skew(com) @ skew(com)
    return mass, com, [float(inertia_body[index, index]) for index in range(3)]


def print_body_table(body_spec: Body) -> None:
    mass, com, inertia_diag = inertia_fields(body_spec.inertia)
    print()
    print("[[cases.bodies]]")
    print(f"parent = {-1 if body_spec.parent is None else body_spec.parent}")
    print(f'joint = "{body_spec.joint}"')
    if body_spec.axis is not None:
        print(f"axis = {vector(body_spec.axis)}")
    print(f"fixed_rotation_row_major = {vector(flat_row_major(body_spec.fixed_rot))}")
    print(f"fixed_translation_m = {vector(body_spec.fixed_translation)}")
    print(f"mass_kg = {fmt(mass)}")
    print(f"center_of_mass_body_m = {vector(com)}")
    print(f"inertia_diag_kg_m2 = {vector(inertia_diag)}")


def body(parent: int | None, joint: str, axis=None, rot=None, translation=None, inertia_matrix=None) -> Body:
    return Body(
        parent=parent,
        joint=joint,
        axis=None if axis is None else tuple(axis),
        inertia=spatial_inertia() if inertia_matrix is None else inertia_matrix,
        fixed_rot=np.eye(3) if rot is None else rot,
        fixed_translation=np.zeros(3) if translation is None else np.asarray(translation, dtype=float),
    )


def random_unit_vector(rng: np.random.Generator) -> tuple[float, float, float]:
    value = rng.normal(size=3)
    value = value / np.linalg.norm(value)
    return tuple(float(component) for component in value)


def random_quaternion(rng: np.random.Generator) -> list[float]:
    value = rng.normal(size=4)
    value = value / np.linalg.norm(value)
    if value[0] < 0.0:
        value = -value
    return [float(component) for component in value]


def random_rotation(rng: np.random.Generator) -> np.ndarray:
    return rotation_about_axis(random_unit_vector(rng), float(rng.uniform(-0.75, 0.75)))


def random_inertia(rng: np.random.Generator) -> np.ndarray:
    mass = float(rng.uniform(3.0, 40.0))
    com = rng.uniform(-0.35, 0.35, size=3)
    base = float(rng.uniform(1.0, 8.0))
    spread = rng.uniform(-0.35, 0.35, size=3)
    inertia_diag = np.array(
        [
            base + 0.1 + abs(spread[0]),
            base + 0.2 + abs(spread[1]),
            base + 0.3 + abs(spread[2]),
        ],
        dtype=float,
    )
    return spatial_inertia(mass, com, inertia_diag)


def random_body(rng: np.random.Generator, parent: int | None, joint: str) -> Body:
    axis = random_unit_vector(rng) if joint in {"revolute", "prismatic"} else None
    return body(
        parent,
        joint,
        axis=axis,
        rot=random_rotation(rng),
        translation=rng.uniform(-0.8, 0.8, size=3),
        inertia_matrix=random_inertia(rng),
    )


def coordinates_for_bodies(rng: np.random.Generator, bodies: list[Body]) -> tuple[list[float], list[float]]:
    q: list[float] = []
    qd: list[float] = []
    for body_spec in bodies:
        if body_spec.joint == "free":
            q.extend(random_quaternion(rng))
            q.extend(float(value) for value in rng.uniform(-0.6, 0.6, size=3))
            qd.extend(float(value) for value in rng.uniform(-0.9, 0.9, size=6))
        elif body_spec.joint == "spherical":
            q.extend(random_quaternion(rng))
            qd.extend(float(value) for value in rng.uniform(-0.8, 0.8, size=3))
        elif body_spec.joint in {"revolute", "prismatic"}:
            q.append(float(rng.uniform(-0.7, 0.7)))
            qd.append(float(rng.uniform(-0.9, 0.9)))
        elif body_spec.joint == "welded":
            continue
        else:
            raise ValueError(f"unsupported joint {body_spec.joint}")
    return q, qd


def random_case(
    rng: np.random.Generator,
    case: str,
    description: str,
    joint_parent_pairs: list[tuple[str, int | None]],
) -> OracleCase:
    bodies = [random_body(rng, parent, joint) for joint, parent in joint_parent_pairs]
    q, qd = coordinates_for_bodies(rng, bodies)
    root_acceleration = [float(value) for value in rng.uniform(-0.05, 0.05, size=3)]
    root_acceleration.extend(
        [
            float(rng.uniform(-0.35, 0.35)),
            float(rng.uniform(-0.35, 0.35)),
            float(-9.81 + rng.uniform(-0.25, 0.25)),
        ]
    )
    return OracleCase(
        case=case,
        description=description,
        bodies=bodies,
        q=q,
        qd=qd,
        generalized_forces=[float(value) for value in rng.uniform(-1.0, 1.0, size=len(qd))],
        root_acceleration=root_acceleration,
        external_forces=[
            [float(value) for value in rng.uniform(-0.6, 0.6, size=6)] for _ in bodies
        ],
    )


def cases() -> list[OracleCase]:
    rng = np.random.default_rng(RNG_SEED)
    return [
        OracleCase(
            case="chain_free_revolute_prismatic",
            description="Independent Python Spatial_v2/RBDA ABA oracle for the existing free-flyer/revolute/prismatic sample chain.",
            bodies=[
                body(None, "free"),
                body(0, "revolute", axis=(0.0, 1.0, 0.0), translation=(1.0, 0.0, 0.0)),
                body(1, "prismatic", axis=(0.0, 0.0, 1.0), translation=(0.0, 0.5, 0.0)),
            ],
            q=[1.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, 0.4, 0.15],
            qd=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8],
            generalized_forces=[1.0, -0.5, 0.25, -0.75, 0.6, -0.4, 0.9, -0.2],
            root_acceleration=[0.0, 0.0, 0.0, 0.0, 0.0, -9.81],
            external_forces=[
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.2, -0.1, 0.05, 0.3, -0.2, 0.1],
                [-0.05, 0.08, -0.03, -0.4, 0.2, -0.1],
            ],
        ),
        OracleCase(
            case="branched_free_revolute_prismatic",
            description="Independent Python Spatial_v2/RBDA ABA oracle for a branched free-flyer tree with revolute and prismatic children.",
            bodies=[
                body(None, "free"),
                body(0, "revolute", axis=(1.0, 1.0, 0.0), translation=(0.8, -0.2, 0.15)),
                body(
                    0,
                    "prismatic",
                    axis=(0.0, 1.0, 1.0),
                    rot=rotation_about_axis((0.0, 0.0, 1.0), 0.35),
                    translation=(-0.4, 0.6, -0.1),
                ),
            ],
            q=[
                0.9233805168766387,
                0.10259783520851541,
                -0.20519567041703082,
                0.3077935056255462,
                -0.25,
                0.35,
                -0.45,
                -0.3,
                0.22,
            ],
            qd=[-0.15, 0.25, -0.35, 0.45, -0.55, 0.65, -0.4, 0.3],
            generalized_forces=[-0.6, 0.2, -0.1, 0.4, -0.3, 0.5, -0.7, 0.8],
            root_acceleration=[0.03, -0.02, 0.01, 0.2, -0.1, -9.7],
            external_forces=[
                [0.1, 0.0, -0.05, 0.2, 0.1, -0.3],
                [-0.2, 0.3, 0.1, -0.1, 0.4, 0.2],
                [0.05, -0.04, 0.03, 0.3, -0.2, 0.1],
            ],
        ),
        OracleCase(
            case="spherical_welded_revolute_chain",
            description="Independent Python Spatial_v2/RBDA ABA oracle for a free-flyer, spherical child, welded offset, and terminal revolute body.",
            bodies=[
                body(None, "free"),
                body(
                    0,
                    "spherical",
                    rot=rotation_about_axis((1.0, 0.0, 0.0), -0.2),
                    translation=(0.3, -0.7, 0.2),
                ),
                body(1, "welded", translation=(0.2, 0.1, -0.4)),
                body(
                    2,
                    "revolute",
                    axis=(0.0, 0.0, 1.0),
                    rot=rotation_about_axis((0.0, 1.0, 0.0), 0.45),
                    translation=(0.0, 0.4, 0.6),
                ),
            ],
            q=[
                0.879923176281257,
                -0.2666433867518961,
                0.10665735470075843,
                -0.3733007414526545,
                0.6,
                -0.4,
                0.2,
                0.9659258262890683,
                0.0,
                0.25881904510252074,
                0.0,
                -0.12,
            ],
            qd=[0.33, -0.11, 0.22, -0.44, 0.66, -0.88, 0.5, -0.25, 0.4, -0.35],
            generalized_forces=[0.15, 0.35, -0.45, 0.55, -0.65, 0.75, 0.25, -0.35, 0.45, -0.55],
            root_acceleration=[-0.04, 0.05, -0.02, -0.3, 0.25, -9.6],
            external_forces=[
                [-0.08, 0.06, 0.04, -0.2, 0.3, -0.1],
                [0.12, -0.18, 0.09, 0.4, -0.3, 0.2],
                [-0.07, 0.05, -0.02, -0.5, 0.1, 0.3],
                [0.03, -0.02, 0.01, 0.2, 0.15, -0.25],
            ],
        ),
        random_case(
            rng,
            "seeded_random_chain_4",
            "Seeded independent Python RBDA oracle for a four-body free/revolute/prismatic/revolute chain.",
            [("free", None), ("revolute", 0), ("prismatic", 1), ("revolute", 2)],
        ),
        random_case(
            rng,
            "seeded_random_branch_5",
            "Seeded independent Python RBDA oracle for a branched five-body free/spherical/welded/revolute/prismatic tree.",
            [("free", None), ("spherical", 0), ("welded", 1), ("revolute", 2), ("prismatic", 0)],
        ),
        random_case(
            rng,
            "seeded_random_mixed_6",
            "Seeded independent Python RBDA oracle for a mixed six-body tree with two branches and a welded interior body.",
            [
                ("free", None),
                ("revolute", 0),
                ("revolute", 1),
                ("welded", 1),
                ("spherical", 3),
                ("prismatic", 0),
            ],
        ),
        random_case(
            rng,
            "seeded_random_deep_6",
            "Seeded independent Python RBDA oracle for a deeper six-body free/prismatic/spherical/revolute/welded/revolute chain.",
            [
                ("free", None),
                ("prismatic", 0),
                ("spherical", 1),
                ("revolute", 2),
                ("welded", 3),
                ("revolute", 4),
            ],
        ),
    ]


def main() -> None:
    print("[source]")
    print('solver = "independent-python-rbda-aba"')
    print('export_format = "spatial-v2-aba-qdd-v1"')
    print('independence = "independent-generator"')
    print('method = "Featherstone RBDA Table 7.1 / Spatial_v2 angular-over-linear conventions"')
    print('generator = "scripts/generate_spatial_v2_independent_aba_oracle.py"')
    print('notes = "Generated by a NumPy implementation independent of openbmp-multibody runtime code; no MATLAB/Octave executable was available in the local environment."')
    print()
    print("[tolerances]")
    print("qdd_max_abs = 1.0e-11")
    for case in cases():
        qdd = articulated_body_forward_dynamics(case)
        print()
        print("[[cases]]")
        print(f'case = "{case.case}"')
        print(f'description = "{case.description}"')
        print(f"q = {vector(case.q)}")
        print(f"qd = {vector(case.qd)}")
        print(f"generalized_forces = {vector(case.generalized_forces)}")
        print(f"expected_qdd = {vector(qdd)}")
        print()
        print("[cases.root_acceleration]")
        print(f"angular = {vector(case.root_acceleration[:3])}")
        print(f"linear = {vector(case.root_acceleration[3:])}")
        for force in case.external_forces:
            print()
            print("[[cases.external_forces]]")
            print(f"moment = {vector(force[:3])}")
            print(f"force = {vector(force[3:])}")
        for body_spec in case.bodies:
            print_body_table(body_spec)


if __name__ == "__main__":
    main()
