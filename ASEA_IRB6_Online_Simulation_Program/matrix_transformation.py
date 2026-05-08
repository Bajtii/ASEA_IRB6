from functools import reduce
import numpy as np
from numpy import typing as npt


def rot_z(theta: float) -> npt.NDArray:
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def trans_z(d: float) -> npt.NDArray:
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])


def trans_x(a: float) -> npt.NDArray:
    return np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def rot_x(alpha: float) -> npt.NDArray:
    return np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(alpha), -np.sin(alpha), 0],
            [0, np.sin(alpha), np.cos(alpha), 0],
            [0, 0, 0, 1],
        ]
    )


def DH_elementary_matrix(
    *, theta: float = 0, d: float = 0, a: float = 0, alpha: float = 0
) -> npt.NDArray:
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)

    A = (rot_z(theta), trans_z(d), trans_x(a), rot_x(alpha))

    return reduce(np.dot, A)

