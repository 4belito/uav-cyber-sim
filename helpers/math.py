import numpy as np
from numpy.typing import NDArray


def manhattan_distance(
    x: NDArray[np.float64], y: NDArray[np.float64]
) -> float | NDArray[np.float64]:
    """
    Compute the Manhattan distance between:
    - Two vectors → returns a float
    - Two arrays → returns an array of floats
    """
    return np.sum(np.abs(x - y), axis=-1).squeeze()
