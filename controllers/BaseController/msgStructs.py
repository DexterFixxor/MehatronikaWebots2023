from dataclasses import dataclass
import numpy as np


@dataclass
class LidarMsg:
    ranges      : np.ndarray
    angles      : np.ndarray
    fov         : float
    n_samples   : int
