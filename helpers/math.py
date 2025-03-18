import numpy as np

def manhattan_distance(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Compute Manhattan distance between:
    - Two vectors (returns a float)
    - A set of vectors (returns an array of floats)
    """
    
    distances = np.sum(np.abs(x - y), axis=-1)
    
    # Ensure float output for single distance
    return distances.item() if distances.shape == () else distances