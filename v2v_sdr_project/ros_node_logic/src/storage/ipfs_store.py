import subprocess
import numpy as np

def store_to_ipfs(np_array):
    np.save("/tmp/fused.npy", np_array)
    out = subprocess.check_output(["ipfs", "add", "/tmp/fused.npy"])
    return out.decode().strip()
