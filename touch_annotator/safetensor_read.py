from safetensors.numpy import save_file, load_file
import numpy as np


if __name__ == '__main__':
    loaded_data = load_file("numpy_topic_data.safetensors")
    numpy_data = {k: np.array(v) for k, v in loaded_data.items()}
    print(numpy_data["timestamps"].shape)  # (1000, 10)