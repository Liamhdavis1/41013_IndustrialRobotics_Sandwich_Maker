import os
import numpy as np

ENV_OFFSET = np.array([1.5, 0.5, 0.0])

current_path = os.path.abspath(os.path.dirname(__file__))
INGREDIENTS = {
    "glass": {
        "path": os.path.join(current_path, "env", "glass.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 0.4]
    },
    "bench": {
        "path": os.path.join(current_path, "env", "benchv2.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 1]
    },
    "bread_rack": {
        "path": os.path.join(current_path, "env", "bread_rack.stl"),
        "scale": (1, 1, 1),
        "color": [0.6, 0.6, 0.6, 1]
    },
        "tray": {
        "path": os.path.join(current_path, "env", "tray.stl"),
        "scale": (0.065, 0.08, 0.08),
        "color": [0.8, 0.8, 0.8, 1]
        },
    "bread_bottom": {
        "path": os.path.join(current_path, "env", "sandwich", "bread-bottom.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.95, 0.77, 0.53, 1]
    },
    "bread_top": {
        "path": os.path.join(current_path, "env", "sandwich", "bread-top.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.95, 0.77, 0.53, 1]
    },
    "ham": {
        "path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.7, 0.5, 0.7, 1]
    },
    "lettuce": {
        "path": os.path.join(current_path, "env", "sandwich", "lettace.stl"),
        "scale": (0.01, 0.01, 0.01),
        "color": [0.25, 0.86, 0.37, 1]
    },
    "tomato": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.05),
        "color": [0.8, 0.1, 0.1, 1]
    },
    "salami": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.03),
        "color": [0.7, 0.3, 0.3, 1]
    },
    "beef": {
        "path": os.path.join(current_path, "env", "sandwich", "ham.stl"),
        "scale": (0.1, 0.1, 0.1),
        "color": [0.56, 0.3, 0.04, 1]
    },
    "chicken": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.05, 0.05, 0.03),
        "color": [0.9, 0.8, 0.6, 1]
    },
    "cucumber": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.04, 0.04, 0.04),
        "color": [0.2, 0.6, 0.3, 1]
    },
    "beetroot": {
        "path": os.path.join(current_path, "env", "sandwich", "tomato.stl"),
        "scale": (0.04, 0.04, 0.04),
        "color": [0.5, 0.09, 0.4, 1]
    }
}