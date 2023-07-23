from importlib import reload

from rospy import init_node, get_param


# Node names
LIVE_LOADER = "follow_the_drow_live_loader"
FILE_LOADER = "follow_the_drow_file_loader"
VISUALIZER = "follow_the_drow_visualizer"
ALGORITHMIC_DETECTOR = "follow_the_drow_algorithmic_detector"

# General arguments
HEARTBEAT_RATE: int = ...
_HEARTBEAT_RATE_NAME = "heartbeat_rate"
RAW_DATA_TOPIC: str = ...
_RAW_DATA_TOPIC_NAME = "raw_data_topic"
ALGORITHMIC_DETECTOR_TOPIC: str = ...
_ALGORITHMIC_DETECTOR_TOPIC_NAME = "algorithmic_detector_topic"

# File loader node arguments
DATASET_PATH: str = ...
_DATASET_PATH_NAME = "dataset_path"


def load_args_for_node(name: str, this_module):
    global HEARTBEAT_RATE, RAW_DATA_TOPIC, ALGORITHMIC_DETECTOR_TOPIC, DATASET_PATH
    init_node(name)
    HEARTBEAT_RATE = get_param(f"/{_HEARTBEAT_RATE_NAME}")
    RAW_DATA_TOPIC = get_param(f"/{_RAW_DATA_TOPIC_NAME}")
    ALGORITHMIC_DETECTOR_TOPIC = get_param(f"/{_ALGORITHMIC_DETECTOR_TOPIC_NAME}")

    if name == FILE_LOADER:
        DATASET_PATH = get_param(f"/{name}/{_DATASET_PATH_NAME}")
    else:
        raise RuntimeError(f"Unknown node name '{name}'!")
    reload(this_module)
