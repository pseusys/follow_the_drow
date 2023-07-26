from typing import Any, Callable

from rospy import init_node, get_param


def _set_attribute(name: str) -> Callable[["Params", Any], None]:
    def set(self, value: Any) -> None:
        setattr(self, name, value)
    return set


def _get_attribute(name: str) -> Callable[["Params"], Any]:
    def get(self: "Params") -> Any:
        try:
            return getattr(self, name)
        except AttributeError:
            raise AttributeError(f"Parameter '{name}' wasn't initialized!!")
    return get


def _property(name: str):
    return property(_get_attribute(name), _set_attribute(name))


class Params:
    # Node names
    LIVE_LOADER = "follow_the_drow_live_loader"
    FILE_LOADER = "follow_the_drow_file_loader"
    VISUALIZER = "follow_the_drow_visualizer"
    ALGORITHMIC_DETECTOR = "follow_the_drow_algorithmic_detector"
    DROW_DETECTOR = "follow_the_drow_drow_detector"

    # General arguments
    HEARTBEAT_RATE = _property("HEARTBEAT_RATE")
    _HEARTBEAT_RATE_NAME = "heartbeat_rate"
    RAW_DATA_TOPIC = _property("RAW_DATA_TOPIC")
    _RAW_DATA_TOPIC_NAME = "raw_data_topic"
    ALGORITHMIC_DETECTOR_TOPIC = _property("ALGORITHMIC_DETECTOR_TOPIC")
    _ALGORITHMIC_DETECTOR_TOPIC_NAME = "algorithmic_detector_topic"
    DROW_DETECTOR_TOPIC = _property("DROW_DETECTOR_TOPIC")
    _DROW_DETECTOR_TOPIC_NAME = "drow_detector_topic"

    # File loader node arguments
    DATASET_PATH = _property("DATASET_PATH")
    _DATASET_PATH_NAME = "dataset_path"


def load_args_for_node(name: str):
    init_node(name)
    Params.HEARTBEAT_RATE = get_param(f"/{Params._HEARTBEAT_RATE_NAME}")
    Params.RAW_DATA_TOPIC = get_param(f"/{Params._RAW_DATA_TOPIC_NAME}")
    Params.ALGORITHMIC_DETECTOR_TOPIC = get_param(f"/{Params._ALGORITHMIC_DETECTOR_TOPIC_NAME}")
    Params.DROW_DETECTOR_TOPIC = get_param(f"/{Params._DROW_DETECTOR_TOPIC_NAME}")

    if name == Params.FILE_LOADER:
        Params.DATASET_PATH = get_param(f"/{name}/{Params._DATASET_PATH_NAME}")
    elif name == Params.DROW_DETECTOR:
        pass
    else:
        raise RuntimeError(f"Unknown node name '{name}'!")
