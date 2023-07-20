#!/usr/bin python3

from rospy import Publisher, Rate, is_shutdown, init_node, spin, loginfo
from geometry_msgs.msg import Point

# TODO: FIX!!
import sys, os
sys.path += ["/~/catkin_ws/devel/lib/python3/dist-packages"]
print(sys.path)

from follow_the_drow.msg import raw_data
from data_outrigger.datasets import Dataset
from data_outrigger.utils.drow_utils import _laser_angles as laser_angles, _rphi_to_xy as rphi_to_xy

RAW_DATA_TOPIC = "raw_data"


class FileLoader:
    def __init__(self) -> None:
        self.dataset = Dataset()
        loginfo(f"Dataset includes {len(self.dataset.scan_id)} scans and {len(self.dataset.det_id)} detections")

    def update(self, counter):
        bottom_lidar_points = list()
        scans = self.dataset.scans[counter]
        angles = laser_angles(scans.shape[-1])
        bottom_lidar_points = [Point(x=x, y=y) for x, y in zip(*rphi_to_xy(scans, angles))]
        self.raw_data.publish(raw_data(bottom_lidar=bottom_lidar_points))

    def __call__(self) -> None:
        self.raw_data = Publisher(RAW_DATA_TOPIC, raw_data, queue_size=10)
        counter = 0
        rate = Rate(10)
        while not is_shutdown():
            if counter >= len(self.dataset.scans):
                counter = 0
            self.update(counter)
            counter += 1
            rate.sleep()


if __name__ == "__main__":
    init_node("file_loader")
    FileLoader().__call__()
    spin()
