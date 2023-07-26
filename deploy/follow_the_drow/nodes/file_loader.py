#!/usr/bin python3

from pathlib import Path

from rospy import Publisher, Rate, is_shutdown, spin, loginfo
from geometry_msgs.msg import Point

import follow_the_drow as ftd
from follow_the_drow.msg import raw_data
from follow_the_drow.datasets import DROW_Dataset
from follow_the_drow.utils.drow_utils import laser_angles


class FileLoader:
    def __init__(self, dataset: Path) -> None:
        self.dataset = DROW_Dataset(dataset=dataset)
        loginfo(f"Dataset includes {len(self.dataset.scan_id)} scans and {len(self.dataset.det_id)} detections")
        self.raw_data = Publisher(ftd.RAW_DATA_TOPIC, raw_data, queue_size=10)

    def update(self, file_counter, scan_counter):
        bottom_lidar_points = list()
        scans = self.dataset.scans[file_counter][scan_counter]
        angles = laser_angles(scans.shape[-1])
        bottom_lidar_points = [Point(x=x, y=y) for x, y in zip(scans, angles)]
        odom_x, odom_y, odom_a = self.dataset.odoms[file_counter][scan_counter]["xya"]
        odometry_data = Point(x=odom_x, y=odom_y, z=odom_a)
        self.raw_data.publish(raw_data(bottom_lidar=bottom_lidar_points, odometry=odometry_data))

    def __call__(self) -> None:
        file_counter, scan_counter = 0, 0
        rate = Rate(ftd.HEARTBEAT_RATE)
        while not is_shutdown():
            if scan_counter >= len(self.dataset.scans[file_counter]):
                file_counter = file_counter + 1 if file_counter < len(self.dataset.scans) - 1 else 0
                scan_counter = 0
            self.update(file_counter, scan_counter)
            scan_counter += 1
            rate.sleep()


if __name__ == "__main__":
    ftd.load_args_for_node(ftd.FILE_LOADER, ftd)
    FileLoader(ftd.DATASET_PATH).__call__()
    spin()
