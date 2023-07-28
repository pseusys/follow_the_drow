#!/usr/bin python3

from pathlib import Path

from rospy import Publisher, Rate, is_shutdown, spin, loginfo
from geometry_msgs.msg import Point

from follow_the_drow import Params, load_args_for_node
from follow_the_drow.msg import raw_data
from follow_the_drow.datasets import DROW_Dataset
from follow_the_drow.utils.drow_utils import laser_angles


class FileLoader:
    def __init__(self, dataset: Path, verbose: bool) -> None:
        self.dataset = DROW_Dataset(dataset=dataset, verbose=verbose)
        self.raw_data = Publisher(Params.RAW_DATA_TOPIC, raw_data, queue_size=10)
        self.rate = Rate(Params.HEARTBEAT_RATE)
        if verbose:
            loginfo(f"Dataset includes {len(self.dataset.scan_id)} scans and {len(self.dataset.det_id)} detections")

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
        while not is_shutdown():
            if scan_counter >= len(self.dataset.scans[file_counter]):
                file_counter = file_counter + 1 if file_counter < len(self.dataset.scans) - 1 else 0
                scan_counter = 0
            self.update(file_counter, scan_counter)
            scan_counter += 1
            self.rate.sleep()


if __name__ == "__main__":
    load_args_for_node(Params.FILE_LOADER)
    FileLoader(Params.DATASET_PATH, Params.FILE_LOADER_VERBOSE).__call__()
    spin()
