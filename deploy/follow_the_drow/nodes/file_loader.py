#!/usr/bin python3

from rospy import Publisher, Rate, is_shutdown, init_node, spin, loginfo, get_param
from geometry_msgs.msg import Point

from follow_the_drow import HEARTBEAT_RATE, RAW_DATA_TOPIC, FILE_LOADER, DATASET_PATH
from follow_the_drow.msg import raw_data
from data_outrigger.datasets import DROW_Dataset
from data_outrigger.utils.drow_utils import laser_angles
from data_outrigger.utils.file_utils import DROW_TEST_SET


class FileLoader:
    def __init__(self) -> None:
        self.dataset = DROW_Dataset(dataset=get_param(f"/{FILE_LOADER}/{DATASET_PATH}", DROW_TEST_SET))
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
        self.raw_data = Publisher(RAW_DATA_TOPIC, raw_data, queue_size=10)
        file_counter, scan_counter = 0, 0
        rate = Rate(HEARTBEAT_RATE)
        while not is_shutdown():
            if scan_counter >= len(self.dataset.scans[file_counter]):
                file_counter = file_counter + 1 if file_counter < len(self.dataset.scans) - 1 else 0
                scan_counter = 0
            self.update(file_counter, scan_counter)
            scan_counter += 1
            rate.sleep()


if __name__ == "__main__":
    init_node(FILE_LOADER)
    FileLoader().__call__()
    spin()
