from tqdm.auto import trange

from ..datasets import DROW_Dataset
from ..utils.drow_utils import laser_minimum, laser_increment
from ..cpp_binding import DetectorFactory

from .detector import Detector


class AlgorithmicDetector(Detector):
    def __init__(self, time_frame_size: int = DROW_Dataset.TIME_FRAME, verbose: bool = True, *args, **kwargs):
        Detector.__init__(self, verbose)
        self.time_frame = time_frame_size
        self._detector = DetectorFactory(laser_minimum, laser_increment, verbose)

    def forward_one(self, scans, odoms):
        return self._detector.forward_one(scans, odoms)

    def forward_all(self, va: DROW_Dataset):
        people = list()
        for iseq in trange(len(va.det_id), desc="Sequences", disable=not self._verbose):
            for idet in trange(len(va.det_id[iseq]), desc="Scans", disable=not self._verbose, leave=False):
                iscan = va.idet2iscan[iseq][idet]
                scans, odoms = va.get_scan(iseq, iscan, self.time_frame)
                for scan, odom in zip(scans, odoms):
                    people += [self.forward_one(scan, odom["xya"])]
        return people
